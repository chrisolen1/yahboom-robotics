"""
LOAD NECESSARY MODULES
"""

import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0,parentdir) 

import pandas as pd
import numpy as np
import os
import warnings
from scipy.io import wavfile
from hmmlearn import hmm  
import pomegranate
from python_speech_features import mfcc
from sklearn.externals import joblib
import hmm_model_feature_extraction
from model_utils import HMM_Model
import random
from tqdm import tqdm
from ast import literal_eval
from google.cloud import storage

"""
MODEL CONFIGURATIONS
"""

import argparse
parser = argparse.ArgumentParser(description='train parser')
parser.add_argument('--gcp', action='store_true', dest='gcp', help='affects whether to configure to running on the cloud')
parser.add_argument('--local', action='store_false', dest='gcp', help='affects whether to configure to running on the cloud')
parser.add_argument('--pre_populated_validation_set', action='store_true',dest='pre_populated_validation_set', \
help='affects whether we use our pre-populated validation set for validation - recommended for cloud')
parser.add_argument('--random_validation_set', action='store_false',dest='pre_populated_validation_set', \
help='affects whether we use our pre-populated validation set for validation - recommended for cloud')
parser.add_argument('--trial_name', action='store', dest='trial_name', help='indicate name of trial')
parser.add_argument('--num_iterations', action='store', dest='num_iterations', type=int, \
                    help='indicate max number of iterations to train')
parser.add_argument('--num_cep_coefs', action='store', dest='num_cep_coefs', type=int, \
                    help='indicate number of cepstral coefficients to use')
parser.add_argument('--multidimensional_input', action='store_true', dest='multidimensional_input', default=True, \
                    help='indicate whether we will feed concatenated, 2-dimensional cepstral coefficients for each track as a sample')
parser.add_argument('--unidimensional_input', action='store_false', dest='multidimensional_input', default=False, \
                    help='indicate whether we will feed 1-dimensional cepstral coefficients for each window as a sample')
parser.add_argument('--use_pomegranate', action='store_true', dest='use_pomegranate', default=True, \
                    help='indicate whether to use the pomegranate package for fitting hmm')
parser.add_argument('--use_hmmlearn', action='store_false', dest='use_pomegranate', default=False, \
                    help='indicate whether to use hmmlearn for fitting hmm')
parser.add_argument('--distribution', action='store', dest='distribution', help='indicate what distribution to use for the hidden states')
parser.add_argument('--n_threads', action='store', dest='n_threads', type=int,\
                    help='indicate how many threads should we use to train the model')
parser.add_argument('--training_prop', action='store', dest='training_prop', type=float, \
                    help='training-validation split: 1 if using the set aside validation set')

parse_results = parser.parse_args()

### SPECIFY WHERE WE'RE RUNNING ###
gcp = parse_results.gcp
### WHERE SAMPLES ARE LOCATED ###
pre_populated_validation_set = parse_results.pre_populated_validation_set
### name this particular trial ###
trial_name = parse_results.trial_name
print("trial_name: ", trial_name)
### max number of iterations the E-M algorithm performs during training ### 
num_iterations = parse_results.num_iterations
print("num_iterations: ", num_iterations)
### number of cepstral coefficients to return from mfcc (13 is typical) ###
num_cep_coefs = parse_results.num_cep_coefs
print("num_cep_coefs: ", num_cep_coefs)
### whether we will feed concatenated, 2-dimensional cepstral coefficients for each track as a sample ###
multidimensional_input = parse_results.multidimensional_input
print("multidimensional_input: ", multidimensional_input)
### whether to use the pomegranate package or hmmlearn for fitting hmm ###
use_pomegranate = parse_results.use_pomegranate
print("use_pomegranate: ", use_pomegranate)
### what distribution to use for the hidden states ###
distribution = parse_results.distribution
if distribution == "MultivariateGaussianDistribution":
    distribution = pomegranate.distributions.MultivariateGaussianDistribution
elif distribution == "DirichletDistribution":
    distribution = pomegranate.distributions.DirichletDistribution
elif distribution == "NormalDistribution":
    distribution = pomegranate.distributions.NormalDistribution
print("distribution: ", distribution)
### how many threads should we use to train the model ###
n_threads = parse_results.n_threads
print("n_threads: ", n_threads)
### training-validation split: 1 if using the set aside validation set ###
training_prop = parse_results.training_prop
print("training proportion: ", training_prop)

"""
ADDITIONAL CONFIGURATIONS BASED ON COMPUTE LOCATION
"""

if gcp == True:
    
    import gcsfs
    import pickle
    import cupy
    
    # specify gcs bucket
    bucket_name = "ad-bucket-15730"
    # set cloud based mixed dir
    gcs_mixed_dir = "gs://{}/mixed_20k".format(bucket_name)
    # set cloud based hmm model dir
    gcs_hmm_model_dir = "gs://{}/hmm_models".format(bucket_name)
    # set cloud based validation dir
    gcs_validation_dir = "gs://{}/validation".format(bucket_name)
    # set local, vm-based mixed dir
    local_mixed_dir = "mixed_local"
    # set local, vm-based validation dir
    local_validation_dir = "validation_local"
    # initialize gcsfs object
    fs = gcsfs.GCSFileSystem(project = 'audio-detection-1')
    metadata = pd.read_csv(gcs_mixed_dir + "/mixed_metadata.csv")
	
    # enable gpus for pomegranate
    pomegranate.utils.enable_gpu()
    print("communicating with GPU: ", pomegranate.utils.is_gpu_enabled())
    
    ### moving pre-populated validation and training set ###
    ### for faster model validation ###
    
    # need to create folder on vm instance for validation set
    if "validation_local" not in os.listdir():
        
        os.mkdir("validation_local")
        
    # need to create folder on vm instance for training (mixed) set 
    if "mixed_local" not in os.listdir():
        
        os.mkdir("mixed_local")
        
    # if fewer then ten (arbitrary) files, copy all in from gcs   
    if len(os.listdir("validation_local")) < 10:
        
        os.system("gsutil -m cp {}/* ./{}".format(gcs_validation_dir, local_validation_dir))
        
    # if fewer then ten (arbitrary) files, copy all in from gcs    
    if len(os.listdir("mixed_local")) < 10:
        
        os.system("gsutil -m cp {}/* ./{}".format(gcs_mixed_dir, local_mixed_dir))

    
else:
    
    # configuration when not running on the cloud
    local_mixed_dir = "../../../mixed"
    local_hmm_model_dir = "../../../hmm_models"
    metadata = pd.read_csv(local_mixed_dir + "/mixed_metadata.csv")



### initialize feature extraction class ###
# mixed_dir will depend on whether we are pulling from gcs or locally
# sampling_freq will depend on how we initially processed our audio files
# gcs will depend on whether we want to pull from gcs during training or from local directory
fe = hmm_model_feature_extraction.feature_extraction(mixed_dir=local_mixed_dir, sampling_freq = 20000, gcs = False)

### choose hidden states per self-organizing maps

label_states = pd.Series({"air_conditioner":20, "car_horn":18, "children_playing":11,
             "dog_bark":17, "drilling":20, "engine_idling":20,
             "gun_shot":9, "jackhammer":18, "siren":17,
             "street_music":17})

"""
VALIDATION SET CONFIGURATIONS if pre_populated_validation_set == True
"""

if pre_populated_validation_set == True:
    
    # get list of validation sample files
    validation_samples_list = os.listdir("validation_local")
    # pull just the validation samples from the metadata df
    validation_df = metadata[metadata['slice_file_name'].isin(validation_samples_list)]
    # convert validation_df to list of tuples
    validation_samples = [(validation_df['slice_file_name'].iloc[i], validation_df['class'].iloc[i]) for i in range(len(validation_df))]
    # retain only the training samples in the metadata df 
    metadata = metadata[~metadata['slice_file_name'].isin(validation_samples_list)]
    
        
### Trains a single HMM ###
        
def build_one_model(features_vector, num_states, num_iterations, multidimensional_input, \
                    use_pomegranate, distribution, gpu, n_threads, batches_per_epoch, lr_decay):
    
    """
    features_vector: nparray of features from Class above
    num_states: number of hidden states in HMM
    """
    
    # initiate HMM model object
    with warnings.catch_warnings():
        warnings.filterwarnings('ignore',category=DeprecationWarning)
        model = HMM_Model(num_components=num_states, num_iter=num_iterations, 
                          use_pomegranate=use_pomegranate, distribution=distribution, gpu=gpu)

    # train HMM model, calculate likelihood of the sample by the trained model
    with warnings.catch_warnings():
        warnings.filterwarnings('ignore',category=DeprecationWarning)
        model.train(features_vector, multidimensional_input, n_threads, batches_per_epoch, lr_decay)
        model_score = model.compute_score(features_vector)
        
    return model, model_score


def hmm_aic(LLik, n, k):
    
    """
    Takes in loglikelihood of trained HMM plus number of params
    Returns: Model AIC
    """
    return -2*LLik+2*(n*n+k*n-1)

### Pre-training procedures ###

def build_all_models(label_name, metadata, num_states, num_iterations, training_prop=0.7, \
                     multidimensional_input=False, use_pomegranate=True, \
                     distribution = pomegranate.NormalDistribution, gpu=False, n_threads=2,
                     batches_per_epoch=50, lr_decay=0.01):
    
    """
    Given an input data folder with subfolders for each response label
    """
    
    # empty list for storing model results
    model_results = []
    validation_set = []
    

    # load in filenames relevant to a particular label
    label_file_names = metadata[metadata['class'] == label_name]['slice_file_name'].tolist()
    # reserve 30 percent for validation
    num_to_train = int(np.ceil(len(label_file_names) * training_prop)) # may need to reduce for pomegranate training
    train = random.sample(label_file_names, num_to_train)
    print(label_name,"train length:", len(train))
    validate = list(set(label_file_names) - set(train))
    print(label_name," validation length:", len(validate))
    # append the list of validation filenames and respective label to validation_set
    validation_set.append((validate, label_name))
    
    # for unidimensional inputs
    if multidimensional_input == False:
    
        # create an empty array for appending features
        X = np.array([])
    
        print("loading in files for label: ", label_name)

        for j in tqdm(range(len(train))):
        
            loaded = fe.read(train[j])
            # nfft assumes 44100Hz
            mfcc_features = fe.return_mfcc(loaded, nfft=1200)

            if len(X) == 0:
                
                X = mfcc_features
            
            else:
                
                X = np.append(X, mfcc_features, axis=0)
            
        model = build_one_model(X, num_states, num_iterations, multidimensional_input, \
                                use_pomegranate, distribution, gpu, n_threads, batches_per_epoch, lr_decay)
        
        # add the model to the results list
        model_results.append((model, label_name))
        
        # reset model variable
        model = None

        return model_results, validation_set
    
    # for multidimensional inputs
    else:
        
        # create an empty array for appending features
        X = np.array([])
    
        print("loading in files for label: ", label_name)
        
        # need to count the number of samples not discarded
        counter = 0
        for j in tqdm(range(len(train))):
        
            loaded = fe.read(train[j])
            # nfft assumes 44100Hz
            mfcc_features = fe.return_mfcc(loaded, nfft=1200)
            
            # need to have all mfcc features the same 
            if mfcc_features.shape[0] == 399: # this number likely not robust to other sampling rates

                if len(X) == 0:
                
                    X = mfcc_features
                    counter += 1
            
                else:
                
                    X = np.append(X, mfcc_features, axis=0)
                    counter += 1
                    
            else:
                
                pass
        
        
		# (n_samples x n_windows x n_cepstral_coefs
        X = X.reshape(counter, 399, num_cep_coefs) # this number likely not robust to other sampling rates
        model = build_one_model(X, num_states, num_iterations, multidimensional_input, \
                                use_pomegranate, distribution, gpu, n_threads, batches_per_epoch, lr_decay)
        
        # add the model to the results list
        model_results.append((model, label_name))
        
        # Reset model variable
        model = None

        return model_results, validation_set
                
                
        
"""
INITIATE TRAINING
"""

# empty list to which to append fitted hmms and validation sets

models, validation_sample = [],[]

# loop through each of the labels
for i in range(len(label_states.index)):
    
    # extract the label name
    label_name = label_states.index[i]
    # extract corresponding chosen number of states
    num_states = label_states[label_states.index[i]]
    
    # send through model build functions
    ### we make training_prop = 1 when on the cloud ###
    model_results, validation_set = build_all_models(label_name, metadata, num_states, num_iterations, \
                                                     training_prop = training_prop, \
                                                     multidimensional_input = multidimensional_input, \
                                                     use_pomegranate = use_pomegranate, \
                                                     distribution = distribution, \
                                                    gpu=False, n_threads = n_threads, \
                                                    batches_per_epoch=10000, lr_decay=0.5)
    models.append(model_results)
    validation_sample.append(validation_set)

"""
VALIDATION CONFIGURATION IF pre_populated_validation_set == FALSE
"""
        
# format validation_sample outputs for validation, only if not using the set-aside validation set

if pre_populated_validation_set == False:
    
    validation_samples = [(validation_sample[0][0][0][i],validation_sample[0][0][1]) for i in range(len(validation_sample[0][0][0]))] + \
[(validation_sample[1][0][0][i],validation_sample[1][0][1]) for i in range(len(validation_sample[1][0][0]))] + \
[(validation_sample[2][0][0][i],validation_sample[2][0][1]) for i in range(len(validation_sample[2][0][0]))] + \
[(validation_sample[3][0][0][i],validation_sample[3][0][1]) for i in range(len(validation_sample[3][0][0]))] + \
[(validation_sample[4][0][0][i],validation_sample[4][0][1]) for i in range(len(validation_sample[4][0][0]))] + \
[(validation_sample[5][0][0][i],validation_sample[5][0][1]) for i in range(len(validation_sample[5][0][0]))] + \
[(validation_sample[6][0][0][i],validation_sample[6][0][1]) for i in range(len(validation_sample[6][0][0]))] + \
[(validation_sample[7][0][0][i],validation_sample[7][0][1]) for i in range(len(validation_sample[7][0][0]))] + \
[(validation_sample[8][0][0][i],validation_sample[8][0][1]) for i in range(len(validation_sample[8][0][0]))] + \
[(validation_sample[9][0][0][i],validation_sample[9][0][1]) for i in range(len(validation_sample[9][0][0]))]    
        
"""
SAVING VALIDATION SAMPLE LIST FOR SUBSEQUENT VALIDATION 
"""
        
# save validation_sample to txt 
        
if gcp == False:
    
    with open(local_hmm_model_dir + '/validation_samples_{}.txt'.format(trial_name),'w') as file:
    
        for ele in validation_samples:
        
            file.write(str(ele)+'\n')
    
        file.close()
         
else:
    
    with fs.open(gcs_hmm_model_dir + '/validation_samples_{}.txt'.format(trial_name),'w') as file:
    
        for ele in validation_samples:
        
            file.write(str(ele) + "\n")
    
    file.close()
    

"""
SAVE MODEL

"""

# model save to pkl

if gcp == True:

    with fs.open(gcs_hmm_model_dir + '/model_{}.pkl'.format(trial_name), 'wb') as file:
        
        pickle.dump(models, file)
        
else:
    
    joblib.dump(models, local_hmm_model_dir +'/model_{}.pkl'.format(trial_name))
    









