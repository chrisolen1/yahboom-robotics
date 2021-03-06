nohup python3 -u train_hmm.py --trial_name $TRIAL_NAME \
	--local \
	--random_validation_set \
	--num_iterations 150 \
	--num_cep_coefs 13 \
	--unidimensional_input \
	--use_pomegranate \
	--distribution NormalDistribution \
	--n_threads 5 --training_prop 0.3 > logs/training_logs_$TRIAL_NAME.txt 2>&1 &

