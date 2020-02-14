import sounddevice as sd
import soundfile as sf

duration = 60
fs = 48000
sd.default.device = 1

myrecording = sd.rec(int(duration * fs), samplerate=fs, channels=1, blocking=True)

sf.write("noise.wav",myrecording, samplerate=fs)


