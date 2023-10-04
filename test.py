import subprocess
import matlab.engine

eng = matlab.engine.start_matlab()

connect = 'parallel-ssh -i -h ~/.ssh/hosts.txt -x "-i ~/.ssh/id_rsa" '
command = "arecord -D plughw:0 -c2 -r 48000 -f S32_LE -t wav -V stereo -v -d 10 file_stereo.wav"
fetchOne = "scp craycraypi@craycraypi:/home/craycraypi/file_stereo.wav /mnt/c/Users/danro/Documents/EEE3097S-Group-11/Recording1"
fetchTwo = "scp craycraypi2@craycraypi2:/home/craycraypi2/file_stereo.wav /mnt/c/Users/danro/Documents/EEE3097S-Group-11/Recording2"

print("Started")
completed_process = subprocess.run(["wsl", "bash", "-c", connect + command], capture_output=True, text=True)
print("Finsihed")
print(completed_process.stdout)
subprocess.run(["wsl", "bash", "-c", fetchOne], capture_output=True, text=True)
subprocess.run(["wsl", "bash", "-c", fetchTwo], capture_output=True, text=True)

eng.run('tdoa.m', nargout=0)

#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


eng.quit()
