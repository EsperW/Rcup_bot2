import os

os.system("echo 1 | sudo -S nvpmodel -m 0")
#os.system("cd /bin")
#os.system("echo 1 | sudo -S sudo jetson_clocks")
#os.system("echo 1 | sudo python3 new_sensor_bgr_different_color.py")

os.system("tmux split-window 'sudo echo 1 | python3 /home/nano/Documents/kode_stz/new_sensor_bgr_different_color.py'")

