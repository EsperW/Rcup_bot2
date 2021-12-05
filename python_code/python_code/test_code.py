from os.path import expanduser
import datetime

file = open(expanduser("~") + '/Desktop/HERE.txt', 'w')
file.write("It worked!\n" + str(datetime.datetime.now()))
file.close()

#@reboot sleep 30 && tmux | cd /home/nano/Documents/kode_stz/ | echo 1 sudo | python3 /home/nano/Documents/kode_stz/start_stz.py

