stty -F /dev/ttyUSB0 4800 cs8 -cstopb -parenb
while true; do
    read LINE < /dev/ttyUSB0
    echo $LINE
done
