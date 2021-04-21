# these commands may need to be used once:

# sudo modprobe can_dev
# sudo modprobe can
# sudo modprobe can_raw
# sudo modprobe vcan
# sudo apt-get install can-utils

# NOTE: the above have been tested with PEAK-CAN hardware only


# can0 hardware at 500kbps
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up

# vcan0 for testing without hardware
sudo ip link add dev vcan0 type vcan
sudo ip link set vcan0 up

# to bring down the link to reset
# sudo ip link set can0 down
# sudo ip link delete can0
