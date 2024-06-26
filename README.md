# My Yahboom ros-humble

## Client-side Code

https://github.com/HackingGate/My-Yahboom-Controller

## Demo Video

https://youtu.be/pUTGvqYHX7M

## Build

```sh
git clone https://github.com/HackingGate/my-yahboom-ros-humble
cd my-yahboom-ros-humble/docker
docker build -t hackinggate/ros-humble:4.1.2 .
```

## Setup

```sh
cd
mv ros2_humble.sh ros2_humble.sh.bak
ln -s my-yahboom-ros-humble/ros2_humble.sh .
./ros2_humble.sh

/root/setup_imageprocessor.sh
/root/setup_videoprocessor.sh
source ~/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash
exit

docker commit $(docker ps -a -q | head -n 1) hackinggate/ros-humble:4.1.2
```

## Run

```sh
cd my-yahboom-ros-humble/mediamtx
docker compose up -d --build

cd
./ros2_humble.sh
```

## Modem Setup

Reference: https://www.jeffgeerling.com/blog/2022/using-4g-lte-wireless-modems-on-raspberry-pi

`/usr/local/bin/start_modem.sh`

```sh
#!/bin/bash

# Initialize wwan0
ip link set wwan0 down
echo 'Y' > /sys/class/net/wwan0/qmi/raw_ip
ip link set wwan0 up
sleep 10
qmicli -p -d /dev/cdc-wdm0 --device-open-net='net-raw-ip|net-no-qos-header' --wds-start-network="apn='YOUR_APN',ip-type=4" --client-no-release-cid
sleep 2
udhcpc -q -f -i wwan0

# Initialize GPS
echo -e 'AT+QGPS=1\r' > /dev/ttyUSB2
sleep 2
cat /dev/ttyUSB2 > /var/log/gps_data &
```

`/etc/systemd/system/modem.service`

```service
[Unit]
Description=Initialize 4G modem and GPS
After=network.target

[Service]
Type=simple
ExecStart=/usr/local/bin/start_modem.sh

[Install]
WantedBy=multi-user.target
```

```sh
sudo systemctl daemon-reload
sudo systemctl enable modem.service
```

`/etc/udev/rules.d/99-usb-4g-modem.rules`

```rules
ACTION=="add", SUBSYSTEM=="usb", ATTR{idVendor}=="2c7c", ATTR{idProduct}=="0125", RUN+="/bin/systemctl restart modem.service"
```

```sh
sudo udevadm control --reload-rules
sudo udevadm trigger
```
