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

cd ../nginx-rtmp
docker build -t hackinggate/nginx-rtmp .

cd ../nginx-ffmpeg-hls
docker build -t hackinggate/nginx-ffmpeg-hls .
```

## Setup

```sh
cd
mv ros2_humble.sh ros2_humble.sh.bak
ln -s my-yahboom-ros-humble/ros2_humble.sh .
./ros2_humble.sh

/root/setup_imageprocessor.sh
/root/setup_videoprocessor.sh
/root/setup_videostream.sh
source ~/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash
exit

docker commit $(docker ps -a -q | head -n 1) hackinggate/ros-humble:4.1.2
```

## Run

```sh
cd my-yahboom-ros-humble/nginx-rtmp
docker compose up -d

cd 
cd my-yahboom-ros-humble/nginx-ffmpeg-hls
docker compose up -d

cd
./ros2_humble.sh
```
