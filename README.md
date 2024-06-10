# my-yahboom-ros-humble

## Build

```sh
git clone https://github.com/HackingGate/my-yahboom-ros-humble
cd my-yahboom-ros-humble/docker
docker build -t hackinggate/ros-humble:4.1.2 .
```

## Usage

```sh
cd
mv ros2_humble.sh ros2_humble.sh.bak
ln -s my-yahboom-ros-humble/ros2_humble.sh .
```
