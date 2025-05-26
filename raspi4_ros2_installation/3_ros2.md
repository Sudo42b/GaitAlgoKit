# arm64v8/ros 설치하기

## 도커를 통해 ros2:humble을 설치하기

```bash
docker pull arm64v8/ros:humble
```

## 해당 도커를 실행해보자. (--name 뒤에는 설정하고 싶은 이름으로 설정하여도 됨)

```bash
docker run --name ros -it arm64v8/ros:humble /bin/bash
```

## 이미지를 실행하고 ros2에 있는 리스트를 확인해보자.

```bash
ros2 pkg list
```

아마 많은 사람들이 설치확인시에 활용하는 `demo_nodes_cpp_talker`와 같은 패키지는 확인할 수 없을 것이다. 해당 이미지를 다운로드받으면 모든 패키지가 다운로드가 되지 않으므로 아래 방법을 차례대로 진행하면 된다.

```bash
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop python3-argcomplete
sudo apt install ros-humble-ros-base python3-argcomplete
sudo apt install ros-dev-tools
```

차례대로 진행하고나면  `ros2 run demo_nodes_cpp_talker` 또는 `ros2 run demo_nodes_cpp_listner` 등을 실행할 수 있을것이다. (본인은 `ros2 run turtlesim turtlesim_node`를 활용하여 제대로 설치되었는지 확인하였는데, 이를 위해서는 GUI 설정이 필요하다..)

## 만약 현 상태를 docker 이미지로 저장하고 싶다면 docker commit을 수행하자.

```bash
docker commit [컨테이너명] [저장빌드]
```

시간이 생각보다 오래걸리니 주의하자! 이후에는 해당 컨테이너를 실행하여주면 된다. 
