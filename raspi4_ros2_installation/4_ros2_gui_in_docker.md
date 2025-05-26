# GUI를 실행하기위한 docker 실행

도커를 실행하는 부분에 옵션 내용이 많은데, 해당 옵션들의 설명은 다음과 같다

`-it`: install 과 terminal 옵션을 합친 것으로, 컨테이너를 종료하지않은 채로 터미널의 입력을 컨테이너로 전달하기 위해서 사용되는 옵션
`--name`: 생성할 컨테이너에 임의의 이름을 지정해주는 옵션
`-v /tmp/.X11-unix:/tmp/.X11-unix`: -v는 볼륨 지정 옵션으로 호스트와 컨테이너의 디렉토리를 연결하는 옵션.(<Host 절대경로>:<Container 절대경로> 순서) ROS는 기본적으로 GUI가 필요므로 x11을 사용하기 위해 지정해줌.
`-e DISPLAY=$DISPLAY`: e는 컨테이너 내에서 사용할 환경변수를 설정하는 옵션. GUI로 x11을 활용하므로 DISPLAY에 x11로 지정해줌.
`/bin/bash` : 이미지 안의 /bin/bash를 실행하는 옵션임

```bash
docker pull arm64v8/ros:humble
docker run --name ros_gui -it -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY arm64v8/ros:humble /bin/bash
```

## GUI를 위해 x-host 설정해주기

1. 도커상에서 해당 부분 실행

```bash
docker start [컨테이너명]
docker attach [컨테이너명]

apt-get update && apt-get upgrade -y
apt-get install x11-apps
```

2. 라즈베리파이상에서 해당 부분 실행

```bash
#라즈베리파이
$ xhost +local:docker

`xhost:  unable to open display`
```

만약 위와 같은 오류가 발생한다면 아래 부분을 차례대로 실행시켜주자

```bash
#라즈베리파이
export DISPLAY=:0.0
sudo apt install x11-xserver-utils
xhost +local:docker
```

* 만약 Display 설정 중에 docker를 실행했다면 docker에서 export DISPLAY=:0.0을 실행하면 됨



## 정리 : 도커를 활용하여 GUI를 활용할 시 
위의 방법대로 모든 과정이 끝났다면, 아래와 같은 방식으로 진행된다고 볼 수 있음

```bash
docker pull arm64v8/ros:humble
xhost +local:docker
docker run --name ros_gui -it -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY --device="/dev/dri:/dev/dri" arm64v8/ros:humble /bin/bash
 
docker run -it --name ros_gui --env="DISPLAY=$DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --volume="$XAUTHORITY:$XAUTHORITY"  --env="XAUTHORITY=$XAUTHORITY" arm64v8/ros:humble /bin/bash
```

```bash
# 도커상에서
apt update && apt upgrade -y
apt-get install x11-apps
apt-get update && apt-get install -y libgl1-mesa-glx libgl1-mesa-dri mesa-utils
apt-get update && apt-get install -y libxext6 libx11-6 libglvnd0 libgl1 libglx0 libegl1 libxcb1 libxkbcommon0
apt install ros-humble-turtlesim
export DISPLAY=:0
export QT_X11_NO_MITSHM=1

```

위와 같은 방식으로 docker run 실행후에 아래 부분을 진행하면 라즈베리파이에서 GUI를 통해 거북이를 볼 수 있다!

```bash
ros2 run turtlesim turtlesim_node
```

## yml로 docker-compose 실행하기

사용 방법:

이 파일을 docker-compose.yml로 저장합니다.
다음 명령어로 컨테이너를 시작합니다:

```bash
docker-compose up -d
```

컨테이너에 접속하려면:

```bash
docker-compose exec ros_gui bash
```

새로운 터미널 창에서 별도의 세션으로 접속하려면:

```bash
docker exec -it ros_gui bash
```

이제 모든 터미널 세션에서 ROS2 명령어(예: ros2 run turtlesim turtlesim_node)가 제대로 작동할 것입니다.