# Docker 설치하기

```bash
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
```
위를 순서대로 시작하자
만약 The repository `'http://packages.ros.org/ros/ubuntu bullseye Release' does not have a Release file` 오류가 발생한다면

`/etc/apt/sources.list.d`의 `.list` 확장자 파일들을 삭제하면 됨

## Docker 비루트 계정 권한 설정

docker를 설치하고난 뒤 일반 계정에서 docker를 실행할려면 계속 sudo를 앞에 붙여줘야 한다. sudo 명령어 없이 docker 이미지를 실행하기 위해서는 docker 그룹에 사용자를 추가해주면 된다.

```bash
sudo usermod -aG docker ${USER}
```

해당 `${USER}` 부분에 일반 계정에 해당하는 이름을 작성해주면 된다.

계정이 그룹에 잘 할당되었는지 확인하려면 아래와 같은 코드를 실행해부면 계정이 할당된 그룹명들이 뜨게 되고 여기에 docker 그룹이 있는지 확인해주면 된다.

```bash
groups ${user}
```

계정 권한 설정 후에는 reboot을 실행해주어야 적용이되니 꼭 reboot을 해주자!

## 도커 설치 확인
위의 과정까지 진행한 뒤, 도커가 제대로 설치되어있는지 확인하기 위해 도커 버전을 확인해보자.

```
docker version
```

아래 그림과 같이 나오는지 확인해보자

만약 부팅시 도커를 자동 실행하고 싶다면?
아래와 같이 서비스유닛에 docker를 등록해주면 된다.

```bash
sudo systemctl enable docker
```
