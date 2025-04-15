0. 들어가며
GNU screen은 여러 개의 가상 터미널 세션을 사용할 수 있는 소프트웨어다.
이 글은 screen을 사용하는 방법을 다룬다.

1. 설치
아래 명령어로 screen을 설치할 수 있다.
E: Unable to locate package screen 에러가 발생하면 apt-get update로 업데이트 후 다시 시도한다.
code
```bash
apt-get install screen
```
2. 주요 명령어
screen 외부
```bash
screen --help # 도움말
screen # screen 생성 후 진입 (이름은 무작위로 생성)
screen -r {name} # 기존 screen 진입 (reattach)
screen -S {name} # screen 생성 후 진입 (이름 지정)
screen -R (name} # 해당 이름의 screen이 있으면 reattach, 없으면 생성 후 진입

screen -S {name} -X quit # detached screen 제거

screen -ls # 현재 screen 목록 확인
screen -list # 현재 screen 목록 확인
```
screen 내부

```bash

exit # screen 종료
ctrl + d # screen 종료 (exit과 같음)

ctrl + a, d # screen 나가기 (detach, 종료 x)
```

3. Bash 사용
* screen의 기본 shell은 sh이다.
* 이를 bash로 변경하는 방법은 다음과 같다.

일시적 변경
* screen 접속 후 bash를 입력한다.

영구적 변경
* screenrc 파일에 들어간다.
```bash
vim ~/.screenrc
```
* 아래의 명령어를 추가한다.
```bash
shell bash
```