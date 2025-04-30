# conda activate actnew로 하셈

## usbipd 설치

### 1. usbipd 설치 (PowerShell)

```powershell
winget install usbipd
```

### 2. usbipd 구동

```powershell
# 관리자 모드로 실행
usbipd list # usbipd list에서 Userial 확인
usbipd bind --busid <busid> # busid는 usbipd list에서 확인
usbipd attach --wsl --busid <busid> # busid는 usbipd list에서 확인
# 해제하고 싶을 경우
usbipd unbind --busid <busid> # busid는 usbipd list에서 확인
```

### 3. usbipd 확인
```bash
lsusb # usbipd list에서 Userial 확인
```

## 4. USB 장치 확인
```bash
dmesg | grep tty 
# Ex: /dev/ttyUSB0
```
