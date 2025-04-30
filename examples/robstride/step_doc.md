# 설명서(Docstring)

이 스크립트는 Robstride 모터를 제어하여 다양한 걷기 시나리오를 테스트하는 데 사용됩니다.

**1. 각도 제한 기능**

- `right_min_angle`, `right_max_angle`: 오른쪽 다리의 최소/최대 각도 제한
- `left_min_angle`, `left_max_angle`: 왼쪽 다리의 최소/최대 각도 제한
- 각도는 도(degree) 단위로 입력받아 내부적으로 라디안으로 변환

**2. 오프셋 조절**

- `right_offset`, `left_offset`: 각 다리의 중심 위치를 조정할 수 있는 오프셋
- 기본 사인파 운동의 중심점을 이동시켜 비대칭 움직임 구현 가능

**3. 부상 정도 조절**

- `right_scale`, `left_scale`: 각 다리의 움직임 진폭을 조절하는 스케일 팩터
- 부상 시뮬레이션에서 움직임 제한 정도를 세밀하게 조절 가능

**4. 명령행 인자 추가**

```bash
    --right-min-angle: 오른쪽 다리 최소 각도(도)
    --right-max-angle: 오른쪽 다리 최대 각도(도)
    --left-min-angle: 왼쪽 다리 최소 각도(도)
    --left-max-angle: 왼쪽 다리 최대 각도(도)
    --right-offset: 오른쪽 다리 중심 오프셋(도)
    --left-offset: 왼쪽 다리 중심 오프셋(도)
    --right-scale: 오른쪽 다리 부상 스케일(기본값: 0.25)
    --left-scale: 왼쪽 다리 부상 스케일(기본값: 0.25)
```

## 사용 예시

다음과 같이 다양한 파라미터를 조합하여 모터 움직임을 세밀하게 조절할 수 있습니다:

```bash
    # 기본 실행 (모든 기본값 사용)
    python script.py --port /dev/ttyUSB0

    # 각도 제한 적용
    python script.py --right-min-angle -30 --right-max-angle 45 --left-min-angle -45 --left-max-angle 30

    # 오프셋 적용 (중심점 이동)
    python script.py --right-offset 15 --left-offset -15

    # 부상 정도 조절
    python script.py --right-scale 0.1 --left-scale 0.4

    # 모든 파라미터 조합
    python script.py --period 1.5 --amplitude 0.8 --duration 4.0 \
                    --right-min-angle -20 --right-max-angle 40 \
                    --left-min-angle -40 --left-max-angle 20 \
                    --right-offset 10 --left-offset -5 \
                    --right-scale 0.15 --left-scale 0.3
```

### 파라미터 파인튜닝

이 코드 구조를 통해 다음과 같이 파인튜닝이 가능합니다:

1. **걸음걸이 조절**:

- `period`: 주기를 조절하여 걸음 속도 변경
- `amplitude`: 진폭을 조절하여 걸음 크기 변경

2. **비대칭 걸음걸이**:

- `right_offset`/`left_offset`: 다리 중심점을 이동시켜 비대칭 걸음 구현
- 예: 한쪽으로 치우친 걸음걸이, 안짱다리 모션 등

3. **부상 정도 조절**:

- `right_scale`/`left_scale`: 부상 시뮬레이션에서 움직임 제한 정도 조절
- 0에 가까울수록 더 심한 부상 상태 (움직임 제한)

4. **움직임 범위 제한**:

- `right_min_angle`/`right_max_angle`: 오른쪽 다리 움직임 범위 제한
- `left_min_angle`/`left_max_angle`: 왼쪽 다리 움직임 범위 제한

이러한 파라미터들을 다양하게 조합하여 여러 보행 패턴을 실험하고 최적의 값을 찾을 수 있습니다.
