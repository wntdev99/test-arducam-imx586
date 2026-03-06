# Arducam IMX586 장치 잠금 버그 — 이슈·해결 전체 기록

## 프로젝트 목표

Arducam IMX586 카메라(8000x6000, 3fps, MJPG, USB3)를 `stress.sh`로
반복 캡처·SIGINT 종료할 때, **장치 잠금(device lock)** 없이
100+ run 연속 PASS를 달성하는 것.

---

## Phase 1: 버그 분석 (`error_capture.py`)

### 발견된 버그 3가지

| # | 버그 | 원인 |
|---|------|------|
| ① | `kill -SIGINT` 무효 | bash `&` 실행 시 `SIGINT=SIG_IGN` 상속, 복원 코드 없음 |
| ② | 장치 잠금 | `cap.read()` 도중 SIGINT → `cap.release()` 미호출 → uvcvideo 드라이버 불완전 해제 |
| ③ | 불완전 release | `sys.exit()` 소멸자 경유 `release()`는 mid-ioctl 상태에서 V4L2 드라이버 완전 해제 불가 |

### stress.sh 실패 유형

- **SIGINT_FAIL**: `kill -SIGINT` 후 15초 내 미종료
- **DEVICE_LOCK**: 다음 run에서 "Cannot open" / RuntimeError
- **CRASH**: SIGINT 전에 Python이 스스로 죽음

---

## Phase 2: OpenCV 기반 수정 시도 (`safe_capture.py`)

### 수정 ①~④: 시그널 안전성 확보

**이슈**: SIGINT가 `cap.read()` ioctl 도중 도착하면 V4L2 드라이버가 불완전 상태로 남음.

**해결**:
1. SIGINT SIG_IGN 상속 복원 (bash `&` 대응)
2. `open_camera()` 전체 구간 `pthread_sigmask`로 SIGINT 블로킹
3. 캡처 전용 스레드 + SIGINT 영구 블로킹 → `cap.read()` 절대 중단 불가
4. SIGINT 핸들러를 `stop_event.set()`으로 교체 (SIGABRT 방지)

**결과**: SIGINT 안전성 확보. 그러나 `cap.release()` 후 다음 run에서 EIO 발생 → DEVICE_LOCK 지속.

---

### 수정 ⑤: VideoCapture() D-state hang 대응

**이슈**: `cv2.VideoCapture()` 생성자가 커널 D-state(uninterruptible sleep)에 빠지면 SIGALRM/SIGINT/SIGKILL 모두 전달 불가. 프로세스가 영구 hang.

**해결**: VideoCapture()를 daemon 스레드에서 실행 + `join(timeout)` 감시. 타임아웃 시 `os._exit(1)` → OS가 fd 정리.

**결과**: hang 방지 성공. 그러나 근본적인 cleanup 문제(EIO)는 미해결.

---

### 수정 ⑥~⑧: 에러 경로 강화

- 에러 경로에서 SIGINT SIG_IGN 처리
- `open_camera()` 내 모든 예외 경로에서 `cap.release()` 보장
- SIGTERM 핸들러 추가

---

### 수정 ⑨: cap.release() 후 USB 리셋 추가

**이슈**: `cap.release()` 후 Arducam IMX586 펌웨어가 EIO 상태에 빠짐. 다음 run에서 `S_FMT` ioctl이 `errno 5 (EIO)` 반환.

**해결**: `cap.release()` 후 `USBDEVFS_RESET` ioctl로 카메라 펌웨어 강제 초기화.

**결과**: EIO 빈도 감소. 그러나 OpenCV 내부 cleanup 순서와 충돌하는 경우 여전히 발생.

---

### 수정 ⑩: VIDIOC_STREAMOFF 수동 호출

**이슈**: OpenCV `cap.release()` 내부에서 STREAMOFF 타이밍이 불확실. 비동기 URB가 남아 있으면 다음 세션 EIO.

**해결**: `cap.release()` 전에 수동으로 `VIDIOC_STREAMOFF` ioctl 호출.

**결과**: OpenCV 내부 상태와 충돌 — release()가 이미 닫힌 스트림에 대해 재시도하면서 에러 발생.

---

### 수정 ⑪: dup2(/dev/null)로 fd 원자적 닫기

**이슈**: `cap.release()` 내부의 cleanup 순서를 외부에서 제어할 수 없음. STREAMOFF → close 사이에 다른 스레드가 fd를 재사용할 위험.

**해결**: OpenCV의 장치 fd를 `/dev/null`로 `dup2` 교체 → 원본 fd를 커널에 반환, OpenCV는 `/dev/null`을 cleanup.

**결과**: OpenCV 내부 상태와 불일치 — release()가 잘못된 fd로 ioctl 시도. 근본적으로 OpenCV 내부를 속이는 방식의 한계.

---

### 수정 ⑫: dup2 제거, cap.release() + USBDEVFS_RESET 단순화

**이슈**: dup2 트릭이 OpenCV 내부 상태와 충돌.

**해결**: dup2 제거. `cap.release()` (OpenCV 표준 cleanup) → `USBDEVFS_RESET` (펌웨어 초기화) 2단계로 단순화.

**결과**: 대부분의 run 성공. 그러나 `cap.release()` 내부 STREAMOFF가 D-state에 빠지는 경우 여전히 SIGINT_FAIL 발생. OpenCV 내부를 제어할 수 없는 근본적 한계.

---

## Phase 3: OpenCV 제거, Raw V4L2 전면 교체

### 결정 배경

OpenCV `VideoCapture`의 V4L2 백엔드는 내부 cleanup 순서를 외부에서 제어할 수 없음.
수동 STREAMOFF, dup2 등 개입 시도마다 OpenCV 내부 상태와 충돌.
**raw V4L2 ioctl로 전면 교체**하여 모든 단계를 직접 제어하기로 결정.

### 구현: `v4l2_camera.py` (V4L2Camera 클래스)

```
open()       → open → QUERYCAP → S_FMT → S_PARM → REQBUFS → QUERYBUF+mmap → QBUF → STREAMON
read()       → DQBUF → 데이터 복사 → QBUF → (True, jpeg_bytes)
set_focus()  → S_CTRL(FOCUS_ABSOLUTE)
close()      → STREAMOFF → munmap → REQBUFS(0) → close(fd) → USB reset
```

각 cleanup 단계는 개별 try/except로 다음 단계 보장.

### `safe_capture.py` 수정

- `import cv2` 제거 → `from v4l2_camera import V4L2Camera`
- `_videocapture_with_timeout` → `_v4l2_open_with_timeout`
- `_release_cap_safe(cap)` → `_release_cam_safe(cam)`
- `_diagnose_device`, `_usb_reset` 제거 (V4L2Camera로 통합)

---

### 이슈 A: fps float→int TypeError

**증상**: `struct.pack_into('I', ..., self.fps)` — `config.json`의 `frame_fps: 3`이 float(3.0)으로 로드됨. `'I'` 포맷은 정수만 허용.

```
required argument is not an integer
```

**해결**: `int(self.fps)` 캐스팅 추가.

---

### 이슈 B: STREAMOFF D-state hang → SIGINT_FAIL

**증상**: Run 16에서 SIGINT 후 `cam.close()` → STREAMOFF ioctl이 D-state hang (10초+). 캡처 스레드가 `phase=release`에서 멈춤. `os._exit(1)` 호출되지만 D-state 스레드 대기로 15초 초과 → SIGINT_FAIL.

```
[Signal] SIGINT received (phase=read)
[Main] frame=7 focus=226
[Main] Capture thread stuck after 10s (phase=release), forcing exit.
```

**해결**: STREAMOFF를 daemon 스레드에서 실행 + 3초 타임아웃. 타임아웃 시 skip하고 `close(fd)`로 진행 (커널이 release 시 cleanup). `JOIN_TIMEOUT` 10→8초로 축소.

---

## 현재 상태 (2026-03-06)

### 코드 구조

```
script/
├── v4l2_camera.py      ← 신규: Raw V4L2 ioctl 카메라 클래스
├── safe_capture.py      ← 수정: OpenCV 제거, V4L2Camera 사용
├── error_capture.py     ← 버그 재현용 (변경 없음)
└── stress.sh            ← 스트레스 테스트 (변경 없음)
```

### 마지막 테스트 결과

- 단독 실행: 50프레임 캡처 + SIGTERM graceful shutdown + cleanup 정상
- stress.sh: Run 16에서 SIGINT_FAIL 발생 → STREAMOFF 타임아웃 수정 적용
- **STREAMOFF 타임아웃 수정 후 stress.sh 미검증** (카메라 USB 분리 상태)

### 검증 필요

```bash
./stress.sh safe
```

목표: 100+ run 연속 PASS.
