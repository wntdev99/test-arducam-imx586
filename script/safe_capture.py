#!/usr/bin/env python3
"""
safe_capture.py — 장치 잠금을 완전히 차단하는 스크립트

[수정 내역]

수정 ① — SIGINT 핸들러 복원
    bash `&` 실행 시 SIGINT = SIG_IGN 상속 → kill -SIGINT 무효.
    main() 진입 시 SIG_IGN이면 default_int_handler로 복원.

수정 ② — open_camera() 전체 구간 SIGINT 블로킹
    cv2.VideoCapture() 초기화 및 워밍업 cap.read() 내부에서 호출되는
    V4L2 ioctl이 SIGINT(EINTR)로 중단되면 장치가 반열린 상태로 남음.
    open_camera() 호출 전 pthread_sigmask(SIG_BLOCK)으로 전체 보호.

수정 ③ — 캡처 전용 스레드 + SIGINT 영구 블로킹
    캡처 스레드 시작 즉시 SIGINT를 영구 블로킹 → cap.read() 절대 중단 불가.
    stop_event 세팅 후 현재 read 완료 → finally에서 cap.release() 보장.

수정 ④ — SIGINT 핸들러를 stop_event.set()으로 교체 (KI 발생 제거)
    메인 스레드에서 t.join() 중 KeyboardInterrupt 발생 시 Python 내부
    스레딩 충돌로 SIGABRT(exit=134) 크래시 발생.
    SIGINT 핸들러를 lambda로 stop_event.set()만 실행하도록 교체하면
    KI가 발생하지 않아 SIGABRT 방지.

수정 ⑤ — SIGALRM 타임아웃을 VideoCapture() 생성자만 감쌈
    이전 버전: SIGALRM을 전체 open_camera()에 걸어 워밍업과 충돌.
    개선: VideoCapture()만 5초 SIGALRM으로 감쌈.
      - 정상 장치: VideoCapture() < 1s → alarm 해제 → 워밍업 정상 진행
      - 잠금 장치: VideoCapture() hang → 5s 후 SIGALRM → RuntimeError → 빠른 종료
      - stress.sh SIGKILL(5s) 이전에 Python이 스스로 종료 → DEVICE_LOCK 정상 감지

수정 ⑥ — 에러 경로 SIGINT SIG_IGN 처리
    RuntimeError/SIGALRM timeout 처리 시 SIGINT 언블로킹 직전에
    SIG_IGN을 설치하여 대기 중인 SIGINT가 KI로 변환되는 것을 방지.
"""

import cv2
import sys
import json
import time
import signal
import random
import threading
from pathlib import Path

# ── config ──────────────────────────────────────────────────────────────────
def _load_config():
    for path in [Path(__file__).parent.parent / "config.json", Path("config.json")]:
        if path.exists():
            with open(path) as f:
                return json.load(f)
    raise FileNotFoundError("config.json not found")

_cfg        = _load_config()
DEVICE_PATH = _cfg["device_path"]
WIDTH       = int(_cfg["frame_width"])
HEIGHT      = int(_cfg["frame_height"])
FPS         = float(_cfg["frame_fps"])
FOCUS_MIN   = int(_cfg["focus_min"])
FOCUS_MAX   = int(_cfg["focus_max"])

_SIGSET = {signal.SIGINT}

VIDEOCAP_TIMEOUT = 5  # VideoCapture() 생성자 최대 허용 시간 (초)


class _VideoCapTimeout(Exception):
    pass


def _alarm_handler(sig, frame):
    raise _VideoCapTimeout(f"VideoCapture() timed out after {VIDEOCAP_TIMEOUT}s (device locked?)")


# ── camera open ──────────────────────────────────────────────────────────────
def open_camera():
    """SIGINT가 블로킹된 상태에서 호출되어야 함 (main에서 보장).

    Phase 1: VideoCapture() — SIGALRM 5초 타임아웃 (잠금 장치 hang 탈출)
    Phase 2: 설정 + 워밍업 cap.read() — SIGINT 블로킹만 (정상 시 SIGALRM 불필요)
    """
    # Phase 1: VideoCapture() 생성자만 SIGALRM으로 보호
    old_alarm = signal.signal(signal.SIGALRM, _alarm_handler)
    signal.alarm(VIDEOCAP_TIMEOUT)
    try:
        cap = cv2.VideoCapture(DEVICE_PATH, cv2.CAP_V4L2)
    finally:
        signal.alarm(0)
        signal.signal(signal.SIGALRM, old_alarm)

    if not cap.isOpened():
        raise RuntimeError(f"Cannot open {DEVICE_PATH}")

    # Phase 2: 설정 + 워밍업 (SIGINT 블로킹 중, SIGALRM 없음)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    cap.set(cv2.CAP_PROP_FPS,          FPS)
    cap.set(cv2.CAP_PROP_BUFFERSIZE,   1)
    cap.set(cv2.CAP_PROP_AUTOFOCUS,    0)

    w   = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h   = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    print(f"[Camera] Opened: {w}x{h} @ {fps} fps", flush=True)

    print("[Camera] Warming up ...", flush=True)
    for i in range(10):
        ret, _ = cap.read()   # SIGINT 블로킹 중 → ioctl 절대 중단 불가
        print(f"[Camera] Warm-up {i+1}/10 {'OK' if ret else 'FAIL'}", flush=True)
        time.sleep(0.1)       # SIGINT 블로킹 중 → sleep은 그냥 대기

    return cap


# ── 캡처 전용 스레드 ─────────────────────────────────────────────────────────
def _capture_worker(cap, interval, stop_event):
    """캡처 전용 스레드.

    시작 즉시 SIGINT를 영구 블로킹 → cap.read() 절대 중단 불가.
    stop_event 세팅 확인 후 루프 종료, finally에서 cap.release() 보장.
    """
    signal.pthread_sigmask(signal.SIG_BLOCK, _SIGSET)

    frame_n = 0
    try:
        while not stop_event.is_set():
            t0 = time.time()

            focus = random.randint(FOCUS_MIN, FOCUS_MAX)
            cap.set(cv2.CAP_PROP_FOCUS, focus)
            ret, frame = cap.read()
            frame_n += 1

            if ret:
                print(f"[Main] frame={frame_n} focus={focus}", flush=True)
            else:
                print(f"[Main] frame={frame_n} READ FAILED", flush=True)

            elapsed = time.time() - t0
            wait = interval - elapsed
            if wait > 0:
                time.sleep(wait)
    finally:
        cap.release()
        print("[Capture] cap.release() OK", flush=True)


# ── main ─────────────────────────────────────────────────────────────────────
def main():
    # 수정 ① — bash `&` 실행 시 SIGINT=SIG_IGN 상속 복원
    if signal.getsignal(signal.SIGINT) == signal.SIG_IGN:
        signal.signal(signal.SIGINT, signal.default_int_handler)

    # 수정 ② — open_camera() 전체를 SIGINT 블로킹 상태에서 실행
    # (내부에서 VideoCapture() 구간만 SIGALRM 5초 타임아웃 추가)
    signal.pthread_sigmask(signal.SIG_BLOCK, _SIGSET)
    try:
        cap = open_camera()
    except (RuntimeError, _VideoCapTimeout) as e:
        # 수정 ⑥ — 언블로킹 전 SIG_IGN: 대기 중인 SIGINT가 KI가 되는 것을 방지
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.pthread_sigmask(signal.SIG_UNBLOCK, _SIGSET)
        print(f"[Main] {e}", flush=True)
        sys.exit(1)

    interval = 1.0 / FPS
    stop_event = threading.Event()

    # 수정 ④ — SIGINT 핸들러를 stop_event.set()으로 교체 (KI 발생 제거)
    # t.join() 중 KI 발생 시 Python 스레딩 내부 충돌 → SIGABRT 방지
    signal.signal(signal.SIGINT, lambda sig, frame: stop_event.set())

    # 수정 ③ — 캡처 스레드 시작 (내부에서 SIGINT 영구 블로킹)
    t = threading.Thread(target=_capture_worker, args=(cap, interval, stop_event))
    t.start()
    print(f"[Main] Capture thread started (interval={interval:.3f}s)", flush=True)

    # 메인 스레드 SIGINT 언블로킹 — 이제부터 SIGINT → stop_event.set()
    signal.pthread_sigmask(signal.SIG_UNBLOCK, _SIGSET)

    t.join()   # 캡처 스레드가 cap.release() 완료 후 종료될 때까지 대기
    print("[Main] Done.", flush=True)
    sys.exit(0)


if __name__ == "__main__":
    main()
