#!/usr/bin/env python3
"""
error_capture.py — 장치 잠금 버그를 재현하는 스크립트

[버그 설명]
bash에서 `python3 error_capture.py &` 로 실행하면 SIGINT가 SIG_IGN으로 상속된다.
이를 복원하지 않으면 kill -SIGINT 가 동작하지 않는다.

SIGINT를 받아도 메인 루프의 cap.read() 도중 인터럽트된 경우,
cap.release()를 명시적으로 호출하지 않고 sys.exit()에 의존하면
uvcvideo 드라이버가 불완전한 상태로 남아 다음 프로세스가 장치를 열지 못한다.

[재현 조건]
stress_test.sh 로 반복 실행 시, 메인 루프 cap.read() 도중 SIGINT 수신 →
다음 run에서 Cannot open /dev/arducam_imx586 발생
"""

import cv2
import sys
import json
import time
import random
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

# ── 버그 ① ──────────────────────────────────────────────────────────────────
# bash `&` 실행 시 SIGINT = SIG_IGN 상속. 복원하지 않으면 kill -SIGINT 무효.
# (이 스크립트에서는 복원 코드를 의도적으로 생략)

# ── camera open ──────────────────────────────────────────────────────────────
def open_camera():
    cap = cv2.VideoCapture(DEVICE_PATH, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open {DEVICE_PATH}")

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

    # 버그 ② — 워밍업 루프에 KeyboardInterrupt 핸들러 없음
    # SIGINT가 cap.read() 도중 오면 cap.release() 미호출
    print("[Camera] Warming up ...", flush=True)
    for i in range(10):
        ret, _ = cap.read()          # ← SIGINT 시 KI 전파, cap.release() 없음
        print(f"[Camera] Warm-up {i+1}/10 {'OK' if ret else 'FAIL'}", flush=True)
        time.sleep(0.1)

    return cap


# ── main ─────────────────────────────────────────────────────────────────────
def main():
    cap = open_camera()
    interval = 1.0 / FPS
    print(f"[Main] Capture loop started (interval={interval:.3f}s)", flush=True)

    try:
        frame_n = 0
        while True:
            t0 = time.time()

            focus = random.randint(FOCUS_MIN, FOCUS_MAX)
            cap.set(cv2.CAP_PROP_FOCUS, focus)

            ret, frame = cap.read()   # ← SIGINT가 여기 도중 오면 장치 잠금 발생
            frame_n += 1
            if ret:
                print(f"[Main] frame={frame_n} focus={focus}", flush=True)
            else:
                print(f"[Main] frame={frame_n} READ FAILED", flush=True)

            elapsed = time.time() - t0
            wait = interval - elapsed
            if wait > 0:
                time.sleep(wait)

    except KeyboardInterrupt:
        print("\n[Main] Interrupted by user.", flush=True)
        # 버그 ③ — cap.release() 명시적 호출 없음
        # sys.exit() → 소멸자 경유 cap.release() 는
        # cap.read() mid-ioctl 인터럽트 상태에서 V4L2 드라이버를 완전히 해제하지 못함

    sys.exit(0)


if __name__ == "__main__":
    main()
