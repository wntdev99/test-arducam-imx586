#!/usr/bin/env python3
"""
safe_capture.py — 장치 잠금을 완전히 차단하는 스크립트

[핵심 설계]
  - Raw V4L2 ioctl로 모든 단계를 직접 제어 (OpenCV 제거)
  - V4L2Camera.open()이 커널 D-state(uninterruptible sleep)에 빠질 수 있음
  - D-state에서는 SIGALRM/SIGINT/SIGKILL 모두 전달 불가
  - 따라서 V4L2Camera.open()은 daemon 스레드에서 실행하고,
    메인 스레드가 join(timeout)으로 감시 → 타임아웃 시 os._exit()

[수정 내역]
  ① SIGINT SIG_IGN 상속 복원 (bash & 실행 대응)
  ② open_camera() 전체 구간 SIGINT 블로킹 (pthread_sigmask)
  ③ 캡처 전용 스레드 + SIGINT 영구 블로킹
  ④ SIGINT 핸들러를 stop_event.set()으로 교체 (SIGABRT 방지)
  ⑤ V4L2Camera.open()을 daemon 스레드에서 실행 + join(timeout) 감시
     → D-state hang 시 os._exit(1)로 프로세스 종료
  ⑥ 에러 경로 SIGINT SIG_IGN 처리
  ⑦ open_camera() 내 모든 예외 경로에서 cam.close() 보장
  ⑧ SIGTERM 핸들러 추가
  ⑨ 캡처 루프 종료 후 join(timeout) 워치독
  ⑩ V4L2Camera.close(): STREAMOFF → munmap → REQBUFS(0) → close → USB reset
  ⑪ 연속 read 실패 시 자동 종료
  ⑫ open_camera() 재시도 (이전 run cleanup 미완 대비)
"""

import os
import re
import sys
import json
import time
import signal
import random
import subprocess
import threading
from pathlib import Path
from v4l2_camera import V4L2Camera

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
CONSEC_FAIL = int(_cfg.get("consec_fail_threshold", 3))

_SIGSET_INT = {signal.SIGINT}

VIDEOCAP_TIMEOUT = 8   # V4L2Camera.open() 최대 허용 시간 (초)
JOIN_TIMEOUT     = 8   # stop_event 후 cam.close() 완료 최대 대기 (초)
OPEN_RETRIES     = 3   # 장치 열기 재시도 횟수
OPEN_RETRY_WAIT  = 2   # 재시도 간격 (초)

_DMESG_FILTER = re.compile(
    r"uvcvideo|usb\s+\d|xhci|video4linux|v4l2|bulk transfer|URB|VIDIOC|04b4:0478",
    re.IGNORECASE,
)


def _dmesg_since(boot_ts):
    """boot_ts 이후의 USB/V4L2 관련 dmesg 라인을 반환."""
    try:
        out = subprocess.run(
            ["dmesg", "--time-format=raw", "--nopager"],
            capture_output=True, text=True, timeout=3,
        ).stdout
    except Exception:
        return []
    lines = []
    for line in out.splitlines():
        parts = line.split(None, 1)
        if len(parts) < 2:
            continue
        try:
            ts = float(parts[0].rstrip(":"))
        except ValueError:
            continue
        if ts >= boot_ts and _DMESG_FILTER.search(parts[1]):
            lines.append(line)
    return lines


def _dmesg_boot_ts():
    """현재 시각에 대응하는 dmesg raw timestamp (부팅 이후 초) 반환."""
    try:
        out = subprocess.run(
            ["dmesg", "--time-format=raw", "--nopager"],
            capture_output=True, text=True, timeout=3,
        ).stdout
        for line in reversed(out.splitlines()):
            parts = line.split(None, 1)
            if parts:
                return float(parts[0].rstrip(":"))
    except Exception:
        pass
    return 0.0


def _print_dmesg(tag, lines):
    """dmesg 라인을 태그와 함께 출력."""
    if not lines:
        print(f"[dmesg:{tag}] (clean)", flush=True)
        return
    print(f"[dmesg:{tag}] {len(lines)} line(s):", flush=True)
    for line in lines[-20:]:
        print(f"  {line}", flush=True)


def _release_cam_safe(cam):
    """cam.close() 호출. V4L2Camera.close()가 모든 cleanup을 수행."""
    if cam is None:
        return
    t0 = time.time()
    try:
        cam.close()
    except Exception as e:
        print(f"[Capture] close exception: {e}", flush=True)
    phase_info = f" sig_phase={_signal_phase}" if _signal_phase else ""
    print(f"[Capture] close done{phase_info} ({time.time() - t0:.2f}s)", flush=True)


# ── V4L2Camera.open() with thread-based timeout ─────────────────────────────
def _v4l2_open_with_timeout(device_path, width, height, fps, timeout):
    """V4L2Camera.open()을 daemon 스레드에서 실행.

    커널 D-state(uninterruptible sleep) hang 시 SIGALRM은 전달 불가.
    대신 daemon 스레드 + join(timeout)으로 감시.
    타임아웃 시 os._exit(1) → OS가 프로세스 fd 정리.
    """
    result = [None]
    exc = [None]

    def _worker():
        try:
            cam = V4L2Camera(device_path, width, height, fps)
            cam.open()
            result[0] = cam
        except Exception as e:
            exc[0] = e

    t = threading.Thread(target=_worker, daemon=True)
    t.start()
    t.join(timeout=timeout)

    if t.is_alive():
        print(f"[Camera] V4L2Camera.open() timed out after {timeout}s (device locked?)", flush=True)
        os._exit(1)

    if exc[0] is not None:
        raise exc[0]

    return result[0]


# ── camera open ──────────────────────────────────────────────────────────────
def open_camera():
    """SIGINT가 블로킹된 상태에서 호출되어야 함 (main에서 보장).

    Phase 1: V4L2Camera.open() — daemon 스레드 + join(timeout) (D-state hang 탈출)
             실패 시 재시도 (이전 run의 USB cleanup 미완 대비)
    Phase 2: 워밍업 5프레임 — SIGINT 블로킹만
    """
    cam = None
    last_err = None

    for attempt in range(OPEN_RETRIES):
        try:
            print(f"[Camera] Opening {DEVICE_PATH} (attempt {attempt + 1}/{OPEN_RETRIES}) ...", flush=True)
            cam = _v4l2_open_with_timeout(DEVICE_PATH, WIDTH, HEIGHT, FPS, VIDEOCAP_TIMEOUT)

            if cam.is_open:
                break

            cam.close()
            cam = None
            last_err = RuntimeError(f"Cannot open {DEVICE_PATH}")

            if attempt < OPEN_RETRIES - 1:
                print(f"[Camera] Device not ready, retry in {OPEN_RETRY_WAIT}s ...", flush=True)
                time.sleep(OPEN_RETRY_WAIT)

        except Exception as e:
            if cam is not None:
                try:
                    cam.close()
                except Exception:
                    pass
                cam = None
            last_err = e
            if attempt < OPEN_RETRIES - 1:
                print(f"[Camera] Open failed: {e}, retry in {OPEN_RETRY_WAIT}s ...", flush=True)
                time.sleep(OPEN_RETRY_WAIT)

    if cam is None or not cam.is_open:
        raise last_err or RuntimeError(f"Cannot open {DEVICE_PATH}")

    # Phase 2: 워밍업 (SIGINT 블로킹 중)
    try:
        print(f"[Camera] Opened: {WIDTH}x{HEIGHT} @ {FPS} fps", flush=True)
        print("[Camera] Warming up ...", flush=True)
        for i in range(5):
            ret, _ = cam.read()
            print(f"[Camera] Warm-up {i+1}/5 {'OK' if ret else 'FAIL'}", flush=True)

        return cam

    except BaseException:
        _release_cam_safe(cam)
        raise


# ── 캡처 전용 스레드 ─────────────────────────────────────────────────────────
_capture_phase = "init"
_signal_phase = ""


def _capture_worker(cam, interval, stop_event):
    """캡처 전용 스레드.

    시작 즉시 SIGINT를 영구 블로킹 → cam.read() 절대 중단 불가.
    stop_event 세팅 확인 후 루프 종료, finally에서 cam.close() 보장.
    """
    global _capture_phase
    signal.pthread_sigmask(signal.SIG_BLOCK, _SIGSET_INT)

    frame_n = 0
    consec_fail = 0
    try:
        while not stop_event.is_set():
            t0 = time.time()

            _capture_phase = "focus"
            focus = random.randint(FOCUS_MIN, FOCUS_MAX)
            cam.set_focus(focus)

            _capture_phase = "read"
            ret, data = cam.read()
            frame_n += 1

            _capture_phase = "process"
            if ret:
                consec_fail = 0
                print(f"[Main] frame={frame_n} focus={focus}", flush=True)
            else:
                consec_fail += 1
                print(f"[Main] frame={frame_n} READ FAILED ({consec_fail}/{CONSEC_FAIL})", flush=True)
                if consec_fail >= CONSEC_FAIL:
                    print(f"[Main] {CONSEC_FAIL} consecutive failures, stopping.", flush=True)
                    break

            _capture_phase = "wait"
            elapsed = time.time() - t0
            wait = interval - elapsed
            if wait > 0 and not stop_event.is_set():
                stop_event.wait(timeout=wait)
    finally:
        _capture_phase = "release"
        _release_cam_safe(cam)
        _capture_phase = "done"


# ── main ─────────────────────────────────────────────────────────────────────
def main():
    # ① bash `&` 실행 시 SIGINT=SIG_IGN 상속 복원
    if signal.getsignal(signal.SIGINT) == signal.SIG_IGN:
        signal.signal(signal.SIGINT, signal.default_int_handler)

    # dmesg 기준 타임스탬프 기록
    dmesg_t0 = _dmesg_boot_ts()
    _print_dmesg("start", _dmesg_since(dmesg_t0))

    # ② open_camera() 전체를 SIGINT 블로킹 상태에서 실행
    signal.pthread_sigmask(signal.SIG_BLOCK, _SIGSET_INT)
    try:
        cam = open_camera()
    except Exception as e:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.pthread_sigmask(signal.SIG_UNBLOCK, _SIGSET_INT)
        print(f"[Main] {e}", flush=True)
        _print_dmesg("error", _dmesg_since(dmesg_t0))
        sys.exit(1)

    interval = 1.0 / FPS
    stop_event = threading.Event()

    # ④ SIGINT/SIGTERM 핸들러를 stop_event.set()으로 교체
    def _stop_handler(sig, frame):
        global _signal_phase
        signame = signal.Signals(sig).name
        _signal_phase = _capture_phase
        print(f"[Signal] {signame} received (phase={_capture_phase})", flush=True)
        stop_event.set()

    signal.signal(signal.SIGINT, _stop_handler)
    signal.signal(signal.SIGTERM, _stop_handler)

    # ③ 캡처 스레드 시작
    t = threading.Thread(target=_capture_worker, args=(cam, interval, stop_event), daemon=True)
    try:
        t.start()
    except Exception as e:
        print(f"[Main] Thread start failed: {e}", flush=True)
        _release_cam_safe(cam)
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.pthread_sigmask(signal.SIG_UNBLOCK, _SIGSET_INT)
        sys.exit(1)

    print(f"[Main] Capture thread started (interval={interval:.3f}s)", flush=True)

    # 메인 스레드 SIGINT 언블로킹
    signal.pthread_sigmask(signal.SIG_UNBLOCK, _SIGSET_INT)

    # ⑨ 워치독: stop 후 cam.read() hang 감지
    while t.is_alive():
        t.join(timeout=0.5)
        if stop_event.is_set() and t.is_alive():
            t.join(timeout=JOIN_TIMEOUT)
            if t.is_alive():
                print(f"[Main] Capture thread stuck after {JOIN_TIMEOUT}s (phase={_capture_phase}), forcing exit.", flush=True)
                _print_dmesg("exit", _dmesg_since(dmesg_t0))
                os._exit(1)

    _print_dmesg("exit", _dmesg_since(dmesg_t0))
    print("[Main] Done.", flush=True)
    sys.exit(0)


if __name__ == "__main__":
    main()
