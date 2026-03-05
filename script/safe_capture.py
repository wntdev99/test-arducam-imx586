#!/usr/bin/env python3
"""
safe_capture.py — 장치 잠금을 완전히 차단하는 스크립트

[핵심 설계]
  - VideoCapture()가 커널 D-state(uninterruptible sleep)에 빠질 수 있음
  - D-state에서는 SIGALRM/SIGINT/SIGKILL 모두 전달 불가
  - 따라서 VideoCapture()는 daemon 스레드에서 실행하고,
    메인 스레드가 join(timeout)으로 감시 → 타임아웃 시 os._exit()

[수정 내역]
  ① SIGINT SIG_IGN 상속 복원 (bash & 실행 대응)
  ② open_camera() 전체 구간 SIGINT 블로킹 (pthread_sigmask)
  ③ 캡처 전용 스레드 + SIGINT 영구 블로킹
  ④ SIGINT 핸들러를 stop_event.set()으로 교체 (SIGABRT 방지)
  ⑤ VideoCapture()를 daemon 스레드에서 실행 + join(timeout) 감시
     → D-state hang 시 os._exit(1)로 프로세스 종료 (SIGALRM 대체)
  ⑥ 에러 경로 SIGINT SIG_IGN 처리
  ⑦ open_camera() 내 모든 예외 경로에서 cap.release() 보장
  ⑧ SIGTERM 핸들러 추가
  ⑨ 캡처 루프 종료 후 join(timeout) 워치독
  ⑩ dup2(/dev/null) + USBDEVFS_RESET: fd 닫기 → 커널 cleanup → 펌웨어 초기화
  ⑪ 연속 read 실패 시 자동 종료
  ⑫ open_camera() 재시도 (이전 run cleanup 미완 대비)
"""

import os
import re
import cv2
import sys
import json
import time
import signal
import random
import subprocess
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
CONSEC_FAIL = int(_cfg.get("consec_fail_threshold", 3))

_SIGSET_INT = {signal.SIGINT}

VIDEOCAP_TIMEOUT = 8   # VideoCapture() 생성자 최대 허용 시간 (초)
JOIN_TIMEOUT     = 10  # stop_event 후 cap.read() 완료 최대 대기 (초)
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
        # raw format: "<timestamp> <message>"
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
    for line in lines[-20:]:  # 최대 20줄
        print(f"  {line}", flush=True)


def _find_device_fd(device_path):
    """프로세스가 열고 있는 장치의 fd 번호를 찾아 반환. 없으면 None."""
    real_device = os.path.realpath(device_path)
    proc_fd = Path(f"/proc/{os.getpid()}/fd")
    for entry in proc_fd.iterdir():
        try:
            if os.path.realpath(str(entry)) == real_device:
                return int(entry.name)
        except (OSError, ValueError):
            continue
    return None


def _usb_reset(device_path):
    """USB 장치 리셋으로 카메라 펌웨어를 초기 상태로 복원.

    Arducam IMX586(Cypress 컨트롤러)는 스트리밍 종료 후 펌웨어가
    EIO 상태에 빠지는 버그가 있음. USBDEVFS_RESET으로 펌웨어를
    강제 초기화하여 다음 세션이 깨끗한 상태에서 시작하도록 보장.
    """
    import fcntl

    video_name = os.path.basename(os.path.realpath(device_path))
    sysfs_device = Path(f"/sys/class/video4linux/{video_name}/device").resolve()
    usb_dev = sysfs_device.parent
    busnum = int((usb_dev / "busnum").read_text().strip())
    devnum = int((usb_dev / "devnum").read_text().strip())
    usb_path = f"/dev/bus/usb/{busnum:03d}/{devnum:03d}"

    fd = os.open(usb_path, os.O_RDWR)
    try:
        USBDEVFS_RESET = 0x5514  # _IO('U', 20)
        fcntl.ioctl(fd, USBDEVFS_RESET, 0)
    finally:
        os.close(fd)


def _release_cap_safe(cap):
    """장치 fd 닫기 → OpenCV 정리 → USB 리셋으로 완전한 cleanup.

    1. dup2(/dev/null): 장치 fd 원자적 닫기 → 커널 V4L2 cleanup
    2. cap.release(): OpenCV 내부 상태 정리 (/dev/null에 대해 무해)
    3. USBDEVFS_RESET: 카메라 펌웨어 초기화 (S_FMT EIO 방지)
    """
    if cap is None:
        return
    t0 = time.time()
    steps = []

    # Step 1: dup2로 장치 fd 닫기 → 커널 V4L2 cleanup
    try:
        dev_fd = _find_device_fd(DEVICE_PATH)
        if dev_fd is not None:
            devnull_fd = os.open("/dev/null", os.O_RDWR)
            os.dup2(devnull_fd, dev_fd)
            os.close(devnull_fd)
            steps.append("dup2=ok")
        else:
            steps.append("dup2=no_fd")
    except Exception as e:
        steps.append(f"dup2=err({e})")

    # Step 2: OpenCV 내부 상태 정리
    try:
        cap.release()
    except Exception:
        pass

    # Step 3: USB 리셋 → 카메라 펌웨어 초기화
    try:
        _usb_reset(DEVICE_PATH)
        steps.append("usb_reset=ok")
    except Exception as e:
        steps.append(f"usb_reset=err({e})")

    phase_info = f" sig_phase={_signal_phase}" if _signal_phase else ""
    print(f"[Capture] {' '.join(steps)}{phase_info} ({time.time() - t0:.2f}s)", flush=True)


def _diagnose_device(device_path):
    """isOpened() 실패 시 raw V4L2 ioctl로 실패 단계 진단.

    순서: open → QUERYCAP → S_FMT(8000x6000 MJPG) → REQBUFS(4, MMAP)
    각 단계 결과를 기록하고, 실패 시 해당 errno 포함.
    fd close 시 커널이 할당된 버퍼 등 모든 리소스 자동 해제.
    """
    import errno
    import fcntl
    import struct

    real_path = os.path.realpath(device_path)

    # Step 1: open
    try:
        fd = os.open(real_path, os.O_RDWR)
    except OSError as e:
        if e.errno == errno.EBUSY:
            return "kernel EBUSY"
        elif e.errno == errno.ENOENT:
            return "device missing"
        else:
            return f"open err{e.errno}({os.strerror(e.errno)})"

    results = []
    try:
        # Step 2: VIDIOC_QUERYCAP — _IOR('V', 0, 104)
        try:
            buf = bytearray(104)
            fcntl.ioctl(fd, 0x80685600, buf)
            results.append("QUERYCAP=OK")
        except OSError as e:
            results.append(f"QUERYCAP=err{e.errno}")
            return " ".join(results)

        # Step 3: VIDIOC_S_FMT — _IOWR('V', 5, 208)
        try:
            fmt = bytearray(208)
            struct.pack_into('I', fmt, 0, 1)           # type = VIDEO_CAPTURE
            struct.pack_into('I', fmt, 4, WIDTH)       # width
            struct.pack_into('I', fmt, 8, HEIGHT)      # height
            struct.pack_into('I', fmt, 12, 0x47504A4D) # pixelformat = MJPG
            fcntl.ioctl(fd, 0xc0d05605, fmt)
            actual_w = struct.unpack_from('I', fmt, 4)[0]
            actual_h = struct.unpack_from('I', fmt, 8)[0]
            results.append(f"S_FMT=OK({actual_w}x{actual_h})")
        except OSError as e:
            results.append(f"S_FMT=err{e.errno}({os.strerror(e.errno)})")
            return " ".join(results)

        # Step 4: VIDIOC_REQBUFS — _IOWR('V', 8, 20)
        try:
            reqbuf = bytearray(20)
            struct.pack_into('I', reqbuf, 0, 4)  # count = 4
            struct.pack_into('I', reqbuf, 4, 1)  # type = VIDEO_CAPTURE
            struct.pack_into('I', reqbuf, 8, 1)  # memory = MMAP
            fcntl.ioctl(fd, 0xc0145608, reqbuf)
            granted = struct.unpack_from('I', reqbuf, 0)[0]
            results.append(f"REQBUFS=OK({granted})")
        except OSError as e:
            results.append(f"REQBUFS=err{e.errno}({os.strerror(e.errno)})")
            return " ".join(results)

        # 모든 ioctl 성공 — OpenCV 내부 문제
        results.append("cv2_internal_fail")
        return " ".join(results)

    finally:
        os.close(fd)


# ── VideoCapture with thread-based timeout ───────────────────────────────────
def _videocapture_with_timeout(device_path, timeout):
    """VideoCapture()를 daemon 스레드에서 실행.

    커널 D-state(uninterruptible sleep) hang 시 SIGALRM은 전달 불가.
    대신 daemon 스레드 + join(timeout)으로 감시.
    타임아웃 시 os._exit(1) → OS가 프로세스 fd 정리.
    (생성자 단계이므로 스트리밍 미시작 → in-flight URB 없음 → STREAMOFF 불필요)
    """
    result = [None]
    exc = [None]

    def _worker():
        try:
            result[0] = cv2.VideoCapture(device_path, cv2.CAP_V4L2)
        except Exception as e:
            exc[0] = e

    t = threading.Thread(target=_worker, daemon=True)
    t.start()
    t.join(timeout=timeout)

    if t.is_alive():
        # D-state: 스레드가 커널에서 멈춤. 시그널로 깨울 수 없음.
        # daemon 스레드이므로 os._exit() 시 함께 종료.
        # OS가 프로세스의 모든 fd를 정리하여 장치 해제.
        print(f"[Camera] VideoCapture() timed out after {timeout}s (device locked?)", flush=True)
        os._exit(1)

    if exc[0] is not None:
        raise exc[0]

    return result[0]


# ── camera open ──────────────────────────────────────────────────────────────
def open_camera():
    """SIGINT가 블로킹된 상태에서 호출되어야 함 (main에서 보장).

    Phase 1: VideoCapture() — daemon 스레드 + join(timeout) (D-state hang 탈출)
             !isOpened 시 재시도 (이전 run의 USB cleanup 미완 대비)
    Phase 2: 설정 + 워밍업 5프레임 — SIGINT 블로킹만
    """
    cap = None
    last_err = None

    for attempt in range(OPEN_RETRIES):
        try:
            print(f"[Camera] Opening {DEVICE_PATH} (attempt {attempt + 1}/{OPEN_RETRIES}) ...", flush=True)
            cap = _videocapture_with_timeout(DEVICE_PATH, VIDEOCAP_TIMEOUT)
            # _videocapture_with_timeout은 D-state 시 os._exit() 호출하므로
            # 여기 도달하면 VideoCapture()는 정상 반환됨

            if cap.isOpened():
                break  # 성공

            # 장치가 반환됐지만 열리지 않음 — raw open으로 원인 진단
            diag = _diagnose_device(DEVICE_PATH)
            try:
                cap.release()
            except Exception:
                pass
            cap = None
            last_err = RuntimeError(f"Cannot open {DEVICE_PATH} ({diag})")

            if attempt < OPEN_RETRIES - 1:
                print(f"[Camera] Device not ready, retry in {OPEN_RETRY_WAIT}s ...", flush=True)
                time.sleep(OPEN_RETRY_WAIT)

        except Exception as e:
            if cap is not None:
                try:
                    cap.release()
                except Exception:
                    pass
                cap = None
            last_err = e
            if attempt < OPEN_RETRIES - 1:
                print(f"[Camera] Open failed: {e}, retry in {OPEN_RETRY_WAIT}s ...", flush=True)
                time.sleep(OPEN_RETRY_WAIT)

    if cap is None or not cap.isOpened():
        raise last_err or RuntimeError(f"Cannot open {DEVICE_PATH}")

    # Phase 2: 설정 + 워밍업 (SIGINT 블로킹 중)
    try:
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

        # 워밍업: 5프레임으로 제한, sleep 없음
        print("[Camera] Warming up ...", flush=True)
        for i in range(5):
            ret, _ = cap.read()   # SIGINT 블로킹 중 → ioctl 절대 중단 불가
            print(f"[Camera] Warm-up {i+1}/5 {'OK' if ret else 'FAIL'}", flush=True)

        return cap

    except BaseException:
        # 워밍업/설정 중 예외 → cap 해제
        _release_cap_safe(cap)
        raise


# ── 캡처 전용 스레드 ─────────────────────────────────────────────────────────
# phase 변수: 시그널 수신 시 어떤 단계에 있었는지 기록
_capture_phase = "init"
_signal_phase = ""  # 시그널 수신 시점의 phase 기록


def _capture_worker(cap, interval, stop_event):
    """캡처 전용 스레드.

    시작 즉시 SIGINT를 영구 블로킹 → cap.read() 절대 중단 불가.
    stop_event 세팅 확인 후 루프 종료, finally에서 cap.release() 보장.
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
            cap.set(cv2.CAP_PROP_FOCUS, focus)

            _capture_phase = "read"
            ret, frame = cap.read()
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
        _release_cap_safe(cap)
        _capture_phase = "done"


# ── main ─────────────────────────────────────────────────────────────────────
def main():
    # ① bash `&` 실행 시 SIGINT=SIG_IGN 상속 복원
    if signal.getsignal(signal.SIGINT) == signal.SIG_IGN:
        signal.signal(signal.SIGINT, signal.default_int_handler)

    # dmesg 기준 타임스탬프 기록 (이후 발생한 커널 메시지만 필터)
    dmesg_t0 = _dmesg_boot_ts()
    _print_dmesg("start", _dmesg_since(dmesg_t0))

    # ② open_camera() 전체를 SIGINT 블로킹 상태에서 실행
    signal.pthread_sigmask(signal.SIG_BLOCK, _SIGSET_INT)
    try:
        cap = open_camera()
    except Exception as e:
        # ⑥ 언블로킹 전 SIG_IGN: 대기 중인 SIGINT가 KI가 되는 것을 방지
        # ⑦ 모든 Exception을 잡음 — open_camera() 내부에서 이미 cap.release() 완료
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.pthread_sigmask(signal.SIG_UNBLOCK, _SIGSET_INT)
        print(f"[Main] {e}", flush=True)
        _print_dmesg("error", _dmesg_since(dmesg_t0))
        sys.exit(1)

    interval = 1.0 / FPS
    stop_event = threading.Event()

    # ④ SIGINT/SIGTERM 핸들러를 stop_event.set()으로 교체
    # + 시그널 수신 시점의 capture phase 기록
    def _stop_handler(sig, frame):
        global _signal_phase
        signame = signal.Signals(sig).name
        _signal_phase = _capture_phase
        print(f"[Signal] {signame} received (phase={_capture_phase})", flush=True)
        stop_event.set()

    signal.signal(signal.SIGINT, _stop_handler)
    # ⑧ SIGTERM도 정상 종료 유도
    signal.signal(signal.SIGTERM, _stop_handler)

    # ③ 캡처 스레드 시작 (daemon=True: os._exit() 시 함께 종료)
    t = threading.Thread(target=_capture_worker, args=(cap, interval, stop_event), daemon=True)
    try:
        t.start()
    except Exception as e:
        print(f"[Main] Thread start failed: {e}", flush=True)
        _release_cap_safe(cap)
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.pthread_sigmask(signal.SIG_UNBLOCK, _SIGSET_INT)
        sys.exit(1)

    print(f"[Main] Capture thread started (interval={interval:.3f}s)", flush=True)

    # 메인 스레드 SIGINT 언블로킹 — 이제부터 SIGINT → stop_event.set()
    signal.pthread_sigmask(signal.SIG_UNBLOCK, _SIGSET_INT)

    # ⑨ 워치독: stop 후 cap.read() hang 감지 (0.5s 폴링)
    while t.is_alive():
        t.join(timeout=0.5)
        if stop_event.is_set() and t.is_alive():
            t.join(timeout=JOIN_TIMEOUT)
            if t.is_alive():
                print(f"[Main] Capture thread stuck after {JOIN_TIMEOUT}s (phase={_capture_phase}), forcing exit.", flush=True)
                # capture thread의 finally가 실행되지 않으므로
                # 메인 스레드에서 장치 fd를 직접 닫아 커널 cleanup 유도
                try:
                    dev_fd = _find_device_fd(DEVICE_PATH)
                    if dev_fd is not None:
                        devnull_fd = os.open("/dev/null", os.O_RDWR)
                        os.dup2(devnull_fd, dev_fd)
                        os.close(devnull_fd)
                except Exception:
                    pass
                _print_dmesg("exit", _dmesg_since(dmesg_t0))
                os._exit(1)

    _print_dmesg("exit", _dmesg_since(dmesg_t0))
    print("[Main] Done.", flush=True)
    sys.exit(0)


if __name__ == "__main__":
    main()
