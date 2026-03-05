#!/usr/bin/env python3
"""
v4l2_camera.py — Raw V4L2 ioctl 기반 카메라 제어

OpenCV VideoCapture의 내부 cleanup 순서를 제어할 수 없어 발생하는
Arducam IMX586 펌웨어 EIO 문제를 해결하기 위해,
모든 V4L2 ioctl을 직접 수행.

cleanup 순서 (핵심):
  1. VIDIOC_STREAMOFF — URB 동기 취소
  2. munmap — 모든 버퍼 언맵
  3. VIDIOC_REQBUFS(0) — 커널 버퍼 해제
  4. os.close(fd) — fd 닫기
  5. USBDEVFS_RESET — 카메라 펌웨어 초기화
"""

import os
import mmap
import fcntl
import struct
import threading
from pathlib import Path

# ── ioctl 번호 (x86_64, 기존 코드에서 검증됨) ────────────────────────────────
VIDIOC_QUERYCAP  = 0x80685600
VIDIOC_G_FMT     = 0xc0d05604
VIDIOC_S_FMT     = 0xc0d05605
VIDIOC_REQBUFS   = 0xc0145608
VIDIOC_QUERYBUF  = 0xc0585609
VIDIOC_QBUF      = 0xc058560f
VIDIOC_DQBUF     = 0xc0585611
VIDIOC_STREAMON   = 0x40045612
VIDIOC_STREAMOFF  = 0x40045613
VIDIOC_S_PARM    = 0xc0cc5616
VIDIOC_S_CTRL    = 0xc008561c
USBDEVFS_RESET   = 0x5514

# V4L2 상수
V4L2_BUF_TYPE_VIDEO_CAPTURE = 1
V4L2_MEMORY_MMAP = 1
V4L2_PIX_FMT_MJPEG = 0x47504A4D  # 'MJPG'
V4L2_CID_FOCUS_ABSOLUTE = 0x009A090A

# struct 크기
SIZEOF_V4L2_FORMAT = 208
SIZEOF_V4L2_BUFFER = 88
SIZEOF_V4L2_REQUESTBUFFERS = 20
SIZEOF_V4L2_STREAMPARM = 204
SIZEOF_V4L2_CONTROL = 8
SIZEOF_V4L2_CAPABILITY = 104

NUM_BUFFERS = 4


class V4L2Camera:
    """Raw V4L2 ioctl 기반 카메라.

    Usage:
        cam = V4L2Camera("/dev/video0", 8000, 6000, 3)
        cam.open()
        ok, data = cam.read()  # data = JPEG bytes
        cam.set_focus(200)
        cam.close()
    """

    def __init__(self, device_path, width, height, fps):
        self.device_path = device_path
        self.width = width
        self.height = height
        self.fps = fps

        self._fd = -1
        self._buffers = []     # list of mmap objects
        self._buf_lengths = [] # buffer lengths for munmap
        self._streaming = False

    def open(self):
        """장치 열기 → 포맷 설정 → 버퍼 할당 → 스트리밍 시작."""
        real_path = os.path.realpath(self.device_path)
        self._fd = os.open(real_path, os.O_RDWR)

        try:
            self._querycap()
            self._s_fmt()
            self._s_parm()
            self._reqbufs()
            self._mmap_buffers()
            self._qbuf_all()
            self._streamon()
        except Exception:
            self.close()
            raise

    def read(self):
        """프레임 1장 캡처. (True, jpeg_bytes) 또는 (False, None) 반환."""
        if self._fd < 0 or not self._streaming:
            return False, None

        try:
            # DQBUF
            buf = bytearray(SIZEOF_V4L2_BUFFER)
            struct.pack_into('I', buf, 0, 0)                          # index (커널이 채움)
            struct.pack_into('I', buf, 4, V4L2_BUF_TYPE_VIDEO_CAPTURE)  # type
            struct.pack_into('I', buf, 60, V4L2_MEMORY_MMAP)           # memory
            fcntl.ioctl(self._fd, VIDIOC_DQBUF, buf)

            index = struct.unpack_from('I', buf, 0)[0]
            bytesused = struct.unpack_from('I', buf, 8)[0]

            # 데이터 복사
            data = bytes(self._buffers[index][:bytesused])

            # QBUF — 버퍼 반환
            qbuf = bytearray(SIZEOF_V4L2_BUFFER)
            struct.pack_into('I', qbuf, 0, index)
            struct.pack_into('I', qbuf, 4, V4L2_BUF_TYPE_VIDEO_CAPTURE)
            struct.pack_into('I', qbuf, 60, V4L2_MEMORY_MMAP)
            fcntl.ioctl(self._fd, VIDIOC_QBUF, qbuf)

            return True, data

        except OSError:
            return False, None

    def set_focus(self, value):
        """포커스 절대값 설정."""
        if self._fd < 0:
            return
        ctrl = bytearray(SIZEOF_V4L2_CONTROL)
        struct.pack_into('I', ctrl, 0, V4L2_CID_FOCUS_ABSOLUTE)
        struct.pack_into('i', ctrl, 4, value)
        try:
            fcntl.ioctl(self._fd, VIDIOC_S_CTRL, ctrl)
        except OSError:
            pass  # 포커스 실패는 무시

    def close(self):
        """안전한 cleanup: STREAMOFF → munmap → REQBUFS(0) → close → USB reset.

        각 단계는 개별 try/except — 이전 단계 실패해도 다음 단계 진행.
        STREAMOFF는 D-state hang 가능 → daemon 스레드 + 3초 타임아웃.
        타임아웃 시 skip하고 close(fd)로 진행 (커널이 release 시 cleanup).
        """
        steps = []

        # Step 1: STREAMOFF (daemon 스레드 + 타임아웃)
        if self._streaming and self._fd >= 0:
            fd = self._fd
            done = threading.Event()

            def _streamoff():
                try:
                    buf_type = struct.pack('I', V4L2_BUF_TYPE_VIDEO_CAPTURE)
                    fcntl.ioctl(fd, VIDIOC_STREAMOFF, buf_type)
                except Exception:
                    pass
                done.set()

            t = threading.Thread(target=_streamoff, daemon=True)
            t.start()
            t.join(timeout=3)

            if done.is_set():
                steps.append("streamoff=ok")
            else:
                steps.append("streamoff=timeout(3s)")
            self._streaming = False

        # Step 2: munmap
        for i, m in enumerate(self._buffers):
            try:
                m.close()
            except Exception as e:
                steps.append(f"munmap{i}=err({e})")
        if self._buffers:
            steps.append(f"munmap={len(self._buffers)}bufs")
        self._buffers.clear()
        self._buf_lengths.clear()

        # Step 3: REQBUFS(0) — 커널 버퍼 해제
        if self._fd >= 0:
            try:
                reqbuf = bytearray(SIZEOF_V4L2_REQUESTBUFFERS)
                struct.pack_into('I', reqbuf, 0, 0)   # count = 0
                struct.pack_into('I', reqbuf, 4, V4L2_BUF_TYPE_VIDEO_CAPTURE)
                struct.pack_into('I', reqbuf, 8, V4L2_MEMORY_MMAP)
                fcntl.ioctl(self._fd, VIDIOC_REQBUFS, reqbuf)
                steps.append("reqbufs0=ok")
            except Exception as e:
                steps.append(f"reqbufs0=err({e})")

        # Step 4: close(fd)
        fd = self._fd
        self._fd = -1
        if fd >= 0:
            try:
                os.close(fd)
                steps.append("close=ok")
            except Exception as e:
                steps.append(f"close=err({e})")

        # Step 5: USB reset
        try:
            self._usb_reset()
            steps.append("usb_reset=ok")
        except Exception as e:
            steps.append(f"usb_reset=err({e})")

        summary = " ".join(steps) if steps else "noop"
        print(f"[Capture] {summary}", flush=True)

    @property
    def is_open(self):
        return self._fd >= 0

    # ── 내부 메서드 ───────────────────────────────────────────────────────────

    def _querycap(self):
        buf = bytearray(SIZEOF_V4L2_CAPABILITY)
        fcntl.ioctl(self._fd, VIDIOC_QUERYCAP, buf)

    def _s_fmt(self):
        fmt = bytearray(SIZEOF_V4L2_FORMAT)
        struct.pack_into('I', fmt, 0, V4L2_BUF_TYPE_VIDEO_CAPTURE)
        struct.pack_into('I', fmt, 4, self.width)
        struct.pack_into('I', fmt, 8, self.height)
        struct.pack_into('I', fmt, 12, V4L2_PIX_FMT_MJPEG)
        fcntl.ioctl(self._fd, VIDIOC_S_FMT, fmt)

        actual_w = struct.unpack_from('I', fmt, 4)[0]
        actual_h = struct.unpack_from('I', fmt, 8)[0]
        self._sizeimage = struct.unpack_from('I', fmt, 24)[0]
        print(f"[V4L2] S_FMT: {actual_w}x{actual_h}, sizeimage={self._sizeimage}", flush=True)

    def _s_parm(self):
        parm = bytearray(SIZEOF_V4L2_STREAMPARM)
        struct.pack_into('I', parm, 0, V4L2_BUF_TYPE_VIDEO_CAPTURE)
        struct.pack_into('I', parm, 12, 1)          # timeperframe.numerator
        struct.pack_into('I', parm, 16, int(self.fps)) # timeperframe.denominator
        fcntl.ioctl(self._fd, VIDIOC_S_PARM, parm)

    def _reqbufs(self):
        reqbuf = bytearray(SIZEOF_V4L2_REQUESTBUFFERS)
        struct.pack_into('I', reqbuf, 0, NUM_BUFFERS)
        struct.pack_into('I', reqbuf, 4, V4L2_BUF_TYPE_VIDEO_CAPTURE)
        struct.pack_into('I', reqbuf, 8, V4L2_MEMORY_MMAP)
        fcntl.ioctl(self._fd, VIDIOC_REQBUFS, reqbuf)
        granted = struct.unpack_from('I', reqbuf, 0)[0]
        print(f"[V4L2] REQBUFS: requested={NUM_BUFFERS}, granted={granted}", flush=True)
        if granted == 0:
            raise RuntimeError("REQBUFS returned 0 buffers")
        self._num_buffers = granted

    def _mmap_buffers(self):
        for i in range(self._num_buffers):
            buf = bytearray(SIZEOF_V4L2_BUFFER)
            struct.pack_into('I', buf, 0, i)
            struct.pack_into('I', buf, 4, V4L2_BUF_TYPE_VIDEO_CAPTURE)
            struct.pack_into('I', buf, 60, V4L2_MEMORY_MMAP)
            fcntl.ioctl(self._fd, VIDIOC_QUERYBUF, buf)

            length = struct.unpack_from('I', buf, 72)[0]
            offset = struct.unpack_from('I', buf, 64)[0]

            m = mmap.mmap(self._fd, length,
                          mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE,
                          offset=offset)
            self._buffers.append(m)
            self._buf_lengths.append(length)

        print(f"[V4L2] mmap: {len(self._buffers)} buffers", flush=True)

    def _qbuf_all(self):
        for i in range(self._num_buffers):
            buf = bytearray(SIZEOF_V4L2_BUFFER)
            struct.pack_into('I', buf, 0, i)
            struct.pack_into('I', buf, 4, V4L2_BUF_TYPE_VIDEO_CAPTURE)
            struct.pack_into('I', buf, 60, V4L2_MEMORY_MMAP)
            fcntl.ioctl(self._fd, VIDIOC_QBUF, buf)

    def _streamon(self):
        buf_type = struct.pack('I', V4L2_BUF_TYPE_VIDEO_CAPTURE)
        fcntl.ioctl(self._fd, VIDIOC_STREAMON, buf_type)
        self._streaming = True
        print(f"[V4L2] STREAMON", flush=True)

    def _usb_reset(self):
        """USB 장치 리셋으로 카메라 펌웨어를 초기 상태로 복원."""
        real_path = os.path.realpath(self.device_path)
        video_name = os.path.basename(real_path)
        sysfs_device = Path(f"/sys/class/video4linux/{video_name}/device").resolve()
        usb_dev = sysfs_device.parent
        busnum = int((usb_dev / "busnum").read_text().strip())
        devnum = int((usb_dev / "devnum").read_text().strip())
        usb_path = f"/dev/bus/usb/{busnum:03d}/{devnum:03d}"

        fd = os.open(usb_path, os.O_RDWR)
        try:
            fcntl.ioctl(fd, USBDEVFS_RESET, 0)
        finally:
            os.close(fd)
