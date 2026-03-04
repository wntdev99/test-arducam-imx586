#!/bin/bash
set -m   # job control 활성화 — bash & 백그라운드 자식이 SIGINT=SIG_IGN을 상속하지 않음
# stress.sh — capture 스크립트 반복 실행, 문제 발생 즉시 중단·보고
#
# 사용법:
#   ./stress.sh error   — error_capture.py (버그 재현)
#   ./stress.sh safe    — safe_capture.py  (정상 동작 검증)
#
# 감지하는 실패 유형:
#   [SIGINT_FAIL]   kill -SIGINT 후 5초 내 미종료 → SIGINT 무시 (버그 ①)
#   [DEVICE_LOCK]   다음 run에서 Cannot open / RuntimeError → 장치 잠금 (버그 ②③)
#   [CRASH]         SIGINT 보내기 전에 Python이 스스로 죽음 → 비정상 종료

SCRIPT_DIR="$(cd "$(dirname "$0")/.." && pwd)"

MODE="${1:-}"
case "$MODE" in
    error) CAPTURE="$SCRIPT_DIR/script/error_capture.py" ;;
    safe)  CAPTURE="$SCRIPT_DIR/script/safe_capture.py"  ;;
    *)
        echo "Usage: $0 <error|safe>"
        echo "  error — error_capture.py (버그 재현)"
        echo "  safe  — safe_capture.py  (정상 동작 검증)"
        exit 1
        ;;
esac

LOG_DIR="$SCRIPT_DIR/log/stress_${MODE}"
mkdir -p "$LOG_DIR"

echo "[Stress] Mode: $MODE  →  $CAPTURE"

pid=""
tail_pid=""

cleanup() {
    echo ""
    echo "[Stress] Caught signal — killing child processes."
    [ -n "$tail_pid" ] && kill "$tail_pid" 2>/dev/null
    [ -n "$pid" ] && kill -SIGKILL "$pid" 2>/dev/null
    exit 0
}
trap cleanup SIGINT SIGTERM

# ─────────────────────────────────────────────────────────────────────────────
fail() {
    local reason="$1"
    local run="$2"
    local outfile="$3"
    echo ""
    echo "╔══════════════════════════════════════════════════════════╗"
    echo "║  FAIL [$reason]  run #${run}                                "
    echo "╚══════════════════════════════════════════════════════════╝"
    echo "--- Python output (last 30 lines) ---"
    tail -30 "$outfile"
    echo "--- lsusb ---"
    lsusb | grep -E "04b4|arducam" || echo "(camera not found)"
    echo "--- /dev/arducam* ---"
    ls -la /dev/arducam* /dev/video* 2>&1
    local logfile="$LOG_DIR/fail_$(date -u +%Y%m%dT%H%M%S)_run${run}_${reason}.log"
    {
        echo "mode: $MODE"
        echo "reason: $reason"
        echo "run: $run"
        echo "timestamp: $(date -u +%Y-%m-%dT%H:%M:%SZ)"
        echo "--- python output ---"
        cat "$outfile"
        echo "--- lsusb ---"
        lsusb
        echo "--- /dev/video* ---"
        ls -la /dev/arducam* /dev/video* 2>&1
    } > "$logfile"
    echo ""
    echo "→ log: $logfile"
    [ -n "$tail_pid" ] && kill "$tail_pid" 2>/dev/null
    rm -f "$outfile"
    exit 1
}

# ─────────────────────────────────────────────────────────────────────────────
run=0
success=0

while true; do
    run=$((run + 1))
    runtime=$(( RANDOM % 19 + 2 ))
    echo ""
    echo "═══ Run #${run} (planned: ${runtime}s | ok so far: ${success}) ═══"

    outfile=$(mktemp /tmp/stress_cap_XXXX.out)

    python3 "$CAPTURE" >"$outfile" 2>&1 &
    pid=$!
    tail -f "$outfile" &
    tail_pid=$!

    start=$(date +%s)
    dead=0

    while true; do
        sleep 0.5
        if ! kill -0 "$pid" 2>/dev/null; then
            dead=1; break
        fi
        if [ $(( $(date +%s) - start )) -ge "$runtime" ]; then
            break
        fi
    done

    kill "$tail_pid" 2>/dev/null; tail_pid=""

    if [ $dead -eq 1 ]; then
        wait "$pid"; ec=$?
        elapsed=$(( $(date +%s) - start ))
        echo ""
        echo "[Run #${run}] Python died after ${elapsed}s (exit=$ec)"

        if grep -q "Cannot open\|RuntimeError\|Device not ready" "$outfile"; then
            fail "DEVICE_LOCK" "$run" "$outfile"
        else
            fail "CRASH" "$run" "$outfile"
        fi
    fi

    echo "[Run #${run}] Sending SIGINT ..."
    kill -SIGINT "$pid"

    sigint_wait=0
    while kill -0 "$pid" 2>/dev/null; do
        sleep 0.5
        sigint_wait=$(( sigint_wait + 1 ))
        if [ $sigint_wait -ge 10 ]; then
            kill -SIGKILL "$pid" 2>/dev/null
            wait "$pid" 2>/dev/null
            fail "SIGINT_FAIL" "$run" "$outfile"
        fi
    done
    wait "$pid" 2>/dev/null
    ec=$?

    echo "[Run #${run}] Exited (exit=$ec) after ${runtime}s"

    if grep -q "Cannot open\|RuntimeError\|Device not ready" "$outfile"; then
        fail "DEVICE_LOCK" "$run" "$outfile"
    fi

    # PASS 시에도 종료 직전 출력 확인 (cap.release() OK / Done 검증)
    tail_lines=$(tail -3 "$outfile")
    rm -f "$outfile"
    success=$((success + 1))
    echo "[Run #${run}] PASS  (last: $(echo "$tail_lines" | tr '\n' '|'))"
    sleep 0.2
done
