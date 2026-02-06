#!/usr/bin/env bash
# Check whether realtime scheduling is properly configured
# for libfranka / franka_control on a PREEMPT_RT system.

set -euo pipefail

RT_GROUP="realtime"
USER_NAME="$(whoami)"

echo "=== Franka Realtime Configuration Check ==="
echo

# ------------------------------------------------------------
# 1) Kernel check
# ------------------------------------------------------------
echo "[1] Kernel check"
KERNEL_INFO="$(uname -a)"
echo "  Kernel: $KERNEL_INFO"

if echo "$KERNEL_INFO" | grep -qi "PREEMPT_RT"; then
  echo "  ✔ PREEMPT_RT kernel detected"
else
  echo "  ⚠ PREEMPT_RT not detected (not fatal, but recommended)"
fi
echo

# ------------------------------------------------------------
# 2) Group membership check
# ------------------------------------------------------------
echo "[2] Group membership"
if id -nG "$USER_NAME" | grep -qw "$RT_GROUP"; then
  echo "  ✔ User '$USER_NAME' is in '$RT_GROUP' group"
else
  echo "  ✖ User '$USER_NAME' is NOT in '$RT_GROUP' group"
  echo "    -> Run the setup script and re-login"
fi
echo

# ------------------------------------------------------------
# 3) ulimit checks
# ------------------------------------------------------------
echo "[3] Realtime limits (ulimit)"

RTPRIO="$(ulimit -r)"
MEMLOCK="$(ulimit -l)"

echo "  rtprio (ulimit -r): $RTPRIO"
echo "  memlock (ulimit -l): $MEMLOCK"

if [ "$RTPRIO" -gt 0 ]; then
  echo "  ✔ Realtime priority allowed"
else
  echo "  ✖ Realtime priority is 0 (this WILL break libfranka)"
fi

if [ "$MEMLOCK" = "unlimited" ] || [ "$MEMLOCK" -ge 1048576 ] 2>/dev/null; then
  echo "  ✔ memlock looks sufficient"
else
  echo "  ⚠ memlock may be too low (recommended: unlimited)"
fi
echo

# ------------------------------------------------------------
# 4) PAM limits check
# ------------------------------------------------------------
echo "[4] PAM limits configuration"

PAM_OK=true

check_pam_file() {
  FILE="$1"
  if [ -f "$FILE" ]; then
    if grep -q "pam_limits.so" "$FILE"; then
      echo "  ✔ pam_limits.so present in $FILE"
    else
      echo "  ✖ pam_limits.so NOT present in $FILE"
      PAM_OK=false
    fi
  else
    echo "  ⚠ $FILE not found"
  fi
}

check_pam_file "/etc/pam.d/common-session"
check_pam_file "/etc/pam.d/common-session-noninteractive"

echo

# ------------------------------------------------------------
# 5) Effective process limits (ground truth)
# ------------------------------------------------------------
echo "[5] Effective limits from /proc"
grep -i "Max realtime priority" /proc/$$/limits || true
grep -i "Max locked memory" /proc/$$/limits || true
echo

# ------------------------------------------------------------
# Final verdict
# ------------------------------------------------------------
echo "=== Verdict ==="

if id -nG "$USER_NAME" | grep -qw "$RT_GROUP" \
   && [ "$RTPRIO" -gt 0 ]; then
  echo "✔ System looks correctly configured for libfranka realtime."
  echo "You should be able to start franka_control without the scheduling error."
else
  echo "✖ Realtime configuration is NOT fully correct."
  echo "Fix the issues above, then log out and log back in."
fi

echo

