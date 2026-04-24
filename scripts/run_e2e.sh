#!/usr/bin/env bash
#
# Run the full end-to-end test matrix against a freshly-launched
# camera_ws_server. Each suite (Python internal, telefacet-web Node.js,
# telefacet C++ headless) is optional — the script skips any suite whose
# source directory or build artifact isn't present.
#
# Usage:
#   scripts/run_e2e.sh
#
# Env vars:
#   TELEFACET_WEB_DIR   Path to webWs/telefacet-web (enables Option 2 if set).
#   TELEFACET_DIR       Path to cppWs/telefacet     (enables Option 3 if set).
#   CHERUPI_BUILD_DIR   Server build dir            (default: build/).
#   CHERUPI_WS_URL      WebSocket URL               (default: ws://localhost:9001).
#
# Requires a Pi 5 with IMX519 cameras connected. No fake-camera mode exists.

set -u -o pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUILD_DIR="${CHERUPI_BUILD_DIR:-$ROOT/build}"
SERVER_BIN="$BUILD_DIR/camera_ws_server"
WS_URL="${CHERUPI_WS_URL:-ws://localhost:9001}"

color() { if [ -t 1 ]; then printf '\033[%sm%s\033[0m' "$1" "$2"; else printf '%s' "$2"; fi; }
info() { echo; color "1;34" "== $1 =="; echo; }
ok()   { color "1;32" "  PASS: $1"; echo; }
fail() { color "1;31" "  FAIL: $1"; echo; }

if [ ! -x "$SERVER_BIN" ]; then
  echo "error: server binary not found at $SERVER_BIN" >&2
  echo "       build it first: cmake -B $BUILD_DIR && cmake --build $BUILD_DIR" >&2
  exit 2
fi

# -- Start server -----------------------------------------------------------
info "Starting camera_ws_server"
"$SERVER_BIN" >"$ROOT/.e2e-server.log" 2>&1 &
SERVER_PID=$!
cleanup() {
  if kill -0 "$SERVER_PID" 2>/dev/null; then
    echo "Stopping server PID $SERVER_PID"
    kill -INT "$SERVER_PID" 2>/dev/null || true
    wait "$SERVER_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

# Wait for port 9001 to open (up to 10s).
HOST_PORT="${WS_URL#ws://}"; HOST_PORT="${HOST_PORT%%/*}"
HOST="${HOST_PORT%:*}"; PORT="${HOST_PORT##*:}"
for i in $(seq 1 100); do
  if (echo > /dev/tcp/"$HOST"/"$PORT") >/dev/null 2>&1; then
    break
  fi
  sleep 0.1
done
if ! (echo > /dev/tcp/"$HOST"/"$PORT") >/dev/null 2>&1; then
  fail "server did not open $HOST:$PORT (see $ROOT/.e2e-server.log)"
  exit 1
fi
echo "  server listening on $HOST:$PORT (PID $SERVER_PID)"

SUITES_RUN=0
SUITES_FAILED=0
FAILED_NAMES=()

# -- Option 1: Python internal ---------------------------------------------
info "Option 1: Python internal harness"
if command -v pytest >/dev/null 2>&1 || python3 -c 'import pytest' 2>/dev/null; then
  if pytest "$ROOT/tests/e2e/" --server-url="$WS_URL" -v; then
    ok "python internal"
  else
    fail "python internal"
    FAILED_NAMES+=("python-internal")
    SUITES_FAILED=$((SUITES_FAILED + 1))
  fi
  SUITES_RUN=$((SUITES_RUN + 1))
else
  echo "  SKIP: pytest not installed (pip install -r $ROOT/tests/e2e/requirements.txt)"
fi

# -- Option 2: telefacet-web Node.js ----------------------------------------
info "Option 2: telefacet-web Node.js harness"
if [ -n "${TELEFACET_WEB_DIR:-}" ] && [ -f "$TELEFACET_WEB_DIR/package.json" ]; then
  if [ -d "$TELEFACET_WEB_DIR/node_modules" ]; then
    if (cd "$TELEFACET_WEB_DIR" && TELEFACET_WS_URL="$WS_URL" npm run test:e2e); then
      ok "telefacet-web"
    else
      fail "telefacet-web"
      FAILED_NAMES+=("telefacet-web")
      SUITES_FAILED=$((SUITES_FAILED + 1))
    fi
    SUITES_RUN=$((SUITES_RUN + 1))
  else
    echo "  SKIP: $TELEFACET_WEB_DIR/node_modules missing — run 'npm install' there first"
  fi
else
  echo "  SKIP: set TELEFACET_WEB_DIR to enable"
fi

# -- Option 3: telefacet C++ headless ---------------------------------------
info "Option 3: telefacet C++ headless harness"
TELEFACET_BIN=""
if [ -n "${TELEFACET_DIR:-}" ]; then
  for candidate in \
      "$TELEFACET_DIR/build/tests/telefacet_e2e_tests" \
      "$TELEFACET_DIR/build-tests/tests/telefacet_e2e_tests" \
      "$TELEFACET_DIR/build-e2e/tests/telefacet_e2e_tests"; do
    if [ -x "$candidate" ]; then TELEFACET_BIN="$candidate"; break; fi
  done
fi
if [ -n "$TELEFACET_BIN" ]; then
  if TELEFACET_WS_URL="$WS_URL" "$TELEFACET_BIN"; then
    ok "telefacet-headless"
  else
    fail "telefacet-headless"
    FAILED_NAMES+=("telefacet-headless")
    SUITES_FAILED=$((SUITES_FAILED + 1))
  fi
  SUITES_RUN=$((SUITES_RUN + 1))
else
  echo "  SKIP: build telefacet_e2e_tests first — cmake -DTELEFACET_BUILD_TESTS=ON -DTELEFACET_BUILD_GUI=OFF"
fi

# -- Summary ----------------------------------------------------------------
echo
info "Summary"
echo "  suites run:    $SUITES_RUN"
echo "  suites failed: $SUITES_FAILED"
if [ $SUITES_FAILED -gt 0 ]; then
  echo "  failed: ${FAILED_NAMES[*]}"
  exit 1
fi
exit 0
