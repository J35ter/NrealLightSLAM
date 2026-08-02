#!/usr/bin/env bash
# start-tracker.sh — run the Nreal Light head tracker in HUD mode with the
# Opentrack UDP output, including the protocol switch (Phase 1, 3-DoF).
#
#   HUD is always on (2 Hz readout, spec G7).
#   UDP goes to $HOST:$PORT as $PROTOCOL frames at $UDP_RATE Hz in $UNITS.
#
# Usage:
#   ./start-tracker.sh [extra tracker args...]      # e.g. --invert-yaw --sensitivity 1.5
#   PROTOCOL=extended ./start-tracker.sh            # env overrides, defaults below
#
# Overridable environment variables:
#   PROTOCOL  classic|extended   (default classic)   --protocol switch (OQ1)
#   HOST      Opentrack host     (default 127.0.0.1) --host
#   PORT      Opentrack UDP port (default 4242)      --port
#   UDP_RATE  Hz                 (default 60)        --udp-rate
#   UNITS     deg|rad            (default deg)       --units
#   BIN       tracker binary     (default target/release/neuromancer-tracker)
#   BUILD     auto|always        (default auto: build only if binary missing)

set -euo pipefail

# Run from the project root no matter where the script is invoked from.
cd "$(dirname "$0")"

BIN="${BIN:-target/release/neuromancer-tracker}"
HOST="${HOST:-127.0.0.1}"
PORT="${PORT:-4242}"
UDP_RATE="${UDP_RATE:-60}"
UNITS="${UNITS:-deg}"
PROTOCOL="${PROTOCOL:-classic}"

case "$PROTOCOL" in
    classic|extended) ;;
    *) echo "error: PROTOCOL must be 'classic' or 'extended' (got '$PROTOCOL')" >&2; exit 2 ;;
esac

case "$UNITS" in
    deg|rad) ;;
    *) echo "error: UNITS must be 'deg' or 'rad' (got '$UNITS')" >&2; exit 2 ;;
esac

if [[ "${BUILD:-auto}" == "always" ]] || [[ ! -x "$BIN" ]]; then
    echo "building release binary..." >&2
    cargo build --release
fi

echo "tracker: HUD on, UDP -> ${HOST}:${PORT} (protocol=${PROTOCOL}, ${UDP_RATE} Hz, ${UNITS})" >&2
echo "extra args: $*" >&2

# exec so Ctrl-C / SIGINT reaches the tracker directly (its own signal handling).
exec "$BIN" \
    --hud \
    --host "$HOST" \
    --port "$PORT" \
    --protocol "$PROTOCOL" \
    --udp-rate "$UDP_RATE" \
    --units "$UNITS" \
    "$@"
