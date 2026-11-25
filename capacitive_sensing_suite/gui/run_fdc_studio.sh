#!/usr/bin/env bash
# Launch FDC Studio. First run creates ~/cal-venv (shared with the other tools) and installs the requirements.
#   Usage: ./run_fdc_studio.sh [--simulate]
set -e
here="$(cd "$(dirname "$0")" && pwd)"
venv="${CAL_VENV:-$HOME/cal-venv}"
[ -x "$venv/bin/python" ] || python3 -m venv "$venv"
"$venv/bin/python" -c "import PyQt5, pyqtgraph, serial, numpy" 2>/dev/null || "$venv/bin/pip" install -r "$here/requirements.txt"
if [ -z "$QT_QPA_PLATFORM" ] && [ "$XDG_SESSION_TYPE" = "wayland" ]; then export QT_QPA_PLATFORM=wayland; fi
exec "$venv/bin/python" "$here/fdc_studio.py" "$@"
