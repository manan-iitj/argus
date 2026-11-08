#!/usr/bin/env bash
# Pre-warm the Gazebo world (server + GUI), then run PX4
# SITL against it.
#
# Why: px4-rc.gzsim spawns the drone with a 5 s-timeout
# EntityFactory service call whose result it ignores, and
# it sends the camera-follow message to /gui/track exactly
# once.  When the world pulls hundreds of models from
# Gazebo Fuel, the server and GUI are still loading meshes
# at that moment, so the spawn silently times out and the
# camera follow is missed -- PX4 reports "Ready for
# takeoff!" with no drone in the scene.
#
# How: instead of sleeping a fixed number of seconds and
# hoping the load finished, both phases block on a signal
# that only fires once the work is actually done:
#
#   server -> /world/<name>/scene/info reports a model count
#             equal to the number of models the world file
#             declares (top-level <model> plus every
#             <include>, each of which becomes one model)
#   GUI    -> /gui/camera/pose publishes, which only the
#             GUI's camera scene plugin does once it has
#             attached and started drawing
#
# Both have a timeout that falls back to a short settle, so a
# slow or partly-unavailable world still starts PX4 instead of
# hanging forever.
#
# Usage: prewarm_gz_world.sh <world.sdf> <world_name> <model_target> <px4_dir>
set -u

WORLD_FILE="$1"
WORLD_NAME="$2"
MODEL_TARGET="$3"
PX4_DIR="$4"

# `make px4_sitl` only exists in the PX4 makefile, so build from there
# rather than relying on the caller having set the cwd.  Resolve the
# world file against the caller's cwd first, so a relative argument
# keeps working across the chdir.
case "$WORLD_FILE" in
  /*) ;;
  *) WORLD_FILE="$PWD/${WORLD_FILE}" ;;
esac
cd "$PX4_DIR"

# Upper bounds for each wait.  The real world pulls ~414 models
# from Gazebo Fuel and is much slower than the procedural one,
# so the server gets the larger budget.
SCENE_TIMEOUT="${PREWARM_SCENE_TIMEOUT:-420}"
GUI_TIMEOUT="${PREWARM_GUI_TIMEOUT:-180}"
# Grace period after a signal fires, so the renderer can finish
# the frame it is in the middle of.
SETTLE_SECONDS="${PREWARM_SETTLE_SECONDS:-3}"

# Gazebo environment that PX4 itself uses: cmake generates
# build/px4_sitl_default/gz_env.sh from gz_env.sh.in; it
# exports PX4_GZ_MODELS / PX4_GZ_WORLDS / PX4_GZ_PLUGINS
# and appends the models+worlds dirs to GZ_SIM_RESOURCE_PATH.
# Without it the server cannot resolve model://x500 /
# model://gimbal when PX4 spawns the drone.
GZ_ENV_SH="${PX4_DIR}/build/px4_sitl_default/gz_env.sh"
if [ -f "$GZ_ENV_SH" ]; then
  # gz_env.sh appends to these paths; initialise them so
  # `set -u` does not trip on a first-time unset value.
  export GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH:-}"
  export GZ_SIM_SYSTEM_PLUGIN_PATH="${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
  # shellcheck disable=SC1090
  . "$GZ_ENV_SH"
else
  echo "[prewarm] WARNING: ${GZ_ENV_SH} not found, model:// URIs may not resolve" >&2
fi

# The GUI needs a display; the virtual X server here is :1.
export DISPLAY="${DISPLAY:-:1}"

SERVER_PID=""
GUI_PID=""
MAKE_PID=""

cleanup() {
  trap - EXIT INT TERM
  for pid in "$MAKE_PID" "$GUI_PID" "$SERVER_PID"; do
    [ -n "$pid" ] && kill "$pid" 2>/dev/null
  done
  wait 2>/dev/null
  exit 0
}
trap cleanup EXIT INT TERM

# Models the world file declares: top-level <model> elements plus
# every <include>, since Gazebo turns each include into one model.
# Echoes nothing if the file cannot be parsed, which makes the
# caller fall back to a plain settle.
expected_models() {
  python3 - "$1" <<'PY' 2>/dev/null
import sys
import xml.etree.ElementTree as ET

world = ET.parse(sys.argv[1]).getroot().find('world')
if world is not None:
    print(len(world.findall('model')) + len(world.findall('include')))
PY
}

# Models Gazebo currently reports in the loaded scene.  gz service
# prints the message as protobuf text, where each top-level model
# starts a line with "model {".
scene_models() {
  gz service -s "/world/${WORLD_NAME}/scene/info" \
    --reqtype gz.msgs.Empty --reptype gz.msgs.Scene \
    --timeout 20000 --req '' 2>/dev/null | grep -c '^model {'
}

# Reuse the world if a previous launch left it running;
# refuse to start a second server for a different world.
running=$(gz topic -l 2>/dev/null | grep -m 1 -e '^/world/.*/clock' \
  | sed 's|/world/||; s|/clock||')
if [ -n "$running" ]; then
  if [ "$running" = "$WORLD_NAME" ]; then
    echo "[prewarm] world '${WORLD_NAME}' is already running -- reusing it"
  else
    echo "[prewarm] ERROR: another Gazebo world ('${running}') is" >&2
    echo "[prewarm] already running.  Stop it first, then re-run." >&2
    exit 1
  fi
else
  echo "[prewarm] starting Gazebo server ..."
  gz sim --verbose=1 -r -s "$WORLD_FILE" &
  SERVER_PID=$!

  # Wait for the scene (the same /world/<name>/scene/info
  # check px4-rc.gzsim uses; 90 s budget).
  ready=0
  for _ in $(seq 1 90); do
    if gz service -i --service "/world/${WORLD_NAME}/scene/info" \
        >/dev/null 2>&1; then
      ready=1
      break
    fi
    sleep 1
  done
  if [ "$ready" -ne 1 ]; then
    echo "[prewarm] ERROR: world did not load within 90 s" >&2
    exit 1
  fi

  # The service existing only means the world is up; Gazebo still
  # inserts models one at a time as their Fuel meshes resolve.
  # Block until the scene holds every model the world declares,
  # so the GUI never gets to render a partial scene.
  want=$(expected_models "$WORLD_FILE")
  if [ -z "$want" ]; then
    echo "[prewarm] WARNING: could not parse ${WORLD_FILE} --" \
         "settling 30 s instead of counting models" >&2
    sleep 30
  else
    echo "[prewarm] scene loaded -- waiting for all ${want} models to render"
    have=0
    deadline=$((SECONDS + SCENE_TIMEOUT))
    while [ "$SECONDS" -lt "$deadline" ]; do
      have=$(scene_models)
      [ "${have:-0}" -ge "$want" ] && break
      sleep 3
    done
    if [ "${have:-0}" -ge "$want" ]; then
      echo "[prewarm] all ${want} models loaded (${have} in scene)"
    else
      echo "[prewarm] WARNING: only ${have:-0}/${want} models loaded after" \
           "${SCENE_TIMEOUT} s -- starting PX4 anyway" >&2
    fi
  fi
fi

# px4-rc.gzsim skips the GUI when it finds an already-running
# world, so start it ourselves.
echo "[prewarm] starting Gazebo GUI"
gz sim -g &
GUI_PID=$!

# The GUI attaches asynchronously and then has to build the
# rendering scene from what the server has.  Its camera scene
# plugin starts publishing /gui/camera/pose once that is done,
# which is the first point at which frames actually reach the
# screen -- a fixed sleep cannot know when that happens.
echo "[prewarm] waiting for the GUI to render (up to ${GUI_TIMEOUT} s)"
if timeout "$GUI_TIMEOUT" gz topic -e -t /gui/camera/pose -n 1 \
     >/dev/null 2>&1; then
  echo "[prewarm] GUI is rendering"
else
  echo "[prewarm] WARNING: GUI published no camera pose within" \
       "${GUI_TIMEOUT} s -- settling 10 s instead" >&2
  sleep 10
fi
sleep "$SETTLE_SECONDS"

echo "[prewarm] world is warm -- starting PX4 SITL (${MODEL_TARGET})"
make px4_sitl "$MODEL_TARGET" &
MAKE_PID=$!
wait "$MAKE_PID"
RC=$?
echo "[prewarm] PX4 exited with ${RC}"
exit "$RC"