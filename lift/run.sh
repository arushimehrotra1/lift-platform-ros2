#!/usr/bin/env bash
# ============================================================================
#  One script for everything.            ./run.sh help   for the list
#    ./run.sh            flash firmware (only if changed) + build ROS 2 + bridge + keyboard
#    ./run.sh sim        same stack, but against the firmware running on this PC (no hardware)
#    ./run.sh selftest   closed-loop firmware tests + protocol tests (no hardware, no ROS)
# ============================================================================
set -euo pipefail
cd "$(dirname "$0")"
ROOT="$PWD"
TOOLS="$ROOT/.tools"; BUILD="$ROOT/.build"
FQBN="esp32:esp32:esp32"
ESP_CORE="3.0.7"                       # PWM calls (ledcAttach/ledcWrite) need the 3.x API
ESP_URL="https://espressif.github.io/arduino-esp32/package_esp32_index.json"
export ARDUINO_DIRECTORIES_DATA="$ROOT/.arduino/data"
export ARDUINO_DIRECTORIES_USER="$ROOT/.arduino/user"
export ARDUINO_DIRECTORIES_DOWNLOADS="$ROOT/.arduino/dl"
export PATH="$TOOLS:$PATH"
mkdir -p "$TOOLS" "$BUILD"

say()  { printf '\033[1;36m==> %s\033[0m\n' "$*"; }
die()  { printf '\033[1;31mERROR: %s\033[0m\n' "$*" >&2; exit 1; }

find_port() {
  local p; p=$(ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null | head -n1 || true)
  [ -n "$p" ] || die "No ESP32 serial port found. Plug it in (or pass the port: ./run.sh $1 /dev/ttyUSB0)."
  echo "$p"
}
check_dialout() {
  id -nG | tr ' ' '\n' | grep -qx dialout || \
    echo "NOTE: you are not in the 'dialout' group. Run: sudo usermod -aG dialout \$USER  (then log out/in)"
}

setup_arduino() {
  if ! command -v arduino-cli >/dev/null; then
    say "Installing arduino-cli into .tools/"
    curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR="$TOOLS" sh
  fi
  if ! arduino-cli core list 2>/dev/null | grep -q "esp32:esp32 *$ESP_CORE"; then
    say "Installing ESP32 core $ESP_CORE (one-time, a few hundred MB)"
    arduino-cli core update-index --additional-urls "$ESP_URL"
    arduino-cli core install "esp32:esp32@$ESP_CORE" --additional-urls "$ESP_URL"
  fi
}

flash() {
  local port="${1:-$(find_port flash)}"
  setup_arduino
  local sum; sum=$(cat firmware/LiftPlatform/* | sha256sum | cut -d' ' -f1)
  if [ -f "$BUILD/fw.sha" ] && [ "$(cat "$BUILD/fw.sha")" = "$sum" ] && [ "${FORCE:-0}" != 1 ]; then
    say "Firmware unchanged since last flash (FORCE=1 ./run.sh flash to override)"; return
  fi
  say "Compiling firmware"
  arduino-cli compile --fqbn "$FQBN" firmware/LiftPlatform --build-path "$BUILD/fw"
  say "Uploading to $port"
  arduino-cli upload --fqbn "$FQBN" -p "$port" --input-dir "$BUILD/fw" firmware/LiftPlatform
  echo "$sum" > "$BUILD/fw.sha"
  sleep 2   # let the ESP32 reboot
}

source_ros() {
  command -v ros2 >/dev/null && return
  local f; f=$(ls /opt/ros/*/setup.bash 2>/dev/null | tail -n1 || true)
  [ -n "$f" ] || die "ROS 2 not found. Install Humble/Jazzy (https://docs.ros.org) and 'sudo apt install python3-serial'."
  set +u; source "$f"; set -u
}
build_ros() {
  source_ros
  python3 -c "import serial" 2>/dev/null || die "Missing pyserial: sudo apt install python3-serial"
  say "Building ROS 2 package"
  ( cd ros2_ws && colcon build --symlink-install --packages-select lift_platform 2>&1 | tail -n 3 )
  set +u; source ros2_ws/install/setup.bash; set -u
}

run_stack() {                       # $1 = serial port
  local port="$1"
  say "Starting bridge on $port"
  ros2 run lift_platform serial_bridge --ros-args -p port:="$port" &
  local bp=$!
  trap 'kill $bp 2>/dev/null || true' EXIT
  sleep 1.5
  kill -0 $bp 2>/dev/null || die "Bridge failed to start (check port / permissions)"
  say "Keyboard control (q to quit)"
  ros2 run lift_platform keyboard_teleop
}

selftest() {
  say "Building firmware simulator and running closed-loop checks"
  ( cd tools/sim && g++ -std=c++17 -O1 -Wall -Wextra -Wno-unused-parameter -Wno-misleading-indentation \
      -I. -o fw_sim fw_sim.cpp && ./fw_sim --selftest )
  say "Protocol tests"
  ( cd ros2_ws/src/lift_platform && python3 -m pytest -q test )
}

sim() {
  ( cd tools/sim && g++ -std=c++17 -O1 -Wno-misleading-indentation -I. -o fw_sim fw_sim.cpp )
  build_ros
  say "Starting firmware simulator"
  tools/sim/fw_sim --pty > "$BUILD/pty.txt" &
  local sp=$!
  trap 'kill $sp 2>/dev/null || true' EXIT
  sleep 0.7
  local port; port=$(awk '/PTY/{print $2}' "$BUILD/pty.txt")
  [ -n "$port" ] || die "Simulator did not start"
  run_stack "$port"
}

cmd="${1:-all}"; shift || true
case "$cmd" in
  all)      check_dialout; port="${1:-$(find_port all)}"; flash "$port"; build_ros; run_stack "$port" ;;
  setup)    setup_arduino; say "Setup complete" ;;
  flash)    check_dialout; flash "${1:-}" ;;
  build)    build_ros ;;
  bridge)   build_ros; ros2 run lift_platform serial_bridge --ros-args -p port:="${1:-$(find_port bridge)}" ;;
  keys)     build_ros; ros2 run lift_platform keyboard_teleop ;;
  sim)      sim ;;
  selftest) selftest ;;
  monitor)  setup_arduino; arduino-cli monitor -p "${1:-$(find_port monitor)}" -c baudrate=115200 ;;
  help|-h|--help|*)
    sed -n '2,7p' "$0" | sed 's/^# \{0,2\}//'
    echo "Other:  setup | flash [port] | build | bridge [port] | keys | monitor [port]" ;;
esac
