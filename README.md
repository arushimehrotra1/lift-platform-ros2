# Lift platform: ESP32 + ROS 2 + keyboard

```
 keyboard_teleop ──/lift/cmd──▶ serial_bridge ══ USB serial ══▶ ESP32 firmware ──▶ 2× Cytron MD10C ──▶ 2× actuators
   (terminal)    ◀─/lift/status─    (ROS 2)        115200        height PID → sync → PWM      ▲
                                                                                              └── pot feedback, E-stop, limits
```

The control loop runs **on the ESP32 at 50 Hz** (ROS never sits in the safety path). ROS 2 gives you topics, logging
and a place to add more nodes later.

## Run it

```bash
./run.sh selftest      # no hardware needed: runs the REAL firmware against a simulated plant + protocol tests
./run.sh sim           # full ROS 2 stack + keyboard against the simulated firmware (needs ROS 2)
./run.sh               # real thing: installs arduino-cli, flashes (only if changed), builds ROS, bridge + keyboard
```
Prerequisites: Linux, ROS 2 (Humble/Jazzy), `sudo apt install python3-serial`, and
`sudo usermod -aG dialout $USER` (then log out/in) for serial access.
`./run.sh flash /dev/ttyUSB0` if the port isn't auto-detected.

## Keys

| key | action | key | action |
|---|---|---|---|
| `w` / ↑ | raise (latched) | `space` / `x` | STOP |
| `s` / ↓ | lower (latched) | `r` | RESET latched fault |
| `g` | type height in mm + Enter → **closed-loop GOTO** | `1`…`5` | GOTO 0/25/50/75/100 % |
| `[` | calibrate: fully **retracted** | `]` | calibrate: fully **extended** |
| `j`/`k`, `n`/`m` | recovery jog left / right actuator | `?` | print gains |

## What the firmware does

1. Reads both pots → extension `xL`, `xR` (mm); height `H = (xL+xR)/2`.
2. **Outer loop:** PID on `H_target − H` → common command `u0` (conditional-integration anti-windup, derivative on measurement, 1 mm deadband).
3. **Inner loop (sync):** `e = xL − xR`, `uL = u0 − Ks·e`, `uR = u0 + Ks·e`. The leader is slowed, never reversed.
4. Limits (switches + soft limits) zero `u0` so both sides stop together. Soft ramp (0.4 s) and min-drive on the way out.
5. **Latched faults** (stop both motors until `r`): E-stop, pot wire break, sync mismatch > 12 mm, jam (driven but not moving 1.5 s). RESET is refused while E-stop is pressed.
6. Manual UP/DOWN stop by themselves 0.4 s after the last command, so a closed terminal or crashed node can't leave it running.

## ROS 2 interface

| topic | type | |
|---|---|---|
| `/lift/cmd` | `std_msgs/String` in | `UP DOWN STOP RESET GOTO 50 CALMIN CALMAX PID kp ki kd KS x VMAX x JOG L 1 GET` (whitelisted) |
| `/lift/set_height` | `std_msgs/Float32` in | mm, same as `GOTO` |
| `/lift/joint_states` | `sensor_msgs/JointState` | actuator extension (m), effort = drive −1..1 |
| `/lift/height`, `/lift/status`, `/lift/device_log` | Float32 / String | |

Example: `ros2 topic pub --once /lift/set_height std_msgs/msg/Float32 "{data: 60.0}"`

## Wiring (pins in `firmware/LiftPlatform/config.h`)

| signal | ESP32 | note |
|---|---|---|
| MD10C #1 (LEFT) PWM / DIR | GPIO 25 / 26 | |
| MD10C #2 (RIGHT) PWM / DIR | GPIO 27 / 13 | 10 kΩ pull-down on each PWM line |
| Left / right pot wiper | GPIO 34 / 35 | pot ends to 3V3 and GND, **never 12 V** |
| E-stop | GPIO 4 → GND | normally-closed; open = triggered |
| Upper / lower limit | GPIO 32 / 33 → GND | normally-closed |
| MD10C GND, ESP32 GND, 12 V supply − | common | |

Because the inputs are NC-to-GND, **a missing switch triggers a fault.** On the bench, jumper each input to GND.

## Bring-up order (do not skip)

1. `./run.sh selftest`, confirm PASS.
2. Flash with motors **unpowered**. Jumper E-stop/limits to GND. Press `?`; watch `/lift/status`; move the actuators by hand and confirm both extensions change.
3. Power the motors with the platform **unloaded**. Hold `w` briefly. If an actuator goes the wrong way, set `ACT_L_INVERT`/`ACT_R_INVERT` to 1 and reflash.
4. Drive fully down → `[`. Drive fully up → `]`. Status changes from UNCALIBRATED to `cal`. Until then GOTO, sync and soft limits are disabled and speed is limited to 50 %.
5. `g` → `50` → Enter. Tune with `PID 0.08 0.01 0`, `KS 0.1` (saved to flash). Raise `Ki` only if it stops short; lower `Kp` if it hunts.
6. Fault-test: press E-stop mid-move, unplug a pot wire, block one actuator.

## Things to check on your actuator (ARF03-12-A1-205-100-C11, from your photo)

* **Feedback type:** the label doesn't say. This code needs a built-in **potentiometer**. If yours has Hall/encoder wires or none, tell me and the position module changes.
* **Duty cycle 10 %:** for example 1 min of motion then 9 min rest. The firmware does **not** enforce this; repeated bench tests will overheat it.
* **7 mm/s:** a full 100 mm stroke takes about 14 s, so GOTO moves are slow by design. Gains assume this.
* The e-stop must also cut motor power in hardware. The software stop is not enough.

## Not included (your slides list them; they plug in later)

ToF height sensing (height here is mean actuator extension, not scissor/four-bar height), IMU tilt, load cells, current sensing, locking pin.
To use a ToF later, replace the `H` computation in `readPots()`.

## Layout

```
run.sh                       one script for everything
firmware/LiftPlatform/       config.h (pins/constants) + LiftPlatform.ino (all control logic)
ros2_ws/src/lift_platform/   serial_bridge, keyboard_teleop, protocol (tested), launch file
tools/sim/                   fw_sim: real firmware on your PC against a simulated plant
```
