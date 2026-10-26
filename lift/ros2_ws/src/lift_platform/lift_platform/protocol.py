"""Pure-Python helpers for the ESP32 text protocol (no ROS imports, easy to test)."""
import re

STATES = ['IDLE', 'MANUAL', 'GOTO', 'JOG', 'FAULT']
FAULTS = ['none', 'estop', 'pot_wire', 'sync_mismatch', 'jam']

# Only these lines are ever forwarded to the ESP32. Anything else is dropped.
_NUM = r'-?\d+(\.\d+)?'
_ALLOWED = [
    r'STOP', r'RESET', r'UP', r'DOWN', r'GET', r'CALMIN', r'CALMAX',
    rf'GOTO {_NUM}',
    rf'PID {_NUM} {_NUM} {_NUM}',
    rf'KS {_NUM}',
    rf'VMAX {_NUM}',
    r'JOG [LR] -?1',
]
_ALLOWED_RE = re.compile('^(?:' + '|'.join(_ALLOWED) + ')$')


def clean_command(text):
    """Return a normalised command string, or None if it is not allowed."""
    t = ' '.join(str(text).strip().upper().split())
    return t if _ALLOWED_RE.match(t) else None


def parse_telemetry(line):
    """'T,ms,xL,xR,H,target,uL,uR,state,fault,cal' -> dict, or None."""
    p = line.strip().split(',')
    if len(p) != 11 or p[0] != 'T':
        return None
    try:
        st, fa = int(p[8]), int(p[9])
        return {
            'ms': int(p[1]), 'x_left': float(p[2]), 'x_right': float(p[3]),
            'height': float(p[4]), 'target': float(p[5]),
            'u_left': float(p[6]), 'u_right': float(p[7]),
            'state': STATES[st], 'fault': FAULTS[fa], 'calibrated': p[10] == '1',
        }
    except (ValueError, IndexError):
        return None


def status_text(t):
    s = t['state']
    if t['fault'] != 'none':
        s += f" ({t['fault']})"
    cal = 'cal' if t['calibrated'] else 'UNCALIBRATED'
    return f"{s} | H={t['height']:.1f} mm tgt={t['target']:.1f} | L={t['x_left']:.1f} R={t['x_right']:.1f} | {cal}"
