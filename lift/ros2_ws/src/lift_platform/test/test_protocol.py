from lift_platform.protocol import clean_command, parse_telemetry, status_text


def test_whitelist_accepts():
    for c in ['up', 'DOWN', 'stop', 'GOTO 50', 'goto 12.5', 'PID 0.08 0.01 0', 'JOG R -1', 'CALMIN']:
        assert clean_command(c) is not None, c


def test_whitelist_rejects():
    for c in ['', 'rm -rf /', 'GOTO', 'GOTO abc', 'UP\nDOWN', 'JOG X 1', 'JOG L 5', 'PID 1 2']:
        assert clean_command(c) is None, c


def test_telemetry():
    t = parse_telemetry('T,1234,10.50,11.00,10.75,50.00,0.40,0.35,2,0,1')
    assert t['state'] == 'GOTO' and t['calibrated'] and abs(t['height'] - 10.75) < 1e-9
    assert 'GOTO' in status_text(t)


def test_telemetry_bad():
    assert parse_telemetry('garbage') is None
    assert parse_telemetry('T,1,2,3') is None
    assert parse_telemetry('T,1,a,b,c,d,e,f,0,0,0') is None
    assert parse_telemetry('T,1,0,0,0,0,0,0,9,0,0') is None
