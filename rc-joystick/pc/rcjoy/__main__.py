"""python -m rcjoy {monitor,bridge,fake-rc,selftest} ... (see rcjoy/__init__.py)."""

import argparse
import sys

from . import protocol as P


def _args(argv):
    ap = argparse.ArgumentParser(
        prog="python -m rcjoy",
        description="PC side of the LIS RC Joystick (DJI RC Pro as the swarm joystick).")
    sub = ap.add_subparsers(dest="cmd")

    def rc_opts(p):
        p.add_argument("--rc", default=None, metavar="IP",
                       help="the joystick RC's IP. Default: find it, trying the last RC "
                            "this PC used and a broadcast. Every RC locked onto is "
                            "remembered, so one --rc is enough for later launches")
        p.add_argument("--port", type=int, default=P.DEFAULT_PORT,
                       help="the RC app's UDP port (default %d)" % P.DEFAULT_PORT)

    p = sub.add_parser("monitor", help="live readout of every input + link health")
    rc_opts(p)
    p.add_argument("--rate", type=int, default=P.RATE_DEFAULT_HZ,
                   help="requested state rate, Hz (default %d). To measure the "
                        "gamepad's ~70 reports/s, ask for %d: the RC samples its input "
                        "at this rate, so a slower one overwrites reports"
                        % (P.RATE_DEFAULT_HZ, P.RATE_MAX_HZ))
    p.add_argument("--csv", metavar="PATH",
                   help="also write one row per received state to PATH (the record "
                        "of a bench run: receive time, seq, fresh, raw inputs, and the "
                        "app's own gamepad rate)")

    p = sub.add_parser("bridge", help="re-emit readController.py's JSON (default to :5055)")
    rc_opts(p)
    p.add_argument("--out", action="append", metavar="HOST:PORT",
                   help="where to send the legacy JSON; repeatable "
                        "(default 127.0.0.1:5055). One consumer per port: Unity's "
                        "receiver and JoystickReceiver cannot share one.")
    p.add_argument("--rate", type=float, default=20.0,
                   help="output rate, Hz, at most 20 (default 20)")
    p.add_argument("--stick-mode", type=int, choices=(1, 2, 3), default=2,
                   help="DJI stick mode (default 2: throttle/yaw on the left)")
    p.add_argument("--profile", choices=("flocking", "joystick", "sim"), default="flocking",
                   help="flocking: right dial = spacing (angular.x, 0.6-1.4), left dial "
                        "= s2. joystick: left (gimbal) dial = angular.x, which "
                        "joystick_controller.py reads as the gimbal. sim: flocking's "
                        "dials with spacing 0.4-1.6, readController.py's range, for the "
                        "Unity sim (it uses angular.x unclamped as its spread)")
    p.add_argument("--knob-sweep-s", type=float, default=3.0,
                   help="seconds of full dial deflection to sweep a knob's whole "
                        "range (default 3)")
    p.add_argument("--ax-range", default=None, metavar="LO,HI",
                   help="the angular.x knob's range, overriding the profile's "
                        "(flocking/joystick 0.6,1.4, where swarm_flocking.py clamps it; "
                        "sim 0.4,1.6)")

    p = sub.add_parser("fake-rc", help="the RC side of the protocol, synthetic inputs")
    p.add_argument("--bind", default="0.0.0.0", help="address to listen on")
    p.add_argument("--port", type=int, default=P.DEFAULT_PORT)
    p.add_argument("--pattern", choices=("steps", "sweep", "still"), default="steps",
                   help="steps: one labelled control at a time (default). sweep: "
                        "everything moving, the right stick circling once a second")
    p.add_argument("--input-hz", type=float, default=70.0,
                   help="sweep: how many times a second the inputs step, like the "
                        "gamepad's reports (default 70)")
    p.add_argument("--aircraft-linked", action="store_true",
                   help="report an aircraft link (the PC must block)")
    p.add_argument("--no-rc-ok", action="store_true", help="report rc_ok=false")
    p.add_argument("--null", default="", metavar="KEYS",
                   help="comma-separated fields to report as not served, e.g. lv,r")
    p.add_argument("--msdk-sticks", action="store_true",
                   help="report the sticks as coming from MSDK (~10 Hz), as the app does "
                        "when its gamepad path is not usable (the PC must warn)")

    sub.add_parser("selftest", help="loopback checks of protocol, client and bridge")
    a = ap.parse_args(argv)
    if not a.cmd:
        ap.print_help()
        sys.exit(2)
    return a


def main(argv=None):
    a = _args(sys.argv[1:] if argv is None else argv)
    if a.cmd == "monitor":
        from .monitor import main as run
    elif a.cmd == "bridge":
        from .bridge import main as run
    elif a.cmd == "fake-rc":
        from .fake_rc import main as run
    else:
        from .selftest import main as run
    return run(a)


if __name__ == "__main__":
    sys.exit(main())
