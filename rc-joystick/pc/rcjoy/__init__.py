"""rcjoy: the PC side of the LIS RC Joystick, a DJI RC Pro used as the swarm joystick.

Stdlib only, Python >= 3.7. Imports nothing from AOS server/ or lis-swarm-app/.
Run from rc-joystick/pc:

    python -m rcjoy monitor  [--rc IP]      live readout of every input + link health
    python -m rcjoy bridge   [--rc IP] ...  re-emit readController.py's JSON on UDP :5055
    python -m rcjoy fake-rc  [...]          the RC side of the protocol, synthetic inputs
    python -m rcjoy selftest                loopback checks, no hardware

Wire protocol: rc-joystick/PROTOCOL.md
"""

__version__ = "1.0"
