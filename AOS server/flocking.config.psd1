# flocking.config.psd1 — default settings for .\dji-flocking.ps1
#
# This is a PowerShell data file (a restricted, code-free hashtable literal that
# dji-flocking.ps1 reads with Import-PowerShellDataFile). Edit the values here to
# change how the flocking swarm launches without touching the launcher.
#
# Precedence: baked-in launcher defaults  <  this file  <  a CLI flag.
# So any flag you pass explicitly (e.g. .\dji-flocking.ps1 -Drones 2) OVERRIDES
# the matching value below for that one run; everything you don't pass falls
# back to this file. Comment out or delete a key to fall back to the launcher's
# baked-in default for it.

@{
    # Number of drones in the swarm (creates IDs 1..N). CLI: -Drones
    Drones = 4

    # RC/broker IP addresses in DRONE-ID ORDER (index 1 = drone 1, ...). When
    # set, swarm_flocking.py publishes commands DIRECTLY to each RC's MQTT
    # broker over a persistent connection (20 Hz capable). The IPs are fixed
    # per switch port; whichever RC is plugged into a port gets that port's IP
    # and therefore that drone ID. List at least as many IPs as Drones (only
    # the first N are used, so keep all ports listed and just lower Drones).
    # Empty array @() = legacy path via DroneSwarmServer (~4.5 Hz commands).
    # CLI: -DroneIPs 192.168.100.173,192.168.100.176
    DroneIPs = @('192.168.100.173', '192.168.100.176', '192.168.100.247', '192.168.100.211')
    # '192.168.100.150'

    # Browser GUI (swarm_gui.py) HTTP port. CLI: -HttpPort
    HttpPort = 8000

    # Slow test mode: uniformly scale ALL commanded velocities + the yaw/climb
    # rates for slow, controlled tuning. 1.0 = full speed; e.g. 0.3 = 30%.
    # CLI: -Slow
    Slow = 0.2

    # Initial gimbal pitch/tilt (deg) for every drone and the GUI slider's start
    # position. DJI Mini 3 Pro range [-90 (down), 60 (up)]. CLI: -GimbalPitch
    GimbalPitch = -2.0

    # Heading-control mode seed (live-switchable from the GUI afterwards):
    #   'manual'     - stick angular.z steers one shared target heading
    #   'convexhull' - GLOBAL_CONVEXHULL: hull drones face outward, interior
    #                  drones hold heading, stick yaw ignored
    # CLI: -ConvexHull forces 'convexhull' for that run.
    Heading = 'manual'

    # convexhull mode only: boundary drones face the swarm centroid instead of
    # outward. Ignored in manual mode. CLI: -PointInwards
    PointInwards = $false

    # Skip the browser-GUI map pane and pass --no-gui to the controller so it
    # does not push telemetry. CLI: -NoGui
    NoGui = $false

    # Publish 800x450 frames to the DroneFeedSharedMemory feed pipeline from
    # INSIDE swarm_flocking.py (--image-stream). Runs off the telemetry
    # threads' existing image fetches, so it cannot slow the cmd/telem rates
    # (unlike the old standalone image_stream.py process). CLI: -ImageStream
    ImageStream = $false

    # --- Olfati-Saber flocking tuning (seed values; the GUI can retune live) ---
    # Velocity-matching gain, swarm_flocking.py --c-vm (default 0.0).
    Cvm = 0.0
    # Cohesion neighbour radius r0_coh, --r0 (default 150.0).
    R0 = 150.0
    # Distance scale factor, --scale (default 10.0, matches the Unity sim).
    Scale = 10.0
}
