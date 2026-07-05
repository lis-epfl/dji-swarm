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
    Drones = 3

    # RC/broker IP addresses in DRONE-ID ORDER (index 1 = drone 1, ...). When
    # listed, swarm_flocking.py publishes commands DIRECTLY to each RC's MQTT
    # broker over a persistent connection (20 Hz capable). The IPs are fixed
    # per switch port; whichever RC is plugged into a port gets that port's IP
    # and therefore that drone ID — stable numbering across sessions, verified
    # against the server's slot order by the identity probe (see IdentityCheck).
    # List at least as many IPs as Drones (only the first N are used, so keep
    # all ports listed and just lower Drones).
    #
    # Empty array @() = AUTO-DISCOVER: the controller reads the connected RC
    # IPs off the running DroneSwarmServer (its per-slot RTSP connections) and
    # binds them to slots with the marker probe. Zero maintenance and immune
    # to ordering mistakes by construction, but drone numbering then follows
    # the server's slot order for that session instead of the switch ports.
    # @('server') = force the legacy command path via DroneSwarmServer (~4.5 Hz).
    # CLI: -DroneIPs 192.168.100.173,192.168.100.176  /  -DroneIPs server
    # DroneIPs = @('192.168.100.150', '192.168.100.173', '192.168.100.176', '192.168.100.247', '192.168.100.211')
    DroneIPs = @()

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

    # Verify at startup and on every swarming Start that the DroneIPs order
    # matches DroneSwarmServer's slot order (an inert MQTT marker probe;
    # mismatch = cross-wired control loops, auto-corrected when resolvable).
    # $false skips it and trusts the order. CLI: -NoIdentityCheck disables for
    # one run.
    IdentityCheck = $true

    # Minimum-separation failsafe (physical metres): swarming auto-STOPs if
    # any drone pair gets closer than this. 0 disables. CLI: -MinSeparation
    MinSeparation = 3.0

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

    # --- Virtual obstacles / geofence (shapes are drawn on the GUI map and ---
    # --- persist in shapes.json; these tune the beta-agent repulsion)      ---
    # Repulsion cutoff in PHYSICAL metres: the push is maximal at contact and
    # exactly 0 beyond this distance from an obstacle edge (or inside the
    # geofence, from its boundary). CLI: -DObs
    DObs = 5.0
    # Detection radius in PHYSICAL metres (>= DObs); beyond it an obstacle is
    # ignored entirely. CLI: -R0Obs
    R0Obs = 6.0
    # Repulsion gain (max push ~= CObs * 1.45 m/s at contact; 4.3 matches the
    # Unity sim). CLI: -CObs
    CObs = 4.3
}
