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
    DroneIPs = @()

    # Browser GUI (swarm_gui.py) HTTP port. CLI: -HttpPort
    HttpPort = 8000

    # Slow test mode: uniformly scale ALL commanded velocities + the yaw/climb
    # rates for slow, controlled tuning. 1.0 = full speed; e.g. 0.3 = 30%.
    # CLI: -Slow
    Slow = 1.0

    # Initial gimbal pitch/tilt (deg) for every drone and the GUI slider's start
    # position. DJI Mini 3 Pro range [-90 (down), 60 (up)]. CLI: -GimbalPitch
    GimbalPitch = -2.0

    # Heading-control mode seed (live-switchable from the GUI afterwards):
    #   'manual'     - stick angular.z steers one shared target heading
    #   'convexhull' - GLOBAL_CONVEXHULL: hull drones face outward, interior
    #                  drones hold heading, stick yaw ignored
    #   'demostitch' - the laterally-middle drone points at the stick-steered
    #                  heading; each neighbour fans out by StitchOffset deg per
    #                  rank, keeping adjacent camera views overlapped for
    #                  image stitching
    # CLI: -ConvexHull / -DemoStitch force the respective mode for that run.
    Heading = 'manual'

    # convexhull mode only: boundary drones face the swarm centroid instead of
    # outward. Ignored in manual mode. CLI: -PointInwards
    PointInwards = $false

    # demostitch mode only: per-rank heading offset between laterally adjacent
    # drones (deg), live-adjustable in the GUI afterwards. Range [5, 90].
    # CLI: -StitchOffset
    StitchOffset = 30.0

    # Verify at startup and on every swarming Start that the DroneIPs order
    # matches DroneSwarmServer's slot order (an inert MQTT marker probe;
    # mismatch = cross-wired control loops, auto-corrected when resolvable).
    # $false skips it and trusts the order. CLI: -NoIdentityCheck disables for
    # one run.
    IdentityCheck = $true

    # Minimum-separation failsafe (physical metres): swarming auto-STOPs if
    # any drone pair gets closer than this, measured in 3D (horizontal + the
    # altitude difference). 0 disables. CLI: -MinSeparation
    MinSeparation = 2.0

    # --- Vertical-plane ("wall") swarming --------------------------------- ---
    # Port of the Unity sim's SwarmPlaneController: the swarm re-forms as a
    # VERTICAL wall facing the stick-steered heading, so the operator looks at a
    # billboard of drones instead of standing inside a ring of them. The
    # cohesion law is unchanged — only the plane it is constrained to swaps —
    # but the vertical part of the in-plane force becomes a PER-DRONE altitude
    # setpoint (DJI VS gives absolute-altitude control and no vertical velocity
    # channel). Toggleable live from the GUI; this key only seeds the toggle.
    #
    # SAFETY, read before flying it:
    #   - A wall stacks drones vertically, so the upper one's rotor downwash
    #     lands on the lower one. Nothing in the sim models this. The GUI shows
    #     a DOWNWASH advisory; the 3D MinSeparation above is the hard failsafe.
    #   - Altitude is takeoff-relative PER AIRCRAFT, so a wall built in altitude
    #     space is skewed by launch-pad height differences (and 3D separation is
    #     wrong by the same amount). Launch from one flat pad. Entry is refused
    #     if the reported altitudes disagree by more than 3 m.
    #   - Heading is forced to 'manual' while a wall is up (convexhull's hull
    #     collapses to a line and demostitch's lateral ranking degenerates).
    # CLI: -PlaneMode
    PlaneMode = $false

    # Restoring pull onto the plane: m/s of horizontal command per metre a drone
    # sits off the wall. Range [0.02, 1.0], live-tunable in the GUI. The term is
    # clamped internally so obstacle/geofence repulsion always outranks it.
    # Higher = a crisper wall but more sensitivity to the ~5 Hz GPS.
    # CLI: -PlaneGain
    PlaneGain = 0.25

    # How far one drone's altitude setpoint may sit from the wall's reference
    # altitude (metres). Bounds the wall's vertical extent and stops a runaway
    # climb or descent. CLI: -PlaneLeash
    PlaneLeash = 12.0

    # Ceiling for every commanded altitude (metres), horizontal or wall. Raise
    # it for a tall wall: N drones at d_ref spacing need roughly (N-1)*d_ref of
    # vertical room, centred well above the 1 m floor (5 drones at 8 m ~= 32 m,
    # so ~20 m of centre altitude). CHECK THE SITE'S LEGAL CEILING FIRST.
    # CLI: -MaxAlt
    MaxAlt = 100.0

    # --- AirLink / RF management (sent once per drone at controller startup ---
    # --- as an "AIRLINK:" MQTT one-shot; the RC shows applied/rejected on   ---
    # --- its status line — bench-test before relying on it in the field)    ---
    #
    # IMPORTANT: @() / '' means "send nothing", NOT "restore DJI defaults".
    # Band and bandwidth persist in the RC/aircraft firmware, so whatever a
    # previous run applied stays in force until it is explicitly overwritten.
    # To undo an earlier experiment you must set the value back, not blank it.
    #
    # There is deliberately no manual-channel setting: DJI does not support
    # manual image-transmission channel selection on the Mini 3 Pro, so the
    # old AirlinkChannels key could only ever be rejected by the aircraft.

    # Per-drone RF band in drone-id order: '2G4' | '5G8' | 'DUAL' | '-' (leave
    # unchanged). ONE value applies to every drone; @() sends nothing. With
    # ~10 co-located OcuSync links, splitting the fleet across the two bands
    # halves the contenders per band, e.g.
    # @('2G4','2G4','2G4','2G4','2G4','5G8','5G8','5G8','5G8','5G8').
    # 'DUAL' is the DJI default (firmware picks per packet) — use it to undo an
    # earlier band pin. CLI: -AirlinkBands 2G4,2G4,5G8
    AirlinkBands = @()

    # Per-drone AirLink channel bandwidth in MHz, drone-id order:
    # '40' | '20' | '10' | '5' | '-' (leave unchanged); one value = all drones.
    # This is the highest-value knob for a crowded site: it narrows how much
    # spectrum each link actually OCCUPIES, whereas VideoMode below only lowers
    # the bitrate carried inside whatever channel width is in use. It also
    # works with DJI's AUTO channel selection, so it is not blocked by the
    # Mini 3 Pro's lack of manual channel support. Narrower = more robust link,
    # lower video data rate. @() sends nothing. CLI: -AirlinkBandwidth 10
    AirlinkBandwidth = @()

    # Camera stream cap applied on every RC, e.g. '1920x1080@24' — a lower
    # encoded bitrate leaves more airlink headroom per link (and less RTSP
    # load on the PC). '' leaves the camera as-is. CLI: -VideoMode
    VideoMode = ''

    # Read-only pre-flight link scan: ask every RC for its current radio config
    # (band / channel mode / bandwidth / frequency point) plus the aircraft's
    # own per-frequency interference measurement, and print + log the answers
    # before anything flies. Changes no setting; costs a few seconds at
    # startup. Needs the direct MQTT command path (DroneIPs @() or an IP list).
    # CLI: -NoLinkScan disables for one run.
    LinkScan = $true

    # BENCH EXPERIMENT — leave $false for normal flying.
    # Set ChannelSelectionMode MANUAL before applying band/bandwidth. Bench-
    # tested 2026-08-07: the Mini 3 Pro ACCEPTS a 10 MHz bandwidth in AUTO
    # mode, reports it back, then reverts to 40 MHz within 3 s — which matches
    # DJI documenting bandwidth as manual-mode-only. This tests whether the
    # airframe will enter MANUAL at all. If the value still does not hold, AUTO
    # is restored automatically so a failed experiment cannot leave the fleet
    # off DJI's own channel selection. DJI recommends AUTO; do not fly a fleet
    # on MANUAL without knowing why. Only sent alongside a band/bandwidth
    # request — never on its own. CLI: -AirlinkManualChannel
    AirlinkManualChannel = $false

    # Skip the browser-GUI map pane and pass --no-gui to the controller so it
    # does not push telemetry. CLI: -NoGui
    NoGui = $false

    # Publish 800x450 frames to the DroneFeedSharedMemory feed pipeline from
    # INSIDE swarm_flocking.py (--image-stream). Runs off the telemetry
    # threads' existing image fetches, so it cannot slow the cmd/telem rates
    # (unlike the old standalone image_stream.py process). CLI: -ImageStream
    ImageStream = $true

    # Include a per-frame camera pose (GPS + gimbal attitude -> Unity world) in
    # each published block (--image-stream-pose; implies ImageStream). The sim's
    # PLANAR stitcher computes its homographies from pose and cannot run without
    # this; STABSTITCH ignores it. Costs one small conversion per frame and no
    # extra ds_wrapper calls. CLI: -ImageStreamPose
    ImageStreamPose = $false

    # --- Clip recording (GUI "Record clip" button) --------------------------
    # Root folder for recorded CLIPS: a short, operator-triggered section of a
    # flight saved WITH pictures — one 1080p MP4 per drone, a per-frame index
    # CSV (time / lat / lon / alt / heading / gimbal), and a copy of the four
    # flight-data CSVs restricted to the recording window. Each clip gets its
    # own clip_YYYYMMDD_HHMMSS folder here.
    #
    # SEPARATE from the always-on flight log (flight_logs/), which is unchanged
    # and keeps running throughout. Gitignored. Created on the first Record
    # press, so a run where the button is never pressed writes nothing.
    # Relative paths resolve against "AOS server/"; an absolute path (e.g.
    # 'D:\Flight clips') works and may quote spaces. Keep it on a LOCAL disk —
    # this writes video for every drone at once. CLI: -RecordingDir
    RecordingDir = 'recordings'

    # Hard cap on ONE clip in seconds [1, 900]; the controller auto-stops and
    # finalises the MP4s there, so a forgotten recording cannot fill the disk.
    # Budget roughly 2-3 MB/s per drone at 1080p (measure on your fleet and
    # correct this note). CLI: -RecordMaxSeconds
    RecordMaxSeconds = 120.0

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
