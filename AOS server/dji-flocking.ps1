# Launch swarm_flocking.py, readController.py, and the browser GUI in Windows
# Terminal. Runs the multi-drone Olfati-Saber flocking controller (vs.
# dji-joystick.ps1 which runs the single-drone direct-stick controller).
#
# Layout: flocking (left) | controller (right top) / swarm-gui (right bottom).
# The flocking controller pushes telemetry to the GUI by default; swarm_gui.py
# serves the map and (with --open) opens it in the browser automatically.
# Image streaming to the stitcher pipeline (BlockSharedMemory) is no longer a
# separate image_stream.py pane — it runs INSIDE swarm_flocking.py when
# ImageStream is enabled (config key / -ImageStream -> --image-stream), so it
# cannot contend with the controller for the ds_wrapper protocol.
#
# SETTINGS COME FROM flocking.config.psd1 (same folder). Edit that file to change
# the swarm's default launch settings. Any CLI flag you pass here OVERRIDES the
# config file for that one run; anything you omit falls back to the config.
# Precedence: baked-in defaults < flocking.config.psd1 < CLI flag.
#
# Usage:
#   .\dji-flocking.ps1                # use flocking.config.psd1 as-is
#   .\dji-flocking.ps1 -Drones 2      # override just the drone count for this run
#   .\dji-flocking.ps1 -HttpPort 9000
#   .\dji-flocking.ps1 -NoGui         # no map; also passes --no-gui to the controller
#   .\dji-flocking.ps1 -Slow 0.3      # slow test mode: scale all velocities to 30%
#   .\dji-flocking.ps1 -GimbalPitch -30   # start gimbal (and slider) tilted to -30deg
#   .\dji-flocking.ps1 -ConvexHull    # start in GLOBAL_CONVEXHULL heading mode (boundary
#                                     # drones face outward); -PointInwards faces the centroid.
#                                     # Both are just seeds — switchable live from the GUI.
#   .\dji-flocking.ps1 -ImageStream   # stream frames to the stitcher (in-process)
#   .\dji-flocking.ps1 -DroneIPs 192.168.100.173,192.168.100.176   # direct-MQTT command
#                                     # path to these RC brokers (order = drone id);
#                                     # normally set via DroneIPs in flocking.config.psd1.
#                                     # Empty DroneIPs (config @()) = AUTO-DISCOVER the RC
#                                     # IPs from the running DroneSwarmServer (id = slot);
#                                     # -DroneIPs server = force the legacy server path.
#   .\dji-flocking.ps1 -NoIdentityCheck   # skip the command<->telemetry identity probe
#   .\dji-flocking.ps1 -MinSeparation 5   # auto-STOP swarming if any pair < 5 m (0 = off)
#   .\dji-flocking.ps1 -Config .\my-other.psd1   # use a different config file
#
# If PowerShell blocks the script, either run once with:
#   powershell -ExecutionPolicy Bypass -File .\dji-flocking.ps1
# or relax policy for the current user:
#   Set-ExecutionPolicy -Scope CurrentUser RemoteSigned

param(
    [int]$Drones,
    [int]$HttpPort,
    [double]$Slow,
    [double]$GimbalPitch,
    [double]$Cvm,
    [double]$R0,
    [double]$Scale,
    [double]$MinSeparation,
    [switch]$ConvexHull,
    [switch]$PointInwards,
    [switch]$NoGui,
    [switch]$ImageStream,
    [switch]$NoIdentityCheck,
    [string[]]$DroneIPs,
    [string]$Config = "$PSScriptRoot\flocking.config.psd1"
)

# --- Resolve settings: baked-in defaults < config file < CLI flags -----------
# The baked-in defaults are the fallback for any key missing from the config
# file, so a stale/partial config can never resolve a setting to $null/0.
$settings = @{
    Drones = 3; HttpPort = 8000; Slow = 1.0; GimbalPitch = -10.0
    Heading = 'manual'; PointInwards = $false; NoGui = $false
    ImageStream = $false
    Cvm = 0.0; R0 = 150.0; Scale = 10.0
    DroneIPs = @()
    IdentityCheck = $true; MinSeparation = 3.0
}

if (-not (Test-Path $Config)) {
    throw "Config file not found: $Config (expected flocking.config.psd1 next to this launcher)"
}
$cfg = Import-PowerShellDataFile -Path $Config
foreach ($k in $cfg.Keys) { $settings[$k] = $cfg[$k] }

# A CLI flag that was actually passed wins over the config. $PSBoundParameters
# only contains parameters the caller supplied, so untouched flags fall through
# to the config value above.
if ($PSBoundParameters.ContainsKey('Drones'))       { $settings.Drones = $Drones }
if ($PSBoundParameters.ContainsKey('HttpPort'))     { $settings.HttpPort = $HttpPort }
if ($PSBoundParameters.ContainsKey('Slow'))         { $settings.Slow = $Slow }
if ($PSBoundParameters.ContainsKey('GimbalPitch'))  { $settings.GimbalPitch = $GimbalPitch }
if ($PSBoundParameters.ContainsKey('Cvm'))          { $settings.Cvm = $Cvm }
if ($PSBoundParameters.ContainsKey('R0'))           { $settings.R0 = $R0 }
if ($PSBoundParameters.ContainsKey('Scale'))        { $settings.Scale = $Scale }
if ($PSBoundParameters.ContainsKey('NoGui'))        { $settings.NoGui = [bool]$NoGui }
if ($PSBoundParameters.ContainsKey('ImageStream'))  { $settings.ImageStream = [bool]$ImageStream }
if ($PSBoundParameters.ContainsKey('PointInwards')) { $settings.PointInwards = [bool]$PointInwards }
if ($PSBoundParameters.ContainsKey('DroneIPs'))     { $settings.DroneIPs = $DroneIPs }
if ($PSBoundParameters.ContainsKey('MinSeparation')){ $settings.MinSeparation = $MinSeparation }
# -ConvexHull is a convenience alias that forces convexhull heading mode.
if ($ConvexHull)                                    { $settings.Heading = 'convexhull' }
# -NoIdentityCheck disables the command<->telemetry identity probe for one run.
if ($NoIdentityCheck)                               { $settings.IdentityCheck = $false }

$Drones       = [int]$settings.Drones
$HttpPort     = [int]$settings.HttpPort
$Slow         = [double]$settings.Slow
$GimbalPitch  = [double]$settings.GimbalPitch
$Cvm          = [double]$settings.Cvm
$R0           = [double]$settings.R0
$Scale        = [double]$settings.Scale
$NoGui        = [bool]$settings.NoGui
$ImageStream  = [bool]$settings.ImageStream
$PointInwards = [bool]$settings.PointInwards
$Heading      = ("$($settings.Heading)").ToLower()
$DroneIPs     = @($settings.DroneIPs | Where-Object { "$_".Trim() -ne '' })
$IdentityCheck = [bool]$settings.IdentityCheck
$MinSeparation = [double]$settings.MinSeparation

# Command-path mode from the DroneIPs value:
#   @()                     -> 'auto'    (swarm_flocking.py discovers the RC IPs from
#                                         DroneSwarmServer's RTSP connections; id = slot)
#   @('server')             -> 'server'  (legacy path via DroneSwarmServer, ~4.5 Hz)
#   @('<ip>', '<ip>', ...)  -> 'explicit' (port-pinned drone ids, verified by the probe)
$CmdMode = 'explicit'
if ($DroneIPs.Count -eq 0) { $CmdMode = 'auto' }
elseif ($DroneIPs.Count -eq 1 -and @('server','legacy') -contains "$($DroneIPs[0])".ToLower()) { $CmdMode = 'server' }

# --- Validate resolved settings ----------------------------------------------
if ($Drones -lt 1) { throw "Drones must be >= 1 (got $Drones)" }
if ($Slow -le 0) { throw "Slow must be > 0 (e.g. 0.3 for 30% speed) (got $Slow)" }
if ($GimbalPitch -lt -90.0 -or $GimbalPitch -gt 60.0) {
    throw "GimbalPitch must be in [-90, 60] (DJI Mini 3 Pro tilt range) (got $GimbalPitch)"
}
if ($Heading -ne 'manual' -and $Heading -ne 'convexhull') {
    throw "Heading must be 'manual' or 'convexhull' (got '$Heading')"
}
if ($CmdMode -eq 'explicit' -and $DroneIPs.Count -lt $Drones) {
    throw ("DroneIPs lists only $($DroneIPs.Count) address(es) but Drones is $Drones " +
           "(order = drone id; extras beyond Drones are fine)")
}
if ($MinSeparation -lt 0) { throw "MinSeparation must be >= 0 (0 disables) (got $MinSeparation)" }

$CmdPathDesc = switch ($CmdMode) {
    'auto'     { "auto-discover RC IPs from DroneSwarmServer (drone id = server slot)" }
    'server'   { "via DroneSwarmServer (legacy, ~4.5 Hz, forced)" }
    'explicit' { "direct MQTT [$($DroneIPs -join ', ')]" }
}
Write-Host ("[dji-flocking] config $Config -> drones=$Drones slow=$Slow gimbal=$GimbalPitch " +
            "heading=$Heading pointInwards=$PointInwards noGui=$NoGui imageStream=$ImageStream " +
            "c_vm=$Cvm r0=$R0 scale=$Scale minSep=$MinSeparation identityCheck=$IdentityCheck " +
            "httpPort=$HttpPort cmdPath=$CmdPathDesc")

# --- Build the swarm_flocking.py CLI -----------------------------------------
# Format doubles invariantly so the decimal point survives locales that use a
# comma separator, and only forward a flag when it differs from the script's own
# default so the command line stays clean.
function Inv([double]$v) { $v.ToString([System.Globalization.CultureInfo]::InvariantCulture) }

$SlowArg   = if ($Slow -ne 1.0)         { " --slow " + (Inv $Slow) }               else { "" }
$GimbalArg = if ($GimbalPitch -ne -10.0){ " --gimbal-pitch " + (Inv $GimbalPitch) } else { "" }
$CvmArg    = if ($Cvm -ne 0.0)          { " --c-vm " + (Inv $Cvm) }                else { "" }
$R0Arg     = if ($R0 -ne 150.0)         { " --r0 " + (Inv $R0) }                   else { "" }
$ScaleArg  = if ($Scale -ne 10.0)       { " --scale " + (Inv $Scale) }             else { "" }

# GLOBAL_CONVEXHULL heading control (heading_convexhull.py): convexhull seeds
# the mode (boundary drones face outward; -PointInwards flips to the centroid),
# default is manual stick yaw. Both settings stay switchable from the GUI.
$HeadingArg = ""
if ($Heading -eq 'convexhull') { $HeadingArg += " --heading convexhull" }
if ($PointInwards)             { $HeadingArg += " --point-inwards" }

# In-process image streaming to the stitcher pipeline (replaces the old
# standalone image_stream.py pane, which starved the controller's ds_wrapper
# access and collapsed the cmd/telem rates).
$ImageStreamArg = if ($ImageStream) { " --image-stream" } else { "" }

# Command path: explicit IP list or forced legacy 'server' get forwarded;
# auto mode passes nothing (it is swarm_flocking.py's default — the script
# discovers the RC IPs from DroneSwarmServer's RTSP connections itself).
$DroneIPsArg = switch ($CmdMode) {
    'auto'     { "" }
    'server'   { " --drone-ips server" }
    'explicit' { " --drone-ips " + ($DroneIPs -join ',') }
}

# Identity probe (on by default in the script; only forward the opt-out) and
# the min-separation failsafe (script default 3.0 m; forward when different).
$IdentityArg = if (-not $IdentityCheck)   { " --no-identity-check" }                     else { "" }
$MinSepArg   = if ($MinSeparation -ne 3.0){ " --min-separation " + (Inv $MinSeparation) } else { "" }

$FlockArgs = "$SlowArg$GimbalArg$HeadingArg$CvmArg$R0Arg$ScaleArg$ImageStreamArg$DroneIPsArg$IdentityArg$MinSepArg"

# The readController pane sources the conda hook and activates this env
# before launching the script. Edit if your miniconda lives elsewhere.
$CondaHook = "C:\Users\jarvis\AppData\Local\miniconda3\shell\condabin\conda-hook.ps1"
$EnvName   = "stitching"
$AosDir    = "C:\Users\jarvis\Documents\DJI_Swarm\AOS server"
$CtrlDir   = "C:\Users\jarvis\Documents\vr_swarm_simulation\Assets\Scripts\Control"

if ($NoGui) {
    wt.exe --size 240,60 `
      new-tab --title "flocking" `
        -d "$AosDir" `
        PowerShell -NoExit -Command "python swarm_flocking.py --drones $Drones$FlockArgs --no-gui" `
      `; split-pane -V --size 0.25 --title "controller" `
        -d "$CtrlDir" `
        PowerShell -NoExit -Command "& '$CondaHook' \; conda activate $EnvName \; python readController.py"
}
else {
    wt.exe --size 240,60 `
      new-tab --title "flocking" `
        -d "$AosDir" `
        PowerShell -NoExit -Command "python swarm_flocking.py --drones $Drones$FlockArgs" `
      `; split-pane -V --size 0.25 --title "controller" `
        -d "$CtrlDir" `
        PowerShell -NoExit -Command "& '$CondaHook' \; conda activate $EnvName \; python readController.py" `
      `; split-pane -H --size 0.45 --title "swarm-gui" `
        -d "$AosDir" `
        PowerShell -NoExit -Command "python swarm_gui.py --http-port $HttpPort --open"
}
