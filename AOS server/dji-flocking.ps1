# Launch swarm_flocking.py, readController.py, and the browser GUI in Windows
# Terminal. Runs the multi-drone Olfati-Saber flocking controller (vs.
# dji-joystick.ps1 which runs the single-drone direct-stick controller).
#
# Layout: flocking (left) | controller (right top) / swarm-gui (right bottom).
# The flocking controller pushes telemetry to the GUI by default; swarm_gui.py
# serves the map and (with --open) opens it in the browser automatically.
#
# Usage:
#   .\dji-flocking.ps1                # default 3 drones, GUI on http://127.0.0.1:8000
#   .\dji-flocking.ps1 -Drones 2
#   .\dji-flocking.ps1 -HttpPort 9000
#   .\dji-flocking.ps1 -NoGui         # no map; also passes --no-gui to the controller
#   .\dji-flocking.ps1 -Slow 0.3      # slow test mode: scale all velocities to 30%
#   .\dji-flocking.ps1 -GimbalPitch -30   # start gimbal (and slider) tilted to -30deg
#   .\dji-flocking.ps1 -ConvexHull    # start in GLOBAL_CONVEXHULL heading mode (boundary
#                                     # drones face outward); -PointInwards faces the centroid.
#                                     # Both are just seeds — switchable live from the GUI.
#
# If PowerShell blocks the script, either run once with:
#   powershell -ExecutionPolicy Bypass -File .\dji-flocking.ps1
# or relax policy for the current user:
#   Set-ExecutionPolicy -Scope CurrentUser RemoteSigned

param(
    [int]$Drones = 3,
    [int]$HttpPort = 8000,
    [double]$Slow = 1.0,
    [double]$GimbalPitch = -10.0,
    [switch]$ConvexHull,
    [switch]$PointInwards,
    [switch]$NoGui
)

if ($Slow -le 0) { throw "-Slow must be > 0 (e.g. -Slow 0.3 for 30% speed)" }
if ($GimbalPitch -lt -90.0 -or $GimbalPitch -gt 60.0) {
    throw "-GimbalPitch must be in [-90, 60] (DJI Mini 3 Pro tilt range)"
}

# Forward --slow to swarm_flocking.py only when slowing down. Format invariantly
# so the decimal point survives locales that use a comma separator.
$SlowArg = ""
if ($Slow -ne 1.0) {
    $SlowArg = " --slow " + $Slow.ToString([System.Globalization.CultureInfo]::InvariantCulture)
}

# Forward --gimbal-pitch only when it differs from the script default (-10), so
# it seeds both the drones' initial gimbal and the GUI slider position.
$GimbalArg = ""
if ($GimbalPitch -ne -10.0) {
    $GimbalArg = " --gimbal-pitch " + $GimbalPitch.ToString([System.Globalization.CultureInfo]::InvariantCulture)
}

# GLOBAL_CONVEXHULL heading control (heading_convexhull.py): -ConvexHull seeds
# the mode (boundary drones face outward; -PointInwards flips to the centroid),
# default is manual stick yaw. Both settings stay switchable from the GUI.
$HeadingArg = ""
if ($ConvexHull) { $HeadingArg += " --heading convexhull" }
if ($PointInwards) { $HeadingArg += " --point-inwards" }

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
        PowerShell -NoExit -Command "python swarm_flocking.py --drones $Drones$SlowArg$GimbalArg$HeadingArg --no-gui" `
      `; split-pane -V --size 0.25 --title "controller" `
        -d "$CtrlDir" `
        PowerShell -NoExit -Command "& '$CondaHook' \; conda activate $EnvName \; python readController.py"
}
else {
    wt.exe --size 240,60 `
      new-tab --title "flocking" `
        -d "$AosDir" `
        PowerShell -NoExit -Command "python swarm_flocking.py --drones $Drones$SlowArg$GimbalArg$HeadingArg" `
      `; split-pane -V --size 0.25 --title "controller" `
        -d "$CtrlDir" `
        PowerShell -NoExit -Command "& '$CondaHook' \; conda activate $EnvName \; python readController.py" `
      `; split-pane -H --size 0.45 --title "swarm-gui" `
        -d "$AosDir" `
        PowerShell -NoExit -Command "python swarm_gui.py --http-port $HttpPort --open"
}
