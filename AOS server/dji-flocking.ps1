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
#
# If PowerShell blocks the script, either run once with:
#   powershell -ExecutionPolicy Bypass -File .\dji-flocking.ps1
# or relax policy for the current user:
#   Set-ExecutionPolicy -Scope CurrentUser RemoteSigned

param(
    [int]$Drones = 3,
    [int]$HttpPort = 8000,
    [switch]$NoGui
)

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
        PowerShell -NoExit -Command "python swarm_flocking.py --drones $Drones --no-gui" `
      `; split-pane -V --size 0.25 --title "controller" `
        -d "$CtrlDir" `
        PowerShell -NoExit -Command "& '$CondaHook' \; conda activate $EnvName \; python readController.py"
}
else {
    wt.exe --size 240,60 `
      new-tab --title "flocking" `
        -d "$AosDir" `
        PowerShell -NoExit -Command "python swarm_flocking.py --drones $Drones" `
      `; split-pane -V --size 0.25 --title "controller" `
        -d "$CtrlDir" `
        PowerShell -NoExit -Command "& '$CondaHook' \; conda activate $EnvName \; python readController.py" `
      `; split-pane -H --size 0.45 --title "swarm-gui" `
        -d "$AosDir" `
        PowerShell -NoExit -Command "python swarm_gui.py --http-port $HttpPort --open"
}
