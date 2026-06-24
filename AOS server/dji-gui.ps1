# Launch the LIS_Swarm browser GUI server (swarm_gui.py) and open it.
#
# The GUI only DISPLAYS telemetry pushed by a running flight controller
# (joystick_controller.py / swarm_flocking.py). Start one of those too — they
# push to the GUI by default. The GUI never touches ds_wrapper, so this can run
# in its own (non-elevated) terminal on any Python >= 3.7.
#
# Usage:
#   .\dji-gui.ps1                 # serve on 127.0.0.1:8000 and open a browser
#   .\dji-gui.ps1 -HttpPort 9000
#   .\dji-gui.ps1 -Lan            # bind 0.0.0.0 so a tablet on the LAN can view
#
# If PowerShell blocks the script, either run once with:
#   powershell -ExecutionPolicy Bypass -File .\dji-gui.ps1
# or relax policy for the current user:
#   Set-ExecutionPolicy -Scope CurrentUser RemoteSigned

param(
    [int]$HttpPort = 8000,
    [switch]$Lan
)

$HttpHost = if ($Lan) { "0.0.0.0" } else { "127.0.0.1" }

wt.exe --size 240,60 `
  new-tab --title "swarm-gui" `
    -d "C:\Users\jarvis\Documents\DJI_Swarm\AOS server" `
    PowerShell -NoExit -Command "python swarm_gui.py --http-host $HttpHost --http-port $HttpPort --open"
