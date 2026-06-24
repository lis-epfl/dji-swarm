# Launch joystick_controller.py and readController.py side by side in Windows Terminal.
# Runs the single-drone direct joystick controller (vs. dji-flocking.ps1 which
# runs the multi-drone Olfati-Saber flocking controller).
#
# Usage:
#   .\dji-joystick.ps1
# If PowerShell blocks the script, either run once with:
#   powershell -ExecutionPolicy Bypass -File .\dji-joystick.ps1
# or relax policy for the current user:
#   Set-ExecutionPolicy -Scope CurrentUser RemoteSigned

# The readController pane sources the conda hook and activates this env
# before launching the script. Edit if your miniconda lives elsewhere.
$CondaHook = "C:\Users\jarvis\AppData\Local\miniconda3\shell\condabin\conda-hook.ps1"
$EnvName   = "stitching"

wt.exe --size 240,60 `
  new-tab --title "joystick" `
    -d "C:\Users\jarvis\Documents\DJI_Swarm\AOS server" `
    PowerShell -NoExit -Command "python joystick_controller.py" `
  `; split-pane -V --size 0.25 --title "controller" `
    -d "C:\Users\jarvis\Documents\vr_swarm_simulation\Assets\Scripts\Control" `
    PowerShell -NoExit -Command "& '$CondaHook' \; conda activate $EnvName \; python readController.py"
