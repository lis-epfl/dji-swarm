# Open (or close) DJI's gate on the RC Pro's built-in gamepad, for LIS_CONTROLLER.
#
# DJI's modified Android (com.dji.comkey.ComKeyManager, inside system_server) drops the
# "DJI embedded joystick" events for every non-system app unless three global settings
# are on. Without them LIS_CONTROLLER still works, but its sticks and dials come from MSDK
# at ~10 Hz instead of the gamepad's ~70 Hz (README, "The RC Pro's built-in gamepad").
#
#   .\enable-gamepad.ps1                 turn the three settings on, then show them
#   .\enable-gamepad.ps1 -Status         only show them
#   .\enable-gamepad.ps1 -Disable        put them back as this RC shipped (all off)
#   .\enable-gamepad.ps1 -Serial <id>    pick the RC when several devices are attached
#
# ONLY on the dedicated joystick RC. It refuses an RC with lis-swarm-app installed: on a
# fleet RC, game mode would also hand the flying sticks to whatever app is in front.
# The settings survive a reboot. A DJI firmware update may reset them; the app's screen
# and trace then say "DJI gate: game mode OFF", and the sticks fall back to MSDK.
# No app restart is needed either way: the app re-reads the gate every second.
#
# If PowerShell blocks the script:
#   powershell -ExecutionPolicy Bypass -File .\enable-gamepad.ps1
# NB: keep string literals ASCII-only (no BOM; PS 5.1 reads the file as ANSI).

param(
    [switch]$Status,
    [switch]$Disable,
    [string]$Serial = ''
)

# --- adb and the device (as collect-debug.ps1) ------------------------------------------
$adb = $null
$found = Get-Command adb -ErrorAction SilentlyContinue
if ($found) { $adb = $found.Source }
if (-not $adb) { $adb = Join-Path $env:LOCALAPPDATA 'Android\Sdk\platform-tools\adb.exe' }
if (-not (Test-Path $adb)) { throw "adb not found. Install the Android SDK platform-tools." }

$rows = @(& $adb devices | Select-Object -Skip 1 | Where-Object { $_.Trim() -ne '' })
$ready = @($rows | Where-Object { $_ -match '^\S+\s+device$' } | ForEach-Object { ($_ -split '\s+')[0] })
if (-not $Serial) {
    $real = @($ready | Where-Object { $_ -notlike 'emulator-*' })
    if ($real.Count -eq 1) {
        $Serial = $real[0]
    } elseif ($real.Count -eq 0) {
        throw ("No RC visible to adb. Plug the joystick RC in by USB-C with USB debugging on, " +
               "accept the prompt on its screen, then run this again.")
    } else {
        throw ("Several devices attached: " + ($real -join ', ') + ". Choose one with -Serial.")
    }
}
$dev = @('-s', $Serial)

function Invoke-Shell([string]$cmd) {
    # One quoted remote command; "$_" turns PS 5.1's stderr ErrorRecords back into text.
    (& $adb @dev shell $cmd 2>&1 | ForEach-Object { "$_" }) -join "`n"
}

$gate = @('dji_lab_game_mode', 'dji_motion_via_left_joystick_enabled', 'dji_motion_via_right_joystick_enabled')

if (-not $Status) {
    $packages = Invoke-Shell 'pm list packages'
    if ($packages -match 'package:com\.lisswarm\b') {
        throw ("$Serial has lis-swarm-app (com.lisswarm) installed: it is a FLEET RC, not the " +
               "joystick RC. Refusing to change its gamepad gate.")
    }
    foreach ($k in $gate) {
        if (-not $Disable) {
            Invoke-Shell "settings put global $k 1" | Out-Null
        } elseif ($k -eq 'dji_lab_game_mode') {
            Invoke-Shell "settings delete global $k" | Out-Null     # it shipped unset
        } else {
            Invoke-Shell "settings put global $k 0" | Out-Null
        }
    }
}

Write-Host "[gamepad] DJI gate on $Serial :"
$open = $true
foreach ($k in $gate) {
    $v = (Invoke-Shell "settings get global $k").Trim()
    if ($v -ne '1') { $open = $false }
    Write-Host ("  {0,-40} {1}" -f $k, $v)
}
if ($open) {
    Write-Host "[gamepad] open: LIS_CONTROLLER takes the sticks and dials from the gamepad (~70 Hz)"
} else {
    Write-Host "[gamepad] closed: LIS_CONTROLLER takes the sticks and dials from MSDK (~10 Hz)"
}
exit 0
