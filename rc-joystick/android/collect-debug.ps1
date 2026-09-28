# Collect everything needed to debug LIS_CONTROLLER on the RC Pro over USB (adb).
# The procedure that goes with it is DEBUGGING.md (same folder).
#
#   .\collect-debug.ps1                     snapshot: device, packages, crash log, app files
#   .\collect-debug.ps1 -Launch             also: force-stop, clear logcat, start the app,
#                                           wait -Seconds, then capture (reproduces a crash)
#   .\collect-debug.ps1 -Launch -Safe       start it in SAFE MODE (nothing auto-starts)
#   .\collect-debug.ps1 -Install -Launch    (re)install the freshly built debug APK first
#   .\collect-debug.ps1 -Serial <id>        pick the RC when several devices are attached
#
# Output: rc-joystick\android\debug-logs\<timestamp>\ (gitignored). Send the whole folder.
#
# If PowerShell blocks the script:
#   powershell -ExecutionPolicy Bypass -File .\collect-debug.ps1 -Launch
# NB: keep string literals ASCII-only (no BOM; PS 5.1 reads the file as ANSI).

param(
    [switch]$Launch,
    [switch]$Safe,
    [switch]$Install,
    [int]$Seconds = 20,
    [string]$Serial = ''
)

$pkg = 'com.liscontroller'
$apk = Join-Path $PSScriptRoot 'app\build\outputs\apk\debug\app-debug.apk'

# --- adb and the device --------------------------------------------------------------
$adb = $null
$found = Get-Command adb -ErrorAction SilentlyContinue
if ($found) { $adb = $found.Source }
if (-not $adb) { $adb = Join-Path $env:LOCALAPPDATA 'Android\Sdk\platform-tools\adb.exe' }
if (-not (Test-Path $adb)) { throw "adb not found. Install the Android SDK platform-tools." }

$rows = @(& $adb devices | Select-Object -Skip 1 | Where-Object { $_.Trim() -ne '' })
$ready = @($rows | Where-Object { $_ -match '^\S+\s+device$' } | ForEach-Object { ($_ -split '\s+')[0] })
$unauth = @($rows | Where-Object { $_ -match 'unauthorized' } | ForEach-Object { ($_ -split '\s+')[0] })
if (-not $Serial) {
    $real = @($ready | Where-Object { $_ -notlike 'emulator-*' })
    if ($real.Count -eq 1) {
        $Serial = $real[0]
    } elseif ($real.Count -eq 0) {
        if ($unauth.Count) {
            throw ("The RC (" + ($unauth -join ', ') + ") has not authorized this PC yet: accept the " +
                   "'Allow USB debugging' prompt on the RC screen, then run this again.")
        }
        throw ("No RC visible to adb (seen: '" + ($ready -join ', ') + "'). Plug the RC in by USB-C, " +
               "turn on Settings > Developer options > USB debugging on it, accept the prompt on " +
               "the RC screen, then run this again. (An emulator alone does not count.)")
    } else {
        throw ("Several devices attached: " + ($real -join ', ') + ". Choose one with -Serial.")
    }
}
$dev = @('-s', $Serial)

$stamp = Get-Date -Format 'yyyyMMdd-HHmmss'
$out = Join-Path $PSScriptRoot "debug-logs\$stamp"
New-Item -ItemType Directory -Force -Path $out | Out-Null
Write-Host "[collect] device $Serial -> $out"

function Invoke-Adb([string[]]$a) {
    # In Windows PowerShell 5.1, 2>&1 on a native command yields ErrorRecords whose
    # rendering is PowerShell noise; "$_" turns each back into adb's plain stderr line.
    & $adb @dev @a 2>&1 | ForEach-Object { "$_" }
}
function Save-Adb([string]$name, [string[]]$a) {
    Invoke-Adb $a | Out-File -Encoding utf8 (Join-Path $out $name)
}

# --- optional install + launch -----------------------------------------------------------
if ($Install) {
    if (-not (Test-Path $apk)) { throw "No APK at $apk - build it first (see README)." }
    Write-Host "[collect] installing $apk ..."
    Invoke-Adb @('install', '-r', $apk) | Tee-Object -FilePath (Join-Path $out 'install.txt')
}

if ($Launch) {
    Invoke-Adb @('shell', 'am', 'force-stop', $pkg) | Out-Null
    Invoke-Adb @('logcat', '-b', 'all', '-c') | Out-Null
    $start = @('shell', 'am', 'start', '-n', "$pkg/.MainActivity")
    if ($Safe) { $start += @('--ez', 'safe', 'true') }
    Invoke-Adb $start | Out-File -Encoding utf8 (Join-Path $out 'am-start.txt')
    Write-Host "[collect] app started$(if ($Safe) { ' in SAFE MODE' }); waiting $Seconds s - answer any permission prompt on the RC"
    Start-Sleep -Seconds $Seconds
}

# --- capture ---------------------------------------------------------------------------
Save-Adb 'getprop.txt' @('shell', 'getprop')
Save-Adb 'packages.txt' @('shell', 'pm', 'list', 'packages')
Save-Adb 'package-liscontroller.txt' @('shell', 'dumpsys', 'package', $pkg)
Save-Adb 'exit-info.txt' @('shell', 'dumpsys', 'activity', 'exit-info', $pkg)   # Android 11+
Save-Adb 'logcat-crash.txt' @('logcat', '-d', '-b', 'crash', '-v', 'threadtime')
Save-Adb 'logcat-app.txt' @('logcat', '-d', '-v', 'threadtime', '-s', 'LIS_CONTROLLER:V',
                            'AndroidRuntime:V', 'DEBUG:V', 'libc:V', 'ActivityManager:W')
Save-Adb 'logcat-all.txt' @('logcat', '-d', '-v', 'threadtime')
foreach ($f in @('crash.txt', 'trace.txt', 'trace.prev.txt')) {
    Save-Adb ("app-" + $f) @('shell', 'run-as', $pkg, 'cat', "files/$f")
}

# --- summary ---------------------------------------------------------------------------
Write-Host ""
$model = ((Invoke-Adb @('shell', 'getprop', 'ro.product.model')) -join '').Trim()
$android = ((Invoke-Adb @('shell', 'getprop', 'ro.build.version.release')) -join '').Trim()
Write-Host "[collect] $model, Android $android"
$ver = Select-String -Path (Join-Path $out 'package-liscontroller.txt') -Pattern 'versionName=' |
       Select-Object -First 1
if ($ver) { Write-Host ("[collect] installed " + $pkg + " " + $ver.Line.Trim()) }
else { Write-Host "[collect] $pkg is NOT installed" -ForegroundColor Red }

$pkgs = Get-Content (Join-Path $out 'packages.txt')
if ($pkgs -match 'package:com\.lisswarm$') {
    Write-Host "[collect] WARNING: lis-swarm-app (com.lisswarm) is installed on this RC too - two MSDK apps contend for the RC's USB link. Uninstall it here." -ForegroundColor Yellow
}
$fly = ((Invoke-Adb @('shell', 'pidof', 'dji.go.v5')) -join '').Trim()
if ($fly -match '^\d') {
    Write-Host "[collect] WARNING: DJI Fly is running (pid $fly) and can hold the RC's USB link: adb -s $Serial shell am force-stop dji.go.v5" -ForegroundColor Yellow
}

$files = @((Join-Path $out 'logcat-crash.txt'), (Join-Path $out 'app-crash.txt'))
$hits = Select-String -Path $files -Pattern 'FATAL EXCEPTION|Fatal signal|Abort message|Caused by|^\s+at com\.liscontroller|Exception: ' |
        Select-Object -First 15
if ($hits) {
    Write-Host "[collect] CRASH FOUND - first lines (full text in logcat-crash.txt / app-crash.txt):" -ForegroundColor Red
    $hits | ForEach-Object { Write-Host ("   " + $_.Line.Trim()) }
} else {
    Write-Host "[collect] no crash in the crash log or crash.txt" -ForegroundColor Green
}
foreach ($t in @('app-trace.txt', 'app-trace.prev.txt')) {
    $p = Join-Path $out $t
    # Real trace lines start with a timestamp; anything else is adb/run-as saying the
    # file (or the app) is not there.
    $lines = @(Get-Content $p -ErrorAction SilentlyContinue | Where-Object { $_ -match '^\d\d:\d\d:\d\d\.\d{3} ' })
    if ($lines.Count) {
        Write-Host "[collect] last steps in $t :"
        $lines | Select-Object -Last 4 | ForEach-Object { Write-Host "   $_" }
    }
}
Write-Host "[collect] done - send the folder $out"
exit 0      # a failed run-as (file not there yet) must not read as the script failing
