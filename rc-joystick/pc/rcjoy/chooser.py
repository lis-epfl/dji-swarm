"""Ask the operator which RC to use when several answer: a small Windows dialog.

    choose_rc([(ip, info), ...], remembered=None, cancelled=lambda: False) -> ip or None

One button per RC, labelled with what tells them apart: the IP (the RC shows it in a chip at
the top of its screen), the serial number, the battery and the app version. The RC used last
is marked, and Enter picks it. "Not now" and Esc return None.

It is PowerShell + Windows Forms, which every Windows 10/11 PC has, so the bridge still needs
nothing beyond the Python stdlib (python.org's 3.7 on the operator PC has no tkinter). On any
other system, or when no window can be shown, it returns None and the client falls back to
its non-interactive rule (the remembered RC, else "pass --rc").

Only RCs running LIS_CONTROLLER answer at all: the fleet's RCs run lis-swarm-app, which does
not listen on :5070, so flying the swarm does not by itself bring this up.
"""

import base64
import json
import os
import subprocess
import time

# Data comes in on stdin as ASCII JSON (json.dumps escapes anything else), so the console
# code page never matters. The answer goes out on stdout: the chosen IP, or nothing.
_SCRIPT = r"""
$ErrorActionPreference = 'Stop'
Add-Type -AssemblyName System.Windows.Forms
Add-Type -AssemblyName System.Drawing
[System.Windows.Forms.Application]::EnableVisualStyles()
$data = [Console]::In.ReadToEnd() | ConvertFrom-Json

$form = New-Object System.Windows.Forms.Form
$form.Text = 'LIS_CONTROLLER - choose the joystick RC'
$form.StartPosition = 'CenterScreen'
$form.TopMost = $true
$form.FormBorderStyle = 'FixedDialog'
$form.MaximizeBox = $false
$form.MinimizeBox = $false
$form.ShowInTaskbar = $true
$form.Font = New-Object System.Drawing.Font('Segoe UI', 10)
$form.AutoSize = $true
$form.AutoSizeMode = 'GrowAndShrink'
$form.Padding = New-Object System.Windows.Forms.Padding(18)

$panel = New-Object System.Windows.Forms.FlowLayoutPanel
$panel.FlowDirection = 'TopDown'
$panel.WrapContents = $false
$panel.AutoSize = $true
$panel.AutoSizeMode = 'GrowAndShrink'

$title = New-Object System.Windows.Forms.Label
$title.Text = 'Several RCs are running LIS_CONTROLLER'
$title.Font = New-Object System.Drawing.Font('Segoe UI Semibold', 12)
$title.AutoSize = $true
$panel.Controls.Add($title)

$hint = New-Object System.Windows.Forms.Label
$hint.Text = 'Click the one in your hands. Its address is in the chip at the top of its screen.'
$hint.ForeColor = [System.Drawing.Color]::DimGray
$hint.AutoSize = $true
$hint.Margin = New-Object System.Windows.Forms.Padding(3, 4, 3, 14)
$panel.Controls.Add($hint)

$state = @{ choice = '' }   # a shared object: handlers run in their own scope
foreach ($rc in $data.options) {
    $b = New-Object System.Windows.Forms.Button
    $b.Text = $rc.label
    $b.Tag = $rc.ip
    $b.Size = New-Object System.Drawing.Size(520, 58)
    $b.TextAlign = 'MiddleLeft'
    $b.Padding = New-Object System.Windows.Forms.Padding(10, 0, 0, 0)
    $b.Margin = New-Object System.Windows.Forms.Padding(3, 3, 3, 6)
    $b.Add_Click({ $state.choice = $this.Tag; $form.Close() })
    $panel.Controls.Add($b)
    if ($rc.ip -eq $data.remembered) { $form.AcceptButton = $b }
}

$cancel = New-Object System.Windows.Forms.Button
$cancel.Text = 'Not now'
$cancel.Size = New-Object System.Drawing.Size(110, 34)
$cancel.Margin = New-Object System.Windows.Forms.Padding(3, 10, 3, 0)
$cancel.Add_Click({ $form.Close() })
$form.CancelButton = $cancel
$panel.Controls.Add($cancel)

$form.Controls.Add($panel)
$form.Add_Shown({ $form.Activate() })
[void]$form.ShowDialog()
[Console]::Out.Write($state.choice)
"""

_CREATE_NO_WINDOW = 0x08000000      # no console window for powershell.exe; the form still shows


def label(ip, info, last_used=False):
    """The button text for one RC: its IP first, then what else tells RCs apart."""
    info = info or {}
    first = ip + ("      (last used)" if last_used else "")
    parts = [str(info.get("rc_type") or "RC")]
    if info.get("sn"):
        parts.append("sn %s" % info["sn"])
    bat = info.get("battery")
    if isinstance(bat, (int, float)) and not isinstance(bat, bool):
        parts.append("battery %d%%" % bat)
    if info.get("app"):
        parts.append("app %s" % info["app"])
    return first + "\n" + "   |   ".join(parts)


def payload(options, remembered=None):
    """The dialog's input: ASCII JSON, whatever the RCs report."""
    return json.dumps({
        "remembered": remembered or "",
        "options": [{"ip": ip, "label": label(ip, info, ip == remembered)} for ip, info in options],
    })


class ChooserUnavailable(RuntimeError):
    """No dialog could be shown (not Windows, no PowerShell, no desktop). The caller should
    fall back to its non-interactive rule rather than treat this as "not now"."""


def choose_rc(options, remembered=None, cancelled=lambda: False):
    """options: [(ip, info dict)]. Returns the chosen IP, or None when the operator
    dismissed the dialog or `cancelled()` turned true (the client stopping). Raises
    ChooserUnavailable when no dialog could be shown at all."""
    if os.name != "nt":
        raise ChooserUnavailable("the pop-up needs Windows")
    if not options:
        return None
    script = base64.b64encode(_SCRIPT.encode("utf-16-le")).decode("ascii")
    try:
        p = subprocess.Popen(
            ["powershell.exe", "-NoProfile", "-ExecutionPolicy", "Bypass", "-EncodedCommand", script],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
            creationflags=_CREATE_NO_WINDOW)
    except OSError as e:
        raise ChooserUnavailable("cannot start powershell.exe: %s" % e)
    try:
        p.stdin.write(payload(options, remembered).encode("ascii"))
        p.stdin.close()
    except OSError:
        pass
    while p.poll() is None:
        if cancelled():
            p.kill()
            p.wait()
            return None
        time.sleep(0.1)
    if p.returncode != 0:
        raise ChooserUnavailable("the dialog could not be shown (powershell exit %d)" % p.returncode)
    out = p.stdout.read().decode("ascii", "replace").strip()
    return out if out in {ip for ip, _ in options} else None
