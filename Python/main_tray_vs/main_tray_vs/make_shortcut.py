"""
make_shortcut.py - creates a "YuMi Control" shortcut on the Desktop and in the Start menu.

Run it ONCE with the same Python you normally run the panel with (that is the Python that has
opencv / depthai installed):

    python make_shortcut.py

The shortcut starts the panel with pythonw.exe, so no console window appears. Double-click it,
or pin it to the taskbar / Start. Re-run this script if you move the folder or change Python.
"""
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPT = os.path.join(HERE, "yumi_shortcut.py")
ICON = os.path.join(HERE, "yumi.ico")
NAME = "YuMi Control"


def pythonw_path():
    exe = sys.executable
    if exe.lower().endswith("python.exe"):
        w = exe[:-len("python.exe")] + "pythonw.exe"
        if os.path.isfile(w):
            return w
    return exe


def ps_quote(text):
    """Escape a string for use inside a PowerShell double-quoted string."""
    return text.replace("`", "``").replace("$", "`$").replace('"', '`"')


def main():
    if os.name != "nt":
        print("This script creates Windows shortcuts; run it on the Windows PC.")
        return 1
    if not os.path.isfile(SCRIPT):
        print(f"Cannot find {SCRIPT}")
        return 1
    target = pythonw_path()
    ps = f'''
$ErrorActionPreference = "Stop"
$w = New-Object -ComObject WScript.Shell
foreach ($folder in @("Desktop", "Programs")) {{
    $dir = [Environment]::GetFolderPath($folder)
    $lnk = Join-Path $dir "{ps_quote(NAME)}.lnk"
    $s = $w.CreateShortcut($lnk)
    $s.TargetPath = "{ps_quote(target)}"
    $s.Arguments = '"{SCRIPT.replace("'", "''")}"'
    $s.WorkingDirectory = "{ps_quote(HERE)}"
    $s.IconLocation = "{ps_quote(ICON)},0"
    $s.Description = "YuMi cell control panel"
    $s.Save()
    Write-Output $lnk
}}
'''
    with tempfile.NamedTemporaryFile("w", suffix=".ps1", delete=False, encoding="utf-8-sig") as f:
        f.write(ps)
        ps1 = f.name
    try:
        r = subprocess.run(["powershell", "-NoProfile", "-ExecutionPolicy", "Bypass", "-File", ps1],
                           capture_output=True, text=True)
    finally:
        os.remove(ps1)
    if r.returncode != 0:
        print("Could not create the shortcut:\n", r.stderr or r.stdout)
        return 1
    print("Python used :", target)
    print("Shortcuts   :")
    for line in r.stdout.strip().splitlines():
        print("   ", line)
    print("\nDone. Double-click 'YuMi Control' on the Desktop (or find it in the Start menu).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
