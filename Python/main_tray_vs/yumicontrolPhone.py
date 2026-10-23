"""
YuMi Cell Control - control panel for the YuMi (RobotWare 6.16) + vision cell.
Keeps the proven start sequence: mastership only for motors-on / pointer reset,
RELEASED before the start command (holding it makes the start a silent no-op).

Added: detection-model dropdown, camera window embedded in the panel (Windows),
and main_tray.py running without a visible console (output -> yumi_logs/main_tray.log).
Also: pause/resume, move home, speed slider, snapshot/record, arm view, F-key hotkeys.
"""
import datetime, hmac, json, math, os, queue, re, secrets, socket, subprocess, sys, threading, time
import urllib.parse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import tkinter as tk
from tkinter import filedialog, messagebox, ttk
import requests
from requests.auth import HTTPDigestAuth
try:
    import winsound
except ImportError:
    winsound = None

if os.name == "nt":
    import ctypes
    from ctypes import wintypes
    user32 = ctypes.windll.user32
else:
    user32 = None

# ------------------------------ CONFIG ------------------------------
CONTROLLER_URL = "http://192.168.125.1"
AUTH = HTTPDigestAuth("Zebastian", "robotics")
HEADERS = {"Content-Type": "application/x-www-form-urlencoded;v=2.0", "Accept": "application/json"}
MAIN_TRAY_FILE = "main_tray.py"
TIMEOUT = 5
HERE = os.path.dirname(os.path.abspath(__file__))
LOG_DIR = os.path.join(HERE, "yumi_logs")

# --- main_tray.py start-up prompts ---
# Prompt 1 "Run detection model v5, v5s, or v8?"          -> answered from the GUI dropdown
# Prompt 2 "Do you want to save the test protocol? (y/n)" -> always SAVE_PROTOCOL_ANSWER
# (Saving the protocol asks more questions later, which a hidden console cannot answer.)
DETECTION_MODELS = {"v5": "5", "v5s": "5s", "v8": "8"}   # GUI label -> typed answer
DEFAULT_MODEL = "v5s"
SAVE_PROTOCOL_ANSWER = "n"
# True  = answers typed automatically, console hidden (output -> yumi_logs/main_tray.log)
# False = old behaviour: main_tray.py opens in its own console and you type the answers (dropdown ignored)
AUTO_ANSWER_PROMPTS = True

# --- embedded camera window (Windows only) ---
EMBED_CAMERA = True
EMBED_TIMEOUT = 40                 # seconds to wait for main_tray.py's camera window
CAMERA_WINDOW_TITLE_HINT = "RGB"   # main_tray.py opens cv.imshow("RGB", ...)

# --- camera snapshot / recording: needs yumi_capture.py next to main_tray.py + 2 lines in main_tray.py ---
CAPTURE_DIR = os.path.join(HERE, "yumi_captures")
SNAP_FLAG = os.path.join(CAPTURE_DIR, ".snapshot_request")
REC_FLAG = os.path.join(CAPTURE_DIR, ".recording")

# --- move home: uses the same calls as main_tray.py (Communication().connectV2() / .MoveHome()) ---
HOME_TIMEOUT = 90   # seconds

# --- phone web page: status + live camera + STOP button ---
# The link (with its secret key) is written to the live log at start-up and behind the "Phone link" button.
WEB_ENABLED = True
WEB_PORT = 8080
LIVE_FLAG = os.path.join(CAPTURE_DIR, ".live")      # touched by the web server while someone is watching
LIVE_JPG = os.path.join(CAPTURE_DIR, "live.jpg")    # written by yumi_capture.py

BG, PANEL, PANEL2, FG, MUTED = "#0d1117", "#161b22", "#1f2630", "#e6edf3", "#7d8590"
GREEN, RED, AMBER, BLUE, CYAN = "#2ea043", "#da3633", "#d29922", "#388bfd", "#39c5cf"
STEPS = ["Connect to controller", "Check AUTO mode", "Stop RAPID", "Take mastership",
         "Motors on", "Reset program pointers", "Release mastership",
         "Start RAPID", "Verify all tasks", "Launch vision"]

# --- arm view: SCHEMATIC forward kinematics from the joint angles ---
# Approximate geometry, NOT CAD-accurate. Tune these to match the real robot.
MECH_UNITS = ("ROB_L", "ROB_R")
ARM_COLORS = {"ROB_L": CYAN, "ROB_R": AMBER}
ARM_BASE = {                      # x, y, z (m), roll about x (deg), pitch about y (deg) in the robot base frame
    "ROB_L": (0.05, 0.07, 0.52, -35.0, 0.0),
    "ROB_R": (0.05, -0.07, 0.52, 35.0, 0.0),
}
ARM_LINKS = (0.25, 0.22, 0.09)            # upper arm, forearm, hand (m)
ARM_JOINT_SIGN = (1, 1, 1, 1, 1, 1, 1)    # j1..j7: flip a 1 to -1 if a joint bends the wrong way
ARM_JOINT_OFFSET = (0, 0, 0, 0, 0, 0, 0)  # degrees added to j1..j7


# ------------------------------ Win32 helpers for the embedded camera ------------------------------
GWL_STYLE = -16
WS_CHILD, WS_VISIBLE, WS_POPUP = 0x40000000, 0x10000000, 0x80000000
WS_CAPTION, WS_THICKFRAME, WS_SYSMENU = 0x00C00000, 0x00040000, 0x00080000
WS_MINIMIZEBOX, WS_MAXIMIZEBOX = 0x00020000, 0x00010000

if user32 is not None:
    WNDENUMPROC = ctypes.WINFUNCTYPE(wintypes.BOOL, wintypes.HWND, wintypes.LPARAM)
    user32.EnumWindows.argtypes = [WNDENUMPROC, wintypes.LPARAM]
    user32.IsWindowVisible.argtypes = [wintypes.HWND]
    user32.GetWindowThreadProcessId.argtypes = [wintypes.HWND, ctypes.POINTER(wintypes.DWORD)]
    user32.GetWindowTextLengthW.argtypes = [wintypes.HWND]
    user32.GetWindowTextW.argtypes = [wintypes.HWND, wintypes.LPWSTR, ctypes.c_int]
    user32.GetWindowLongW.argtypes = [wintypes.HWND, ctypes.c_int]
    user32.GetWindowLongW.restype = ctypes.c_long
    user32.SetWindowLongW.argtypes = [wintypes.HWND, ctypes.c_int, ctypes.c_long]
    user32.SetWindowLongW.restype = ctypes.c_long
    user32.SetParent.argtypes = [wintypes.HWND, wintypes.HWND]
    user32.MoveWindow.argtypes = [wintypes.HWND, ctypes.c_int, ctypes.c_int,
                                  ctypes.c_int, ctypes.c_int, wintypes.BOOL]


def find_window_for_pid(pid):
    """First visible, titled top-level window owned by process `pid` (matching the title hint)."""
    if user32 is None:
        return None
    found = []

    def cb(hwnd, _lparam):
        if not user32.IsWindowVisible(hwnd):
            return True
        wpid = wintypes.DWORD()
        user32.GetWindowThreadProcessId(hwnd, ctypes.byref(wpid))
        if wpid.value != pid:
            return True
        n = user32.GetWindowTextLengthW(hwnd)
        if n == 0:
            return True
        buf = ctypes.create_unicode_buffer(n + 1)
        user32.GetWindowTextW(hwnd, buf, n + 1)
        if CAMERA_WINDOW_TITLE_HINT and CAMERA_WINDOW_TITLE_HINT.lower() not in buf.value.lower():
            return True
        found.append(hwnd)
        return True

    user32.EnumWindows(WNDENUMPROC(cb), 0)
    return found[0] if found else None


def _mm(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def _rx(a):
    c, s = math.cos(a), math.sin(a)
    return [[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1]]


def _ry(a):
    c, s = math.cos(a), math.sin(a)
    return [[c, 0, s, 0], [0, 1, 0, 0], [-s, 0, c, 0], [0, 0, 0, 1]]


def _tr(x, y, z):
    return [[1, 0, 0, x], [0, 1, 0, y], [0, 0, 1, z], [0, 0, 0, 1]]


def _dh(theta, d, alpha):
    ct, st, ca, sa = math.cos(theta), math.sin(theta), math.cos(alpha), math.sin(alpha)
    return [[ct, -st * ca, st * sa, 0], [st, ct * ca, -ct * sa, 0], [0, sa, ca, d], [0, 0, 0, 1]]


def arm_points(unit, joints):
    """Shoulder, elbow, wrist, tip (x, y, z in metres) of one arm from its joint angles j1..j7 (deg)."""
    bx, by, bz, roll, pitch = ARM_BASE[unit]
    T = _mm(_tr(bx, by, bz), _mm(_rx(math.radians(roll)), _ry(math.radians(pitch))))
    j = [ARM_JOINT_SIGN[i] * joints[i] + ARM_JOINT_OFFSET[i] for i in range(7)]
    up, fore, hand = ARM_LINKS
    # YuMi kinematic order is joint 1, 2, 7, 3, 4, 5, 6 (joint 7 = the extra axis in the upper arm)
    chain = ((j[0], 0, -90), (j[1], 0, 90), (j[6], up, -90), (j[2], 0, 90),
             (j[3], fore, -90), (j[4], 0, 90), (j[5], hand, 0))
    pts = [(T[0][3], T[1][3], T[2][3])]
    for deg, d, alpha in chain:
        T = _mm(T, _dh(math.radians(deg), d, math.radians(alpha)))
        pts.append((T[0][3], T[1][3], T[2][3]))
    return [pts[0], pts[3], pts[5], pts[7]]


WEB_PAGE = r"""<!doctype html>
<html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">
<title>YuMi</title>
<style>
:root{--bg:#0d1117;--panel:#161b22;--panel2:#1f2630;--fg:#e6edf3;--mut:#7d8590;--green:#2ea043;--red:#da3633;--amber:#d29922;--cyan:#39c5cf}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--fg);font:15px -apple-system,"Segoe UI",Roboto,sans-serif;
padding:env(safe-area-inset-top) 0 calc(86px + env(safe-area-inset-bottom))}
header{display:flex;justify-content:space-between;align-items:center;padding:12px 14px;background:var(--panel)}
header b{font-size:18px} #conn{font-weight:600;color:var(--amber)}
.cam{position:relative;background:#000;aspect-ratio:4/3}
.cam img{width:100%;height:100%;object-fit:contain;display:block}
#camoff{position:absolute;inset:0;display:flex;align-items:center;justify-content:center;color:var(--mut);background:#000}
.card{background:var(--panel);margin:10px;padding:12px 14px;border-radius:10px}
.card h3{margin:0 0 8px;font-size:11px;letter-spacing:.08em;color:var(--mut)}
#headline{font-size:17px;font-weight:700}
.kv{display:flex;justify-content:space-between;padding:3px 0}.kv span:first-child{color:var(--mut)}
.kv span:last-child{font-family:Consolas,monospace;font-weight:700}
#steps{display:flex;flex-wrap:wrap;gap:6px}
.chip{font-size:12px;padding:3px 8px;border-radius:12px;background:var(--panel2);color:var(--mut)}
.chip.ok{background:#1b4d2b;color:#9be9a8}.chip.run{background:#10444a;color:var(--cyan)}
.chip.fail{background:#5c1d1b;color:#ffb3ad}.chip.skip{opacity:.6}
pre{margin:0;font:11px/1.35 Consolas,monospace;white-space:pre-wrap;word-break:break-word;color:var(--fg)}
#stopbar{position:fixed;left:0;right:0;bottom:0;padding:10px 12px calc(10px + env(safe-area-inset-bottom));background:var(--bg)}
#stop{width:100%;padding:18px;font-size:20px;font-weight:800;color:#fff;background:var(--red);border:0;border-radius:12px}
#stop:disabled{opacity:.6}
</style></head><body>
<header><b>YuMi</b><span id="conn">● connecting</span></header>
<div class="cam"><img id="cam" alt=""><div id="camoff">camera off</div></div>
<div class="card"><div id="headline">…</div></div>
<div class="card"><h3>CONTROLLER</h3><div id="vals"></div></div>
<div class="card"><h3>WORKFLOW</h3><div id="steps"></div></div>
<div class="card"><h3>JOINTS (deg)</h3><pre id="joints">—</pre></div>
<div class="card"><h3>LOG</h3><pre id="log"></pre></div>
<div id="stopbar"><button id="stop">■ STOP CELL</button></div>
<script>
const T=new URLSearchParams(location.search).get('t')||'';
const q=encodeURIComponent(T), $=id=>document.getElementById(id), cam=$('cam');
function startCam(){cam.src='/stream?t='+q+'&r='+Date.now();}
cam.onerror=()=>setTimeout(startCam,2000);
startCam();
function kv(el,rows){el.textContent='';for(const [k,v] of rows){const d=document.createElement('div');d.className='kv';
 const a=document.createElement('span'),b=document.createElement('span');a.textContent=k;b.textContent=v;d.append(a,b);el.append(d);}}
async function tick(){
 try{
  const r=await fetch('/state?t='+q,{cache:'no-store'});
  if(r.status===403){$('conn').textContent='● bad key';return;}
  const s=await r.json(); if(!s.values) return;
  $('conn').textContent=s.conn||'●'; $('conn').style.color=(s.conn||'').includes('online')?'var(--green)':'var(--red)';
  $('headline').textContent=s.headline; $('headline').style.color=s.headline_color||'';
  const v=s.values, rows=Object.entries(v).filter(([k])=>k!=='Process');
  rows.push(['Vision',v.Process||'—']); rows.push(['Speed',(s.speed??'—')+' %']);
  for(const [t,x] of Object.entries(s.tasks||{})) rows.push([t,x]);
  kv($('vals'),rows);
  $('steps').textContent='';
  for(const st of s.steps||[]){const c=document.createElement('span');c.className='chip '+st.state;c.textContent=st.name;$('steps').append(c);}
  $('joints').textContent=Object.entries(s.joints||{}).map(([u,a])=>u.slice(-1)+' '+a.map(x=>String(Math.round(x)).padStart(5)).join('')).join('\n')||'—';
  $('log').textContent=(s.log||[]).slice(-14).join('\n');
  $('camoff').style.display=(v.Process||'').startsWith('RUNNING')?'none':'flex';
 }catch(e){$('conn').textContent='● no link';$('conn').style.color='var(--red)';}
}
setInterval(tick,1500);tick();
$('stop').onclick=async()=>{const b=$('stop');b.disabled=true;b.textContent='Stop sent …';
 try{await fetch('/stop?t='+q,{method:'POST'});}catch(e){}
 setTimeout(()=>{b.disabled=false;b.textContent='■ STOP CELL';},3000);};
</script></body></html>"""


def load_web_token():
    """Secret key for the phone page; created once and kept in yumi_logs/web_token.txt."""
    path = os.path.join(LOG_DIR, "web_token.txt")
    try:
        with open(path, encoding="utf-8") as f:
            tok = f.read().strip()
            if tok:
                return tok
    except OSError:
        pass
    tok = secrets.token_urlsafe(9)
    with open(path, "w", encoding="utf-8") as f:
        f.write(tok)
    return tok


def local_ips():
    """This PC's IPv4 addresses a phone could reach (skips loopback and the 192.168.125.x robot network)."""
    ips = []
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            ip = info[4][0]
            if not ip.startswith(("127.", "192.168.125.")) and ip not in ips:
                ips.append(ip)
    except OSError:
        pass
    return ips


class WebHandler(BaseHTTPRequestHandler):
    app = None   # set to the App before the server starts

    def log_message(self, *args):
        pass

    def authed(self):
        qs = urllib.parse.parse_qs(urllib.parse.urlparse(self.path).query)
        tok = (qs.get("t") or [""])[0]
        return hmac.compare_digest(tok.encode(), self.app.web_token.encode())

    def reply(self, code, body, ctype="text/plain; charset=utf-8"):
        if isinstance(body, str):
            body = body.encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        try:
            path = urllib.parse.urlparse(self.path).path
            if path == "/favicon.ico":
                return self.reply(204, b"")
            if not self.authed():
                return self.reply(403, "Forbidden - open the link shown in the YuMi panel (it contains the key).")
            if path == "/":
                self.reply(200, WEB_PAGE, "text/html; charset=utf-8")
            elif path == "/state":
                self.reply(200, json.dumps(self.app.web_cache), "application/json")
            elif path == "/stream":
                self.stream()
            else:
                self.reply(404, "Not found")
        except OSError:
            pass                                    # phone went away

    def do_POST(self):
        try:
            if not self.authed():
                return self.reply(403, "Forbidden")
            if urllib.parse.urlparse(self.path).path == "/stop":
                self.app.q.put(("webstop", None))
                self.reply(200, "stopping")
            else:
                self.reply(404, "Not found")
        except OSError:
            pass

    def stream(self):
        """MJPEG: sends live.jpg (written by yumi_capture.py) whenever it changes."""
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        last_m, last_touch = 0.0, 0.0
        while not self.app.quit.is_set():
            now = time.time()
            if now - last_touch > 1.0:               # tells yumi_capture.py that someone is watching
                last_touch = now
                try:
                    with open(LIVE_FLAG, "w"):
                        pass
                except OSError:
                    pass
            try:
                m = os.path.getmtime(LIVE_JPG)
                if m != last_m and now - m < 3:
                    with open(LIVE_JPG, "rb") as f:
                        data = f.read()
                    last_m = m
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n" % len(data))
                    self.wfile.write(data)
                    self.wfile.write(b"\r\n")
            except FileNotFoundError:
                pass
            except PermissionError:
                pass
            time.sleep(0.08)


class Cancelled(Exception):
    pass


class Yumi:
    """Thin RWS client (RobotWare 6, Digest auth)."""
    def __init__(self):
        self.s = requests.Session()
        self.s.auth, self.s.headers = AUTH, dict(HEADERS)

    def get(self, path):
        sep = "&" if "?" in path else "?"
        return self.s.get(f"{CONTROLLER_URL}{path}{sep}json=1", timeout=TIMEOUT).text

    def states(self, path):
        return json.loads(self.get(path))["_embedded"]["_state"]

    def field(self, path, key):
        m = re.search(rf'"{key}"\s*:\s*"([^"]*)"', self.get(path))
        return m.group(1) if m else "?"

    def post(self, path, data=None):
        return self.s.post(f"{CONTROLLER_URL}{path}", data=data, timeout=TIMEOUT)

    opmode = lambda self: self.field("/rw/panel/opmode", "opmode")
    ctrlstate = lambda self: self.field("/rw/panel/ctrlstate", "ctrlstate")
    execstate = lambda self: self.field("/rw/rapid/execution", "ctrlexecstate")
    tasks = lambda self: self.states("/rw/rapid/tasks")
    elog = lambda self, n=14: self.states(f"/rw/elog/0?lang=en&order=lifo&limit={n}")

    def joints(self, unit):
        """Joint angles j1..j6 + external axis a (= YuMi joint 7), degrees."""
        st = self.states(f"/rw/motionsystem/mechunits/{unit}/jointtarget")[0]
        out = []
        for k in ("rax_1", "rax_2", "rax_3", "rax_4", "rax_5", "rax_6", "eax_a"):
            try:
                v = float(st.get(k, 0))
            except (TypeError, ValueError):
                v = 0.0
            out.append(v if abs(v) < 1000 else 0.0)
        return out

    def mastership(self):
        return {d: self.field(f"/rw/mastership/{d}", "mastership") for d in ("rapid", "motion")}

    def pp(self):
        return {t["name"]: self.field(f"/rw/rapid/tasks/{t['name']}/pcp", "routinename") for t in self.tasks()}

    def release(self):
        for p in ("motion", "rapid", ""):
            try:
                self.post(f"/rw/mastership{'/' + p if p else ''}?action=release")
            except requests.RequestException:
                pass

    def stop_rapid(self):
        return self.post("/rw/rapid/execution?action=stop", {"stopmode": "stop", "usetsp": "alltsk"})

    def start_rapid(self, regain, tsp):
        return self.post("/rw/rapid/execution?action=start", {
            "regain": regain, "execmode": "continue", "cycle": "forever", "condition": "none",
            "stopatbp": "disabled", "alltaskbytsp": tsp})


class App:
    def __init__(self, root):
        self.root, self.q = root, queue.Queue()
        self.cancel, self.quit = threading.Event(), threading.Event()
        self.vision, self.busy, self.run_t0, self.cur, self.spin = None, False, None, 0, 0
        self.vision_log = None
        self.cam_hwnd, self.cam_token = None, 0
        self.model_choice = DEFAULT_MODEL
        self.recording, self.rec_t0 = False, 0.0
        self.speed_drag, self.speed_hold = False, 0.0
        self.joint_data = {}
        self.web_cache, self.web_token, self.web_srv = {}, "", None
        self.step_state = ["pending"] * len(STEPS)
        os.makedirs(LOG_DIR, exist_ok=True)
        os.makedirs(CAPTURE_DIR, exist_ok=True)
        self.logfile = os.path.join(LOG_DIR, datetime.datetime.now().strftime("cell_%Y%m%d_%H%M%S.log"))
        self.build()
        threading.Thread(target=self.poll_loop, daemon=True).start()
        threading.Thread(target=self.joint_loop, daemon=True).start()
        if WEB_ENABLED:
            self.start_web()
        root.after(80, self.drain)
        root.after(250, self.tick)
        root.bind("<Escape>", lambda e: self.stop_cell())
        for key, fn in (("<F5>", self.start_cell), ("<F6>", self.restart_vision), ("<F7>", self.pause_resume),
                        ("<F8>", self.snapshot), ("<F9>", self.toggle_record)):
            root.bind(key, lambda e, f=fn: f())
        root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.log("YuMi Control ready. Esc = STOP · F5 start · F6 vision · F7 pause · F8 snapshot · F9 record", "dim")

    # ------------------------------ UI ------------------------------
    def build(self):
        r = self.root
        r.title("YuMi Control")
        w = min(1720, r.winfo_screenwidth() - 40)      # original 1180 wide + camera column on the right
        h = min(920, r.winfo_screenheight() - 80)
        r.geometry(f"{w}x{h}")
        r.minsize(min(1500, w), 700)
        r.configure(bg=BG)
        st = ttk.Style()
        st.theme_use("clam")
        st.configure("G.Horizontal.TProgressbar", troughcolor=PANEL2, background=GREEN, bordercolor=PANEL, lightcolor=GREEN, darkcolor=GREEN)
        st.configure("TCombobox", fieldbackground=PANEL2, background=PANEL2, foreground=FG, arrowcolor=FG, bordercolor=PANEL2)
        st.map("TCombobox", fieldbackground=[("readonly", PANEL2)], foreground=[("readonly", FG)],
               selectbackground=[("readonly", PANEL2)], selectforeground=[("readonly", FG)])
        r.option_add("*TCombobox*Listbox.background", PANEL2)
        r.option_add("*TCombobox*Listbox.foreground", FG)

        head = tk.Frame(r, bg=PANEL)
        head.pack(fill="x")
        tk.Label(head, text="YuMi Control", font=("Segoe UI", 20, "bold"), bg=PANEL, fg=FG).pack(side="left", padx=20, pady=12)
        tk.Label(head, text="IRC5 · RobotWare 6.16 · 192.168.125.1", font=("Segoe UI", 10), bg=PANEL, fg=MUTED).pack(side="left")
        self.clock = tk.Label(head, font=("Consolas", 12), bg=PANEL, fg=MUTED)
        self.clock.pack(side="right", padx=20)
        self.conn = tk.Label(head, text="● connecting", font=("Segoe UI", 11, "bold"), bg=PANEL, fg=AMBER)
        self.conn.pack(side="right", padx=10)

        bar = tk.Frame(r, bg=BG)
        bar.pack(fill="x", padx=16, pady=(14, 6))
        self.b_start = self.button(bar, "▶  START / RESTART CELL", GREEN, self.start_cell, 24)
        self.b_stop = self.button(bar, "■  STOP CELL", RED, self.stop_cell, 16)
        self.b_vis = self.button(bar, "Stop vision", PANEL2, self.stop_vision_click, 12, small=True)
        self.b_save = self.button(bar, "Save log", PANEL2, self.save_log, 10, small=True)
        self.b_clear = self.button(bar, "Clear", PANEL2, lambda: self.text_clear(self.logbox), 8, small=True)
        self.b_web = self.button(bar, "Phone link", PANEL2, self.show_web_link, 11, small=True)
        self.motors_off = tk.BooleanVar(value=False)
        tk.Checkbutton(bar, text="also motors OFF on stop", variable=self.motors_off, bg=BG, fg=MUTED, selectcolor=PANEL2,
                       activebackground=BG, activeforeground=FG, font=("Segoe UI", 9)).pack(side="right", padx=8)

        body = tk.Frame(r, bg=BG)
        body.pack(fill="both", expand=True, padx=(0, 16), pady=6)
        body.columnconfigure(0, weight=0, minsize=180)
        body.columnconfigure(1, weight=3, minsize=580)
        body.columnconfigure(2, weight=2, minsize=360)   # camera, to the right of everything
        body.rowconfigure(0, weight=0)
        body.rowconfigure(1, weight=1)

        left = tk.Frame(body, bg=BG)
        left.grid(row=0, column=0, rowspan=2, sticky="nsew", padx=(0, 12))
        self.val = {}
        c = self.card(left, "CONTROLLER")
        for k in ("Mode", "Motors", "RAPID", "Mastership", "Running for"):
            self.val[k] = self.row(c, k)
        c = self.card(left, "RAPID TASKS")
        self.task_lbl = {}
        for t in ("T_ROB_R", "T_ROB_L", "communication"):
            self.task_lbl[t] = self.row(c, t)
        c = self.card(left, "VISION  (main_tray.py)")
        self.val["Process"] = self.row(c, "Process")
        mrow = tk.Frame(c, bg=PANEL)
        mrow.pack(fill="x", pady=2)
        tk.Label(mrow, text="Detection model", font=("Segoe UI", 10), bg=PANEL, fg=MUTED).pack(side="left")
        self.model_var = tk.StringVar(value=DEFAULT_MODEL)
        self.model_combo = ttk.Combobox(mrow, textvariable=self.model_var, values=list(DETECTION_MODELS),
                                        state="readonly", width=7, font=("Consolas", 11, "bold"))
        self.model_combo.pack(side="right")
        if not AUTO_ANSWER_PROMPTS:
            self.model_combo.config(state="disabled")
        c = self.card(left, "CONTROL")
        self.b_pause = self.wide_button(c, "‖  PAUSE", BLUE, self.pause_resume)
        self.b_home = self.wide_button(c, "⌂  HOME", PANEL2, self.move_home)
        self.b_rvis = self.wide_button(c, "↻  RESTART VISION", PANEL2, self.restart_vision)
        self.speed_lbl = tk.Label(c, text="Speed  —", font=("Segoe UI", 10), bg=PANEL, fg=MUTED, anchor="w")
        self.speed_lbl.pack(fill="x", pady=(8, 0))
        self.speed_var = tk.IntVar(value=100)
        self.speed_scale = tk.Scale(c, from_=5, to=100, resolution=5, orient="horizontal", showvalue=False,
                                    variable=self.speed_var, bg=PANEL, fg=FG, troughcolor=PANEL2, highlightthickness=0,
                                    bd=0, activebackground=BLUE, sliderrelief="flat", length=120,
                                    command=lambda v: self.speed_lbl.config(text=f"Speed  {int(float(v))} %", fg=FG))
        self.speed_scale.pack(fill="x")
        self.speed_scale.bind("<ButtonPress-1>", lambda e: setattr(self, "speed_drag", True), add="+")
        self.speed_scale.bind("<ButtonRelease-1>", self.speed_release, add="+")
        tk.Label(c, text="F5 start · F6 vision · F7 pause\nF8 snapshot · F9 record · Esc stop", justify="left",
                 font=("Segoe UI", 8), bg=PANEL, fg=MUTED).pack(anchor="w", pady=(8, 0))
        tk.Label(left, text="Software stop is not a safety function.\nUse the E-stop in an emergency.", justify="left",
                 font=("Segoe UI", 9), bg=BG, fg=AMBER).pack(anchor="w", pady=(10, 0))

        wf = self.card(body, "WORKFLOW", grid=(0, 1))
        self.step_w = []
        grid = tk.Frame(wf, bg=PANEL)
        grid.pack(fill="x")
        for i, name in enumerate(STEPS):
            icon = tk.Label(grid, text="○", font=("Segoe UI", 12), bg=PANEL, fg=MUTED, width=2)
            txt = tk.Label(grid, text=name, font=("Segoe UI", 10), bg=PANEL, fg=MUTED, anchor="w")
            icon.grid(row=i % 5, column=(i // 5) * 2, sticky="w")
            txt.grid(row=i % 5, column=(i // 5) * 2 + 1, sticky="w", padx=(0, 30))
            self.step_w.append((icon, txt))
        self.prog = ttk.Progressbar(wf, style="G.Horizontal.TProgressbar", maximum=len(STEPS))
        self.prog.pack(fill="x", pady=(10, 2))
        self.headline = tk.Label(wf, text="Idle", font=("Segoe UI", 12, "bold"), bg=PANEL, fg=FG, anchor="w")
        self.headline.pack(fill="x")

        # camera card: main_tray.py's OpenCV window gets placed inside cam_frame
        rightcol = tk.Frame(body, bg=BG)
        rightcol.grid(row=0, column=2, rowspan=2, sticky="nsew", padx=(12, 0))
        cc = self.card(rightcol, "CAMERA")
        cc.pack_configure(fill="both", expand=True)
        self.cam_frame = tk.Frame(cc, bg="black", width=460, height=345)
        self.cam_frame.pack(fill="both", expand=True)
        self.cam_frame.pack_propagate(False)
        self.cam_frame.bind("<Configure>", self.fit_camera)
        self.cam_note = tk.Label(self.cam_frame, text="Camera view appears here after vision starts",
                                 bg="black", fg=MUTED, font=("Segoe UI", 11))
        self.cam_note.place(relx=0.5, rely=0.5, anchor="center")
        camtools = tk.Frame(cc, bg=PANEL)
        camtools.pack(fill="x", pady=(8, 0))
        self.b_snap = tk.Button(camtools, text="◉  Snapshot", command=self.snapshot, bg=PANEL2, fg="white",
                                activebackground=PANEL2, activeforeground="white", relief="flat", bd=0, cursor="hand2",
                                font=("Segoe UI", 10, "bold"), padx=12, pady=5)
        self.b_snap.pack(side="left", padx=(0, 8))
        self.b_rec = tk.Button(camtools, text="●  Record", command=self.toggle_record, bg=PANEL2, fg="white",
                               activebackground=PANEL2, activeforeground="white", relief="flat", bd=0, cursor="hand2",
                               font=("Segoe UI", 10, "bold"), padx=12, pady=5)
        self.b_rec.pack(side="left")

        ac = self.card(rightcol, "ARMS  (schematic, from joint angles)")
        cv_row = tk.Frame(ac, bg=PANEL)
        cv_row.pack(fill="x")
        self.arm_front = tk.Canvas(cv_row, width=150, height=190, bg=PANEL2, highlightthickness=0)
        self.arm_side = tk.Canvas(cv_row, width=150, height=190, bg=PANEL2, highlightthickness=0)
        self.arm_front.pack(side="left", fill="both", expand=True, padx=(0, 4))
        self.arm_side.pack(side="left", fill="both", expand=True, padx=(4, 0))
        self.arm_front.bind("<Configure>", lambda e: self.draw_arms())
        self.arm_side.bind("<Configure>", lambda e: self.draw_arms())
        tk.Label(ac, text="deg " + "".join(f"{'j' + str(i):>5}" for i in range(1, 8)), font=("Consolas", 9),
                 bg=PANEL, fg=MUTED, anchor="w").pack(fill="x", pady=(6, 0))
        self.joint_lbl = {}
        for u in MECH_UNITS:
            self.joint_lbl[u] = tk.Label(ac, text=f"{u[-1]}   —", font=("Consolas", 9), bg=PANEL,
                                         fg=ARM_COLORS[u], anchor="w")
            self.joint_lbl[u].pack(fill="x")

        lower = tk.Frame(body, bg=BG)
        lower.grid(row=1, column=1, sticky="nsew", pady=(12, 0))
        lower.columnconfigure(0, weight=3)
        lower.columnconfigure(1, weight=2)
        lower.rowconfigure(0, weight=1)
        lc = self.card(lower, "LIVE LOG", grid=(0, 0), fill=True)
        self.logbox = self.textbox(lc)
        ec = self.card(lower, "CONTROLLER EVENT LOG", grid=(0, 1), fill=True, padx=(10, 0))
        self.elogbox = self.textbox(ec)
        for box in (self.logbox, self.elogbox):
            for tag, col in (("info", FG), ("ok", GREEN), ("warn", AMBER), ("err", RED), ("dim", MUTED), ("cyan", CYAN)):
                box.tag_config(tag, foreground=col)
        self.refresh_buttons()

    def button(self, parent, text, color, cmd, w, small=False):
        b = tk.Button(parent, text=text, command=cmd, bg=color, fg="white", activebackground=color, activeforeground="white",
                      relief="flat", bd=0, cursor="hand2", width=w, font=("Segoe UI", 10 if small else 13, "bold"),
                      pady=6 if small else 12)
        b.pack(side="left", padx=(0, 8))
        b.bind("<Enter>", lambda e: b.config(bg=self.shade(color)) if str(b["state"]) == "normal" else None)
        b.bind("<Leave>", lambda e: b.config(bg=color))
        return b

    def wide_button(self, parent, text, color, cmd):
        b = tk.Button(parent, text=text, command=cmd, bg=color, fg="white", activebackground=color,
                      activeforeground="white", relief="flat", bd=0, cursor="hand2",
                      font=("Segoe UI", 9, "bold"), pady=6)
        b.pack(fill="x", pady=(0, 6))
        b.bind("<Enter>", lambda e: b.config(bg=self.shade(color)))
        b.bind("<Leave>", lambda e: b.config(bg=color))
        return b

    @staticmethod
    def shade(h):
        r, g, b = (int(h[i:i + 2], 16) for i in (1, 3, 5))
        return "#%02x%02x%02x" % tuple(min(255, int(v * 1.18) + 8) for v in (r, g, b))

    def card(self, parent, title, grid=None, fill=False, padx=0):
        f = tk.Frame(parent, bg=PANEL, padx=14, pady=10)
        if grid:
            f.grid(row=grid[0], column=grid[1], sticky="nsew", padx=padx)
        else:
            f.pack(fill="x", pady=(0, 10))
        tk.Label(f, text=title, font=("Segoe UI", 9, "bold"), bg=PANEL, fg=MUTED).pack(anchor="w", pady=(0, 6))
        if fill:
            f.pack_propagate(False)
        return f

    def row(self, card, name):
        f = tk.Frame(card, bg=PANEL)
        f.pack(fill="x", pady=2)
        tk.Label(f, text=name, font=("Segoe UI", 10), bg=PANEL, fg=MUTED).pack(side="left")
        v = tk.Label(f, text="—", font=("Consolas", 11, "bold"), bg=PANEL, fg=MUTED)
        v.pack(side="right")
        return v

    def textbox(self, parent):
        box = tk.Text(parent, bg=PANEL2, fg=FG, relief="flat", font=("Consolas", 9), wrap="word", state="disabled",
                      insertbackground=FG, padx=8, pady=6)
        sb = tk.Scrollbar(parent, command=box.yview)
        box.config(yscrollcommand=sb.set)
        sb.pack(side="right", fill="y")
        box.pack(fill="both", expand=True)
        return box

    def text_clear(self, box):
        box.config(state="normal")
        box.delete("1.0", "end")
        box.config(state="disabled")

    def text_add(self, box, line, tag="info"):
        box.config(state="normal")
        box.insert("end", line + "\n", tag)
        if int(box.index("end-1c").split(".")[0]) > 1500:
            box.delete("1.0", "200.0")
        box.see("end")
        box.config(state="disabled")

    # ------------------------------ embedded camera (GUI thread only) ------------------------------
    def fit_camera(self, _event=None):
        if self.cam_hwnd and user32 is not None:
            user32.MoveWindow(self.cam_hwnd, 0, 0, self.cam_frame.winfo_width(), self.cam_frame.winfo_height(), True)

    def embed_camera(self, hwnd):
        user32.SetParent(hwnd, self.cam_frame.winfo_id())
        style = user32.GetWindowLongW(hwnd, GWL_STYLE)
        style &= ~(WS_CAPTION | WS_THICKFRAME | WS_POPUP | WS_SYSMENU | WS_MINIMIZEBOX | WS_MAXIMIZEBOX)
        style |= WS_CHILD | WS_VISIBLE
        user32.SetWindowLongW(hwnd, GWL_STYLE, style)
        self.cam_hwnd = hwnd
        self.cam_note.place_forget()
        self.fit_camera()
        self.log("Camera window embedded in panel", "ok")

    def poll_camera(self, token, deadline):
        if token != self.cam_token:
            return                                   # a newer launch took over
        p = self.vision
        if p is None or p.poll() is not None:
            self.on_camreset("main_tray.py exited - see yumi_logs/main_tray.log")
            return
        hwnd = find_window_for_pid(p.pid)
        if hwnd:
            self.embed_camera(hwnd)
            return
        if time.time() > deadline:
            self.cam_note.config(text='Camera window not found.\nCheck CAMERA_WINDOW_TITLE_HINT in the script.')
            return
        self.root.after(300, self.poll_camera, token, deadline)

    def on_embed(self, deadline):
        self.cam_token += 1
        self.cam_hwnd = None
        self.cam_note.config(text="Waiting for camera window…")
        self.cam_note.place(relx=0.5, rely=0.5, anchor="center")
        self.poll_camera(self.cam_token, deadline)

    def on_camreset(self, note):
        self.cam_token += 1
        self.cam_hwnd = None
        if self.recording:
            self.recording = False
            self.b_rec.config(text="●  Record", bg=PANEL2)
        self.cam_note.config(text=note or "Camera view appears here after vision starts")
        self.cam_note.place(relx=0.5, rely=0.5, anchor="center")

    # ------------------------------ thread-safe helpers ------------------------------
    def log(self, msg, level="info"):
        self.q.put(("log", (msg, level)))

    def ask(self, title, msg):
        ev, res = threading.Event(), []
        self.q.put(("ask", (title, msg, ev, res)))
        ev.wait()
        return res[0] if res else False

    def mark(self, i, state):
        self.cur = i
        self.q.put(("step", (i, state)))

    def chk(self):
        if self.cancel.is_set():
            raise Cancelled()

    def beep(self, ok):
        if winsound:
            threading.Thread(target=lambda: winsound.Beep(1100 if ok else 400, 250 if ok else 600), daemon=True).start()

    # ------------------------------ GUI queue / timers ------------------------------
    def drain(self):
        try:
            while True:
                kind, p = self.q.get_nowait()
                getattr(self, "on_" + kind)(p)
        except queue.Empty:
            pass
        if not self.quit.is_set():
            self.root.after(80, self.drain)

    def on_log(self, p):
        msg, level = p
        line = f"{datetime.datetime.now():%H:%M:%S}  {msg}"
        self.text_add(self.logbox, line, level)
        with open(self.logfile, "a", encoding="utf-8") as f:
            f.write(line + "\n")

    def on_ask(self, p):
        title, msg, ev, res = p
        res.append(messagebox.askokcancel(title, msg))
        ev.set()

    def on_step(self, p):
        i, state = p
        self.step_state[i] = state
        done = sum(s in ("ok", "skip") for s in self.step_state)
        self.prog["value"] = done
        self.prog.configure(style="G.Horizontal.TProgressbar")
        if state == "run":
            self.headline.config(text=STEPS[i] + " …", fg=CYAN)
        self.paint_steps()

    def paint_steps(self):
        col = {"pending": MUTED, "run": CYAN, "ok": GREEN, "skip": MUTED, "fail": RED}
        ico = {"pending": "○", "ok": "✔", "skip": "–", "fail": "✖"}
        for (icon, txt), s in zip(self.step_w, self.step_state):
            icon.config(text=ico.get(s, "◐◓◑◒"[self.spin % 4]), fg=col[s])
            txt.config(fg=FG if s in ("run", "ok", "fail") else MUTED)

    def on_busy(self, p):
        self.busy = p
        self.refresh_buttons()

    def on_headline(self, p):
        self.headline.config(text=p[0], fg=p[1])

    def on_error(self, msg):
        self.beep(False)
        messagebox.showerror("Cell start failed", msg)

    def on_elog(self, entries):
        self.text_clear(self.elogbox)
        for e in reversed(entries):
            tag = {"1": "dim", "2": "warn", "3": "err"}.get(str(e.get("msgtype")), "info")
            self.text_add(self.elogbox, f"{str(e.get('tstamp', '')).replace(' T ', ' ')[11:]} [{e.get('code')}] {e.get('title')}", tag)

    def on_poll(self, d):
        if not d["ok"]:
            self.conn.config(text="● offline", fg=RED)
            for k in ("Mode", "Motors", "RAPID", "Mastership"):
                self.val[k].config(text="—", fg=MUTED)
            return
        self.conn.config(text="● online", fg=GREEN)
        self.val["Mode"].config(text=d["mode"], fg=GREEN if d["mode"] == "AUTO" else AMBER)
        self.val["Motors"].config(text=d["motors"].upper(), fg=GREEN if d["motors"] == "motoron" else AMBER)
        running = d["rapid"] == "running"
        self.val["RAPID"].config(text=d["rapid"].upper(), fg=GREEN if running else MUTED)
        m = d["msh"]
        held = {k for k, v in m.items() if v != "nomaster"}
        self.val["Mastership"].config(text=("held: " + ",".join(sorted(held))) if held else "free", fg=AMBER if held else MUTED)
        if running and self.run_t0 is None:
            self.run_t0 = time.time()
        if not running:
            self.run_t0 = None
        for name, lab in self.task_lbl.items():
            s = d["tasks"].get(name, "?").lower()
            lab.config(text="● RUNNING" if s.startswith("star") else "● stopped" if s.startswith("stop") else f"● {s}",
                       fg=GREEN if s.startswith("star") else MUTED if s.startswith("stop") else AMBER)
        self.b_pause.config(text="‖  PAUSE" if running else "▶  RESUME")

    def tick(self):
        self.spin += 1
        if "run" in self.step_state:
            self.paint_steps()
        if self.spin % 4 == 0:
            self.clock.config(text=f"{datetime.datetime.now():%Y-%m-%d  %H:%M:%S}")
            self.val["Running for"].config(
                text=str(datetime.timedelta(seconds=int(time.time() - self.run_t0))) if self.run_t0 else "—",
                fg=GREEN if self.run_t0 else MUTED)
            alive = self.vision is not None and self.vision.poll() is None
            self.val["Process"].config(text=f"RUNNING  pid {self.vision.pid}" if alive else "not running", fg=GREEN if alive else MUTED)
            if self.cam_hwnd and not alive:          # e.g. 'q' pressed in the camera window
                self.on_camreset("main_tray.py exited - see yumi_logs/main_tray.log")
            if self.recording:
                t = int(time.time() - self.rec_t0)
                self.b_rec.config(text=f"■  Stop  {t // 60:02d}:{t % 60:02d}")
            if self.web_srv:
                self.web_cache = self.build_web_cache()
        if not self.quit.is_set():
            self.root.after(250, self.tick)

    def refresh_buttons(self):
        self.b_start.config(state="disabled" if self.busy else "normal", bg=PANEL2 if self.busy else GREEN)

    def poll_loop(self):
        y, n = Yumi(), 0
        while not self.quit.is_set():
            d = {"ok": False}
            try:
                d.update(ok=True, mode=y.opmode(), motors=y.ctrlstate(), rapid=y.execstate(),
                         tasks={t["name"]: t.get("excstate", "?") for t in y.tasks()}, msh=y.mastership())
                if n % 3 == 0:
                    self.q.put(("elog", y.elog()))
            except Exception:
                pass
            self.q.put(("poll", d))
            n += 1
            time.sleep(1.2)

    # ------------------------------ actions ------------------------------
    def start_cell(self):
        if self.busy:
            return
        self.model_choice = self.model_var.get()      # read on the GUI thread
        self.cancel.clear()
        self.step_state = ["pending"] * len(STEPS)
        self.paint_steps()
        self.prog["value"] = 0
        self.on_busy(True)
        threading.Thread(target=self.workflow, daemon=True).start()

    def stop_cell(self):
        self.cancel.set()
        self.log("STOP requested", "warn")
        self.on_headline(("Stopping …", AMBER))
        threading.Thread(target=self.stop_worker, args=(self.motors_off.get(),), daemon=True).start()

    def stop_worker(self, motors_off):
        y = Yumi()
        try:
            self.kill_vision()
            y.stop_rapid()
            for _ in range(15):
                if y.execstate() != "running":
                    break
                time.sleep(0.3)
            self.log("RAPID stopped", "ok")
            if motors_off:
                y.post("/rw/mastership?action=request")
                r = y.post("/rw/panel/ctrlstate?action=setctrlstate", {"ctrl-state": "motoroff"})
                y.release()
                self.log(f"Motors OFF requested ({r.status_code})", "ok")
            self.q.put(("headline", ("Stopped", AMBER)))
        except Exception as e:
            self.log(f"Stop error: {e}", "err")

    def stop_vision_click(self):
        threading.Thread(target=self.kill_vision, daemon=True).start()

    def kill_vision(self):
        if os.path.exists(REC_FLAG):                 # let main_tray.py close the video file first
            try:
                os.remove(REC_FLAG)
            except OSError:
                pass
            time.sleep(0.7)
        p = self.vision
        if p is not None and p.poll() is None:
            p.terminate()
            try:
                p.wait(3)
            except subprocess.TimeoutExpired:
                p.kill()
            self.log("Vision (main_tray.py) stopped", "warn")
        self.vision = None
        if self.vision_log:
            try:
                self.vision_log.close()
            except Exception:
                pass
            self.vision_log = None
        self.q.put(("camreset", None))

    def save_log(self):
        path = filedialog.asksaveasfilename(defaultextension=".txt", initialfile="yumi_log.txt")
        if path:
            with open(path, "w", encoding="utf-8") as f:
                f.write(self.logbox.get("1.0", "end"))

    def on_close(self):
        embedded = self.cam_hwnd is not None
        msg = ("Close the control panel?\nThe camera view lives in this window, so vision will be stopped. "
               "The robot keeps running." if embedded else
               "Close the control panel?\nRobot and vision keep running.")
        if messagebox.askokcancel("Close", msg):
            if embedded:
                self.kill_vision()
            self.quit.set()
            self.root.destroy()

    # ------------------------------ extra controls ------------------------------
    def joint_loop(self):
        y, n = Yumi(), 0
        while not self.quit.is_set():
            data = {}
            for u in MECH_UNITS:
                try:
                    data[u] = y.joints(u)
                except Exception:
                    pass
            if data:
                self.q.put(("joints", data))
            if n % 4 == 0:
                try:
                    self.q.put(("speed", int(float(y.field("/rw/panel/speedratio", "speedratio")))))
                except Exception:
                    pass
            n += 1
            time.sleep(0.5)

    def on_joints(self, data):
        self.joint_data = data
        for u, vals in data.items():
            self.joint_lbl[u].config(text=f"{u[-1]}   " + "".join(f"{v:5.0f}" for v in vals))
        self.draw_arms()

    def on_speed(self, v):
        if self.speed_drag or time.time() < self.speed_hold:
            return
        v = max(5, min(100, v))
        self.speed_var.set(v)
        self.speed_lbl.config(text=f"Speed  {v} %", fg=FG)

    def draw_arms(self):
        for canvas, view in ((self.arm_front, "front"), (self.arm_side, "side")):
            w, h = canvas.winfo_width(), canvas.winfo_height()
            if w < 40 or h < 40:
                continue
            canvas.delete("all")
            s = min((w - 12) / 1.3, (h - 14) / 1.15)
            ox = w / 2 if view == "front" else w / 2 - 0.25 * s
            gy = h - 8

            def pt(p, view=view, ox=ox, s=s, gy=gy):
                x, y, z = p
                return (ox + (y if view == "front" else x) * s, gy - z * s)

            canvas.create_line(6, gy, w - 6, gy, fill=MUTED)
            canvas.create_text(6, 4, text=view, anchor="nw", fill=MUTED, font=("Segoe UI", 8))
            lb, rb = ARM_BASE[MECH_UNITS[0]], ARM_BASE[MECH_UNITS[1]]
            canvas.create_line(*pt(lb[:3]), *pt(rb[:3]), fill=MUTED, width=3)                # shoulder bar
            canvas.create_line(*pt((0.0, 0.0, 0.0)), *pt((0.0, 0.0, lb[2])), fill=MUTED, width=3)  # torso
            if not self.joint_data:
                canvas.create_text(w / 2, h / 2, text="no joint data", fill=MUTED)
                continue
            for unit in MECH_UNITS:
                if unit not in self.joint_data:
                    continue
                pts = [pt(q) for q in arm_points(unit, self.joint_data[unit])]
                col = ARM_COLORS[unit]
                canvas.create_line(*[c for xy in pts for c in xy], fill=col, width=4, capstyle="round", joinstyle="round")
                for i, (px, py) in enumerate(pts):
                    r = 5 if i == len(pts) - 1 else 3
                    canvas.create_oval(px - r, py - r, px + r, py + r, fill=col, outline="")

    def speed_release(self, _e=None):
        self.speed_drag = False
        self.speed_hold = time.time() + 3            # ignore polled values until the new one has been applied
        self.root.after(80, lambda: threading.Thread(target=self.set_speed, args=(int(self.speed_var.get()),),
                                                     daemon=True).start())

    def set_speed(self, pct):
        y = Yumi()
        try:
            data = {"speed-ratio": str(pct)}
            r = y.post("/rw/panel/speedratio?action=setspeedratio", data)
            if r.status_code not in (200, 204):
                self.log(f"Speed: plain request rejected ({r.status_code}), retrying with mastership", "dim")
                held = False
                try:
                    y.post("/rw/mastership?action=request")
                    held = True
                    r = y.post("/rw/panel/speedratio?action=setspeedratio", data)
                finally:
                    if held:
                        y.release()
            if r.status_code in (200, 204):
                self.log(f"Speed ratio set to {pct} %", "ok")
            else:
                self.log(f"Speed change rejected ({r.status_code}): {r.text[:150]}", "err")
        except Exception as e:
            self.log(f"Speed error: {e}", "err")

    def pause_resume(self):
        if self.busy:
            self.log("Start sequence is still running - wait for it to finish", "warn")
            return
        threading.Thread(target=self.pause_worker, daemon=True).start()

    def pause_worker(self):
        y = Yumi()
        try:
            if y.execstate() == "running":
                self.log("PAUSE: stopping RAPID …", "warn")
                y.stop_rapid()
                for _ in range(15):
                    if y.execstate() != "running":
                        break
                    time.sleep(0.3)
                self.log("RAPID paused - press RESUME (F7) to continue", "warn")
                self.q.put(("headline", ("Paused", AMBER)))
                return
            self.log("RESUME: continuing RAPID …", "cyan")
            running = False
            for regain in ("continue", "regain"):
                r = y.start_rapid(regain, "false")
                if r.status_code not in (200, 204):
                    self.log(f"Resume rejected ({r.status_code}): {r.text[:150]}", "err")
                    return
                end = time.time() + 5
                while time.time() < end and not running:
                    time.sleep(0.4)
                    running = y.execstate() == "running"
                if running:
                    break
            if running:
                self.log("RAPID resumed", "ok")
                self.q.put(("headline", ("Running", GREEN)))
            else:
                self.log("RAPID did not resume - check the event log / mastership", "err")
        except Exception as e:
            self.log(f"Pause/resume error: {e}", "err")

    def move_home(self):
        if self.busy:
            self.log("Start sequence is still running - wait for it to finish", "warn")
            return
        if self.run_t0 is None:
            messagebox.showinfo("Move home", "RAPID is not running, so the robot cannot take commands.\n"
                                "Start the cell first (F5) or press Resume (F7).")
            return
        alive = self.vision is not None and self.vision.poll() is None
        msg = "Send the robot to its home position?\n\nMake sure the workspace is clear."
        if alive:
            msg += ("\n\nVision (main_tray.py) will be stopped first because it holds the robot connection. "
                    "Restart it afterwards with F6.")
        if messagebox.askokcancel("Move home", msg):
            threading.Thread(target=self.home_worker, daemon=True).start()

    def home_worker(self):
        try:
            if self.vision is not None and self.vision.poll() is None:
                self.kill_vision()
                time.sleep(1.0)                      # let the RAPID socket server accept a new client
            self.log("Moving home …", "cyan")
            code = (
                "import sys, time\n"
                "from updated_communication import Communication\n"
                "c = Communication()\n"
                "if not c.connectV2():\n"
                "    print('could not connect to the RAPID socket server')\n"
                "    sys.exit(2)\n"
                "c.MoveHome()\n"
                "time.sleep(1.0)\n"
                "print('MoveHome sent')\n"
            )
            flags = subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0
            r = subprocess.run([sys.executable, "-c", code], cwd=HERE, capture_output=True, text=True,
                               timeout=HOME_TIMEOUT, creationflags=flags)
            for line in (r.stdout + r.stderr).strip().splitlines()[-6:]:
                self.log("  " + line, "dim")
            if r.returncode == 0:
                self.log("Home command finished", "ok")
            else:
                self.log(f"Home failed (exit {r.returncode}) - see lines above", "err")
        except subprocess.TimeoutExpired:
            self.log(f"Home timed out after {HOME_TIMEOUT} s", "err")
        except Exception as e:
            self.log(f"Home error: {e}", "err")

    def restart_vision(self):
        if self.busy:
            self.log("Start sequence is still running - wait for it to finish", "warn")
            return
        if self.run_t0 is None:
            self.log("RAPID is not running - use START CELL (F5) first", "warn")
            return
        self.model_choice = self.model_var.get()
        self.log(f"Restarting vision (model {self.model_choice}) …", "cyan")
        threading.Thread(target=self.restart_vision_worker, daemon=True).start()

    def restart_vision_worker(self):
        try:
            self.launch_vision()
        except Exception as e:
            self.log(f"Vision restart failed: {e}", "err")

    def snapshot(self):
        if self.vision is None or self.vision.poll() is not None:
            self.log("Snapshot: vision is not running", "warn")
            return
        t0 = time.time()
        open(SNAP_FLAG, "w").close()
        self.root.after(1500, lambda: self.snap_report(t0))

    def snap_report(self, t0):
        files = [os.path.join(CAPTURE_DIR, f) for f in os.listdir(CAPTURE_DIR) if f.startswith("snap_")]
        newest = max(files, key=os.path.getmtime, default=None)
        if newest and os.path.getmtime(newest) >= t0 - 1:
            self.log(f"Snapshot saved: {os.path.basename(newest)}  (folder yumi_captures)", "ok")
        else:
            try:
                os.remove(SNAP_FLAG)
            except OSError:
                pass
            self.log("No snapshot appeared - is yumi_capture.handle(frame) added to main_tray.py?", "err")

    def toggle_record(self):
        if not self.recording:
            if self.vision is None or self.vision.poll() is not None:
                self.log("Record: vision is not running", "warn")
                return
            t0 = time.time()
            open(REC_FLAG, "w").close()
            self.recording, self.rec_t0 = True, t0
            self.b_rec.config(text="■  Stop  00:00", bg=RED)
            self.log("Recording started (folder yumi_captures)", "ok")
            self.root.after(2500, lambda: self.rec_report(t0))
        else:
            try:
                os.remove(REC_FLAG)
            except OSError:
                pass
            self.recording = False
            self.b_rec.config(text="●  Record", bg=PANEL2)
            self.log("Recording stopped", "ok")

    def rec_report(self, t0):
        if not self.recording:
            return
        files = [os.path.join(CAPTURE_DIR, f) for f in os.listdir(CAPTURE_DIR) if f.startswith("rec_")]
        if not any(os.path.getmtime(f) >= t0 - 1 for f in files):
            self.log("No video file appeared - is yumi_capture.handle(frame) added to main_tray.py?", "err")

    # ------------------------------ phone web page ------------------------------
    def start_web(self):
        try:
            self.web_token = load_web_token()
            WebHandler.app = self
            srv = ThreadingHTTPServer(("0.0.0.0", WEB_PORT), WebHandler)
            srv.daemon_threads = True
            threading.Thread(target=srv.serve_forever, daemon=True).start()
            self.web_srv = srv
            urls = self.web_urls()
            for u in urls or [f"http://<PC-IP>:{WEB_PORT}/?t={self.web_token}"]:
                self.log(f"Phone page: {u}", "cyan")
        except OSError as e:
            self.log(f"Phone page could not start ({e}) - is port {WEB_PORT} already in use?", "err")

    def web_urls(self):
        return [f"http://{ip}:{WEB_PORT}/?t={self.web_token}" for ip in local_ips()]

    def show_web_link(self):
        if not self.web_srv:
            messagebox.showinfo("Phone page", "The phone page is not running (WEB_ENABLED = False, or the port was busy).")
            return
        urls = self.web_urls() or [f"http://<PC-IP>:{WEB_PORT}/?t={self.web_token}"]
        self.root.clipboard_clear()
        self.root.clipboard_append(urls[0])
        messagebox.showinfo("Phone page",
                            "Open this on a phone on the same Wi-Fi:\n\n" + "\n".join(urls) +
                            "\n\n(The first link was copied to the clipboard.)\n"
                            "Anyone with this link can see the camera and press STOP.")

    def build_web_cache(self):
        """Snapshot of what the phone page shows. Runs on the GUI thread; the web threads only read the result."""
        return {
            "t": time.time(),
            "conn": self.conn.cget("text"),
            "headline": self.headline.cget("text"),
            "headline_color": str(self.headline.cget("fg")),
            "values": {k: self.val[k].cget("text") for k in ("Mode", "Motors", "RAPID", "Mastership", "Running for", "Process")},
            "tasks": {k: lab.cget("text") for k, lab in self.task_lbl.items()},
            "steps": [{"name": STEPS[i], "state": self.step_state[i]} for i in range(len(STEPS))],
            "speed": self.speed_var.get(),
            "joints": self.joint_data,
            "recording": self.recording,
            "log": self.logbox.get("1.0", "end").strip().splitlines()[-25:],
        }

    def on_webstop(self, _p):
        self.log("STOP requested from the phone page", "warn")
        self.stop_cell()

    # ------------------------------ workflow ------------------------------
    def workflow(self):
        y, held = Yumi(), False
        try:
            self.log("=== START CELL ===", "cyan")
            self.mark(0, "run")
            for _ in range(5):
                try:
                    y.opmode()
                    break
                except requests.RequestException:
                    self.chk()
                    time.sleep(1)
            else:
                raise RuntimeError("Controller not reachable. Check the service port cable / IP.")
            self.mark(0, "ok")

            self.mark(1, "run")
            mode = y.opmode()
            if mode.upper() != "AUTO":
                raise RuntimeError(f"Controller is in '{mode}', not AUTO. Turn the mode switch to Auto.")
            self.mark(1, "ok")

            self.chk()
            self.mark(2, "run")
            if y.execstate() == "running":
                y.stop_rapid()
                for _ in range(15):
                    if y.execstate() != "running":
                        break
                    time.sleep(0.3)
                self.mark(2, "ok")
            else:
                self.mark(2, "skip")

            self.chk()
            self.mark(3, "run")
            m = y.mastership()
            if any(v != "nomaster" for v in m.values()):
                raise RuntimeError(f"Mastership already held: {m}  (FlexPendant / RobotStudio / stale session)")
            y.post("/rw/mastership?action=request")
            held = True
            m = y.mastership()
            if any(v != "remote" for v in m.values()):
                raise RuntimeError(f"Mastership not granted: {m}")
            self.mark(3, "ok")

            self.chk()
            self.mark(4, "run")
            if y.ctrlstate() == "motoron":
                self.mark(4, "skip")
            else:
                r = y.post("/rw/panel/ctrlstate?action=setctrlstate", {"ctrl-state": "motoron"})
                if r.status_code not in (200, 204):
                    raise RuntimeError(f"Motors-on rejected ({r.status_code}).")
                for _ in range(8):
                    time.sleep(0.5)
                    if y.ctrlstate() == "motoron":
                        break
                else:
                    raise RuntimeError("Motors did not turn on. Check E-stop, enabling device, gates.")
                self.mark(4, "ok")

            self.chk()
            self.mark(5, "run")
            pp = y.pp()
            self.log(f"Program pointers: {pp}", "dim")
            if any(r != "main" for r in pp.values()):
                r = y.post("/rw/rapid/execution?action=resetpp")
                if r.status_code not in (200, 204):
                    raise RuntimeError(f"Pointer reset rejected ({r.status_code}).")
                time.sleep(3)
                self.mark(5, "ok")
            else:
                self.mark(5, "skip")

            self.mark(6, "run")
            y.release()
            held = False
            time.sleep(0.5)
            self.mark(6, "ok")

            self.chk()
            self.mark(7, "run")
            running = False
            for n, (regain, tsp) in enumerate([("continue", "false"), ("clear", "true"), ("regain", "false")], 1):
                self.log(f"Start attempt {n}/3 (regain={regain})")
                r = y.start_rapid(regain, tsp)
                if r.status_code not in (200, 204):
                    raise RuntimeError(f"Start rejected ({r.status_code}): {r.text[:200]}")
                end = time.time() + 6
                while time.time() < end and not running:
                    self.chk()
                    time.sleep(0.4)
                    running = y.execstate() == "running"
                if running:
                    break
            if not running:
                if not self.ask("Manual start needed", "The remote start was accepted but RAPID did not run.\n\n"
                                "Press START on the FlexPendant (Auto, motors on), then click OK.\nWaiting up to 60 s."):
                    raise Cancelled()
                end = time.time() + 60
                while time.time() < end and not running:
                    self.chk()
                    time.sleep(0.5)
                    running = y.execstate() == "running"
            if not running:
                raise RuntimeError("RAPID did not start (remote and manual start failed).")
            self.mark(7, "ok")

            self.mark(8, "run")
            for _ in range(24):
                self.chk()
                act = [t for t in y.tasks() if t.get("active", "").lower() == "on"]
                if act and all(t.get("excstate", "").lower().startswith("star") for t in act):
                    break
                time.sleep(0.5)
            else:
                bad = [f"{t['name']} ({t.get('excstate')})" for t in act if not t.get("excstate", "").lower().startswith("star")]
                raise RuntimeError("RAPID runs but these tasks did not start: " + ", ".join(bad))
            self.mark(8, "ok")

            self.chk()
            self.mark(9, "run")
            self.launch_vision()
            self.mark(9, "ok")
            self.log("=== CELL RUNNING ===", "ok")
            self.q.put(("headline", ("Cell running — robot + vision active", GREEN)))
            self.beep(True)
        except Cancelled:
            self.mark(self.cur, "fail")
            self.log("Start cancelled", "warn")
            self.q.put(("headline", ("Cancelled", AMBER)))
        except Exception as e:
            self.mark(self.cur, "fail")
            self.log(f"FAILED at '{STEPS[self.cur]}': {e}", "err")
            try:
                for ent in reversed(y.elog(8)):
                    self.log(f"  elog [{ent.get('code')}] {ent.get('title')}", "dim")
            except Exception:
                pass
            self.q.put(("headline", ("Failed — see log", RED)))
            self.q.put(("error", f"Step: {STEPS[self.cur]}\n\n{e}"))
        finally:
            if held:
                y.release()
            self.q.put(("busy", False))

    def launch_vision(self):
        path = os.path.join(HERE, MAIN_TRAY_FILE)
        if not os.path.isfile(path):
            raise RuntimeError(f"{MAIN_TRAY_FILE} not found in {HERE}")
        self.kill_vision()
        for flag in (REC_FLAG, SNAP_FLAG):           # no stale capture requests from an earlier run
            try:
                os.remove(flag)
            except OSError:
                pass
        if AUTO_ANSWER_PROMPTS:
            answers = [DETECTION_MODELS[self.model_choice], SAVE_PROTOCOL_ANSWER]
            flags = subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0
            self.vision_log = open(os.path.join(LOG_DIR, "main_tray.log"), "w", encoding="utf-8")
            p = subprocess.Popen([sys.executable, "-u", path], cwd=HERE, stdin=subprocess.PIPE,
                                 stdout=self.vision_log, stderr=subprocess.STDOUT, text=True, creationflags=flags)
            for a in answers:
                p.stdin.write(a + "\n")
            p.stdin.flush()
            info = f"model {self.model_choice}, inputs {answers}, console hidden"
        else:
            flags = subprocess.CREATE_NEW_CONSOLE if os.name == "nt" else 0
            p = subprocess.Popen([sys.executable, path], cwd=HERE, creationflags=flags)
            info = "manual inputs, own console"
        self.vision = p
        self.log(f"main_tray.py started (pid {p.pid}): {info}", "ok")
        if EMBED_CAMERA and user32 is not None:
            self.q.put(("embed", time.time() + EMBED_TIMEOUT))


if __name__ == "__main__":
    root = tk.Tk()
    App(root)
    root.mainloop()
