"""
YuMi Cell Control - control panel for the YuMi (RobotWare 6.16) + vision cell.
Keeps the proven start sequence: mastership only for motors-on / pointer reset,
RELEASED before the start command (holding it makes the start a silent no-op).
"""
import datetime, json, os, queue, re, subprocess, sys, threading, time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk
import requests
from requests.auth import HTTPDigestAuth
try:
    import winsound
except ImportError:
    winsound = None

# ------------------------------ CONFIG ------------------------------
CONTROLLER_URL = "http://192.168.125.1"
AUTH = HTTPDigestAuth("Zebastian", "robotics")
HEADERS = {"Content-Type": "application/x-www-form-urlencoded;v=2.0", "Accept": "application/json"}
MAIN_TRAY_FILE = "main_tray.py"
MAIN_TRAY_INPUTS = ["5s", "n"]       # typed automatically into main_tray.py; [] = type yourself
TIMEOUT = 5
HERE = os.path.dirname(os.path.abspath(__file__))
LOG_DIR = os.path.join(HERE, "yumi_logs")

BG, PANEL, PANEL2, FG, MUTED = "#0d1117", "#161b22", "#1f2630", "#e6edf3", "#7d8590"
GREEN, RED, AMBER, BLUE, CYAN = "#2ea043", "#da3633", "#d29922", "#388bfd", "#39c5cf"
STEPS = ["Connect to controller", "Check AUTO mode", "Stop RAPID", "Take mastership",
         "Motors on", "Reset program pointers", "Release mastership",
         "Start RAPID", "Verify all tasks", "Launch vision"]


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
        self.step_state = ["pending"] * len(STEPS)
        os.makedirs(LOG_DIR, exist_ok=True)
        self.logfile = os.path.join(LOG_DIR, datetime.datetime.now().strftime("cell_%Y%m%d_%H%M%S.log"))
        self.build()
        threading.Thread(target=self.poll_loop, daemon=True).start()
        root.after(80, self.drain)
        root.after(250, self.tick)
        root.bind("<Escape>", lambda e: self.stop_cell())
        root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.log("YuMi Cell Control ready. Esc = STOP.", "dim")

    # ------------------------------ UI ------------------------------
    def build(self):
        r = self.root
        r.title("YuMi Cell Control")
        r.geometry("1180x800")
        r.minsize(1000, 700)
        r.configure(bg=BG)
        st = ttk.Style()
        st.theme_use("clam")
        st.configure("G.Horizontal.TProgressbar", troughcolor=PANEL2, background=GREEN, bordercolor=PANEL, lightcolor=GREEN, darkcolor=GREEN)

        head = tk.Frame(r, bg=PANEL)
        head.pack(fill="x")
        tk.Label(head, text="YuMi Cell Control", font=("Segoe UI", 20, "bold"), bg=PANEL, fg=FG).pack(side="left", padx=20, pady=12)
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
        self.motors_off = tk.BooleanVar(value=False)
        tk.Checkbutton(bar, text="also motors OFF on stop", variable=self.motors_off, bg=BG, fg=MUTED, selectcolor=PANEL2,
                       activebackground=BG, activeforeground=FG, font=("Segoe UI", 9)).pack(side="right", padx=8)

        body = tk.Frame(r, bg=BG)
        body.pack(fill="both", expand=True, padx=16, pady=6)
        body.columnconfigure(0, weight=0, minsize=330)
        body.columnconfigure(1, weight=1)
        body.rowconfigure(0, weight=0)
        body.rowconfigure(1, weight=1)

        left = tk.Frame(body, bg=BG)
        left.grid(row=0, column=0, rowspan=2, sticky="ns", padx=(0, 12))
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
        self.val["Inputs"] = self.row(c, "Auto-inputs")
        self.val["Inputs"].config(text=", ".join(MAIN_TRAY_INPUTS) or "manual", fg=FG)
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
        p = self.vision
        if p is not None and p.poll() is None:
            p.terminate()
            try:
                p.wait(3)
            except subprocess.TimeoutExpired:
                p.kill()
            self.log("Vision (main_tray.py) stopped", "warn")
        self.vision = None

    def save_log(self):
        path = filedialog.asksaveasfilename(defaultextension=".txt", initialfile="yumi_log.txt")
        if path:
            with open(path, "w", encoding="utf-8") as f:
                f.write(self.logbox.get("1.0", "end"))

    def on_close(self):
        if messagebox.askokcancel("Close", "Close the control panel?\nRobot and vision keep running."):
            self.quit.set()
            self.root.destroy()

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
        flags = subprocess.CREATE_NEW_CONSOLE if os.name == "nt" else 0
        if MAIN_TRAY_INPUTS:
            p = subprocess.Popen([sys.executable, "-u", path], cwd=HERE, stdin=subprocess.PIPE, text=True, creationflags=flags)
            for a in MAIN_TRAY_INPUTS:
                p.stdin.write(a + "\n")
            p.stdin.flush()
        else:
            p = subprocess.Popen([sys.executable, path], cwd=HERE, creationflags=flags)
        self.vision = p
        self.log(f"main_tray.py started (pid {p.pid}) with inputs {MAIN_TRAY_INPUTS or 'manual'}", "ok")


if __name__ == "__main__":
    root = tk.Tk()
    App(root)
    root.mainloop()
