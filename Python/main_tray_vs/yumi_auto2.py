import os
import sys
import json
import re
import subprocess
import time
import tkinter as tk
from tkinter import messagebox
import requests
from requests.auth import HTTPDigestAuth

# --- PHYSICAL YUMI CONFIGURATION ---
CONTROLLER_URL = "http://192.168.125.1"
AUTH = HTTPDigestAuth("Zebastian", "robotics")

RWS_HEADERS = {
    "Content-Type": "application/x-www-form-urlencoded;v=2.0",
    "Accept": "application/json"
}

DO_RESETPP = True          # reset program pointers, but ONLY if a task is not already at main
CLEAR_ELOG = False         # clearing the event log is optional (off: keeps diagnostics visible)
RESETPP_SETTLE_SECONDS = 3.0
# Answers typed automatically into main_tray.py's prompts, in order (one per input()).
# Set to [] to launch main_tray.py interactively in its own console instead.
MAIN_TRAY_INPUTS = ["5s", "n"]
RUN_POLL_SECONDS = 20      # how long to wait for RAPID to reach 'running'
REQUEST_TIMEOUT = 5


# --------------------------------------------------------------------------
# Helpers
# --------------------------------------------------------------------------
_main_tray_proc = None  # keeps main_tray.py's stdin pipe alive


def norm(text):
    """Lowercase and strip whitespace so JSON checks are formatting-proof."""
    return "".join(text.lower().split())


def get_json_text(session, path):
    """GET a RWS resource as JSON text. Returns (status_code, text)."""
    sep = "&" if "?" in path else "?"
    r = session.get(f"{CONTROLLER_URL}{path}{sep}json=1", timeout=REQUEST_TIMEOUT)
    return r.status_code, r.text


def elog_summary(session, count=10):
    """Newest event log entries as compact one-liners (newest first)."""
    try:
        code, text = get_json_text(session, f"/rw/elog/0?lang=en&order=lifo&limit={count}")
        entries = json.loads(text)["_embedded"]["_state"]
        return "\n".join(
            f"  {e.get('tstamp','?')} [{e.get('code','?')}] {e.get('title','?')}"
            for e in entries
        ) or "  (empty)"
    except Exception as e:
        return f"  (could not read elog: {e})"


def get_tasks(session):
    """Return list of RAPID task dicts (name, type, taskstate, excstate, active, ...)."""
    code, text = get_json_text(session, "/rw/rapid/tasks")
    return json.loads(text)["_embedded"]["_state"]


def tasks_summary(session):
    try:
        return "\n".join(
            f"  {t.get('name','?')}: type={t.get('type','?')} taskstate={t.get('taskstate','?')} "
            f"excstate={t.get('excstate','?')} active={t.get('active','?')} "
            f"motiontask={t.get('motiontask','?')}"
            for t in get_tasks(session)
        )
    except Exception as e:
        return f"  (could not read tasks: {e})"


def mastership_states(session):
    """Return {'rapid': 'nomaster'|'local'|'remote', ...} for each domain."""
    out = {}
    for dom in ("rapid", "motion", "cfg"):
        try:
            st = get_json_text(session, f"/rw/mastership/{dom}")[1]
            import re
            m = re.search(r'"mastership"\s*:\s*"([^"]+)"', st)
            out[dom] = m.group(1) if m else "?"
        except Exception as e:
            out[dom] = f"error({e})"
    return out


def pp_routines(session):
    """{task: routine name at the program pointer}"""
    out = {}
    for t in get_tasks(session):
        tname = t.get("name")
        try:
            pcp = get_json_text(session, f"/rw/rapid/tasks/{tname}/pcp")[1]
            m = re.search(r'"routinename"\s*:\s*"([^"]*)"', pcp)
            out[tname] = m.group(1) if m else "?"
        except Exception as e:
            out[tname] = f"error({e})"
    return out


def dump_state(session):
    """Collect opmode, ctrlstate, execution state and latest event log entries."""
    lines = []
    for name, path in [
        ("opmode", "/rw/panel/opmode"),
        ("ctrlstate", "/rw/panel/ctrlstate"),
        ("execution", "/rw/rapid/execution"),
    ]:
        try:
            code, text = get_json_text(session, path)
            lines.append(f"{name}: {code} {text[:400]}")
        except Exception as e:
            lines.append(f"{name}: ERROR {e}")
    lines.append(f"mastership: {mastership_states(session)}")
    lines.append("tasks:\n" + tasks_summary(session))
    lines.append("elog (newest first):\n" + elog_summary(session))
    return "\n".join(lines)


def set_status(text, color="orange"):
    status_label.config(text=f"Status: {text}", fg=color)
    root.update()


# --------------------------------------------------------------------------
# Main workflow
# --------------------------------------------------------------------------
def run_automation_workflow():
    action_button.config(state="disabled")
    session = requests.Session()
    session.auth = AUTH
    session.headers.update(RWS_HEADERS)
    have_mastership = False

    try:
        set_status("Connecting to controller...")

        # 1. Verify controller reachability
        controller_online = False
        for _ in range(5):
            try:
                test_resp = session.get(CONTROLLER_URL, timeout=2)
                if test_resp.status_code < 500:
                    controller_online = True
                    break
            except requests.exceptions.RequestException:
                time.sleep(1.0)

        if not controller_online:
            raise Exception("Connection refused by controller. Check your service port connection.")

        # 2. Check operating mode (must be AUTO for remote motors-on / start)
        set_status("Checking operating mode...")
        code, text = get_json_text(session, "/rw/panel/opmode")
        print("OPMODE:", code, text)
        if "auto" not in norm(text):
            raise Exception(
                "Controller is not in AUTO mode. Turn the mode switch to Auto "
                "and acknowledge on the FlexPendant, then try again.\n\n"
                + dump_state(session)
            )

        # 3. Stop RAPID only if it is actually running
        _, ex0 = get_json_text(session, "/rw/rapid/execution")
        if '"ctrlexecstate":"running"' in norm(ex0):
            set_status("Stopping RAPID execution...")
            stop_resp = session.post(
                f"{CONTROLLER_URL}/rw/rapid/execution?action=stop",
                data={"stopmode": "stop", "usetsp": "alltsk"},
                timeout=REQUEST_TIMEOUT,
            )
            print("STOP:", stop_resp.status_code, stop_resp.text)
            time.sleep(0.5)
        else:
            print("STOP skipped (RAPID not running)")

        # 4. Request mastership
        set_status("Requesting Mastership...")
        before = mastership_states(session)
        print("MASTERSHIP before:", before)
        if any(before.get(d) != "nomaster" for d in ("rapid", "motion")):
            raise Exception(
                f"Mastership is already held: {before}\n"
                "'local' = FlexPendant holds it, 'remote' = another RWS client "
                "(RobotStudio, a stale session from a crashed run).\n"
                "Release it there, or wait for the old session to time out."
            )
        msh_resp = session.post(f"{CONTROLLER_URL}/rw/mastership?action=request",
                                timeout=REQUEST_TIMEOUT)
        print("MASTERSHIP request:", msh_resp.status_code, " ".join(msh_resp.text.split())[:200])
        have_mastership = True
        after = mastership_states(session)
        print("MASTERSHIP after:", after)
        # 'remote' after our request = held by us (this RWS client)
        if after.get("rapid") != "remote" or after.get("motion") != "remote":
            raise Exception(
                f"Mastership request was not granted. State: {after}\n"
                f"Response: {msh_resp.status_code} {msh_resp.text[:300]}"
            )
        have_mastership = True

        # 5. Motors on, then VERIFY it actually happened
        _, cs0 = get_json_text(session, "/rw/panel/ctrlstate")
        if '"ctrlstate":"motoron"' in norm(cs0):
            print("MOTORON skipped (motors already on)")
        else:
            set_status("Setting control state to Motor-On...")
            motor_resp = session.post(
                f"{CONTROLLER_URL}/rw/panel/ctrlstate?action=setctrlstate",
                data={"ctrl-state": "motoron"},
                timeout=REQUEST_TIMEOUT,
            )
            print("MOTORON:", motor_resp.status_code, motor_resp.text)
            if motor_resp.status_code not in (204, 200):
                raise Exception(
                    f"Failed to set control state to motoron. Status: {motor_resp.status_code}\n\n"
                    + motor_resp.text[:400]
                )

        motors_on = False
        last_cs = ""
        for _ in range(6):
            time.sleep(0.5)
            _, last_cs = get_json_text(session, "/rw/panel/ctrlstate")
            if '"ctrlstate":"motoron"' in norm(last_cs):
                motors_on = True
                break
        if not motors_on:
            raise Exception(
                "Motors did not turn on. Check E-stop, enabling device, "
                "gate / auto-stop circuits, then press the physical motors-on/reset button.\n\n"
                + dump_state(session)
            )

        # 6. Clear the event log (optional)
        if CLEAR_ELOG:
            set_status("Clearing controller event logs...")
            for clear_path in ("/rw/elog/0?action=clear", "/rw/elog?action=clearall"):
                try:
                    clr = session.post(f"{CONTROLLER_URL}{clear_path}", timeout=REQUEST_TIMEOUT)
                    print("CLEAR ELOG", clear_path, clr.status_code)
                    if clr.status_code in (200, 204):
                        break
                except requests.exceptions.RequestException as e:
                    print("CLEAR ELOG failed (non-fatal):", e)
            time.sleep(0.5)
        else:
            print("CLEAR ELOG skipped (CLEAR_ELOG = False)")

        # 7. Reset program pointers only if some task is not already at main
        pp = pp_routines(session)
        print("PP before:", pp)
        need_reset = any(r != "main" for r in pp.values())
        if DO_RESETPP and need_reset:
            set_status("Resetting Program Pointers...")
            reset_resp = session.post(
                f"{CONTROLLER_URL}/rw/rapid/execution?action=resetpp",
                timeout=REQUEST_TIMEOUT,
            )
            print("RESETPP:", reset_resp.status_code, reset_resp.text)
            if reset_resp.status_code not in (204, 200):
                raise Exception(
                    f"Failed to reset program pointers. Status: {reset_resp.status_code}\n\n"
                    + reset_resp.text[:400] + "\n\n" + dump_state(session)
                )
            time.sleep(RESETPP_SETTLE_SECONDS)
            print("PP after:", pp_routines(session))
        else:
            print("RESETPP skipped (pointers already at main or DO_RESETPP = False)")

        # 7a. RELEASE mastership BEFORE starting RAPID. Confirmed on this YuMi (RW 6.16):
        # while this client holds mastership ('remote') the controller answers the start
        # with 204 but does a no-op regain and RAPID never runs. Without it, start works.
        for rel_path in ("/rw/mastership/motion?action=release",
                         "/rw/mastership/rapid?action=release",
                         "/rw/mastership?action=release"):
            try:
                session.post(f"{CONTROLLER_URL}{rel_path}", timeout=REQUEST_TIMEOUT)
            except Exception as e:
                print("Mastership release error (non-fatal):", e)
        have_mastership = False
        time.sleep(0.5)
        print("MASTERSHIP released before start:", mastership_states(session))

        # 7b. Show tasks and activate any inactive normal task (TSP)
        set_status("Checking RAPID tasks...")
        print("TASKS before start:\n" + tasks_summary(session))
        try:
            for t in get_tasks(session):
                name = t.get("name")
                if t.get("type", "").lower().startswith("norm") and t.get("active", "").lower() != "on":
                    r = session.post(
                        f"{CONTROLLER_URL}/rw/rapid/tasks/{name}?action=activate",
                        timeout=REQUEST_TIMEOUT,
                    )
                    print(f"ACTIVATE {name}:", r.status_code, r.text[:200])
        except Exception as e:
            print("Task activation failed (non-fatal):", e)
        time.sleep(0.5)

        # 8/9. Start RAPID. The first start can end up doing only a regain
        # movement (robot back onto its path), so retry up to 3 times.
        execution_running = False
        seen_states = []
        for attempt, (regain_mode, use_tsp) in enumerate([("continue", False), ("clear", True), ("regain", False)], start=1):
            set_status(f"Starting RAPID execution (attempt {attempt}/3)...")
            start_payload = {
                "regain": regain_mode,
                "execmode": "continue",
                "cycle": "forever",
                "condition": "none",
                "stopatbp": "disabled",
            }
            start_payload["alltaskbytsp"] = "true" if use_tsp else "false"
            start_resp = session.post(
                f"{CONTROLLER_URL}/rw/rapid/execution?action=start",
                data=start_payload,
                timeout=REQUEST_TIMEOUT,
            )
            print(f"START #{attempt} (regain={regain_mode}, tsp={use_tsp}):", start_resp.status_code, start_resp.text)
            if start_resp.status_code not in (200, 204):
                raise Exception(
                    f"Failed to send start command. Status: {start_resp.status_code}\n\n"
                    + start_resp.text[:600] + "\n\n" + dump_state(session)
                )

            deadline = time.time() + 6
            while time.time() < deadline:
                time.sleep(0.5)
                _, exec_text = get_json_text(session, "/rw/rapid/execution")
                n = norm(exec_text)
                if '"ctrlexecstate":"running"' in n:
                    execution_running = True
                    seen_states.append("running")
                    break
                state = "stopped" if '"ctrlexecstate":"stopped"' in n else "other"
                if not seen_states or seen_states[-1] != state:
                    seen_states.append(state)

            if execution_running:
                break
            print(f"Attempt {attempt} did not reach running.\nTasks:\n{tasks_summary(session)}\nEvent log:\n{elog_summary(session)}")
            time.sleep(1.0)

        if not execution_running:
            # Fallback: let the operator start RAPID from the FlexPendant. Release mastership
            # first so the pendant is not blocked by this script holding it.
            print("REST start did not run RAPID. Falling back to manual start on the pendant.")
            print("State at fallback:\n" + dump_state(session))
            for rel_path in ("/rw/mastership/motion?action=release",
                             "/rw/mastership/rapid?action=release",
                             "/rw/mastership?action=release"):
                try:
                    session.post(f"{CONTROLLER_URL}{rel_path}", timeout=REQUEST_TIMEOUT)
                except Exception:
                    pass
            have_mastership = False
            print("MASTERSHIP released:", mastership_states(session))

            messagebox.showinfo(
                "Manual start needed",
                "The remote start command was accepted but RAPID did not run.\n\n"
                "Press START on the FlexPendant (Auto mode, motors on) now.\n"
                "Click OK, then this tool waits up to 60 s for RAPID to run.",
            )
            set_status("Waiting for manual start on the pendant (60 s)...", "orange")
            manual_deadline = time.time() + 60
            while time.time() < manual_deadline:
                _, exec_text = get_json_text(session, "/rw/rapid/execution")
                if '"ctrlexecstate":"running"' in norm(exec_text):
                    execution_running = True
                    break
                time.sleep(0.5)
                root.update()

        if not execution_running:
            raise Exception(
                "RAPID did not start (remote start and manual pendant start both failed).\n"
                f"States seen: {' -> '.join(seen_states) or 'none'}\n\n"
                + dump_state(session)
            )

        # 10. Short stability check: make sure it does not drop out again
        set_status("Verifying execution is stable...")
        time.sleep(2.0)
        _, exec_text = get_json_text(session, "/rw/rapid/execution")
        if '"ctrlexecstate":"running"' not in norm(exec_text):
            raise Exception(
                "RAPID started but stopped again within 2 seconds. "
                "This usually means a RAPID runtime error.\n\n" + dump_state(session)
            )

        # 10b. All active tasks (incl. 'communication', the socket server) must be started
        set_status("Verifying all tasks started...")
        all_started = False
        for _ in range(20):
            try:
                active = [t for t in get_tasks(session) if t.get("active", "").lower() == "on"]
                if active and all(t.get("excstate", "").lower().startswith("star") for t in active):
                    all_started = True
                    break
            except Exception as e:
                print("Task check error:", e)
            time.sleep(0.5)
        print("TASKS after start:\n" + tasks_summary(session))
        if not all_started:
            try:
                ts = get_tasks(session)
                ok = [t.get("name") for t in ts if t.get("excstate", "").lower().startswith("star")]
                bad = [f"{t.get('name')} ({t.get('excstate')})" for t in ts
                       if t.get("active", "").lower() == "on"
                       and not t.get("excstate", "").lower().startswith("star")]
            except Exception:
                ok, bad = [], ["unknown"]
            raise Exception(
                f"RAPID is running, but not every task started.\n"
                f"Running: {', '.join(ok) or 'none'}\n"
                f"NOT running: {', '.join(bad) or 'none'}\n"
                "(The 'communication' task is the socket server; main_tray.py cannot "
                "connect without it.)\n\n"
                + dump_state(session)
            )

        # 11. Launch main_tray.py in a new visible console window
        set_status("Spawning main_tray.py...", "blue")
        time.sleep(1.0)

        script_dir = os.path.dirname(os.path.abspath(__file__))
        main_tray_path = os.path.join(script_dir, "main_tray.py")
        python_exe = sys.executable

        if not os.path.isfile(main_tray_path):
            raise Exception(f"main_tray.py not found at:\n{main_tray_path}")

        global _main_tray_proc
        new_console = subprocess.CREATE_NEW_CONSOLE if os.name == "nt" else 0
        if MAIN_TRAY_INPUTS:
            # Own console window for output; keyboard answers come from our pipe instead.
            _main_tray_proc = subprocess.Popen(
                [python_exe, "-u", main_tray_path],
                cwd=script_dir,
                stdin=subprocess.PIPE,
                text=True,
                creationflags=new_console,
            )
            for answer in MAIN_TRAY_INPUTS:
                _main_tray_proc.stdin.write(answer + "\n")
            _main_tray_proc.stdin.flush()
            print("main_tray.py started with automatic inputs:", MAIN_TRAY_INPUTS)
        elif os.name == "nt":
            subprocess.Popen(
                f'cmd.exe /k ""{python_exe}" "{main_tray_path}""',
                cwd=script_dir,
                creationflags=new_console,
            )
        else:
            subprocess.Popen([python_exe, main_tray_path], cwd=script_dir)

        set_status("Success! Robot running & vision active.", "green")

    except Exception as e:
        status_label.config(text="Status: Error Occurred", fg="red")
        print("ERROR:", e)
        messagebox.showerror("Workflow Error", str(e))
    finally:
        if have_mastership:
            try:
                for rel_path in ("/rw/mastership/motion?action=release",
                                 "/rw/mastership/rapid?action=release",
                                 "/rw/mastership?action=release"):
                    session.post(f"{CONTROLLER_URL}{rel_path}", timeout=REQUEST_TIMEOUT)
            except Exception:
                pass
        session.close()
        action_button.config(state="normal")


# --------------------------------------------------------------------------
# GUI LAYOUT
# --------------------------------------------------------------------------
root = tk.Tk()
root.title("YuMi Cell Automator")
root.geometry("420x220")

instruction_text = tk.Label(
    root,
    text="Press to fully automate reset, start, and vision:",
    font=("Arial", 10),
)
instruction_text.pack(pady=15)

action_button = tk.Button(
    root,
    text="RESTART YUMI CELL",
    font=("Arial", 12, "bold"),
    bg="#107C41",
    fg="white",
    padx=15,
    pady=10,
    command=run_automation_workflow,
)
action_button.pack(pady=10)

status_label = tk.Label(root, text="Status: Ready", font=("Arial", 10, "italic"), fg="gray")
status_label.pack(pady=15)

root.mainloop()
