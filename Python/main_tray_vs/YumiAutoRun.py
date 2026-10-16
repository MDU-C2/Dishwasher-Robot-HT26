import subprocess
import time
import tkinter as tk
from tkinter import messagebox
import requests
from requests.auth import HTTPDigestAuth

# --- CONFIGURATION ---
# Default Virtual Controller IP (localhost for RobotStudio) and RWS credentials
CONTROLLER_URL = "http://192.168.125.1" 
AUTH = HTTPDigestAuth("Default User", "robotics")

def run_automation_workflow():
    try:
        status_label.config(text="Status: Stopping & Resetting RAPID...", fg="orange")
        root.update()

        # 1. Stop RAPID Execution safely
        requests.post(f"{CONTROLLER_URL}/rw/rapid/execution?action=stop", auth=AUTH, data={"stopmode": "stop", "usetsp": "alltsk"})
        
        # 2. Reset Program Pointer to Main for all tasks
        reset_resp = requests.post(f"{CONTROLLER_URL}/rw/rapid/execution?action=resetpp", auth=AUTH)
        if reset_resp.status_code not in [200, 204]:
            raise Exception("Failed to reset the Program Pointer to Main.")

        # 3. Start RAPID Execution (Automatically encompasses all active tasks)
        start_payload = {
            "regain": "clear",
            "execmode": "continue",
            "cycle": "forever",
            "condition": "none",
            "stopatbp": "disabled",
            "alltaskbytsp": "true" # Ensures all 3 tasks start simultaneously
        }
        start_resp = requests.post(f"{CONTROLLER_URL}/rw/rapid/execution?action=start", auth=AUTH, data=start_payload)
        if start_resp.status_code not in [200, 204]:
            raise Exception("Failed to start RAPID tasks.")

        status_label.config(text="Status: RAPID Running. Spawning maintray.py...", fg="blue")
        root.update()
        
        # Give the controller a brief moment to settle
        time.sleep(1)

        # 4. Programmatically launch maintray.py and handle terminal strings
        # Adjust 'python' to 'python3' if your environment requires it
        process = subprocess.Popen(
            ["python", "maintray.py"],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1
        )

        # Send "5s" followed by Enter, then "n" followed by Enter
        process.stdin.write("5s\n")
        process.stdin.flush()
        time.sleep(0.5) # Slight pause to simulate user cadence if needed by the script
        
        process.stdin.write("n\n")
        process.stdin.flush()

        status_label.config(text="Status: Complete! Vision connected & YuMi moving.", fg="green")

    except Exception as e:
        status_label.config(text="Status: Error Occurred", fg="red")
        messagebox.showerror("Workflow Error", str(e))

# --- GUI LAYOUT ---
root = tk.Tk()
root.title("YuMi RAPID & Vision Automator")
root.geometry("400x200")

instruction_text = tk.Label(root, text="Click below to completely reset and restart the cell:", font=("Arial", 10))
instruction_text.pack(pady=15)

# Large prominent button to recover from kinematic limits / errors instantly
action_button = tk.Button(root, text="🚀 RESET & START SYSTEM", font=("Arial", 12, "bold"), bg="#107C41", fg="white", padx=10, pady=10, command=run_automation_workflow)
action_button.pack(pady=10)

status_label = tk.Label(root, text="Status: Ready", font=("Arial", 10, "italic"), fg="gray")
status_label.pack(pady=15)

root.mainloop()