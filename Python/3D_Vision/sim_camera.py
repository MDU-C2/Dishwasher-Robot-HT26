import time
import threading
from updated_communication import Communication

# ===========================================================
# 1. DEFINE YOUR CONSTANT MUG POSITION HERE
# These values are in ROBOT coordinates (mm) relative to base
# ===========================================================
# Example: Mug sitting on the left side of the table
MOCK_COORDS = [450.0, 150.0, 65.0]  # [X, Y, Z] 

# Orientation: [0, 0, 1] means the mug is standing upright
# [0, 0, -1] means it is upside down
MOCK_NORMAL = [0.0, 0.0, 1.0]

# Dummy quaternion required by the communication class
DUMMY_QUAT = [1.0, 0.0, 0.0, 0.0]

busy = False
lock = threading.Lock()

def trigger_robot(client):
    global busy
    with lock:
        if not busy:
            print(f"[ACTION] Sending Mug at {MOCK_COORDS} to Robot...")
            busy = True
            
            # This triggers the same 'pickupSequence' case in RAPID
            # No need for camera setup or coordinate conversion!
            client.PickUpSequence(MOCK_COORDS, DUMMY_QUAT, MOCK_NORMAL)
            
            print("[INFO] Sequence finished. Ready for next test.")
            busy = False

def run():
    client = Communication()
    
    print("--- YuMi MOCK Vision System ---")
    print(f"Targeting Hardcoded Position: {MOCK_COORDS}")
    
    if client.connectV2():
        print("\nReady! Press 'ENTER' to start the pick-up sequence.")
        print("Press 'Ctrl+C' to exit.")
        
        try:
            while True:
                user_input = input() # Waits for you to press Enter
                
                if not busy:
                    # Start the robot movement in a thread so this script stays responsive
                    threading.Thread(target=trigger_robot, args=(client,), daemon=True).start()
                else:
                    print("Robot is currently busy!")
                    
        except KeyboardInterrupt:
            print("\nExiting...")
            client.disconnect()

if __name__ == "__main__":
    run()