import socket
import time
import ast
import threading
from collections import deque
import cv2 as cv
from config_loader import load_config
lock = threading.Lock()

class Communication():
    """Class to handle communication with RAPID server"""
    max_sleep_duration = 10
    space = False
    activ = True
    def __init__(self):

        self.sleep_time = 0

        self._mutex_variable = threading.RLock()  # re-entrant mutex for accessing shared variables
        self._mutex_function = threading.RLock()  # re-entrant mutex so _handle_response -> disconnect never self-deadlocks
        self.socket = None
        self._rx_queue = deque()
        self._rx_buffer = ""
        self.config = load_config()
        self.port = int(self.config["robot"]["socket_port"])
        self.host = str(self.config["robot"]["ip"])
        self.connected = False


    
        ## What python can recive SEE BELOW
        self.ACKS = ["Ack_succesful","Ack_wait","Ack_Release done","Connection_Confirmed",
                "Ack_succesfull", "ACK","Ack_Coordinate","Ack_Orientation", "Ack_normal", "Ack_EGM",
                "Ack_LeavePosition"]
        
        self.CLOSE = ["Disconnect", ]
        self.ROBOTWANTSTOSENDCOORDINATES = ["Robot_Wants_To_Send_Coordinates", ] # Python will answer with an ACK then receive coordinates then answer with an ACK again
        self.ROBOTWANTSTOSENDORIENTATION = ["Robot_Wants_To_Send_Orientation", ] # RAPID gives the robots coordinates

        self.ASKMUGCOORDINATES = ["Ask_MugCoordinate", "Ask_Coordinate"] # Python gives a mugs coordinates
        self.ASKMUGORIENTATION = ["Ask_MugOrientation", "Ask_Orientation"]
        self.ASKNEXT = ["AskNext",  "Ack_Grip_Done", "Ack_Release done", "Ask_next"] # RAPID is ready for the next command

        self.ASKCALPOINT = ["AskCalPoint", ]
        self.ASKMUGNORMAL = ["Ask_MugNormal", ]
        self.ASKFREESLOT = ["Ask_LeavePosition", "Ask_FreeSlot"]

        self.NEXTSTEP = ["Next_step",]
        self.ERROR = [
            "[ERROR]_wrong_format,try_again(exampel[x,y,z])",
            "[ERROR]_wrong_format,send_normal",
            "[ERROR]_wrong_format,try_again(exampel[q1,q2,q3,q4])",
            "[ERROR] Invalid format, retry",
        ]
        # Known fixed RAPID messages sorted longest-first to split back-to-back TCP coalesced sends
        self._known_tokens = tuple(sorted(
            {
                t for group in (
                    self.ACKS, self.CLOSE, self.ROBOTWANTSTOSENDCOORDINATES,
                    self.ROBOTWANTSTOSENDORIENTATION, self.ASKMUGCOORDINATES,
                    self.ASKMUGORIENTATION, self.ASKNEXT, self.ASKCALPOINT,
                    self.ASKMUGNORMAL, self.ASKFREESLOT, self.NEXTSTEP, self.ERROR,
                )
                for t in group if t
            },
            key=len,
            reverse=True,
        ))

        ## List of what python can send

        '''
        "Connection_test"
        "Get_Coordinates"
        "Move"
        "Grip"
        "Release"
        "Home"
        "Pick_Up_Sequence"
        "Leave_Sequence"
        "Move_Calibration_Position"

        # Special cases, inside a switch case
        "ACK"
        '''
        
        self.MugCoordinates = [200,-200,100] # x,y,z coordinates of the mug
        self.MugOrientation = [1,0,0,0] # quaternion, mug orientation
        self.CalPoint = 1 # calibration point number, corresponds to a point in RAPID
        self.RobTarget = [[200,-200,100],[1,0,0,0],[-1,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]] # RAPID format
        self.Robotcoordinates = [100,100,100] # Robot hand coordinates
        self.Robotorientation = [1,0,0,0] # Robot hand orientation, unused?
        self.MugNormal = [0,0,1]
        self.FreeSlot = None  # [x, y, z] in robot frame; set by main.py before each placement
        self.LatestMugCoordinates = None
        self.LatestMugNormal = None
        self.pick_attempt_count = 0
        self.retry_history = []
        self.last_pick_start_time = None
        self.last_handover_timestamp = None
        self.last_pick_end_time = None
        self.last_left_arm_time_s = 0.0
        self.last_right_arm_time_s = 0.0

        # self._connection_test_thread = threading.Thread(target=self._keep_connection_alive, daemon=True)
        # self._connection_test_thread.start()


    def _handle_response(self):
        """Handle response from RAPID"""

        while True: # It will loop until the response is in "ASKNEXT" or RAPID wants to close the connection (CLOSE).

            response = self._receive_message()
            if response is None:
                self.disconnect()
                return None

            match response:
                case next if next in self.ASKNEXT:
                    return None
                
                case next_step if next_step in self.NEXTSTEP:
                        return True
                case ack if ack in self.ACKS:
                    continue

                case close if close in self.CLOSE:
                    self.disconnect() # Disconnect, don't send anything to RAPID
                    return None
                
                case robot_wants_to_send_coordinates if robot_wants_to_send_coordinates in self.ROBOTWANTSTOSENDCOORDINATES: # RAPID wants to send robot coordinates
                    self._send_message("ACK") # Tell RAPID to send coordinates
                    self.Robotcoordinates =  self._receive_message() # Receive coordinates
                    self._send_message("ACK") # Acknowledge so it can send AskNext or other commands

                case robot_wants_to_send_orientation if robot_wants_to_send_orientation in self.ROBOTWANTSTOSENDORIENTATION: # RAPID wants to send robot orientation
                    self._send_message("ACK")
                    self.Robotorientation = self._receive_message()
                    self._send_message("ACK")

                case askmug_coordinates if askmug_coordinates in self.ASKMUGCOORDINATES: # RAPID wants mug coordinates
                    with self._mutex_variable:  # Lock mutex for thread-safe access
                        self.pick_attempt_count += 1
                        if self.pick_attempt_count > 1 and self.LatestMugCoordinates is not None:
                            print(f"[RETRY #{self.pick_attempt_count}] Grasp miss detected! Refreshing mug coordinates from live camera:")
                            print(f"         Previous: {self.MugCoordinates}")
                            print(f"         Fresh:    {self.LatestMugCoordinates}")
                            self.retry_history.append({
                                "attempt": self.pick_attempt_count,
                                "previous_xyz": list(self.MugCoordinates),
                                "fresh_xyz": list(self.LatestMugCoordinates),
                            })
                            self.MugCoordinates = list(self.LatestMugCoordinates)
                            if self.LatestMugNormal is not None:
                                self.MugNormal = list(self.LatestMugNormal)
                        self._send_message(str(self.MugCoordinates)) # Send mug coordinates

                case askmug_orientation if askmug_orientation in self.ASKMUGORIENTATION: # RAPID wants mug orientation
                    with self._mutex_variable:  # Lock mutex for thread-safe access
                        self._send_message(str(self.MugOrientation)) # Send mug orientation
                    

                case askcal_point if askcal_point in self.ASKCALPOINT: # RAPID wants calibration point number
                    self._send_message(str(self.CalPoint)) # Send calibration point number
                
                case askmug_normal if askmug_normal in self.ASKMUGNORMAL: # RAPID wants mug normal
                    with self._mutex_variable:  # Lock mutex for thread-safe access
                        self._send_message(str(self.MugNormal)) # Send mug normal

                case ask_free_slot if ask_free_slot in self.ASKFREESLOT: # RAPID wants a free tray slot position
                    with self._mutex_variable:
                        if self.last_handover_timestamp is None:
                            self.last_handover_timestamp = time.perf_counter()
                        slot_cb = getattr(self, 'get_free_slot_logic', None)
                        fallback_slot = list(self.FreeSlot) if self.FreeSlot is not None else None
                    # Invoke callback outside _mutex_variable so background slot resolution cannot deadlock
                    if callable(slot_cb):
                        try:
                            slot_coords = slot_cb()
                            if slot_coords is not None:
                                self._send_message(str(list(slot_coords)))
                            else:
                                self._send_message("NO_SLOT")
                        except Exception as exc:
                            print(f" error: {exc}")
                            self._send_message("NO_SLOT")
                    elif fallback_slot is not None:
                        self._send_message(str(fallback_slot))
                    else:
                        # Invalid for rob_coordinates so RAPID retries instead of going to [0,0,0]
                        self._send_message("NO_SLOT")

                case error if error in self.ERROR:
                    print("Known error occured")

                case _: # Error and unexpected response handling
                    #self.ErrorHandling()
                    print(f"[ERROR] Unexpected response: {response}")
                    self.disconnect()
                    # exit(1)
                    # sys.exit(0)
                    return None

    def _keep_connection_alive(self):
        """Sleep 60s then send a test message to RAPID. Windows does not like open sockets without activity."""
        while self.activ:

            start = time.time()
            time.sleep(1) 
            if self.sleep_time > self.max_sleep_duration:
                if self.connected:
                    self.ConnectionTest()
                    self.reset_connection_timer()
            else:
                self.sleep_time += time.time() - start

    def reset_connection_timer(self):
        self.sleep_time = 0
        # print(f"Sleep timer reset: {self.sleep_time}")

    def connect(self):

        with self._mutex_function:  # Lock mutex for function-level thread safety

            """Connect to RAPID server"""
            try:
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                #socket.settimeout(120)  # Increased to 120 seconds for robot movement, Dont use time out
                self.socket.connect((self.host, self.port))
                self._rx_queue.clear()
                self._rx_buffer = ""
                self.connected = True
                print(f"[INFO] Connected to RAPID server at {self.host}:{self.port}")

            except Exception as e:
                print(f"[ERROR] Connection failed: {e}")

                
    def connectV2(self):

        with self._mutex_function:  # Lock mutex for function-level thread safety

            """Connect to RAPID server"""
            try:
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                #socket.settimeout(120)  # Increased to 120 seconds for robot movement, Dont use time out
                self.socket.connect((self.host, self.port))
                self._rx_queue.clear()
                self._rx_buffer = ""
                self.connected = True
                print(f"[INFO] Connected to RAPID server at {self.host}:{self.port}")
                return True

            except Exception as e:
                print(f"[ERROR] Connection failed: {e}")
                return False


    def disconnect(self):

        with self._mutex_function:  # Lock mutex for function-level thread safety
            if not self.socket:
                return
            try:
                self._send_message("disconnect")
                try:
                    self.socket.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass  # socket may already be closed or not connected
                self.socket.close()
                print("[INFO] Socket closed")
            except Exception as e:
                print(f"[ERROR] Close error: {e}")
            finally:
                self.connected = False
                self.socket = None
                self._rx_queue.clear()
                self._rx_buffer = ""

        
    def _send_message(self, message):
        """Send message to RAPID"""
        if not self.connected:
            print("[ERROR] Not connected to robot!")
            return None

        try:
            self.socket.sendall(message.encode())
            print(f"[SENT] {message}")
        except Exception as e:
            print(f"[ERROR] Send error: {e}")
            self.connected = False
            return None

    def _drain_rx_buffer(self):
        """Extract complete protocol messages from TCP stream buffer, splitting coalesced RAPID sends."""
        while self._rx_buffer:
            matched = None
            for tok in self._known_tokens:
                if self._rx_buffer.startswith(tok):
                    matched = tok
                    break
            if matched is not None:
                self._rx_queue.append(matched)
                self._rx_buffer = self._rx_buffer[len(matched):]
                continue
            # If buffer is a partial prefix of a known token split across TCP packets, wait for more bytes
            if any(tok.startswith(self._rx_buffer) and tok != self._rx_buffer for tok in self._known_tokens):
                break
            # Non-tokenized payload (e.g. Robot_Wants_To_Send_* coordinates/orientation)
            self._rx_queue.append(self._rx_buffer)
            self._rx_buffer = ""
            break

    def _receive_message(self):
        """Receive message from RAPID, handling coalesced TCP packets."""
        try:
            while not self._rx_queue:
                chunk = self.socket.recv(1024).decode()
                if not chunk:
                    if self._rx_buffer:
                        msg = self._rx_buffer
                        self._rx_buffer = ""
                        print(f"[RECEIVED] {msg}")
                        return msg
                    self.connected = False
                    return None
                self._rx_buffer += chunk
                self._drain_rx_buffer()
            message = self._rx_queue.popleft()
            print(f"[RECEIVED] {message}")
            return message
        except socket.timeout:
            print(f"[ERROR] Receive timeout - no message received within timeout period")
            self.connected = False
            return None
        except Exception as e:
            print(f"[ERROR] Receive error: {e}")
            self.connected = False
            return None
        
    def UpdateVariables(self, MugCoordinates = None, MugOrientation = None):
        with self._mutex_variable: # Lock mutex for thread-safe access
            if MugCoordinates is not None:
                self.MugCoordinates = list(MugCoordinates)  # make defensive copy, unnecessary?
            if MugOrientation is not None:
                self.MugOrientation = list(MugOrientation)
        return


    def Presentation(self, coordinates, orientation, normalized_vector):
        coiqter = 0
        max_time = 10
        start = 0
        self._mutex_function.acquire()  # Lock mutex for function-level thread safety
        
        self._mutex_variable.acquire()
        self.MugCoordinates = list(coordinates)
        self.MugOrientation = list(orientation)
        self.MugNormal = list(normalized_vector)
        self._mutex_variable.release()

        self._send_message("Presentation")
        _continue = self._handle_response()
        self._mutex_function.release()
        # self.reset_connection_timer()

        while _continue != None:
            start = time.time()
            if self.space:

                self._mutex_function.acquire()
                self._send_message("ACK")
                _continue =  self._handle_response()
                self._mutex_function.release()

                self._mutex_variable.acquire()
                # self.reset_connection_timer()
                self.space = False
                self._mutex_variable.release()
                coiqter = 0
            elif coiqter > max_time:
                self.ConnectionTest()
                coiqter = 0
            coiqter += time.time() - start
        # self._mutex_function.release()
        # self._mutex_function.release()
           


    def GetPosition(self):

        with self._mutex_function:  # Lock mutex for function-level thread safety
            #"""Get current position from RAPID"""
            self._send_message("Get_Coordinates")
            self._handle_response() # stores respone in self.Robotcoordinates
            self.Robotcoordinates = ast.literal_eval(self.Robotcoordinates) # Convert string to variable

            return self.Robotcoordinates # RobotCoordinates can be both string and a variable depending on when you check it.
    
    def Move(self, coordinates, orientation, normalized_vector):

        with self._mutex_function:  # Lock mutex for function-level thread safety

            """Move robot to specified coordinates and orientation"""
            with self._mutex_variable:
                self.MugCoordinates = list(coordinates)
                self.MugOrientation = list(orientation)
                self.MugNormal = list(normalized_vector)
            self._send_message("Move")
            self._handle_response()  # Wait for "AskNext"


            return

    def PickUpSequence(self, coordinates, orientation,normalized_vector):
        with self._mutex_function:
            with self._mutex_variable:
                self.MugCoordinates = list(coordinates)
                self.MugOrientation = list(orientation)
                self.MugNormal = list(normalized_vector)
                self.pick_attempt_count = 0
                self.retry_history = []
                self.last_pick_start_time = time.perf_counter()
                self.last_handover_timestamp = None
                self.last_pick_end_time = None
                self.last_left_arm_time_s = 0.0
                self.last_right_arm_time_s = 0.0
            self._send_message("Pick_Up_Sequence")
            self._handle_response()
            with self._mutex_variable:
                self.last_pick_end_time = time.perf_counter()
                total_seq = self.last_pick_end_time - self.last_pick_start_time
                if self.last_handover_timestamp is not None:
                    self.last_left_arm_time_s = self.last_handover_timestamp - self.last_pick_start_time
                    self.last_right_arm_time_s = self.last_pick_end_time - self.last_handover_timestamp
                else:
                    self.last_left_arm_time_s = total_seq
                    self.last_right_arm_time_s = 0.0
            return None

    def LeaveSequence(self, coordinates, orientation):
        with self._mutex_function:
            self._send_message("Leave_Sequence")
            self.MugCoordinates = coordinates
            self.MugOrientation = orientation
            self._handle_response()

            #"""Perform leave sequence"""
            return None

    def SetFreeSlot(self, slot_coords):
        """Store robot-frame [x, y, z] so RAPID can fetch it via Ask_LeavePosition."""
        with self._mutex_variable:
            if slot_coords is None:
                self.FreeSlot = None
            else:
                self.FreeSlot = [float(v) for v in slot_coords]
            print(f"[TRAY] Free slot set to: {self.FreeSlot}")

    def LeaveSequenceTray(self, slot_coords=None):
        if slot_coords is not None:
            self.SetFreeSlot(slot_coords)
    
    def OpenGripper(self):
        with self._mutex_function:
            self._send_message("Release")
            self._handle_response()
            return None
    
    def CloseGripper(self):
        with self._mutex_function:
            self._send_message("Grip")
            self._handle_response()
            return None
    
    def MoveCalibrationPosition(self, position):
        with self._mutex_function:
            self._send_message("Move_Calibration_Position")
            with self._mutex_variable:
                self.CalPoint = position # make defensive copy
            self._handle_response()
            return None
    
    def MoveHome(self):
        with self._mutex_function:
            self._send_message("Home")
            self._handle_response()
            return None
    
    def ConnectionTest(self):
        with self._mutex_function:
            self._send_message("Connection_test")
            self._handle_response()
            return None

    def EGM_movement(self):
        with self._mutex_function:
            self._send_message("EGM_movement")
            self._handle_response()
            return None
