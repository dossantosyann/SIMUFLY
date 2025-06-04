import gpiozero
import time
import math
import re # For command parsing
import curses # <-- Ajout pour l'interface CLI améliorée
import cv2 # <-- Added for OpenCV webcam capture
import os # <-- Added for path manipulation
import sys
from contextlib import contextmanager

# --- Gestionnaire de contexte pour supprimer les messages stderr de bas niveau ---
@contextmanager
def suppress_c_stderr():
    """
    Un gestionnaire de contexte pour supprimer temporairement la sortie stderr au niveau C.
    Utile pour cacher les avertissements des bibliothèques comme GStreamer via OpenCV.
    """
    original_stderr_fd = sys.stderr.fileno()
    devnull_fd = os.open(os.devnull, os.O_WRONLY)
    saved_stderr_fd = os.dup(original_stderr_fd)
    try:
        os.dup2(devnull_fd, original_stderr_fd)
        yield
    finally:
        os.dup2(saved_stderr_fd, original_stderr_fd)
        os.close(devnull_fd)
        os.close(saved_stderr_fd)

# GPIO Pin Configuration (BCM numbering) - As provided by user
# Motor 1 (often Motor A in CoreXY nomenclature)
PUL_PIN_M1 = 6
DIR_PIN_M1 = 5

# Motor 2 (often Motor B in CoreXY nomenclature)
PUL_PIN_M2 = 27
DIR_PIN_M2 = 17

# --- NEW: Endstop GPIO Pin Configuration ---
ENDSTOP_PIN_X = 16 # For X-axis
ENDSTOP_PIN_Y = 26 # For Y-axis

# --- GPIOZero Device Objects (will be initialized in setup_gpio) ---
pul_device_m1 = None
dir_device_m1 = None
pul_device_m2 = None
dir_device_m2 = None
# --- NEW: Endstop Device Objects ---
endstop_x = None
endstop_y = None

# --- Motor & System Parameters ---
MOTOR_NATIVE_STEPS_PER_REV = 400
DRIVER_MICROSTEP_DIVISOR = 2

MM_PER_MICROSTEP = 0.1
if MM_PER_MICROSTEP == 0:
    raise ValueError("MM_PER_MICROSTEP cannot be zero.")
MICROSTEPS_PER_MM = 1.0 / MM_PER_MICROSTEP

DEFAULT_XLIM_MM = 800.0
DEFAULT_YLIM_MM = 1200.0
XLIM_MM = DEFAULT_XLIM_MM
YLIM_MM = DEFAULT_YLIM_MM

# --- Speed and Timing Parameters ---
DEFAULT_SPEED_MM_S = 400.0       # Default speed in mm/s
MAX_SPEED_MM_S = 400.0           # Maximum allowable speed by user
TARGET_SPEED_MM_S = min(DEFAULT_SPEED_MM_S, MAX_SPEED_MM_S) # Global target speed
HOMING_SPEED_MM_S = 350.0        # Speed for HOME moves in mm/s
CALIBRATION_SPEED_MM_S = 100.0   # Speed for calibration moves
CALIBRATION_BACKOFF_MM = 5.0     # Back-off distance after hitting endstop
MAX_CALIBRATION_TRAVEL_MM = max(DEFAULT_XLIM_MM, DEFAULT_YLIM_MM) + 50.0 # Safety travel limit for calibration

MIN_PULSE_WIDTH = 0.000002      # 2 microseconds (minimum pulse high time)
MINIMUM_PULSE_CYCLE_DELAY = 0.0001 # 100µs -> max 10,000 step cycles/sec

# --- Ramp Parameters ---
RAMP_PERCENTAGE = 0.05  # Percentage of move steps for acceleration/deceleration ramp
RAMP_START_DELAY_MULTIPLIER = 3.0  # Start ramp with delay = target_delay * multiplier
MAX_START_PULSE_CYCLE_DELAY_FOR_RAMP = 0.01  # Max delay for the start/end of ramp (e.g., 10ms)

# --- System State ---
current_x_mm = 0.0
current_y_mm = 0.0
absolute_mode = True # True for ABS (absolute), False for REL (relative)

# --- Curses UI state ---
last_command_output = []



def setup_gpio():
    """Initializes GPIO pins. Returns True on success, False on failure."""
    global pul_device_m1, dir_device_m1, pul_device_m2, dir_device_m2
    global endstop_x, endstop_y
    try:
        pul_device_m1 = gpiozero.OutputDevice(PUL_PIN_M1, active_high=True, initial_value=False)
        dir_device_m1 = gpiozero.OutputDevice(DIR_PIN_M1, active_high=True, initial_value=False)
        pul_device_m2 = gpiozero.OutputDevice(PUL_PIN_M2, active_high=True, initial_value=False)
        dir_device_m2 = gpiozero.OutputDevice(DIR_PIN_M2, active_high=True, initial_value=False)
        
        endstop_x = gpiozero.InputDevice(ENDSTOP_PIN_X, pull_up=True)
        endstop_y = gpiozero.InputDevice(ENDSTOP_PIN_Y, pull_up=True)
        
        print("GPIO initialized successfully (Motors and Endstops).")
        return True
    except Exception as e:
        print(f"Error during GPIO initialization: {e}")
        print("Ensure you are running the script with necessary permissions (e.g., sudo)")
        print("and that the GPIO pins are not already in use.")
        return False

def cleanup_gpio():
    """Cleans up GPIO resources."""
    if pul_device_m1: pul_device_m1.close()
    if dir_device_m1: dir_device_m1.close()
    if pul_device_m2: pul_device_m2.close()
    if dir_device_m2: dir_device_m2.close()
    if endstop_x: endstop_x.close()
    if endstop_y: endstop_y.close()
    print("GPIO cleaned up.")

def move_motors_coordinated(steps_m1_target, steps_m2_target, pulse_cycle_delay_for_move):
    """
    Coordinates the movement of both motors for CoreXY motion with a specific pulse delay,
    including initial acceleration and final deceleration ramps (trapezoidal profile).
    """
    messages = []
    if not all([pul_device_m1, dir_device_m1, pul_device_m2, dir_device_m2]):
        messages.append("Error: GPIO devices not initialized.")
        return messages

    if steps_m1_target > 0: dir_device_m1.off() 
    else: dir_device_m1.on()                  

    if steps_m2_target > 0: dir_device_m2.off() 
    else: dir_device_m2.on()                  
    
    time.sleep(0.001) 

    abs_steps_m1 = abs(steps_m1_target)
    abs_steps_m2 = abs(steps_m2_target)
    total_iterations = max(abs_steps_m1, abs_steps_m2)

    if total_iterations == 0:
        return messages

    # --- Trapezoidal Profile Calculation ---
    target_cruise_delay = pulse_cycle_delay_for_move # Delay for the cruise speed phase

    _start_delay_uncapped = target_cruise_delay * RAMP_START_DELAY_MULTIPLIER
    _calculated_initial_delay = min(_start_delay_uncapped, MAX_START_PULSE_CYCLE_DELAY_FOR_RAMP)
    effective_start_end_delay = max(target_cruise_delay, _calculated_initial_delay)

    use_ramps = False
    ramp_segment_duration_steps = 0 

    if (effective_start_end_delay > target_cruise_delay + MIN_PULSE_WIDTH / 2.0) and total_iterations >= 4: 
        use_ramps = True
        ramp_segment_duration_steps = max(1, int(RAMP_PERCENTAGE * total_iterations))

    accel_phase_actual_steps = 0
    cruise_phase_actual_steps = total_iterations 
    decel_phase_actual_steps = 0
    delay_at_peak_or_cruise_start = target_cruise_delay 

    if use_ramps:
        if total_iterations < 2 * ramp_segment_duration_steps:
            accel_phase_actual_steps = total_iterations // 2
            decel_phase_actual_steps = total_iterations - accel_phase_actual_steps
            cruise_phase_actual_steps = 0
            if accel_phase_actual_steps > 0:
                if accel_phase_actual_steps == 1: 
                     delay_at_peak_or_cruise_start = effective_start_end_delay 
                else:
                     fraction = (accel_phase_actual_steps - 1.0) / float(accel_phase_actual_steps)
                     delay_at_peak_or_cruise_start = effective_start_end_delay + \
                                        (target_cruise_delay - effective_start_end_delay) * fraction
                if target_cruise_delay < effective_start_end_delay: 
                    delay_at_peak_or_cruise_start = max(target_cruise_delay, delay_at_peak_or_cruise_start)
                else: 
                    delay_at_peak_or_cruise_start = min(target_cruise_delay, delay_at_peak_or_cruise_start)
            else: 
                delay_at_peak_or_cruise_start = effective_start_end_delay
        else: 
            accel_phase_actual_steps = ramp_segment_duration_steps
            decel_phase_actual_steps = ramp_segment_duration_steps
            cruise_phase_actual_steps = total_iterations - accel_phase_actual_steps - decel_phase_actual_steps
            delay_at_peak_or_cruise_start = target_cruise_delay 
    
    for i in range(total_iterations):
        current_step_delay = target_cruise_delay 

        if use_ramps and i < accel_phase_actual_steps:
            if accel_phase_actual_steps > 0:
                fraction_completed = i / float(accel_phase_actual_steps)
                current_step_delay = effective_start_end_delay + \
                                   (target_cruise_delay - effective_start_end_delay) * fraction_completed
            else: 
                current_step_delay = target_cruise_delay
        
        elif use_ramps and i >= (accel_phase_actual_steps + cruise_phase_actual_steps):
            if decel_phase_actual_steps > 0:
                step_into_decel = i - (accel_phase_actual_steps + cruise_phase_actual_steps)
                fraction_completed = step_into_decel / float(decel_phase_actual_steps)
                current_step_delay = delay_at_peak_or_cruise_start + \
                                   (effective_start_end_delay - delay_at_peak_or_cruise_start) * fraction_completed
            else: 
                 current_step_delay = target_cruise_delay

        current_step_delay = max(MINIMUM_PULSE_CYCLE_DELAY, current_step_delay)

        perform_pulse_m1 = (i < abs_steps_m1)
        perform_pulse_m2 = (i < abs_steps_m2)

        if perform_pulse_m1: pul_device_m1.on()
        if perform_pulse_m2: pul_device_m2.on()
        time.sleep(MIN_PULSE_WIDTH)
        if perform_pulse_m1: pul_device_m1.off()
        if perform_pulse_m2: pul_device_m2.off()
        
        inter_pulse_delay = current_step_delay - MIN_PULSE_WIDTH
        if inter_pulse_delay > 0:
            time.sleep(inter_pulse_delay)
            
    return messages

def move_corexy(delta_x_mm, delta_y_mm, pulse_cycle_delay_for_move):
    """
    Calculates steps and initiates CoreXY movement with a given pulse cycle delay.
    Returns a list of messages from motor coordination.
    """
    global current_x_mm, current_y_mm
    motor_messages = []
    
    delta_x_steps_cartesian = round(-delta_x_mm * MICROSTEPS_PER_MM)
    delta_y_steps_cartesian = round(delta_y_mm * MICROSTEPS_PER_MM)

    steps_m1 = delta_x_steps_cartesian + delta_y_steps_cartesian
    steps_m2 = delta_x_steps_cartesian - delta_y_steps_cartesian
    
    if steps_m1 == 0 and steps_m2 == 0:
        pass 
    else:
        motor_messages = move_motors_coordinated(int(steps_m1), int(steps_m2), pulse_cycle_delay_for_move)

    current_x_mm += delta_x_mm
    current_y_mm += delta_y_mm
    
    current_x_mm = round(current_x_mm, 3) 
    current_y_mm = round(current_y_mm, 3)
    return motor_messages

def perform_calibration_cycle():
    """
    Performs a homing/calibration cycle for Y and X axes using endstops.
    Moves towards negative Y, then negative X. Sets origin at trigger point, then backs off.
    Returns a list of output messages.
    """
    global current_x_mm, current_y_mm, TARGET_SPEED_MM_S, absolute_mode
    output_messages = ["--- Starting Calibration Cycle ---"]

    if not all([pul_device_m1, dir_device_m1, pul_device_m2, dir_device_m2, endstop_x, endstop_y]):
        output_messages.append("Error: GPIOs or endstop devices not initialized for calibration.")
        return output_messages

    cal_speed = min(CALIBRATION_SPEED_MM_S, MAX_SPEED_MM_S)
    if cal_speed <= 1e-6:
        output_messages.append(f"Error: Calibration speed {cal_speed:.2f} mm/s is too low.")
        return output_messages
        
    pulse_cycle_delay_cal = MM_PER_MICROSTEP / cal_speed
    actual_pulse_cycle_delay_cal = max(MINIMUM_PULSE_CYCLE_DELAY, pulse_cycle_delay_cal)
    max_steps_travel = int(MAX_CALIBRATION_TRAVEL_MM * MICROSTEPS_PER_MM)

    # --- Calibrate Y-axis (moving towards negative Y) ---
    output_messages.append(f"Calibrating Y-axis at {cal_speed:.2f} mm/s...")
    dir_device_m1.on()  
    dir_device_m2.off() 
    time.sleep(0.002)

    homed_y = False
    for i in range(max_steps_travel):
        if endstop_y.is_active:
            output_messages.append("Y-axis endstop triggered.")
            homed_y = True
            break
        
        pul_device_m1.on()
        pul_device_m2.on()
        time.sleep(MIN_PULSE_WIDTH)
        pul_device_m1.off()
        pul_device_m2.off()
        
        inter_pulse_delay = actual_pulse_cycle_delay_cal - MIN_PULSE_WIDTH
        if inter_pulse_delay > 0:
            time.sleep(inter_pulse_delay)
        if i > 0 and i % 2000 == 0: time.sleep(0.001) 

    if not homed_y:
        output_messages.append("Error: Y-axis calibration failed (endstop not triggered). Stopping.")
        return output_messages

    current_y_mm = 0.0
    output_messages.append(f"Y-axis origin set at trigger point: {current_y_mm:.3f} mm.")
    output_messages.append(f"Backing off Y-axis by {CALIBRATION_BACKOFF_MM:.2f} mm...")
    motor_msgs_by = move_corexy(0.0, CALIBRATION_BACKOFF_MM, actual_pulse_cycle_delay_cal) 
    if motor_msgs_by: output_messages.extend(motor_msgs_by)
    output_messages.append(f"Y-axis backed off. New position Y={current_y_mm:.3f} mm.")

    # --- Calibrate X-axis (moving towards negative X) ---
    output_messages.append(f"Calibrating X-axis at {cal_speed:.2f} mm/s...")
    dir_device_m1.off() 
    dir_device_m2.off() 
    time.sleep(0.002)

    homed_x = False
    for i in range(max_steps_travel):
        if endstop_x.is_active: 
            output_messages.append("X-axis endstop triggered.")
            homed_x = True
            break
        
        pul_device_m1.on()
        pul_device_m2.on()
        time.sleep(MIN_PULSE_WIDTH)
        pul_device_m1.off()
        pul_device_m2.off()
        
        inter_pulse_delay = actual_pulse_cycle_delay_cal - MIN_PULSE_WIDTH
        if inter_pulse_delay > 0:
            time.sleep(inter_pulse_delay)
        if i > 0 and i % 2000 == 0: time.sleep(0.001)

    if not homed_x:
        output_messages.append("Error: X-axis calibration failed (endstop not triggered). Stopping.")
        return output_messages
    
    current_x_mm = 0.0 
    output_messages.append(f"X-axis origin set at trigger point: {current_x_mm:.3f} mm.")
    output_messages.append(f"Backing off X-axis by {CALIBRATION_BACKOFF_MM:.2f} mm...")
    motor_msgs_bx = move_corexy(CALIBRATION_BACKOFF_MM, 0.0, actual_pulse_cycle_delay_cal) 
    if motor_msgs_bx: output_messages.extend(motor_msgs_bx)
    output_messages.append(f"X-axis backed off. New position X={current_x_mm:.3f} mm.")

    output_messages.append(f"--- Calibration Cycle Complete ---")
    output_messages.append(f"Final position after back-off: X={current_x_mm:.3f}, Y={current_y_mm:.3f} mm")
    absolute_mode = True 
    output_messages.append("Mode set to ABS (Absolute).")
    return output_messages

def capture_images(num_captures_requested, base_image_path):
    """
    Captures a specified number of images from the webcam.
    Saves images as base_image_path_0.jpg, base_image_path_1.jpg, etc.
    Assumes webcam is at index 0. Attempts to set 4K resolution.
    Returns a list of output messages.
    GStreamer warnings during camera initialization are suppressed.
    """
    messages = []
    cap = None  # Initialiser cap à None pour une gestion correcte dans le bloc finally

    # Tenter d'initialiser la caméra en supprimant les avertissements de console d'OpenCV/GStreamer
    with suppress_c_stderr(): #
        cap = cv2.VideoCapture(0)  # 0 est généralement la webcam par défaut

    if cap is None or not cap.isOpened(): 
        messages.append("Error: Could not open webcam. Is it connected and not in use?") #
        if cap is not None:
            with suppress_c_stderr(): 
                cap.release() 
        return messages 

    # À partir d'ici, on suppose que cap n'est pas None et est ouvert.
    try:
        target_width = 3840
        target_height = 2160
        messages.append(f"  Attempting to set resolution to {target_width}x{target_height}...")
        
        # Tenter de définir la largeur et la hauteur
        # cap.set() peut retourner False si la propriété n'est pas supportée ou ne peut être définie.
        set_w_ok = cap.set(cv2.CAP_PROP_FRAME_WIDTH, target_width)
        set_h_ok = cap.set(cv2.CAP_PROP_FRAME_HEIGHT, target_height)

        # Vérifier la résolution réellement appliquée par la caméra
        actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        
        if set_w_ok and set_h_ok and actual_width == target_width and actual_height == target_height:
            messages.append(f"  Successfully set resolution to {actual_width}x{actual_height}.")
        else:
            messages.append(f"  Warning: Could not set exact resolution to {target_width}x{target_height}.")
            messages.append(f"  Actual resolution reported by camera: {actual_width}x{actual_height}.")
            if not set_w_ok:
                 messages.append("  Note: cap.set(cv2.CAP_PROP_FRAME_WIDTH) returned False.")
            if not set_h_ok:
                 messages.append("  Note: cap.set(cv2.CAP_PROP_FRAME_HEIGHT) returned False.")
        # --- FIN DE L'AJOUT POUR LA RÉSOLUTION ---

        time.sleep(0.5)  # Permettre à la caméra de s'initialiser avec la nouvelle résolution
        actual_captures = 0 
        for i in range(num_captures_requested): 
            ret, frame = cap.read() 
            if not ret: 
                messages.append(f"Error: Could not read frame {i+1}/{num_captures_requested}.") #
                if actual_captures == 0 and i == 0: 
                    return messages 
                break 

            directory = os.path.dirname(base_image_path) 
            if directory and not os.path.exists(directory): 
                try:
                    os.makedirs(directory, exist_ok=True) 
                    messages.append(f"  Created directory: {directory}") 
                except OSError as e:
                    messages.append(f"  Error creating directory {directory}: {e}") 
                    return messages 
            
            filename_base = os.path.basename(base_image_path) 
            image_filename = os.path.join(directory, f"{filename_base}_{i}.jpg") 

            try:
                cv2.imwrite(image_filename, frame) 
                messages.append(f"  Image captured and saved: {image_filename}") 
                actual_captures += 1 
            except Exception as e:
                messages.append(f"  Error saving image {image_filename}: {e}") 
                if actual_captures == 0 and i == 0 : 
                     return messages 
                break 
            
            if i < num_captures_requested - 1: 
                time.sleep(0.2) 
                
        if actual_captures > 0: 
            messages.append(f"--- Capture complete. {actual_captures} image(s) saved. ---") 
        elif not (cap is None or not cap.isOpened()): #
             messages.append(f"--- Capture failed. No images saved. ---") 
        
    except Exception as e_inner: 
        messages.append(f"An unexpected error occurred during capture/saving: {e_inner}") 
    finally:
        if cap is not None: 
            with suppress_c_stderr(): 
                cap.release() 
    
    return messages 


def parse_command_and_execute(line):
    """
    Parses a custom command line and executes it.
    Returns a tuple: (list_of_output_messages, continue_running_boolean).
    """
    global current_x_mm, current_y_mm, absolute_mode, TARGET_SPEED_MM_S, XLIM_MM, YLIM_MM 
    
    output_messages = []
    command = line.strip().upper()
    # Keep original case for path in CAPTURE command
    parts_orig_case = line.strip().split()
    parts = command.split()
    instruction = parts[0] if parts else ""

    output_messages.append(f"CMD: {line.strip()}") # Show original case command

    if instruction == "ABS":
        absolute_mode = True
        output_messages.append("  Mode: Absolute Positioning (ABS)")
    elif instruction == "REL":
        absolute_mode = False
        output_messages.append("  Mode: Relative Positioning (REL)")
    elif instruction == "CALIBRATE" or instruction == "CAL":
        output_messages.append("  Calibration initiated...")
        cal_messages = perform_calibration_cycle()
        output_messages.extend(cal_messages)
    elif (instruction == "SET" and len(parts) > 1 and parts[1] == "HOME") or instruction == "SETHOME":
        output_messages.append("  Setting current position as HOME (0,0)...")
        current_x_mm = 0.0
        current_y_mm = 0.0
        absolute_mode = True 
        output_messages.append(f"  New logical origin set at current physical position.")
        output_messages.append(f"  Current Position: X={current_x_mm:.3f} mm, Y={current_y_mm:.3f} mm")
        output_messages.append("  Mode set to ABS (Absolute).")
    elif instruction == "HOME":
        output_messages.append("  Homing (HOME to logical 0,0):")
        target_x_abs = 0.0
        target_y_abs = 0.0
        
        dx_home = 0.0 
        dy_home = 0.0
        if absolute_mode:
            dx_home = target_x_abs - current_x_mm
            dy_home = target_y_abs - current_y_mm
        
        actual_homing_speed = min(HOMING_SPEED_MM_S, MAX_SPEED_MM_S)
        home_path_length_mm = math.sqrt(dx_home**2 + dy_home**2)

        if home_path_length_mm > (MM_PER_MICROSTEP / 2.0) and absolute_mode:
            output_messages.append(f"    Moving to 0,0 from {current_x_mm:.2f}, {current_y_mm:.2f} at {actual_homing_speed:.2f} mm/s")
            time_for_home_move_s = home_path_length_mm / actual_homing_speed
            
            delta_x_steps_cartesian_home = round(-dx_home * MICROSTEPS_PER_MM) 
            delta_y_steps_cartesian_home = round(dy_home * MICROSTEPS_PER_MM)
            steps_m1_home = delta_x_steps_cartesian_home + delta_y_steps_cartesian_home
            steps_m2_home = delta_x_steps_cartesian_home - delta_y_steps_cartesian_home
            num_iterations_home = max(abs(int(steps_m1_home)), abs(int(steps_m2_home)))

            if num_iterations_home > 0:
                pulse_delay_for_home_move = time_for_home_move_s / num_iterations_home
                actual_pulse_delay_for_home_move = max(MINIMUM_PULSE_CYCLE_DELAY, pulse_delay_for_home_move)
                motor_msgs = move_corexy(dx_home, dy_home, actual_pulse_delay_for_home_move)
                output_messages.extend(motor_msgs)
            else: 
                current_x_mm = target_x_abs
                current_y_mm = target_y_abs
        else: 
            current_x_mm = target_x_abs
            current_y_mm = target_y_abs
            if absolute_mode:
                 output_messages.append("    Already at (or very close to) logical 0,0.")
            else: 
                output_messages.append("    Logical position reset to 0,0. No physical movement in REL for this HOME.")
        output_messages.append(f"  New position: X={current_x_mm:.3f} mm, Y={current_y_mm:.3f} mm")
            
    elif instruction.startswith("S") and instruction != "MOVE": 
        try:
            speed_val_mm_s_req = 0.0
            if len(instruction) > 1 and instruction[1:].replace('.', '', 1).isdigit():
                 speed_val_mm_s_req = float(instruction[1:])
            elif len(parts) > 1 and parts[1].replace('.', '', 1).isdigit():
                 speed_val_mm_s_req = float(parts[1])
            else:
                output_messages.append(f"  Invalid S command format. Use S<value> or S <value>.")
                return output_messages, True

            if speed_val_mm_s_req > 0:
                if speed_val_mm_s_req > MAX_SPEED_MM_S: 
                    TARGET_SPEED_MM_S = MAX_SPEED_MM_S
                    output_messages.append(f"  Requested speed {speed_val_mm_s_req:.2f} mm/s exceeds limit of {MAX_SPEED_MM_S:.2f} mm/s.")
                    output_messages.append(f"  Global target speed set to MAX: {TARGET_SPEED_MM_S:.2f} mm/s")
                else:
                    TARGET_SPEED_MM_S = speed_val_mm_s_req
                    output_messages.append(f"  Global target speed set to: {TARGET_SPEED_MM_S:.2f} mm/s")
            else:
                output_messages.append(f"  Speed S must be positive (and <= {MAX_SPEED_MM_S:.2f} mm/s).")
        except ValueError:
            output_messages.append(f"  Invalid speed value in S command: {parts[1] if len(parts) > 1 else instruction[1:]}")
    
    elif instruction == "LIMITS":
        if len(parts) > 1:
            sub_command = parts[1].upper()
            if sub_command == "OFF":
                XLIM_MM = float('inf')
                YLIM_MM = float('inf')
                output_messages.append("  Coordinate limits DISABLED.")
                output_messages.append("  Warning: Moves can exceed default physical boundaries.")
            elif sub_command == "ON":
                XLIM_MM = DEFAULT_XLIM_MM
                YLIM_MM = DEFAULT_YLIM_MM
                output_messages.append(f"  Coordinate limits ENABLED (X: [0, {XLIM_MM:.0f}], Y: [0, {YLIM_MM:.0f}]).")
            else:
                output_messages.append("  Usage: LIMITS [ON|OFF]")
        else:
            x_lim_display = f"{XLIM_MM:.0f}" if XLIM_MM != float('inf') else "INF (Disabled)"
            y_lim_display = f"{YLIM_MM:.0f}" if YLIM_MM != float('inf') else "INF (Disabled)"
            output_messages.append(f"  Current effective limits: X=[0, {x_lim_display}], Y=[0, {y_lim_display}]")
            output_messages.append("  Use 'LIMITS ON' or 'LIMITS OFF' to change.")

    # --- NEW: CAPTURE Command ---
    elif instruction == "CAPTURE":
        if len(parts_orig_case) == 3: # CAPTURE N PATH
            num_images_str = parts_orig_case[1]
            image_base_path_str = parts_orig_case[2] # Use original case for path
            try:
                num_img = int(num_images_str)
                if num_img <= 0:
                    output_messages.append("  Error: Number of images (N) must be a positive integer.")
                elif not image_base_path_str:
                    output_messages.append("  Error: Image base path (PATH) cannot be empty.")
                else:
                    output_messages.append(f"  Initiating capture of {num_img} image(s) to path starting with '{image_base_path_str}'...")
                    # This function call might take some time
                    capture_msgs = capture_images(num_img, image_base_path_str) 
                    output_messages.extend(capture_msgs)
            except ValueError:
                output_messages.append(f"  Error: Invalid number of images '{num_images_str}'. Must be an integer.")
            except Exception as e: # Catch other potential errors from capture_images
                output_messages.append(f"  An error occurred during image capture: {e}")
        else:
            output_messages.append("  Usage: CAPTURE <N> <PATH>")
            output_messages.append("    N: Number of images to capture (e.g., 1, 3).")
            output_messages.append("    PATH: Base name and path for images (e.g., /tmp/shot or ./captures/img).")
            output_messages.append("          Images will be saved as PATH_0.jpg, PATH_1.jpg, etc.")
            
    elif instruction == "MOVE" :
        target_x_cmd = None
        target_y_cmd = None
        s_value_this_cmd_req = None 
        
        for part in parts[1:]: # Use uppercase parts for parsing X, Y, S parameters
            if part.startswith('X'):
                try: target_x_cmd = float(part[1:])
                except ValueError: output_messages.append(f"  Invalid X value: {part}"); return output_messages, True
            elif part.startswith('Y'):
                try: target_y_cmd = float(part[1:])
                except ValueError: output_messages.append(f"  Invalid Y value: {part}"); return output_messages, True
            elif part.startswith('S'):
                 try: 
                     s_value_this_cmd_req = float(part[1:])
                 except ValueError: output_messages.append(f"  Invalid S value in MOVE: {part}")

        if target_x_cmd is None and target_y_cmd is None:
            output_messages.append("  No X or Y coordinate specified for MOVE.")
            return output_messages, True

        current_move_speed_mm_s = TARGET_SPEED_MM_S 
        if s_value_this_cmd_req is not None:
            if s_value_this_cmd_req > 0:
                if s_value_this_cmd_req > MAX_SPEED_MM_S: 
                    current_move_speed_mm_s = MAX_SPEED_MM_S
                    output_messages.append(f"  Requested MOVE speed {s_value_this_cmd_req:.2f} mm/s exceeds limit of {MAX_SPEED_MM_S:.2f} mm/s.")
                    output_messages.append(f"  MOVE speed clamped to MAX: {current_move_speed_mm_s:.2f} mm/s.")
                else:
                    current_move_speed_mm_s = s_value_this_cmd_req
            else: 
                output_messages.append(f"  Speed S in MOVE must be positive. Using global {TARGET_SPEED_MM_S:.2f} mm/s (max {MAX_SPEED_MM_S:.2f} mm/s).")
        
        current_move_speed_mm_s = min(current_move_speed_mm_s, MAX_SPEED_MM_S)

        final_target_x_mm = current_x_mm
        final_target_y_mm = current_y_mm
        if absolute_mode:
            if target_x_cmd is not None: final_target_x_mm = target_x_cmd
            if target_y_cmd is not None: final_target_y_mm = target_y_cmd
        else: 
            if target_x_cmd is not None: final_target_x_mm = current_x_mm + target_x_cmd
            if target_y_cmd is not None: final_target_y_mm = current_y_mm + target_y_cmd
        
        actual_target_x_mm = max(0.0, min(final_target_x_mm, XLIM_MM))
        actual_target_y_mm = max(0.0, min(final_target_y_mm, YLIM_MM))

        clamped_output_needed = False
        if XLIM_MM != float('inf') or YLIM_MM != float('inf') : 
            if abs(actual_target_x_mm - final_target_x_mm) > 1e-9 or abs(actual_target_y_mm - final_target_y_mm) > 1e-9 : 
                 clamped_output_needed = True
        elif final_target_x_mm < 0 or final_target_y_mm < 0 : 
             clamped_output_needed = True


        if clamped_output_needed:
            output_messages.append(f"  Warning: Target ({final_target_x_mm:.2f}, {final_target_y_mm:.2f}) is out of effective bounds.")
            output_messages.append(f"           Will be clamped to ({actual_target_x_mm:.2f}, {actual_target_y_mm:.2f}).")

        delta_x_to_move = actual_target_x_mm - current_x_mm
        delta_y_to_move = actual_target_y_mm - current_y_mm
        
        path_length_mm = math.sqrt(delta_x_to_move**2 + delta_y_to_move**2)

        if path_length_mm < (MM_PER_MICROSTEP / 2.0):
            output_messages.append(f"  Move too small or already at target. Current: ({current_x_mm:.3f}, {current_y_mm:.3f}), Target: ({actual_target_x_mm:.3f}, {actual_target_y_mm:.3f})")
            current_x_mm = actual_target_x_mm 
            current_y_mm = actual_target_y_mm
            current_x_mm = round(current_x_mm,3) 
            current_y_mm = round(current_y_mm,3)
        else:
            delta_x_steps_cartesian = round(-delta_x_to_move * MICROSTEPS_PER_MM) 
            delta_y_steps_cartesian = round(delta_y_to_move * MICROSTEPS_PER_MM)
            steps_m1 = delta_x_steps_cartesian + delta_y_steps_cartesian
            steps_m2 = delta_x_steps_cartesian - delta_y_steps_cartesian
            num_iterations = max(abs(int(steps_m1)), abs(int(steps_m2)))

            if num_iterations == 0: 
                output_messages.append("  No motor steps required for this move (delta too small).")
                current_x_mm = actual_target_x_mm 
                current_y_mm = actual_target_y_mm
                current_x_mm = round(current_x_mm,3)
                current_y_mm = round(current_y_mm,3)
            else:
                if current_move_speed_mm_s <= 1e-6: 
                    output_messages.append(f"  Target speed {current_move_speed_mm_s:.2e} mm/s is too low. No move executed.")
                else:
                    time_for_move_s = path_length_mm / current_move_speed_mm_s
                    pulse_delay_for_this_move = time_for_move_s / num_iterations
                    actual_pulse_delay_for_this_move = max(MINIMUM_PULSE_CYCLE_DELAY, pulse_delay_for_this_move)
                    
                    effective_speed_mm_s = 0 
                    if (num_iterations * actual_pulse_delay_for_this_move) > 1e-9: 
                        effective_speed_mm_s = path_length_mm / (num_iterations * actual_pulse_delay_for_this_move)
                    else: 
                        effective_speed_mm_s = current_move_speed_mm_s 

                    output_messages.append(f"  Moving by dx={delta_x_to_move:.3f} mm, dy={delta_y_to_move:.3f} mm")
                    output_messages.append(f"  To X={actual_target_x_mm:.3f}, Y={actual_target_y_mm:.3f}")
                    output_messages.append(f"  Target speed (cruise): {current_move_speed_mm_s:.2f} mm/s. Effective cruise speed: {effective_speed_mm_s:.2f} mm/s.")
                    output_messages.append(f"  Cruise pulse cycle delay: {actual_pulse_delay_for_this_move*1000:.4f} ms.")
                    
                    motor_msgs = move_corexy(delta_x_to_move, delta_y_to_move, actual_pulse_delay_for_this_move)
                    output_messages.extend(motor_msgs)
        
        output_messages.append(f"  New position: X={current_x_mm:.3f} mm, Y={current_y_mm:.3f} mm")

    elif instruction == "POS":
        x_lim_display = f"{XLIM_MM:.0f}" if XLIM_MM != float('inf') else "INF (Disabled)"
        y_lim_display = f"{YLIM_MM:.0f}" if YLIM_MM != float('inf') else "INF (Disabled)"
        output_messages.append(f"  Current Position: X={current_x_mm:.3f} mm, Y={current_y_mm:.3f} mm")
        output_messages.append(f"  Target Speed: {TARGET_SPEED_MM_S:.2f} mm/s (Max settable: {MAX_SPEED_MM_S:.2f} mm/s)") 
        output_messages.append(f"  Mode: {'Absolute' if absolute_mode else 'Relative'}")
        output_messages.append(f"  Effective Limits: X=[0, {x_lim_display}], Y=[0, {y_lim_display}]")
        if endstop_x and endstop_y:
            output_messages.append(f"  Endstop Status: X={'ACTIVE (pressed)' if endstop_x.is_active else 'Inactive'}, Y={'ACTIVE (pressed)' if endstop_y.is_active else 'Inactive'}")


    elif instruction in ["EXIT", "QUIT"]:
        output_messages.append("  Exiting program.")
        return output_messages, False 
        
    else:
        if instruction: 
            output_messages.append(f"  Unknown or unsupported command: {instruction}")
    
    return output_messages, True 

def draw_ui(stdscr, header_lines, status_lines, command_output_lines, input_prompt):
    """Draws the entire UI in the curses window."""
    stdscr.clear()
    max_y, max_x = stdscr.getmaxyx()
    current_y = 0

    for i, line in enumerate(header_lines):
        if current_y < max_y:
            stdscr.addstr(current_y, 0, line[:max_x-1]) 
            current_y += 1
    if current_y < max_y: 
        stdscr.addstr(current_y, 0, "-" * (max_x -1) )
        current_y +=1

    for i, line in enumerate(status_lines):
        if current_y < max_y:
            stdscr.addstr(current_y, 0, line[:max_x-1])
            current_y += 1
    if current_y < max_y: 
        stdscr.addstr(current_y, 0, "-" * (max_x-1) )
        current_y +=1
    
    output_start_y = current_y
    available_lines_for_output = max_y - output_start_y - 2 
    
    start_index = 0
    if len(command_output_lines) > available_lines_for_output and available_lines_for_output > 0 :
        start_index = len(command_output_lines) - available_lines_for_output
    
    for i, line in enumerate(command_output_lines[start_index:]):
        if output_start_y + i < max_y - 2: 
            stdscr.addstr(output_start_y + i, 0, line[:max_x-1]) 
    
    current_y = max_y - 2 
    if current_y > 0 : 
         stdscr.addstr(current_y, 0, "-" * (max_x-1) )
    
    input_line_y = max_y - 1
    if input_line_y > 0: 
        stdscr.addstr(input_line_y, 0, input_prompt)
        stdscr.move(input_line_y, len(input_prompt)) 

    stdscr.refresh()

def _curses_main_loop(stdscr):
    """Main loop for the command-line interface, managed by curses."""
    global TARGET_SPEED_MM_S, current_x_mm, current_y_mm, absolute_mode, last_command_output, XLIM_MM, YLIM_MM

    curses.curs_set(1) 
    stdscr.nodelay(False)

    running = True
    while running:
        x_limit_display_str = f"{XLIM_MM:.0f}" if XLIM_MM != float('inf') else "INF (Disabled)"
        y_limit_display_str = f"{YLIM_MM:.0f}" if YLIM_MM != float('inf') else "INF (Disabled)"

        header = [
            "--- CoreXY CLI Controller (Custom Commands) ---",
            f"Max Settable Speed: {MAX_SPEED_MM_S:.2f} mm/s",
            f"Calibration Speed: {CALIBRATION_SPEED_MM_S} mm/s",
            f"Effective Limits: X=[0, {x_limit_display_str}], Y=[0, {y_limit_display_str}] mm",
            f"Resolution: {MM_PER_MICROSTEP} mm/microstep ({MICROSTEPS_PER_MM} microsteps/mm)",
            f"Motor Native Steps/Rev: {MOTOR_NATIVE_STEPS_PER_REV}, Driver Microstepping: 1/{DRIVER_MICROSTEP_DIVISOR}",
            f"Min Pulse Cycle Delay: {MINIMUM_PULSE_CYCLE_DELAY*1000:.3f} ms",
            "Available commands:",
            "  MOVE X<v> Y<v> [S<v>]",
            "  ABS, REL",
            "  HOME                   (Moves to logical 0,0 or resets coords)",
            "  SETHOME / SET HOME     (Sets current physical position as 0,0)",
            "  CALIBRATE / CAL        (Physical homing with endstops)",
            "  CAPTURE <N> <PATH>     (Captures N images to PATH_i.jpg)", # <-- Added CAPTURE help
            "  S<v> or S <v>", 
            "  LIMITS [ON|OFF]", 
            "  POS",
            "  EXIT/QUIT"
        ]
        status_info = [ 
            f"Current Position: X={current_x_mm:.3f} mm, Y={current_y_mm:.3f} mm",
            f"Target Speed: {TARGET_SPEED_MM_S:.2f} mm/s, Mode: {'ABS' if absolute_mode else 'REL'}"
        ]
        if endstop_x and endstop_y:
             status_info.append(f"Endstops: X={'TRIG' if endstop_x.is_active else 'open'}, Y={'TRIG' if endstop_y.is_active else 'open'}")
        
        input_prompt_text = "CoreXY > "
        draw_ui(stdscr, header, status_info, last_command_output, input_prompt_text) 
        
        input_line_y = stdscr.getmaxyx()[0] - 1 
        
        try:
            if input_line_y > 0 and input_line_y > (len(header) + len(status_info) +2) : 
                 stdscr.move(input_line_y, len(input_prompt_text)) 
                 curses.echo() 
                 cmd_line_bytes = stdscr.getstr(input_line_y, len(input_prompt_text), 120) # Increased buffer for longer paths
                 curses.noecho() 
                 cmd_line = cmd_line_bytes.decode('utf-8').strip()
            else: 
                cmd_line = "" 
                last_command_output = ["Terminal too small. Resize or EXIT."]

            if not cmd_line: 
                if not last_command_output or (last_command_output and last_command_output[-1] != "Terminal too small. Resize or EXIT."):
                    last_command_output = [] 
                continue

            last_command_output, running = parse_command_and_execute(cmd_line)

        except curses.error as e:
            curses.noecho() 
            last_command_output = [f"Curses error: {e}", "Try resizing terminal or type 'exit'."]
        except KeyboardInterrupt:
            curses.noecho() 
            last_command_output = ["Keyboard interrupt detected. Exiting..."]
            running = False
        except Exception as e: 
            curses.noecho() 
            last_command_output = [
                f"An unexpected error occurred: {e}",
                "Check command syntax or internal logic."
            ]
            
    status_info_final = [ 
        f"Current Position: X={current_x_mm:.3f} mm, Y={current_y_mm:.3f} mm",
        f"Target Speed: {TARGET_SPEED_MM_S:.2f} mm/s, Mode: {'ABS' if absolute_mode else 'REL'}"
    ]
    if not last_command_output or "Exiting program." not in last_command_output[-1]:
        last_command_output.append("Exiting program.")
    
    x_limit_display_str = f"{XLIM_MM:.0f}" if XLIM_MM != float('inf') else "INF (Disabled)"
    y_limit_display_str = f"{YLIM_MM:.0f}" if YLIM_MM != float('inf') else "INF (Disabled)"
    header_final = [
        "--- CoreXY CLI Controller (Custom Commands) ---",
        f"Max Settable Speed: {MAX_SPEED_MM_S:.2f} mm/s",
        f"Effective Limits: X=[0, {x_limit_display_str}], Y=[0, {y_limit_display_str}] mm",
        f"Resolution: {MM_PER_MICROSTEP} mm/microstep ({MICROSTEPS_PER_MM} microsteps/mm)",
        "Available commands: ... (EXIT/QUIT to close)" 
    ]
    try:
        draw_ui(stdscr, header_final, status_info_final, last_command_output, "Exited. ")
        stdscr.refresh()
        time.sleep(0.5) 
    except curses.error:
        pass 

if __name__ == "__main__":
    if setup_gpio():
        try:
            curses.wrapper(_curses_main_loop)
        except Exception as e: 
            print(f"A critical error occurred outside curses main loop: {e}")
            import traceback
            traceback.print_exc()
        finally:
            print("Cleaning up GPIO before exit...") 
            cleanup_gpio() 
            # It's good practice to also release camera if it was somehow left open,
            # though capture_images should handle its own release.
            # For robustness, you could add a global camera object and try to release it here if it exists.
            print("Program terminated.") 
    else:
        print("GPIO setup failed. Program will not start controller interface.")
        print("Program terminated.")