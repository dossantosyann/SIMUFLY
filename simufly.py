import gpiozero
import time
import math
import re # For command parsing
import curses # <-- Ajout pour l'interface CLI améliorée
import cv2 # <-- Added for OpenCV webcam capture
import os # <-- Added for path manipulation
import sys
from contextlib import contextmanager
from datetime import datetime # <-- Added for timestamp in automatic mode
import csv

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

DEFAULT_XLIM_MM = 850.0
DEFAULT_YLIM_MM = 1150.0
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

# --- Automatic Mode Scan Parameters --- 
POINT_MARGIN_MM = 10.0  # Marge des points de scan par rapport aux limites XLIM_MM, YLIM_MM

# --- System State ---
current_x_mm = 0.0
current_y_mm = 0.0
absolute_mode = True # True for ABS (absolute), False for REL (relative)

# --- Curses UI state ---
last_command_output = [] # For manual mode output

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

    target_cruise_delay = pulse_cycle_delay_for_move 
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

    output_messages.append(f"Calibrating Y-axis at {cal_speed:.2f} mm/s...")
    dir_device_m1.on(); dir_device_m2.off(); time.sleep(0.002)
    homed_y = False
    for i in range(max_steps_travel):
        if endstop_y.is_active:
            output_messages.append("Y-axis endstop triggered."); homed_y = True; break
        pul_device_m1.on(); pul_device_m2.on(); time.sleep(MIN_PULSE_WIDTH)
        pul_device_m1.off(); pul_device_m2.off()
        inter_pulse_delay = actual_pulse_cycle_delay_cal - MIN_PULSE_WIDTH
        if inter_pulse_delay > 0: time.sleep(inter_pulse_delay)
        if i > 0 and i % 2000 == 0: time.sleep(0.001) 
    if not homed_y:
        output_messages.append("Error: Y-axis calibration failed. Stopping."); return output_messages
    current_y_mm = 0.0
    output_messages.append(f"Y-axis origin set: {current_y_mm:.3f} mm. Backing off {CALIBRATION_BACKOFF_MM:.2f} mm...")
    motor_msgs_by = move_corexy(0.0, CALIBRATION_BACKOFF_MM, actual_pulse_cycle_delay_cal) 
    if motor_msgs_by: output_messages.extend(motor_msgs_by)
    output_messages.append(f"Y-axis backed off. New Y={current_y_mm:.3f} mm.")

    output_messages.append(f"Calibrating X-axis at {cal_speed:.2f} mm/s...")
    dir_device_m1.off(); dir_device_m2.off(); time.sleep(0.002)
    homed_x = False
    for i in range(max_steps_travel):
        if endstop_x.is_active: 
            output_messages.append("X-axis endstop triggered."); homed_x = True; break
        pul_device_m1.on(); pul_device_m2.on(); time.sleep(MIN_PULSE_WIDTH)
        pul_device_m1.off(); pul_device_m2.off()
        inter_pulse_delay = actual_pulse_cycle_delay_cal - MIN_PULSE_WIDTH
        if inter_pulse_delay > 0: time.sleep(inter_pulse_delay)
        if i > 0 and i % 2000 == 0: time.sleep(0.001)
    if not homed_x:
        output_messages.append("Error: X-axis calibration failed. Stopping."); return output_messages
    current_x_mm = 0.0 
    output_messages.append(f"X-axis origin set: {current_x_mm:.3f} mm. Backing off {CALIBRATION_BACKOFF_MM:.2f} mm...")
    motor_msgs_bx = move_corexy(CALIBRATION_BACKOFF_MM, 0.0, actual_pulse_cycle_delay_cal) 
    if motor_msgs_bx: output_messages.extend(motor_msgs_bx)
    output_messages.append(f"X-axis backed off. New X={current_x_mm:.3f} mm.")
    output_messages.append(f"--- Calibration Complete --- Final: X={current_x_mm:.3f}, Y={current_y_mm:.3f} mm")
    absolute_mode = True; output_messages.append("Mode set to ABS.")
    return output_messages

def capture_images(num_captures_requested, base_image_path_with_prefix): # Renamed arg for clarity
    """
    Captures a specified number of images from the webcam.
    Saves images as base_image_path_with_prefix_Id00.tiff, _Id01.tiff, etc.
    """
    messages = []
    cap = None
    with suppress_c_stderr(): cap = cv2.VideoCapture(0)
    if cap is None or not cap.isOpened(): 
        messages.append("Erreur: Impossible d'ouvrir la webcam.")
        if cap is not None: 
            with suppress_c_stderr(): cap.release()
        return messages 
    try:
        target_width, target_height = 3840, 2160 # Tentative de résolution 4K
        # ... (logique de réglage de la résolution, identique à avant) ...
        set_w_ok = cap.set(cv2.CAP_PROP_FRAME_WIDTH, target_width)
        set_h_ok = cap.set(cv2.CAP_PROP_FRAME_HEIGHT, target_height)
        actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        if set_w_ok and set_h_ok and actual_width == target_width and actual_height == target_height:
            messages.append(f"  Résolution réglée sur {actual_width}x{actual_height}.")
        else:
            messages.append(f"  Avertissement: Résolution actuelle: {actual_width}x{actual_height}.")
        
        time.sleep(0.5) # Laisser la caméra s'initialiser
        actual_captures = 0 
        for i in range(num_captures_requested): 
            ret, frame = cap.read() 
            if not ret: 
                messages.append(f"Erreur: Lecture frame {i+1}/{num_captures_requested} impossible.")
                if actual_captures == 0 and i == 0: return messages 
                break 
            
            # base_image_path_with_prefix contient déjà le chemin du dossier et le préfixe BXX_ImXX
            directory = os.path.dirname(base_image_path_with_prefix)
            filename_prefix_only = os.path.basename(base_image_path_with_prefix) # Ex: B00_Im00

            # --- MODIFICATION NOMENCLATURE ---
            image_filename = os.path.join(directory, f"{filename_prefix_only}_Id{i:02d}.tiff") 
            
            try:
                # S'assurer que le dossier existe (normalement créé par _automatic_mode_loop)
                if directory and not os.path.exists(directory):
                    os.makedirs(directory, exist_ok=True)
                    messages.append(f"  Création dossier (sécurité): {directory}")

                cv2.imwrite(image_filename, frame)
                messages.append(f"  Image capturée: {os.path.basename(image_filename)}") # Afficher juste le nom du fichier
                actual_captures += 1 
            except Exception as e:
                messages.append(f"  Erreur sauvegarde image {os.path.basename(image_filename)}: {e}") 
                if actual_captures == 0 and i == 0 : return messages 
                break 
            if i < num_captures_requested - 1: time.sleep(0.2) 
        
        if actual_captures > 0: messages.append(f"--- Capture Position: {actual_captures} image(s) TIFF sauvegardée(s). ---") 
        elif not (cap is None or not cap.isOpened()): messages.append(f"--- Capture Position: Échec, aucune image sauvegardée. ---") 
    except Exception as e_inner: messages.append(f"Erreur inattendue pendant capture: {e_inner}") 
    finally:
        if cap is not None: 
            with suppress_c_stderr(): cap.release() 
    return messages

def parse_command_and_execute(line):
    global current_x_mm, current_y_mm, absolute_mode, TARGET_SPEED_MM_S, XLIM_MM, YLIM_MM
    output_messages = []
    command = line.strip().upper()
    parts_orig_case = line.strip().split()
    parts = command.split()
    instruction = parts[0] if parts else ""
    output_messages.append(f"CMD: {line.strip()}")

    if instruction == "ABS":
        absolute_mode = True; output_messages.append("  Mode: Absolute (ABS)")
    elif instruction == "REL":
        absolute_mode = False; output_messages.append("  Mode: Relative (REL)")
    elif instruction == "CALIBRATE" or instruction == "CAL":
        output_messages.append("  Calibration initiated..."); cal_messages = perform_calibration_cycle(); output_messages.extend(cal_messages)
    elif (instruction == "SET" and len(parts) > 1 and parts[1] == "HOME") or instruction == "SETHOME":
        output_messages.append("  Setting current position as HOME (0,0)...")
        current_x_mm = 0.0; current_y_mm = 0.0; absolute_mode = True
        output_messages.append(f"  New logical origin. Current Pos: X={current_x_mm:.3f}, Y={current_y_mm:.3f}. Mode: ABS.")
    elif instruction == "HOME":
        output_messages.append("  Homing (to logical 0,0):")
        target_x_abs, target_y_abs = 0.0, 0.0
        dx_home, dy_home = 0.0, 0.0
        if absolute_mode: dx_home = target_x_abs - current_x_mm; dy_home = target_y_abs - current_y_mm
        actual_homing_speed = min(HOMING_SPEED_MM_S, MAX_SPEED_MM_S)
        home_path_length_mm = math.sqrt(dx_home**2 + dy_home**2)
        if home_path_length_mm > (MM_PER_MICROSTEP / 2.0) and absolute_mode:
            output_messages.append(f"    Moving to 0,0 from {current_x_mm:.2f},{current_y_mm:.2f} at {actual_homing_speed:.2f} mm/s")
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
            else: current_x_mm = target_x_abs; current_y_mm = target_y_abs
        else:
            current_x_mm = target_x_abs; current_y_mm = target_y_abs
            output_messages.append("    Already at logical 0,0." if absolute_mode else "    Logical pos reset to 0,0. No physical move in REL.")
        output_messages.append(f"  New position: X={current_x_mm:.3f}, Y={current_y_mm:.3f}")
    elif instruction.startswith("S") and instruction != "MOVE":
        try:
            speed_val_mm_s_req = 0.0
            if len(instruction) > 1 and instruction[1:].replace('.', '', 1).isdigit(): speed_val_mm_s_req = float(instruction[1:])
            elif len(parts) > 1 and parts[1].replace('.', '', 1).isdigit(): speed_val_mm_s_req = float(parts[1])
            else: output_messages.append(f"  Invalid S format."); return output_messages, True
            if speed_val_mm_s_req > 0:
                if speed_val_mm_s_req > MAX_SPEED_MM_S:
                    TARGET_SPEED_MM_S = MAX_SPEED_MM_S
                    output_messages.append(f"  Requested speed {speed_val_mm_s_req:.2f} > limit {MAX_SPEED_MM_S:.2f}. Speed set to MAX.")
                else: TARGET_SPEED_MM_S = speed_val_mm_s_req; output_messages.append(f"  Global target speed set to: {TARGET_SPEED_MM_S:.2f} mm/s")
            else: output_messages.append(f"  Speed S must be positive (<= {MAX_SPEED_MM_S:.2f} mm/s).")
        except ValueError: output_messages.append(f"  Invalid speed value in S: {parts[1] if len(parts) > 1 else instruction[1:]}")
    elif instruction == "LIMITS":
        if len(parts) > 1:
            sub_command = parts[1].upper()
            if sub_command == "OFF": XLIM_MM, YLIM_MM = float('inf'), float('inf'); output_messages.append("  Coordinate limits DISABLED.")
            elif sub_command == "ON": XLIM_MM, YLIM_MM = DEFAULT_XLIM_MM, DEFAULT_YLIM_MM; output_messages.append(f"  Limits ENABLED (X:[0,{XLIM_MM:.0f}], Y:[0,{YLIM_MM:.0f}]).")
            else: output_messages.append("  Usage: LIMITS [ON|OFF]")
        else:
            x_disp = f"{XLIM_MM:.0f}" if XLIM_MM!=float('inf') else "INF"; y_disp = f"{YLIM_MM:.0f}" if YLIM_MM!=float('inf') else "INF"
            output_messages.append(f"  Limits: X=[0, {x_disp}], Y=[0, {y_disp}]. Use 'LIMITS ON/OFF'.")
    elif instruction == "CAPTURE":
        if len(parts_orig_case) == 3:
            try:
                num_img = int(parts_orig_case[1])
                image_base_path_str = parts_orig_case[2]
                if num_img <= 0: output_messages.append("  Error: N must be positive.")
                elif not image_base_path_str: output_messages.append("  Error: PATH cannot be empty.")
                else:
                    output_messages.append(f"  Capturing {num_img} image(s) to '{image_base_path_str}'...");
                    capture_msgs = capture_images(num_img, image_base_path_str); output_messages.extend(capture_msgs)
            except ValueError: output_messages.append(f"  Error: Invalid N '{parts_orig_case[1]}'. Must be integer.")
            except Exception as e: output_messages.append(f"  Capture error: {e}")
        else: output_messages.append("  Usage: CAPTURE <N> <PATH> (e.g. CAPTURE 1 ./shot)")
    elif instruction == "MOVE" :
        target_x_cmd, target_y_cmd, s_value_this_cmd_req = None, None, None
        for part in parts[1:]:
            # --- CORRECTED SECTION START ---
            if part.startswith('X'):
                try:
                    target_x_cmd = float(part[1:])
                except ValueError:
                    output_messages.append(f"  Invalid X: {part}")
                    return output_messages, True
            elif part.startswith('Y'):
                try:
                    target_y_cmd = float(part[1:])
                except ValueError:
                    output_messages.append(f"  Invalid Y: {part}")
                    return output_messages, True
            elif part.startswith('S'):
                try:
                    s_value_this_cmd_req = float(part[1:])
                except ValueError:
                    output_messages.append(f"  Invalid S in MOVE: {part}")
            # --- CORRECTED SECTION END ---

        if target_x_cmd is None and target_y_cmd is None: output_messages.append("  No X or Y for MOVE."); return output_messages, True
        current_move_speed_mm_s = TARGET_SPEED_MM_S
        if s_value_this_cmd_req is not None:
            if s_value_this_cmd_req > 0:
                if s_value_this_cmd_req > MAX_SPEED_MM_S: current_move_speed_mm_s = MAX_SPEED_MM_S; output_messages.append(f"  MOVE speed {s_value_this_cmd_req:.2f} > limit. Clamped to MAX: {current_move_speed_mm_s:.2f}.")
                else: current_move_speed_mm_s = s_value_this_cmd_req
            else: output_messages.append(f"  Speed S in MOVE must be positive. Using global {TARGET_SPEED_MM_S:.2f}.")
        current_move_speed_mm_s = min(current_move_speed_mm_s, MAX_SPEED_MM_S)
        final_target_x_mm, final_target_y_mm = current_x_mm, current_y_mm
        if absolute_mode:
            if target_x_cmd is not None: final_target_x_mm = target_x_cmd
            if target_y_cmd is not None: final_target_y_mm = target_y_cmd
        else:
            if target_x_cmd is not None: final_target_x_mm = current_x_mm + target_x_cmd
            if target_y_cmd is not None: final_target_y_mm = current_y_mm + target_y_cmd
        actual_target_x_mm = max(0.0, min(final_target_x_mm, XLIM_MM))
        actual_target_y_mm = max(0.0, min(final_target_y_mm, YLIM_MM))
        clamped = False
        if XLIM_MM!=float('inf') or YLIM_MM!=float('inf'):
            if abs(actual_target_x_mm-final_target_x_mm)>1e-9 or abs(actual_target_y_mm-final_target_y_mm)>1e-9 : clamped=True
        elif final_target_x_mm < 0 or final_target_y_mm < 0 : clamped=True
        if clamped: output_messages.append(f"  Warn: Target ({final_target_x_mm:.2f},{final_target_y_mm:.2f}) out of bounds. Clamped to ({actual_target_x_mm:.2f},{actual_target_y_mm:.2f}).")
        delta_x_to_move, delta_y_to_move = actual_target_x_mm - current_x_mm, actual_target_y_mm - current_y_mm
        path_length_mm = math.sqrt(delta_x_to_move**2 + delta_y_to_move**2)
        if path_length_mm < (MM_PER_MICROSTEP / 2.0):
            output_messages.append(f"  Move too small. Current:({current_x_mm:.3f},{current_y_mm:.3f}), Target:({actual_target_x_mm:.3f},{actual_target_y_mm:.3f})")
            current_x_mm,current_y_mm = round(actual_target_x_mm,3), round(actual_target_y_mm,3)
        else:
            delta_x_steps_cartesian = round(-delta_x_to_move * MICROSTEPS_PER_MM)
            delta_y_steps_cartesian = round(delta_y_to_move * MICROSTEPS_PER_MM)
            steps_m1, steps_m2 = (delta_x_steps_cartesian + delta_y_steps_cartesian), (delta_x_steps_cartesian - delta_y_steps_cartesian)
            num_iterations = max(abs(int(steps_m1)), abs(int(steps_m2)))
            if num_iterations == 0:
                output_messages.append("  No motor steps for move."); current_x_mm,current_y_mm = round(actual_target_x_mm,3),round(actual_target_y_mm,3)
            else:
                if current_move_speed_mm_s <= 1e-6: output_messages.append(f"  Speed {current_move_speed_mm_s:.2e} too low. No move.")
                else:
                    time_for_move_s = path_length_mm / current_move_speed_mm_s
                    pulse_delay_for_this_move = time_for_move_s / num_iterations
                    actual_pulse_delay_for_this_move = max(MINIMUM_PULSE_CYCLE_DELAY, pulse_delay_for_this_move)
                    effective_speed_mm_s = path_length_mm / (num_iterations * actual_pulse_delay_for_this_move) if (num_iterations * actual_pulse_delay_for_this_move) > 1e-9 else current_move_speed_mm_s
                    output_messages.append(f"  Moving by dx={delta_x_to_move:.3f}, dy={delta_y_to_move:.3f} to X={actual_target_x_mm:.3f}, Y={actual_target_y_mm:.3f}")
                    output_messages.append(f"  Target speed: {current_move_speed_mm_s:.2f}. Effective: {effective_speed_mm_s:.2f} mm/s. Pulse delay: {actual_pulse_delay_for_this_move*1000:.4f} ms.")
                    motor_msgs = move_corexy(delta_x_to_move, delta_y_to_move, actual_pulse_delay_for_this_move)
                    output_messages.extend(motor_msgs)
        output_messages.append(f"  New position: X={current_x_mm:.3f} mm, Y={current_y_mm:.3f} mm")
    elif instruction == "POS":
        x_disp = f"{XLIM_MM:.0f}" if XLIM_MM!=float('inf') else "INF"; y_disp = f"{YLIM_MM:.0f}" if YLIM_MM!=float('inf') else "INF"
        output_messages.append(f"  Pos: X={current_x_mm:.3f}, Y={current_y_mm:.3f}. Speed: {TARGET_SPEED_MM_S:.2f} (Max: {MAX_SPEED_MM_S:.2f}). Mode: {'ABS' if absolute_mode else 'REL'}.")
        output_messages.append(f"  Limits: X=[0,{x_disp}], Y=[0,{y_disp}]. Endstops: X={'TRIG' if endstop_x.is_active else 'open'}, Y={'TRIG' if endstop_y.is_active else 'open'}")
    elif instruction in ["EXIT", "QUIT", "MENU"]:
        output_messages.append("  Returning to main menu..."); return output_messages, False
    else:
        if instruction: output_messages.append(f"  Unknown command: {instruction}")
    return output_messages, True

def draw_manual_mode_ui(stdscr, header_lines, status_lines, command_output_lines, input_prompt):
    """Draws the entire Manual Mode UI in the curses window."""
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
    # available_lines_for_output: Lignes restantes pour la sortie des commandes,
    # moins la ligne de séparation et la ligne d'invite.
    available_lines_for_output = max_y - output_start_y - 2
    
    # --- Correction/Vérification pour start_index ---
    start_index = 0 # Initialiser start_index par défaut
    if available_lines_for_output < 0: # Si la fenêtre est trop petite
        available_lines_for_output = 0

    if len(command_output_lines) > available_lines_for_output:
        start_index = len(command_output_lines) - available_lines_for_output
    # --- Fin de la correction/vérification ---
    
    for i, line in enumerate(command_output_lines[start_index:]):
        # S'assurer de ne pas écrire sur les deux dernières lignes (séparateur et invite)
        if output_start_y + i < max_y - 2:
            stdscr.addstr(output_start_y + i, 0, line[:max_x-1])
    
    current_y = max_y - 2 # Ligne pour le séparateur au-dessus de l'invite
    if current_y >= 0 and current_y < max_y : # Vérifier que current_y est valide
         stdscr.addstr(current_y, 0, "-" * (max_x-1) )
    
    input_line_y = max_y - 1 # Ligne pour l'invite de commande
    if input_line_y >=0 and input_line_y < max_y: # Vérifier que input_line_y est valide
        stdscr.addstr(input_line_y, 0, input_prompt)
        # S'assurer que le curseur est positionné à un endroit valide
        if len(input_prompt) < max_x:
            stdscr.move(input_line_y, len(input_prompt))
        else: # Cas où le prompt lui-même est trop long (improbable mais défensif)
            stdscr.move(input_line_y, max_x -1)


    stdscr.refresh()

def _manual_mode_loop(stdscr):
    global TARGET_SPEED_MM_S, current_x_mm, current_y_mm, absolute_mode, last_command_output, XLIM_MM, YLIM_MM
    curses.curs_set(1); stdscr.nodelay(False)
    last_command_output = ["--- Switched to Mode Manuel ---", "Type 'MENU' or 'EXIT' to return."]
    running_manual_mode = True
    while running_manual_mode:
        x_lim_disp = f"{XLIM_MM:.0f}" if XLIM_MM!=float('inf') else "INF"; y_lim_disp = f"{YLIM_MM:.0f}" if YLIM_MM!=float('inf') else "INF"
        header = [
            "--- SIMUFLY : Mode Manuel ---",
            f"Max Speed: {MAX_SPEED_MM_S:.2f}, Cal Speed: {CALIBRATION_SPEED_MM_S}, Limits: X=[0,{x_lim_disp}], Y=[0,{y_lim_disp}]",
            f"Resolution: {MM_PER_MICROSTEP} mm/µstep ({MICROSTEPS_PER_MM} µsteps/mm)", "",
            "Cmds: MOVE X Y [S], ABS, REL, HOME, SETHOME, CAL, CAPTURE, S, LIMITS, POS, MENU/EXIT"
        ]
        status_info = [f"Pos: X={current_x_mm:.3f}, Y={current_y_mm:.3f} mm. Speed: {TARGET_SPEED_MM_S:.2f} mm/s. Mode: {'ABS' if absolute_mode else 'REL'}"]
        if endstop_x and endstop_y: status_info.append(f"Endstops: X={'TRIG' if endstop_x.is_active else 'open'}, Y={'TRIG' if endstop_y.is_active else 'open'}")
        input_prompt_text = "Manuel > "
        draw_manual_mode_ui(stdscr, header, status_info, last_command_output, input_prompt_text) 
        input_line_y = stdscr.getmaxyx()[0] - 1 
        try:
            if input_line_y > 0 and input_line_y > (len(header) + len(status_info) +2) : 
                 stdscr.move(input_line_y, len(input_prompt_text)); curses.echo() 
                 cmd_line = stdscr.getstr(input_line_y, len(input_prompt_text), 120).decode('utf-8').strip(); curses.noecho() 
            else: cmd_line = ""; last_command_output = ["Terminal too small. Resize or type 'MENU'."]
            if not cmd_line: 
                if not last_command_output or (last_command_output and last_command_output[-1] != "Terminal too small. Resize or type 'MENU'." ): last_command_output = [] 
                continue
            last_command_output, running_manual_mode = parse_command_and_execute(cmd_line)
        except curses.error as e: curses.noecho(); last_command_output = [f"Curses error: {e}", "Try resize or 'MENU'."]
        except KeyboardInterrupt: curses.noecho(); last_command_output = ["Interruption. Retour au menu..."]; running_manual_mode = False
        except Exception as e: curses.noecho(); last_command_output = [f"Erreur: {e}", "Vérifiez la commande."]
    last_command_output = []


# --- Curses Input Helper Functions ---
def _get_string_input_curses(stdscr, y, x, prompt_text, max_len=50):
    """
    Gets a string input from the user in a curses window.
    Handles ESC for cancellation (returns None).
    Handles Backspace/Delete for editing.
    Handles Left/Right arrow keys for cursor movement within the input.
    """
    # curses.echo() # We are handling echo manually via addstr
    curses.noecho() # Turn off automatic echo
    stdscr.keypad(True) # Ensure special keys are captured

    stdscr.addstr(y, x, prompt_text)
    stdscr.clrtoeol() # Clear anything after the prompt
    
    input_chars = [] # Store characters as a list for easier manipulation
    cursor_display_pos = 0 # Visual cursor position relative to start of input field
    
    # Initial display of prompt
    stdscr.move(y, x + len(prompt_text))
    stdscr.refresh()

    while True:
        # Display current input string
        current_input_str = "".join(input_chars)
        stdscr.move(y, x + len(prompt_text)) # Move to start of input area
        stdscr.clrtoeol() # Clear old input string
        stdscr.addstr(y, x + len(prompt_text), current_input_str)
        
        # Place cursor at its current position within the displayed string
        stdscr.move(y, x + len(prompt_text) + cursor_display_pos)
        stdscr.refresh()

        key = stdscr.getch()

        if key == 27: # ESC key
            # curses.echo() # Restore echo if it was on before, but we set noecho
            return None # Signal cancellation
        elif key == curses.KEY_ENTER or key == 10 or key == 13: # Enter key
            # curses.echo()
            return "".join(input_chars)
        elif key == curses.KEY_BACKSPACE or key == 127 or key == 8: # Backspace
            if cursor_display_pos > 0:
                input_chars.pop(cursor_display_pos - 1)
                cursor_display_pos -= 1
        elif key == curses.KEY_DC: # Delete key
            if cursor_display_pos < len(input_chars):
                input_chars.pop(cursor_display_pos)
        elif key == curses.KEY_LEFT:
            if cursor_display_pos > 0:
                cursor_display_pos -= 1
        elif key == curses.KEY_RIGHT:
            if cursor_display_pos < len(input_chars):
                cursor_display_pos += 1
        elif (key >= 32 and key <= 126) or \
             (key >= 160 and key <= 255): # Caractères ASCII imprimables et Latin-1 courants
            if len(input_chars) < max_len:
                try:
                    input_chars.insert(cursor_display_pos, chr(key))
                    cursor_display_pos += 1
                except ValueError:
                    # Au cas où chr(key) échouerait pour une raison imprévue pour ces plages
                    pass 
        # Le reste de la fonction _get_string_input_curses reste inchangé
        # Ignore other special keys for simplicity

def _get_int_input_curses(stdscr, y, x, prompt_text, min_val=None, max_val=None):
    """Gets an integer input, validates, and returns it or None."""
    # Ensures keypad is true for the string input part too
    stdscr.keypad(True)
    while True:
        # _get_string_input_curses now handles ESC and can return None
        input_str_obj = _get_string_input_curses(stdscr, y, x, prompt_text, 20) # Max 20 chars for an int

        if input_str_obj is None: # ESC was pressed in _get_string_input_curses
            return None

        current_input_str = input_str_obj # It's a string if not None
        
        # Clear potential previous error message line (y+1)
        stdscr.move(y + 1, x); stdscr.clrtoeol()

        if not current_input_str: # User pressed Enter on empty string
            stdscr.addstr(y + 1, x, "Veuillez entrer une valeur. Réessayez.")
            stdscr.clrtoeol()
            stdscr.refresh()
            time.sleep(1) # Show message briefly
            stdscr.move(y + 1, x); stdscr.clrtoeol() # Clear message
            continue # Retry getting string input

        try:
            val = int(current_input_str)
            valid_range = True
            if min_val is not None and val < min_val: valid_range = False
            if max_val is not None and val > max_val: valid_range = False
            
            if not valid_range:
                range_err_msg = ""
                if min_val is not None and max_val is not None:
                    range_err_msg = f" (Doit être entre {min_val} et {max_val})"
                elif min_val is not None:
                    range_err_msg = f" (Doit être >= {min_val})"
                elif max_val is not None:
                    range_err_msg = f" (Doit être <= {max_val})"
                
                stdscr.addstr(y + 1, x, f"Valeur invalide{range_err_msg}. Réessayez.")
                stdscr.clrtoeol()
                stdscr.refresh()
                time.sleep(1.5) # Show error for a bit
                stdscr.move(y + 1, x); stdscr.clrtoeol() # Clear error
                continue # Retry input
            return val
        except ValueError:
            stdscr.addstr(y + 1, x, "Entrée numérique invalide. Réessayez.")
            stdscr.clrtoeol()
            stdscr.refresh()
            time.sleep(1.5) # Show error for a bit
            stdscr.move(y + 1, x); stdscr.clrtoeol() # Clear error
            continue # Retry input

# --- Automatic Mode Functions ---

# NOUVELLE FONCTION
def _calculate_scan_positions(nx, ny, current_xlim_mm, current_ylim_mm, point_margin):
    """
    Calcule les positions de scan en serpentin à l'intérieur des limites données.
    current_xlim_mm et current_ylim_mm définissent les coordonnées maximales utilisables.
    point_margin est la distance des points extérieurs par rapport aux limites.
    Retourne une liste de tuples (x, y) ou (None, message_erreur).
    """
    positions = []
    
    if nx <= 0 or ny <= 0:
        return None, "Le nombre de points X (nx) et Y (ny) doit être positif."

    # Zone effective pour le placement des centres des points de capture
    scan_area_x_min = point_margin
    scan_area_x_max = current_xlim_mm - point_margin
    scan_area_y_min = point_margin
    scan_area_y_max = current_ylim_mm - point_margin

    # Vérifier si la zone de scan est valide
    if scan_area_x_min > scan_area_x_max : # Vérification plus simple
        if nx == 1 and current_xlim_mm >=0: # Un seul point peut être au centre
             pass # Ok pour nx=1 si xlim >= 0, le point sera à xlim/2
        else:
             return None, f"Largeur X ({current_xlim_mm}mm) insuffisante pour {nx} points avec marge de {point_margin}mm."
    
    if scan_area_y_min > scan_area_y_max :
        if ny == 1 and current_ylim_mm >=0:
            pass # Ok pour ny=1
        else:
            return None, f"Hauteur Y ({current_ylim_mm}mm) insuffisante pour {ny} points avec marge de {point_margin}mm."

    x_coords_list = []
    if nx == 1:
        # Centrer le point si current_xlim_mm est défini.
        # S'assurer que XLIM_MM n'est pas inférieur à la marge d'une manière qui rendrait le calcul illogique.
        # Pour un seul point, le placer au centre de la zone totale XLIM_MM.
        x_coords_list.append(current_xlim_mm / 2.0)
    else:
        width_for_points = scan_area_x_max - scan_area_x_min
        step_x = width_for_points / (nx - 1)
        for i in range(nx):
            x_coords_list.append(scan_area_x_min + i * step_x)

    y_coords_list = []
    if ny == 1:
        y_coords_list.append(current_ylim_mm / 2.0)
    else:
        height_for_points = scan_area_y_max - scan_area_y_min
        step_y = height_for_points / (ny - 1)
        for i in range(ny):
            y_coords_list.append(scan_area_y_min + i * step_y)

    # Générer le chemin en serpentin
    for i_y in range(ny):
        current_y_val = y_coords_list[i_y]
        
        # Pour le serpentin, inverser les coordonnées x sur les lignes impaires
        current_x_row_iter = x_coords_list if i_y % 2 == 0 else reversed(x_coords_list)
        
        for current_x_val in current_x_row_iter:
            # S'assurer que les points sont dans les limites absolues [0, XLIM_MM] et [0, YLIM_MM]
            # Le calcul utilisant scan_area_min/max devrait déjà garantir cela.
            # Arrondir pour la propreté et pour éviter les problèmes de flottants minimes.
            final_x = round(max(0.0, min(current_x_val, current_xlim_mm)), 3)
            final_y = round(max(0.0, min(current_y_val, current_ylim_mm)), 3)
            positions.append((final_x, final_y))
            
    return positions, f"{len(positions)} positions calculées avec succès."

def _execute_automatic_scan(stdscr, project_path, positions_list,
                            num_y_bands, num_x_images_per_band, imgs_per_pos_at_location,
                            base_ui_prompts, base_ui_values): # Pour référence de la hauteur de l'UI
    """
    Exécute la séquence de scan automatique : déplacement et capture d'images.
    """
    global current_x_mm, current_y_mm, TARGET_SPEED_MM_S, MM_PER_MICROSTEP, \
           MICROSTEPS_PER_MM, MINIMUM_PULSE_CYCLE_DELAY

    previous_cursor_visibility = curses.curs_set(0) # Cacher le curseur, sauvegarder l'état précédent

    h, w = stdscr.getmaxyx()

    # --- CORRECTION DU CALCUL DE LA LIGNE DE DÉBUT DU STATUT ---
    # L'affichage des paramètres par draw_automatic_mode_ui se structure comme suit:
    # Ligne 1: Titre de la page
    # Ligne 2: Séparateur sous le titre
    # Ligne 3: Vide (implicitement)
    # Ligne 4 à (4 + len(base_ui_prompts) - 1): Les invites de paramètres
    last_prompt_line_idx = 4 + len(base_ui_prompts) - 1
    status_start_line_y = last_prompt_line_idx + 2 # Laisse une ligne vide après les prompts

    # S'assurer que status_start_line_y est dans les limites de l'écran
    if status_start_line_y >= h - 5: # Garder au moins 5 lignes pour le statut du scan
        status_start_line_y = h - 5
    if status_start_line_y < 0 : # Cas extrême d'écran très petit
        status_start_line_y = 0
    # --- FIN CORRECTION ---

    # Effacer la zone de statut avant de commencer
    for i in range(6): # Nombre de lignes à potentiellement utiliser pour le statut
        if status_start_line_y + i < h: # Vérifier les limites
            stdscr.move(status_start_line_y + i, 1)
            stdscr.clrtoeol()
    stdscr.refresh()

    global_image_num_for_prefix = 0
    scan_aborted = False
    final_scan_message = "Scan non initié."

    for pos_idx, target_pos_coords in enumerate(positions_list):
        # Vérifier si l'on a encore de la place pour afficher les messages de statut
        if status_start_line_y + 4 >= h : # Si pas assez de place pour tous les messages
             # Afficher un message minimal et continuer, ou gérer l'erreur
             stdscr.move(h-1, 1); stdscr.clrtoeol()
             stdscr.addstr(h-1, 1, f"Scan {pos_idx+1}/{len(positions_list)}...", curses.A_REVERSE)
             stdscr.refresh()
        else: # Affichage normal du statut
            target_x, target_y = target_pos_coords
            current_band_idx_for_naming = pos_idx // num_x_images_per_band

            stdscr.move(status_start_line_y, 1); stdscr.clrtoeol()
            move_status_msg = f"Scan {pos_idx+1}/{len(positions_list)}: Bande {current_band_idx_for_naming+1}, Pos ({target_x:.1f}, {target_y:.1f})"
            stdscr.addstr(status_start_line_y, 1, move_status_msg[:w-2])
            stdscr.refresh()

            # --- Logique de Déplacement ---
            delta_x_to_move = target_x - current_x_mm
            delta_y_to_move = target_y - current_y_mm
            path_length_mm = math.sqrt(delta_x_to_move**2 + delta_y_to_move**2)

            if path_length_mm >= (MM_PER_MICROSTEP / 2.0):
                current_move_speed = TARGET_SPEED_MM_S
                if current_move_speed <= 1e-6:
                    stdscr.move(status_start_line_y + 1, 1); stdscr.clrtoeol()
                    stdscr.addstr(status_start_line_y + 1, 1, "Erreur: Vitesse de déplacement trop faible. Scan annulé.")
                    stdscr.refresh()
                    final_scan_message = "Erreur: Vitesse de déplacement trop faible. Scan annulé."
                    scan_aborted = True; break
                
                time_for_move_s = path_length_mm / current_move_speed
                _dx_steps_cart = round(-delta_x_to_move * MICROSTEPS_PER_MM)
                _dy_steps_cart = round(delta_y_to_move * MICROSTEPS_PER_MM)
                _steps_m1 = _dx_steps_cart + _dy_steps_cart
                _steps_m2 = _dx_steps_cart - _dy_steps_cart
                num_iterations = max(abs(int(_steps_m1)), abs(int(_steps_m2)))

                if num_iterations > 0:
                    pulse_delay = time_for_move_s / num_iterations
                    actual_pulse_delay = max(MINIMUM_PULSE_CYCLE_DELAY, pulse_delay)
                    move_motors_coordinated(int(_steps_m1), int(_steps_m2), actual_pulse_delay)
            
            current_x_mm = round(target_x, 3)
            current_y_mm = round(target_y, 3)
            # --- Fin Logique de Déplacement ---

            stdscr.move(status_start_line_y + 1, 1); stdscr.clrtoeol()
            capture_info_msg = f"  Capture de {imgs_per_pos_at_location} image(s) à (X:{current_x_mm:.1f}, Y:{current_y_mm:.1f})"
            stdscr.addstr(status_start_line_y + 1, 1, capture_info_msg[:w-2])
            stdscr.refresh()

            # --- Logique de Capture ---
            filename_prefix = f"B{current_band_idx_for_naming:02d}_Im{global_image_num_for_prefix:02d}"
            base_path_for_this_capture = os.path.join(project_path, filename_prefix)
            
            capture_output_msgs = capture_images(imgs_per_pos_at_location, base_path_for_this_capture)
            
            for k_msg_idx in range(min(2, len(capture_output_msgs))):
                if status_start_line_y + 2 + k_msg_idx < h:
                    stdscr.move(status_start_line_y + 2 + k_msg_idx, 1); stdscr.clrtoeol()
                    stdscr.addstr(status_start_line_y + 2 + k_msg_idx, 3, capture_output_msgs[k_msg_idx][:w-4])
            stdscr.refresh()

            critical_capture_error = False
            for msg in capture_output_msgs:
                if "Erreur" in msg and "Résolution" not in msg and "directory" not in msg and "Impossible d'ouvrir la webcam" not in msg:
                    critical_capture_error = True; break
                if "Impossible d'ouvrir la webcam" in msg:
                    critical_capture_error = True; break

            if critical_capture_error:
                final_scan_message = "Erreur critique de capture. Scan interrompu."
                scan_aborted = True; break
            
            global_image_num_for_prefix += 1
            time.sleep(0.1)

            stdscr.nodelay(True)
            key_press = stdscr.getch()
            stdscr.nodelay(False)
            if key_press == 27: # ESC
                final_scan_message = "Scan interrompu par l'utilisateur."
                scan_aborted = True; break
    
    if not scan_aborted:
        final_scan_message = "Scan automatique terminé avec succès."

    final_msg_line_y = status_start_line_y + 4 
    if final_msg_line_y < h :
        stdscr.move(final_msg_line_y, 1); stdscr.clrtoeol()
        stdscr.addstr(final_msg_line_y, 1, final_scan_message[:w-2], curses.A_BOLD)
    else: # Fallback si l'écran est trop petit pour la ligne de message final dédiée
        stdscr.move(h-1,1); stdscr.clrtoeol()
        stdscr.addstr(h-1,1, final_scan_message[:w-2], curses.A_BOLD)
    stdscr.refresh()
    
    curses.curs_set(previous_cursor_visibility)
    return final_scan_message

def draw_automatic_mode_ui(stdscr, title, prompts, current_values, status_message="", help_message=""):
    """Draws the UI for the automatic mode."""
    stdscr.clear()
    h, w = stdscr.getmaxyx()

    # Title
    stdscr.addstr(1, (w - len(title)) // 2, title, curses.A_BOLD)
    stdscr.addstr(2, (w - len(title)) // 2, "-" * len(title))

    # Prompts and current values
    line_y = 4
    for i, prompt_key in enumerate(prompts): # prompts is now a list of keys
        val_str = str(current_values.get(prompt_key, "..."))
        display_str = f"{prompt_key}: {val_str}" # Use key directly for display
        stdscr.addstr(line_y + i, 2, display_str[:w-3])

    # Status Message
    status_line_y = h - 4 # Ligne pour le statut (au-dessus de l'erreur et de l'aide)
    if error_msg_line_explicit: status_line_y = h -5 # Ajuster si ligne d'erreur dédiée
    
    # Error Message (if any, displayed separately)
    # Note: error_msg logic needs to be handled in _automatic_mode_loop to use a specific line
    # For now, status_message can contain errors.

    if status_message:
        # Si le message de statut est une erreur, l'afficher sur une ligne dédiée (h-4)
        # Sinon, l'afficher sur (h-3)
        is_error = "erreur" in status_message.lower() or "impossible" in status_message.lower()
        msg_y = h - 4 if is_error else h - 3
        stdscr.move(msg_y,1); stdscr.clrtoeol() # Clear previous message on that line
        stdscr.addstr(msg_y, 1, status_message[:w-2], curses.A_REVERSE if is_error else curses.A_NORMAL)
    
    # Help Message
    stdscr.move(h-2,1); stdscr.clrtoeol() # Clear previous help message
    if help_message:
         stdscr.addstr(h - 2, 1, help_message[:w-2])
    else: # Default help message
        stdscr.addstr(h - 2, 1, "ESC pour retourner au menu principal."[:w-2])

    stdscr.refresh()
error_msg_line_explicit = False # Global or part of a class state if we need more complex error display

def _automatic_mode_loop(stdscr):
    """Main loop for the Automatic Mode UI and logic."""
    global XLIM_MM, YLIM_MM, POINT_MARGIN_MM # Access global machine limits & margin

    curses.curs_set(1) 
    stdscr.keypad(True) 
    
    project_name = ""
    project_path = ""
    # Les variables internes peuvent garder des noms techniques clairs
    num_x_points = None # Correspond à "Images par bande (nx)" pour l'utilisateur
    num_y_points = None # Correspond à "Bandes (ny)" pour l'utilisateur
    images_per_position = None
    calculated_positions = [] 
    
    current_stage_idx = 0
    stages = ["GET_PROJECT_NAME", "GET_NUM_Y", "GET_NUM_X", # Ordre modifié : Bandes (ny) puis Images/bande (nx)
              "GET_IMAGES_PER_POS", "CONFIRM_PARAMETERS", 
              "CALCULATE_POSITIONS", "READY_TO_EXECUTE"]

    config_values = {} 
    status_msg = "Initialisation du mode automatique..."
    help_text_current = "Entrez les informations. ESC pour annuler."

    while True:
        current_stage = stages[current_stage_idx]
        
        # --- RENOMMAGE DES INVITES UTILISATEUR ---
        prompt_keys_ordered = [
            "Nom du Projet", "Chemin du Projet",
            "Bandes (ny)", "Images par bande (nx)", # MODIFIÉ
            "Images par Position", "Total Positions"
        ]
        
        ui_values = {
            "Nom du Projet": project_name if project_name else "...",
            "Chemin du Projet": project_path if project_path else "...",
            "Bandes (ny)": str(num_y_points) if num_y_points is not None else "...", # MODIFIÉ
            "Images par bande (nx)": str(num_x_points) if num_x_points is not None else "...", # MODIFIÉ
            "Images par Position": str(images_per_position) if images_per_position is not None else "...",
            "Total Positions": str(num_x_points * num_y_points) if num_x_points is not None and num_y_points is not None else "..."
        }

        draw_automatic_mode_ui(stdscr, "-- Mode Automatique --", prompt_keys_ordered, ui_values, status_msg, help_text_current)
        
        input_y_offset = len(prompt_keys_ordered) + 5 

        if current_stage == "GET_PROJECT_NAME":
            help_text_current = "Entrez le nom du projet et validez avec Entrée. ESC pour annuler."
            status_msg = "Configuration du projet..."
            draw_automatic_mode_ui(stdscr, "-- Mode Automatique --", prompt_keys_ordered, ui_values, status_msg, help_text_current)

            temp_name = _get_string_input_curses(stdscr, input_y_offset, 2, "Nom du Projet: ")
            if temp_name is None: status_msg = "Annulation..."; time.sleep(0.5); break 
            if not temp_name:
                status_msg = "Erreur: Le nom du projet ne peut pas être vide. Réessayez."
                continue

            project_name = temp_name
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            project_folder_name = f"{project_name.replace(' ', '_')}_{timestamp}" 
            project_path = os.path.join(".", project_folder_name) 
            
            try:
                os.makedirs(project_path, exist_ok=True)
                status_msg = f"Dossier projet créé: {project_path}"
                config_values["project_name"] = project_name
                config_values["project_path"] = project_path
                current_stage_idx += 1
            except OSError as e:
                status_msg = f"Erreur création dossier {project_path}: {e}"
                project_name = ""; project_path = ""; 

        elif current_stage == "GET_NUM_Y": # MODIFIÉ: Demander Bandes (ny) en premier
            help_text_current = "Nombre de bandes de scan sur l'axe Y. ESC pour annuler." # MODIFIÉ
            status_msg = "Configuration de la grille de scan..."
            draw_automatic_mode_ui(stdscr, "-- Mode Automatique --", prompt_keys_ordered, ui_values, status_msg, help_text_current)

            temp_ny = _get_int_input_curses(stdscr, input_y_offset, 2, "Bandes (ny) (ex: 4): ", min_val=1) # MODIFIÉ
            if temp_ny is None: status_msg = "Annulation..."; time.sleep(0.5); break
                 
            num_y_points = temp_ny # Variable interne reste num_y_points
            config_values["num_y_points"] = num_y_points
            current_stage_idx += 1
            
        elif current_stage == "GET_NUM_X": # MODIFIÉ: Demander Images par bande (nx) ensuite
            help_text_current = "Nombre d'images à prendre par bande (sur l'axe X). ESC pour annuler." # MODIFIÉ
            status_msg = "Configuration de la grille de scan..."
            draw_automatic_mode_ui(stdscr, "-- Mode Automatique --", prompt_keys_ordered, ui_values, status_msg, help_text_current)

            temp_nx = _get_int_input_curses(stdscr, input_y_offset, 2, "Images par bande (nx) (ex: 5): ", min_val=1) # MODIFIÉ
            if temp_nx is None: status_msg = "Annulation..."; time.sleep(0.5); break
            
            num_x_points = temp_nx # Variable interne reste num_x_points
            config_values["num_x_points"] = num_x_points
            current_stage_idx += 1

        elif current_stage == "GET_IMAGES_PER_POS":
            help_text_current = "Nombre de captures d'images à chaque position. ESC pour annuler."
            status_msg = "Configuration de la capture..."
            draw_automatic_mode_ui(stdscr, "-- Mode Automatique --", prompt_keys_ordered, ui_values, status_msg, help_text_current)

            temp_imgs = _get_int_input_curses(stdscr, input_y_offset, 2, "Images par Position (ex: 1): ", min_val=1)
            if temp_imgs is None: status_msg = "Annulation..."; time.sleep(0.5); break
            
            images_per_position = temp_imgs
            config_values["images_per_position"] = images_per_position
            current_stage_idx += 1

        elif current_stage == "CONFIRM_PARAMETERS":
            total_pos_str = ui_values["Total Positions"]
            status_msg = f"Vérifiez les paramètres ({total_pos_str} positions au total)."
            help_text_current = "'Entrée' pour valider et calculer les positions. 'ESC' pour annuler."
            draw_automatic_mode_ui(stdscr, "-- Mode Automatique --", prompt_keys_ordered, ui_values, status_msg, help_text_current)
            
            stdscr.nodelay(False); key = stdscr.getch()
            if key == 27: status_msg = "Annulation..."; time.sleep(0.5); break
            elif key == curses.KEY_ENTER or key in [10, 13]:
                status_msg = "Paramètres confirmés. Calcul des positions..."
                current_stage_idx += 1 
            
        elif current_stage == "CALCULATE_POSITIONS":
            status_msg = "Calcul en cours..."
            help_text_current = "Veuillez patienter."
            draw_automatic_mode_ui(stdscr, "-- Mode Automatique --", prompt_keys_ordered, ui_values, status_msg, help_text_current)
            time.sleep(0.1) 

            calculated_positions, calc_msg = _calculate_scan_positions(
                num_x_points, num_y_points, XLIM_MM, YLIM_MM, POINT_MARGIN_MM
            )
            
            if calculated_positions is None:
                status_msg = calc_msg # Contient déjà le message d'erreur du calcul
                help_text_current = "Erreur de calcul. Appuyez sur ESC et vérifiez les limites/points."
                draw_automatic_mode_ui(stdscr, "-- Mode Automatique --", prompt_keys_ordered, ui_values, status_msg, help_text_current)
                # Attendre que l'utilisateur appuie sur ESC pour quitter
                while True:
                    if stdscr.getch() == 27: break
                status_msg = "Annulation suite à erreur de calcul..."; time.sleep(0.5); break
            else:
                config_values["calculated_positions"] = calculated_positions
                csv_feedback_msg = ""
                
                # --- ÉCRITURE DU FICHIER CSV ---
                if project_path: # S'assurer que le chemin du projet est défini
                    csv_file_path = os.path.join(project_path, "position_list.csv")
                    try:
                        with open(csv_file_path, 'w', newline='') as csvfile:
                            csv_writer = csv.writer(csvfile)
                            csv_writer.writerow(['X_mm', 'Y_mm']) # En-têtes
                            for pos_x, pos_y in calculated_positions:
                                csv_writer.writerow([pos_x, pos_y])
                        csv_feedback_msg = " position_list.csv sauvegardé."
                    except IOError as e:
                        csv_feedback_msg = f" Erreur sauvegarde CSV: {e}."
                else:
                    csv_feedback_msg = " Erreur: Chemin du projet non défini pour CSV."
                # --- FIN ÉCRITURE CSV ---
                
                status_msg = calc_msg + csv_feedback_msg # Concaténer les messages
                current_stage_idx += 1 # Passer à READY_TO_EXECUTE
        
        # ... (dans _automatic_mode_loop)
        elif current_stage == "READY_TO_EXECUTE":
            help_text_current = "'Entrée' pour lancer la séquence. 'ESC' pour retourner au menu."
            # status_msg contient déjà le résultat du calcul et du CSV
            draw_automatic_mode_ui(stdscr, "-- Mode Automatique --", prompt_keys_ordered, ui_values, status_msg, help_text_current)
            
            stdscr.nodelay(False); key = stdscr.getch()
            if key == 27: 
                status_msg = "Séquence annulée avant démarrage."; 
                # Redessiner pour s'assurer que le message est vu avant la pause et la sortie
                draw_automatic_mode_ui(stdscr, "-- Mode Automatique --", prompt_keys_ordered, ui_values, status_msg, "Retour au menu...")
                time.sleep(1); 
                break
            elif key == curses.KEY_ENTER or key in [10, 13]:
                status_msg = "Lancement de la séquence de scan..."
                draw_automatic_mode_ui(stdscr, "-- Mode Automatique --", prompt_keys_ordered, ui_values, status_msg, "Scan en cours... ESC pour interrompre.")
                time.sleep(0.5)

                scan_result_msg = _execute_automatic_scan(
                    stdscr, 
                    project_path, 
                    calculated_positions, # La liste des (x,y)
                    num_y_points,         # Nombre de bandes (ny)
                    num_x_points,         # Nombre d'images par bande (nx)
                    images_per_position,  # Nombre de captures à chaque position (pour _IdXX)
                    prompt_keys_ordered,  # Pour que _execute_automatic_scan puisse redessiner la base
                    ui_values
                )
                
                status_msg = scan_result_msg 
                help_text_current = "Appuyez sur une touche pour retourner au menu."
                # Redessiner l'interface avec le message final du scan
                draw_automatic_mode_ui(stdscr, "-- Mode Automatique --", prompt_keys_ordered, ui_values, status_msg, help_text_current)
                stdscr.nodelay(False)
                stdscr.getch() 
                break # Fin du mode auto après le scan ou annulation

    curses.curs_set(0)
    stdscr.keypad(True)
    return_message = f"Mode Auto: {project_name}" if project_name else "Mode Auto quitté/annulé."
    if calculated_positions and project_name:
        return_message += f" ({len(calculated_positions)} positions)"
    return return_message



# --- Main Menu Functions ---
def draw_main_menu_ui(stdscr, menu_items, selected_idx, status_message=""):
    stdscr.clear(); h, w = stdscr.getmaxyx()
    title = "-- SIMUFLY --"; stdscr.addstr(1, (w - len(title)) // 2, title, curses.A_BOLD)
    stdscr.addstr(2, (w - len(title)) // 2, "-" * len(title))
    for idx, item_text in enumerate(menu_items):
        x = (w - len(item_text)) // 2; y = 4 + idx * 2
        if idx == selected_idx: stdscr.attron(curses.A_REVERSE); stdscr.addstr(y, x, item_text); stdscr.attroff(curses.A_REVERSE)
        else: stdscr.addstr(y, x, item_text)
    if status_message: stdscr.addstr(h - 2, 1, status_message[:w-2])
    stdscr.addstr(h - 1, 1, "HAUT/BAS/ENTRÉE pour sélectionner. 'Q' pour quitter.")
    stdscr.refresh()

def _main_menu_loop(stdscr):
    global last_command_output 
    curses.curs_set(0); stdscr.keypad(True)
    menu_items = ["Mode automatique", "Mode manuel", "Charger un fichier", "Démonstration", "Quitter SimuFly"]
    current_selection = 0
    status_msg = f"GPIO {'OK' if pul_device_m1 else 'ERREUR'}. Pos: X={current_x_mm:.1f}, Y={current_y_mm:.1f}"

    while True:
        draw_main_menu_ui(stdscr, menu_items, current_selection, status_msg)
        key = stdscr.getch()
        if key == curses.KEY_UP: current_selection = (current_selection - 1 + len(menu_items)) % len(menu_items)
        elif key == curses.KEY_DOWN: current_selection = (current_selection + 1) % len(menu_items)
        elif key == ord('q') or key == ord('Q'):
            status_msg = "Quitting SimuFly..."; draw_main_menu_ui(stdscr, menu_items, current_selection, status_msg); time.sleep(0.5); break 
        elif key == curses.KEY_ENTER or key in [10, 13]:
            selected_option = menu_items[current_selection]
            if selected_option == "Mode manuel":
                status_msg = "Lancement Mode Manuel..."; draw_main_menu_ui(stdscr, menu_items, current_selection, status_msg); time.sleep(0.2)
                _manual_mode_loop(stdscr)
                curses.curs_set(0); stdscr.keypad(True)
                status_msg = f"Retour du Mode Manuel. Pos: X={current_x_mm:.1f}, Y={current_y_mm:.1f}"; last_command_output = []
            # --- NEW: Handle Automatic Mode ---
            elif selected_option == "Mode automatique":
                status_msg = "Lancement Mode Automatique..."; draw_main_menu_ui(stdscr, menu_items, current_selection, status_msg); time.sleep(0.2)
                auto_mode_status = _automatic_mode_loop(stdscr) # Call the automatic mode loop
                curses.curs_set(0); stdscr.keypad(True) # Restore curses settings for main menu
                status_msg = f"{auto_mode_status} Pos: X={current_x_mm:.1f}, Y={current_y_mm:.1f}"
                last_command_output = []
            elif selected_option == "Quitter SimuFly":
                status_msg = "Quitting SimuFly..."; draw_main_menu_ui(stdscr, menu_items, current_selection, status_msg); time.sleep(0.5); break 
            else:
                status_msg = f"Option '{selected_option}' non implémentée."
                draw_main_menu_ui(stdscr, menu_items, current_selection, status_msg); stdscr.timeout(1500); stdscr.getch(); stdscr.timeout(-1)
                status_msg = f"Pos: X={current_x_mm:.1f}, Y={current_y_mm:.1f}"
        else: status_msg = f"Pos: X={current_x_mm:.1f}, Y={current_y_mm:.1f}"

if __name__ == "__main__":
    gpio_ok = setup_gpio()
    if gpio_ok: print("GPIO initialisé. Lancement SimuFly Curses...")
    else: print("ERREUR: Init GPIO échouée. Interface Curses démarrera sans fonctions moteur.")
    time.sleep(0.1)
    try:
        curses.wrapper(_main_menu_loop)
    except Exception as e: 
        print(f"Erreur critique hors boucle Curses: {e}")
        import traceback; traceback.print_exc()
    finally:
        print("Nettoyage GPIO avant de quitter..."); cleanup_gpio(); print("SimuFly terminé.")