import gpiozero
import time
import math
import re # For command parsing
import curses 
import cv2
import os 
import sys
from contextlib import contextmanager
from datetime import datetime 
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

# --- Endstop GPIO Pin Configuration ---
ENDSTOP_PIN_X = 16 # For X-axis
ENDSTOP_PIN_Y = 26 # For Y-axis

# --- GPIOZero Device Objects (will be initialized in setup_gpio) ---
pul_device_m1 = None
dir_device_m1 = None
pul_device_m2 = None
dir_device_m2 = None
# --- Endstop Device Objects ---
endstop_x = None
endstop_y = None

# --- Motor & System Parameters ---
MOTOR_NATIVE_STEPS_PER_REV = 400
DRIVER_MICROSTEP_DIVISOR = 2

MM_PER_MICROSTEP = 0.1
if MM_PER_MICROSTEP == 0:
    raise ValueError("MM_PER_MICROSTEP cannot be zero.")
MICROSTEPS_PER_MM = 1.0 / MM_PER_MICROSTEP

DEFAULT_XLIM_MM = 825.0
DEFAULT_YLIM_MM = 1125.0
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
CALIBRATION_LOGICAL_ORIGIN_OFFSET_MM = 20.0 # Décalage pour le nouveau (0,0) après calibration

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
    global current_x_mm, current_y_mm, TARGET_SPEED_MM_S, absolute_mode, CALIBRATION_LOGICAL_ORIGIN_OFFSET_MM
    output_messages = ["--- Démarrage du Cycle de Calibration ---"]

    if not all([pul_device_m1, dir_device_m1, pul_device_m2, dir_device_m2, endstop_x, endstop_y]):
        output_messages.append("Erreur: GPIOs ou capteurs de fin de course non initialisés.")
        return output_messages

    cal_speed = min(CALIBRATION_SPEED_MM_S, MAX_SPEED_MM_S)
    if cal_speed <= 1e-6:
        output_messages.append(f"Erreur: Vitesse de calibration ({cal_speed:.2f} mm/s) trop faible.")
        return output_messages
    
    pulse_cycle_delay_cal = MM_PER_MICROSTEP / cal_speed
    actual_pulse_cycle_delay_cal = max(MINIMUM_PULSE_CYCLE_DELAY, pulse_cycle_delay_cal)
    max_steps_travel = int(MAX_CALIBRATION_TRAVEL_MM * MICROSTEPS_PER_MM)

    # --- Calibration Axe Y (vers Y-min) ---
    output_messages.append(f"Calibration Axe Y à {cal_speed:.2f} mm/s...")
    # Directions pour -Y : M1 négatif, M2 positif (selon votre configuration CoreXY)
    dir_device_m1.on()  # M1 -> dir pour -Y
    dir_device_m2.off() # M2 -> dir pour -Y
    time.sleep(0.002)

    homed_y = False
    for i in range(max_steps_travel):
        if endstop_y.is_active:
            output_messages.append("Fin de course Y activé.")
            homed_y = True
            break
        pul_device_m1.on(); pul_device_m2.on() 
        time.sleep(MIN_PULSE_WIDTH)
        pul_device_m1.off(); pul_device_m2.off()
        inter_pulse_delay = actual_pulse_cycle_delay_cal - MIN_PULSE_WIDTH
        if inter_pulse_delay > 0: time.sleep(inter_pulse_delay)
        if i > 0 and i % 2000 == 0: time.sleep(0.001) 

    if not homed_y:
        output_messages.append("Erreur: Calibration Axe Y échouée (fin de course non atteint).")
        return output_messages
    output_messages.append("Point zéro physique Y trouvé.")

    # --- Calibration Axe X (vers X-min) ---
    output_messages.append(f"Calibration Axe X à {cal_speed:.2f} mm/s...")
    # --- CORRECTION DE LA DIRECTION POUR L'AXE X ---
    # Rétablir la configuration qui fonctionnait pour le mouvement -X
    dir_device_m1.off() 
    dir_device_m2.off() 
    # --- FIN CORRECTION ---
    time.sleep(0.002)

    homed_x = False
    for i in range(max_steps_travel):
        if endstop_x.is_active:
            output_messages.append("Fin de course X activé.")
            homed_x = True
            break
        pul_device_m1.on(); pul_device_m2.on() 
        time.sleep(MIN_PULSE_WIDTH)
        pul_device_m1.off(); pul_device_m2.off()
        inter_pulse_delay = actual_pulse_cycle_delay_cal - MIN_PULSE_WIDTH
        if inter_pulse_delay > 0: time.sleep(inter_pulse_delay)
        if i > 0 and i % 2000 == 0: time.sleep(0.001)

    if not homed_x:
        output_messages.append("Erreur: Calibration Axe X échouée (fin de course non atteint).")
        return output_messages
    output_messages.append("Point zéro physique X trouvé.")

    current_x_mm = 0.0
    current_y_mm = 0.0

    offset_val = CALIBRATION_LOGICAL_ORIGIN_OFFSET_MM    
    motor_msgs_offset = move_corexy(offset_val, offset_val, actual_pulse_cycle_delay_cal)

    current_x_mm = 0.0
    current_y_mm = 0.0
    absolute_mode = True 
    output_messages.append(f"Origine définie.")
    output_messages.append(f"--- Calibration Terminée --- Position logique actuelle: X={current_x_mm:.3f}, Y={current_y_mm:.3f} mm")
    output_messages.append("Mode réglé sur ABS (Absolu).")
    
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
            
            directory = os.path.dirname(base_image_path_with_prefix)
            filename_prefix_only = os.path.basename(base_image_path_with_prefix) 

            image_filename = os.path.join(directory, f"{filename_prefix_only}_Id{i:02d}.tiff") 
            
            try:
                if directory and not os.path.exists(directory):
                    os.makedirs(directory, exist_ok=True)
                    messages.append(f"  Création dossier (sécurité): {directory}")

                cv2.imwrite(image_filename, frame)
                messages.append(f"  Image capturée: {os.path.basename(image_filename)}") 
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
        # In ABS mode, calculate deltas. In REL, target is (0,0) relative to current.
        if absolute_mode: 
            dx_home = target_x_abs - current_x_mm
            dy_home = target_y_abs - current_y_mm
        else: # REL mode home means move by (0,0) effectively just resetting logical coords
            dx_home = 0.0 
            dy_home = 0.0
            # current_x_mm = 0.0 # Position reset is handled after potential move
            # current_y_mm = 0.0

        actual_homing_speed = min(HOMING_SPEED_MM_S, MAX_SPEED_MM_S)
        home_path_length_mm = math.sqrt(dx_home**2 + dy_home**2)

        if home_path_length_mm > (MM_PER_MICROSTEP / 2.0) and absolute_mode : # Only move physically if in ABS and there's a delta
            output_messages.append(f"    Moving to 0,0 from {current_x_mm:.2f},{current_y_mm:.2f} at {actual_homing_speed:.2f} mm/s")
            time_for_home_move_s = home_path_length_mm / actual_homing_speed
            
            # Calculate steps (move_corexy expects deltas)
            # We already have dx_home, dy_home for ABS.
            # For REL, they are 0,0 so move_corexy won't do physical motion.
            
            delta_x_steps_cartesian_home = round(-dx_home * MICROSTEPS_PER_MM)
            delta_y_steps_cartesian_home = round(dy_home * MICROSTEPS_PER_MM)
            steps_m1_home = delta_x_steps_cartesian_home + delta_y_steps_cartesian_home
            steps_m2_home = delta_x_steps_cartesian_home - delta_y_steps_cartesian_home
            num_iterations_home = max(abs(int(steps_m1_home)), abs(int(steps_m2_home)))

            if num_iterations_home > 0:
                pulse_delay_for_home_move = time_for_home_move_s / num_iterations_home
                actual_pulse_delay_for_home_move = max(MINIMUM_PULSE_CYCLE_DELAY, pulse_delay_for_home_move)
                motor_msgs = move_corexy(dx_home, dy_home, actual_pulse_delay_for_home_move) # move_corexy updates current_x, current_y
                output_messages.extend(motor_msgs)
            # current_x_mm and current_y_mm are updated by move_corexy
        
        # After physical move (if any), set logical coordinates to 0,0
        current_x_mm = target_x_abs 
        current_y_mm = target_y_abs
        if not absolute_mode: # If in REL mode and HOME was called
             output_messages.append("    Logical pos reset to 0,0. No physical move in REL unless X0 Y0 was specified.")
        elif home_path_length_mm <= (MM_PER_MICROSTEP / 2.0) and absolute_mode:
             output_messages.append("    Already at logical 0,0.")

        output_messages.append(f"  New position: X={current_x_mm:.3f}, Y={current_y_mm:.3f}")


    elif instruction.startswith("S") and instruction != "MOVE": # S<val> or S <val>
        try:
            speed_val_mm_s_req = 0.0
            if len(instruction) > 1 and instruction[1:].replace('.', '', 1).isdigit(): # S<val>
                speed_val_mm_s_req = float(instruction[1:])
            elif len(parts) > 1 and parts[1].replace('.', '', 1).isdigit(): # S <val>
                speed_val_mm_s_req = float(parts[1])
            else:
                output_messages.append(f"  Invalid S format. Use S<value> or S <value>.")
                return output_messages, True # Continue manual mode

            if speed_val_mm_s_req > 0:
                if speed_val_mm_s_req > MAX_SPEED_MM_S:
                    TARGET_SPEED_MM_S = MAX_SPEED_MM_S
                    output_messages.append(f"  Requested speed {speed_val_mm_s_req:.2f} > limit {MAX_SPEED_MM_S:.2f}. Speed set to MAX.")
                else:
                    TARGET_SPEED_MM_S = speed_val_mm_s_req
                    output_messages.append(f"  Global target speed set to: {TARGET_SPEED_MM_S:.2f} mm/s")
            else:
                output_messages.append(f"  Speed S must be positive (<= {MAX_SPEED_MM_S:.2f} mm/s).")
        except ValueError:
            output_messages.append(f"  Invalid speed value in S: {parts[1] if len(parts) > 1 else instruction[1:]}")
    
    elif instruction == "LIMITS":
        if len(parts) > 1:
            sub_command = parts[1].upper()
            if sub_command == "OFF":
                XLIM_MM, YLIM_MM = float('inf'), float('inf')
                output_messages.append("  Coordinate limits DISABLED.")
            elif sub_command == "ON":
                XLIM_MM, YLIM_MM = DEFAULT_XLIM_MM, DEFAULT_YLIM_MM
                output_messages.append(f"  Limits ENABLED (X:[0,{XLIM_MM:.0f}], Y:[0,{YLIM_MM:.0f}]).")
            else:
                output_messages.append("  Usage: LIMITS [ON|OFF]")
        else:
            x_disp = f"{XLIM_MM:.0f}" if XLIM_MM!=float('inf') else "INF"
            y_disp = f"{YLIM_MM:.0f}" if YLIM_MM!=float('inf') else "INF"
            output_messages.append(f"  Limits: X=[0, {x_disp}], Y=[0, {y_disp}]. Use 'LIMITS ON/OFF'.")

    elif instruction == "CAPTURE": # Usage: CAPTURE <N> <PATH_PREFIX>
        if len(parts_orig_case) == 3: # CAPTURE N PATH
            try:
                num_img = int(parts_orig_case[1])
                image_base_path_str = parts_orig_case[2] # This is base_image_path_with_prefix

                if num_img <= 0:
                    output_messages.append("  Error: N (number of images) must be positive.")
                elif not image_base_path_str:
                    output_messages.append("  Error: PATH_PREFIX cannot be empty.")
                else:
                    output_messages.append(f"  Capturing {num_img} image(s) to '{image_base_path_str}_IdXX.tiff'...");
                    # Ensure directory exists for the prefix
                    capture_dir = os.path.dirname(image_base_path_str)
                    if capture_dir and not os.path.exists(capture_dir):
                        try:
                            os.makedirs(capture_dir, exist_ok=True)
                            output_messages.append(f"  Created directory: {capture_dir}")
                        except OSError as e:
                            output_messages.append(f"  Error creating directory {capture_dir}: {e}")
                            return output_messages, True # Abort capture if dir creation fails

                    capture_msgs = capture_images(num_img, image_base_path_str) # Pass the full prefix
                    output_messages.extend(capture_msgs)
            except ValueError:
                output_messages.append(f"  Error: Invalid N '{parts_orig_case[1]}'. Must be an integer.")
            except Exception as e:
                output_messages.append(f"  Capture error: {e}")
        else:
            output_messages.append("  Usage: CAPTURE <N> <PATH_PREFIX> (e.g. CAPTURE 1 ./capture_dir/shot)")

    elif instruction == "MOVE" : # MOVE X<val> Y<val> S<val> (any order, any missing)
        target_x_cmd, target_y_cmd, s_value_this_cmd_req = None, None, None
        for part in parts[1:]:
            # --- CORRECTED SECTION START ---
            if part.startswith('X'):
                try: target_x_cmd = float(part[1:])
                except ValueError: output_messages.append(f"  Invalid X: {part}"); return output_messages, True
            elif part.startswith('Y'):
                try: target_y_cmd = float(part[1:])
                except ValueError: output_messages.append(f"  Invalid Y: {part}"); return output_messages, True
            elif part.startswith('S'):
                try: s_value_this_cmd_req = float(part[1:])
                except ValueError: output_messages.append(f"  Invalid S in MOVE: {part}"); # Don't return, use global S
            # --- CORRECTED SECTION END ---

        if target_x_cmd is None and target_y_cmd is None:
            output_messages.append("  No X or Y coordinate provided for MOVE."); return output_messages, True

        current_move_speed_mm_s = TARGET_SPEED_MM_S # Default to global
        if s_value_this_cmd_req is not None:
            if s_value_this_cmd_req > 0:
                if s_value_this_cmd_req > MAX_SPEED_MM_S:
                    current_move_speed_mm_s = MAX_SPEED_MM_S
                    output_messages.append(f"  MOVE speed {s_value_this_cmd_req:.2f} > limit. Clamped to MAX: {current_move_speed_mm_s:.2f}.")
                else:
                    current_move_speed_mm_s = s_value_this_cmd_req
            else: # s_value_this_cmd_req <= 0
                output_messages.append(f"  Speed S in MOVE must be positive. Using global {TARGET_SPEED_MM_S:.2f} mm/s.")
        current_move_speed_mm_s = min(current_move_speed_mm_s, MAX_SPEED_MM_S) # Ensure it's not over max

        final_target_x_mm, final_target_y_mm = current_x_mm, current_y_mm # Start with current if an axis is not given
        if absolute_mode:
            if target_x_cmd is not None: final_target_x_mm = target_x_cmd
            if target_y_cmd is not None: final_target_y_mm = target_y_cmd
        else: # Relative mode
            if target_x_cmd is not None: final_target_x_mm = current_x_mm + target_x_cmd
            if target_y_cmd is not None: final_target_y_mm = current_y_mm + target_y_cmd
        
        # Clamp to 0 and XLIM/YLIM
        actual_target_x_mm = max(0.0, min(final_target_x_mm, XLIM_MM))
        actual_target_y_mm = max(0.0, min(final_target_y_mm, YLIM_MM))

        clamped = False
        if XLIM_MM != float('inf') or YLIM_MM != float('inf'): # Check only if limits are active
            if abs(actual_target_x_mm - final_target_x_mm) > 1e-9 or \
               abs(actual_target_y_mm - final_target_y_mm) > 1e-9 :
                clamped = True
        elif final_target_x_mm < 0 or final_target_y_mm < 0 : # Also clamped if trying to go below 0
             clamped = True
        
        if clamped:
            output_messages.append(f"  Warn: Target ({final_target_x_mm:.2f},{final_target_y_mm:.2f}) out of bounds or below 0. Clamped to ({actual_target_x_mm:.2f},{actual_target_y_mm:.2f}).")

        delta_x_to_move = actual_target_x_mm - current_x_mm
        delta_y_to_move = actual_target_y_mm - current_y_mm
        path_length_mm = math.sqrt(delta_x_to_move**2 + delta_y_to_move**2)

        if path_length_mm < (MM_PER_MICROSTEP / 2.0):
            output_messages.append(f"  Move too small or already at target. Current:({current_x_mm:.3f},{current_y_mm:.3f}), Target:({actual_target_x_mm:.3f},{actual_target_y_mm:.3f})")
            current_x_mm,current_y_mm = round(actual_target_x_mm,3), round(actual_target_y_mm,3) # Update logical position
        else:
            delta_x_steps_cartesian = round(-delta_x_to_move * MICROSTEPS_PER_MM)
            delta_y_steps_cartesian = round(delta_y_to_move * MICROSTEPS_PER_MM)
            steps_m1 = delta_x_steps_cartesian + delta_y_steps_cartesian
            steps_m2 = delta_x_steps_cartesian - delta_y_steps_cartesian
            num_iterations = max(abs(int(steps_m1)), abs(int(steps_m2)))

            if num_iterations == 0: # Should not happen if path_length_mm is sufficient, but defensive
                output_messages.append("  No motor steps required for this move after rounding.");
                current_x_mm,current_y_mm = round(actual_target_x_mm,3),round(actual_target_y_mm,3)
            else:
                if current_move_speed_mm_s <= 1e-6: # Speed too low
                    output_messages.append(f"  Effective speed {current_move_speed_mm_s:.2e} mm/s is too low. No physical move.")
                else:
                    time_for_move_s = path_length_mm / current_move_speed_mm_s
                    pulse_delay_for_this_move = time_for_move_s / num_iterations
                    actual_pulse_delay_for_this_move = max(MINIMUM_PULSE_CYCLE_DELAY, pulse_delay_for_this_move)
                    
                    effective_speed_calc_denominator = num_iterations * actual_pulse_delay_for_this_move
                    effective_speed_mm_s = path_length_mm / effective_speed_calc_denominator if effective_speed_calc_denominator > 1e-9 else current_move_speed_mm_s
                    
                    output_messages.append(f"  Moving by dx={delta_x_to_move:.3f}, dy={delta_y_to_move:.3f} to X={actual_target_x_mm:.3f}, Y={actual_target_y_mm:.3f}")
                    output_messages.append(f"  Target speed: {current_move_speed_mm_s:.2f}. Effective: {effective_speed_mm_s:.2f} mm/s. Pulse delay: {actual_pulse_delay_for_this_move*1000:.4f} ms.")
                    
                    motor_msgs = move_corexy(delta_x_to_move, delta_y_to_move, actual_pulse_delay_for_this_move) # This updates current_x_mm, current_y_mm
                    output_messages.extend(motor_msgs)
        
        output_messages.append(f"  New position: X={current_x_mm:.3f} mm, Y={current_y_mm:.3f} mm")

    elif instruction == "POS":
        x_disp = f"{XLIM_MM:.0f}" if XLIM_MM!=float('inf') else "INF"
        y_disp = f"{YLIM_MM:.0f}" if YLIM_MM!=float('inf') else "INF"
        output_messages.append(f"  Pos: X={current_x_mm:.3f}, Y={current_y_mm:.3f}. Speed: {TARGET_SPEED_MM_S:.2f} (Max: {MAX_SPEED_MM_S:.2f}). Mode: {'ABS' if absolute_mode else 'REL'}.")
        output_messages.append(f"  Limits: X=[0,{x_disp}], Y=[0,{y_disp}]. Endstops: X={'TRIG' if endstop_x.is_active else 'open'}, Y={'TRIG' if endstop_y.is_active else 'open'}")
    
    elif instruction in ["EXIT", "QUIT", "MENU"]:
        output_messages.append("  Returning to main menu..."); return output_messages, False # Signal to exit manual mode
    else:
        if instruction: # If there was any instruction at all
            output_messages.append(f"  Unknown command: {instruction}")
        # If input was empty, no message needed here, just loop.
    return output_messages, True # Continue manual mode

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
    available_lines_for_output = max_y - output_start_y - 2
    
    start_index = 0 
    if available_lines_for_output < 0: 
        available_lines_for_output = 0

    if len(command_output_lines) > available_lines_for_output:
        start_index = len(command_output_lines) - available_lines_for_output
    
    for i, line in enumerate(command_output_lines[start_index:]):
        if output_start_y + i < max_y - 2:
            stdscr.addstr(output_start_y + i, 0, line[:max_x-1])
    
    current_y = max_y - 2 
    if current_y >= 0 and current_y < max_y : 
         stdscr.addstr(current_y, 0, "-" * (max_x-1) )
    
    input_line_y = max_y - 1 
    if input_line_y >=0 and input_line_y < max_y: 
        stdscr.addstr(input_line_y, 0, input_prompt)
        if len(input_prompt) < max_x:
            stdscr.move(input_line_y, len(input_prompt))
        else: 
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
            
            if not cmd_line and not (last_command_output and last_command_output[-1] == "Terminal too small. Resize or type 'MENU'.") :
                 last_command_output = [] # Clear output if command is empty and no critical message exists
                 continue

            last_command_output, running_manual_mode = parse_command_and_execute(cmd_line)
        except curses.error as e: curses.noecho(); last_command_output = [f"Curses error: {e}", "Try resize or 'MENU'."]
        except KeyboardInterrupt: curses.noecho(); last_command_output = ["Interruption. Retour au menu..."]; running_manual_mode = False; time.sleep(0.5)
        except Exception as e: curses.noecho(); last_command_output = [f"Erreur: {e}", "Vérifiez la commande."]; import traceback; last_command_output.append(traceback.format_exc(limit=1))

    last_command_output = [] # Clear output when exiting mode

# --- Curses Input Helper Functions ---
def _get_string_input_curses(stdscr, y, x, prompt_text, max_len=50):
    """
    Gets a string input from the user in a curses window.
    Handles ESC for cancellation (returns None).
    Handles Backspace/Delete for editing.
    Handles Left/Right arrow keys for cursor movement within the input.
    """
    curses.noecho() 
    stdscr.keypad(True) 

    stdscr.addstr(y, x, prompt_text)
    stdscr.clrtoeol() 
    
    input_chars = [] 
    cursor_display_pos = 0 
    
    stdscr.move(y, x + len(prompt_text))
    stdscr.refresh()

    while True:
        current_input_str = "".join(input_chars)
        stdscr.move(y, x + len(prompt_text)) 
        stdscr.clrtoeol() 
        stdscr.addstr(y, x + len(prompt_text), current_input_str)
        stdscr.move(y, x + len(prompt_text) + cursor_display_pos)
        stdscr.refresh()
        key = stdscr.getch()
        if key == 27: return None 
        elif key == curses.KEY_ENTER or key == 10 or key == 13: return "".join(input_chars)
        elif key == curses.KEY_BACKSPACE or key == 127 or key == 8: 
            if cursor_display_pos > 0: input_chars.pop(cursor_display_pos - 1); cursor_display_pos -= 1
        elif key == curses.KEY_DC: 
            if cursor_display_pos < len(input_chars): input_chars.pop(cursor_display_pos)
        elif key == curses.KEY_LEFT:
            if cursor_display_pos > 0: cursor_display_pos -= 1
        elif key == curses.KEY_RIGHT:
            if cursor_display_pos < len(input_chars): cursor_display_pos += 1
        elif (key >= 32 and key <= 126) or (key >= 160 and key <= 255): 
            if len(input_chars) < max_len:
                try: input_chars.insert(cursor_display_pos, chr(key)); cursor_display_pos += 1
                except ValueError: pass 

def _get_int_input_curses(stdscr, y, x, prompt_text, min_val=None, max_val=None):
    """Gets an integer input, validates, and returns it or None."""
    stdscr.keypad(True)
    while True:
        input_str_obj = _get_string_input_curses(stdscr, y, x, prompt_text, 20) 
        if input_str_obj is None: return None
        current_input_str = input_str_obj 
        stdscr.move(y + 1, x); stdscr.clrtoeol()
        if not current_input_str: 
            stdscr.addstr(y + 1, x, "Veuillez entrer une valeur. Réessayez.")
            stdscr.clrtoeol(); stdscr.refresh(); time.sleep(1) 
            stdscr.move(y + 1, x); stdscr.clrtoeol()
            continue 
        try:
            val = int(current_input_str)
            valid_range = True
            if min_val is not None and val < min_val: valid_range = False
            if max_val is not None and val > max_val: valid_range = False
            if not valid_range:
                range_err_msg = ""
                if min_val is not None and max_val is not None: range_err_msg = f" (Doit être entre {min_val} et {max_val})"
                elif min_val is not None: range_err_msg = f" (Doit être >= {min_val})"
                elif max_val is not None: range_err_msg = f" (Doit être <= {max_val})"
                stdscr.addstr(y + 1, x, f"Valeur invalide{range_err_msg}. Réessayez.")
                stdscr.clrtoeol(); stdscr.refresh(); time.sleep(1.5) 
                stdscr.move(y + 1, x); stdscr.clrtoeol() 
                continue 
            return val
        except ValueError:
            stdscr.addstr(y + 1, x, "Entrée numérique invalide. Réessayez.")
            stdscr.clrtoeol(); stdscr.refresh(); time.sleep(1.5)
            stdscr.move(y + 1, x); stdscr.clrtoeol()
            continue

# --- Automatic Mode Functions ---
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

    scan_area_x_min = point_margin
    scan_area_x_max = current_xlim_mm - point_margin
    scan_area_y_min = point_margin
    scan_area_y_max = current_ylim_mm - point_margin

    if scan_area_x_min > scan_area_x_max : 
        if nx == 1 and current_xlim_mm >=0: pass 
        else: return None, f"Largeur X ({current_xlim_mm}mm) insuffisante pour {nx} points avec marge de {point_margin}mm."
    if scan_area_y_min > scan_area_y_max :
        if ny == 1 and current_ylim_mm >=0: pass 
        else: return None, f"Hauteur Y ({current_ylim_mm}mm) insuffisante pour {ny} points avec marge de {point_margin}mm."

    x_coords_list = []
    if nx == 1: x_coords_list.append(current_xlim_mm / 2.0)
    else:
        width_for_points = scan_area_x_max - scan_area_x_min
        step_x = width_for_points / (nx - 1)
        for i in range(nx): x_coords_list.append(scan_area_x_min + i * step_x)

    y_coords_list = []
    if ny == 1: y_coords_list.append(current_ylim_mm / 2.0)
    else:
        height_for_points = scan_area_y_max - scan_area_y_min
        step_y = height_for_points / (ny - 1)
        for i in range(ny): y_coords_list.append(scan_area_y_min + i * step_y)

    for i_y in range(ny):
        current_y_val = y_coords_list[i_y]
        current_x_row_iter = x_coords_list if i_y % 2 == 0 else reversed(x_coords_list)
        for current_x_val in current_x_row_iter:
            final_x = round(max(0.0, min(current_x_val, current_xlim_mm)), 3)
            final_y = round(max(0.0, min(current_y_val, current_ylim_mm)), 3)
            positions.append((final_x, final_y))
    return positions, f"{len(positions)} positions calculées avec succès."

def _execute_automatic_scan(stdscr, project_path, positions_list,
                            num_y_bands, num_x_images_per_band, imgs_per_pos_at_location,
                            base_ui_prompts, base_ui_values): 
    global current_x_mm, current_y_mm, TARGET_SPEED_MM_S, MM_PER_MICROSTEP, \
           MICROSTEPS_PER_MM, MINIMUM_PULSE_CYCLE_DELAY, HOMING_SPEED_MM_S, MAX_SPEED_MM_S
    previous_cursor_visibility = curses.curs_set(0) 
    h, w = stdscr.getmaxyx()
    last_prompt_line_idx = 4 + len(base_ui_prompts) - 1
    status_start_line_y = last_prompt_line_idx + 2 
    if status_start_line_y >= h - 6: status_start_line_y = h - 6
    if status_start_line_y < 0 : status_start_line_y = 0
    
    def _clear_status_lines(start_line, num_lines):
        for i in range(num_lines):
            if start_line + i < h: stdscr.move(start_line + i, 1); stdscr.clrtoeol()
        stdscr.refresh()
    _clear_status_lines(status_start_line_y, 6) 

    stdscr.move(status_start_line_y, 1); stdscr.clrtoeol()
    stdscr.addstr(status_start_line_y, 1, "Calibration initiale en cours..."[:w-2], curses.A_BOLD); stdscr.refresh()
    calibration_messages = perform_calibration_cycle() 
    for k_msg_idx, msg in enumerate(calibration_messages[:3]):
        if status_start_line_y + 1 + k_msg_idx < h:
            stdscr.move(status_start_line_y + 1 + k_msg_idx, 1); stdscr.clrtoeol()
            stdscr.addstr(status_start_line_y + 1 + k_msg_idx, 3, msg[:w-4]) 
    stdscr.refresh()
    calibration_failed = any("Error" in msg or "Erreur" in msg or "failed" in msg for msg in calibration_messages)
    if calibration_failed:
        final_scan_message = "Échec de la calibration initiale. Scan annulé."
        error_line = status_start_line_y + 4 if status_start_line_y + 4 < h else h -1
        stdscr.move(error_line, 1); stdscr.clrtoeol()
        stdscr.addstr(error_line, 1, final_scan_message[:w-2], curses.A_REVERSE | curses.A_BOLD); stdscr.refresh()
        time.sleep(2.5); curses.curs_set(previous_cursor_visibility); return final_scan_message
    
    stdscr.move(status_start_line_y + 4 if status_start_line_y + 4 < h else h-1, 1); stdscr.clrtoeol() 
    stdscr.addstr(status_start_line_y + 4 if status_start_line_y + 4 < h else h-1, 1, "Calibration terminée avec succès."[:w-2], curses.A_BOLD)
    stdscr.refresh(); time.sleep(1.5) 
    _clear_status_lines(status_start_line_y, 6) 

    global_image_num_for_prefix = 0
    scan_aborted = False
    final_scan_message = "Scan non complété (raison inconnue)."

    for pos_idx, target_pos_coords in enumerate(positions_list):
        _clear_status_lines(status_start_line_y, 5) 
        if status_start_line_y + 4 >= h : 
             stdscr.move(h-1, 1); stdscr.clrtoeol()
             stdscr.addstr(h-1, 1, f"Scan {pos_idx+1}/{len(positions_list)}...", curses.A_REVERSE); stdscr.refresh()
        else: 
            target_x, target_y = target_pos_coords
            current_band_idx_for_naming = pos_idx // num_x_images_per_band
            stdscr.move(status_start_line_y, 1); stdscr.clrtoeol()
            move_status_msg = f"Scan {pos_idx+1}/{len(positions_list)}: Bande {current_band_idx_for_naming+1}, Pos ({target_x:.1f}, {target_y:.1f})"
            stdscr.addstr(status_start_line_y, 1, move_status_msg[:w-2]); stdscr.refresh()

            delta_x_to_move = target_x - current_x_mm
            delta_y_to_move = target_y - current_y_mm
            path_length_mm = math.sqrt(delta_x_to_move**2 + delta_y_to_move**2)

            if path_length_mm >= (MM_PER_MICROSTEP / 2.0):
                current_move_speed = TARGET_SPEED_MM_S
                if current_move_speed <= 1e-6:
                    stdscr.move(status_start_line_y + 1, 1); stdscr.clrtoeol()
                    stdscr.addstr(status_start_line_y + 1, 1, "Erreur: Vitesse de déplacement trop faible. Scan annulé."); stdscr.refresh()
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
            stdscr.move(status_start_line_y + 1, 1); stdscr.clrtoeol()
            capture_info_msg = f"  Capture de {imgs_per_pos_at_location} image(s) à (X:{current_x_mm:.1f}, Y:{current_y_mm:.1f})"
            stdscr.addstr(status_start_line_y + 1, 1, capture_info_msg[:w-2]); stdscr.refresh()
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
                if "Impossible d'ouvrir la webcam" in msg: critical_capture_error = True; break
            if critical_capture_error:
                final_scan_message = "Erreur critique de capture. Scan interrompu."
                scan_aborted = True; break
            global_image_num_for_prefix += 1; time.sleep(0.1)
            stdscr.nodelay(True); key_press = stdscr.getch(); stdscr.nodelay(False)
            if key_press == 27: final_scan_message = "Scan interrompu par l'utilisateur."; scan_aborted = True; break
    
    if not scan_aborted: final_scan_message = "Scan des positions terminé avec succès."
    _clear_status_lines(status_start_line_y, 6) 

    stdscr.move(status_start_line_y, 1); stdscr.clrtoeol()
    stdscr.addstr(status_start_line_y, 1, "Retour au point HOME (0,0)..."[:w-2], curses.A_BOLD); stdscr.refresh()
    target_x_home, target_y_home = 0.0, 0.0
    delta_x_to_home = target_x_home - current_x_mm
    delta_y_to_home = target_y_home - current_y_mm
    home_path_len = math.sqrt(delta_x_to_home**2 + delta_y_to_home**2)
    if home_path_len >= (MM_PER_MICROSTEP / 2.0) :
        homing_speed_actual = min(HOMING_SPEED_MM_S, MAX_SPEED_MM_S) 
        if homing_speed_actual <= 1e-6:
            stdscr.move(status_start_line_y + 1, 1); stdscr.clrtoeol()
            stdscr.addstr(status_start_line_y + 1, 1, "Erreur: Vitesse de retour HOME trop faible."[:w-2])
        else:
            time_for_home_move = home_path_len / homing_speed_actual
            _dx_steps_cart_h = round(-delta_x_to_home * MICROSTEPS_PER_MM)
            _dy_steps_cart_h = round(delta_y_to_home * MICROSTEPS_PER_MM)
            num_iter_h = max(abs(int(round(-delta_x_to_home * MICROSTEPS_PER_MM) + round(delta_y_to_home * MICROSTEPS_PER_MM))), 
                               abs(int(round(-delta_x_to_home * MICROSTEPS_PER_MM) - round(delta_y_to_home * MICROSTEPS_PER_MM))))

            if num_iter_h > 0:
                pulse_delay_h = time_for_home_move / num_iter_h
                actual_pulse_delay_h = max(MINIMUM_PULSE_CYCLE_DELAY, pulse_delay_h)
                move_corexy(delta_x_to_home, delta_y_to_home, actual_pulse_delay_h)
    else: current_x_mm, current_y_mm = round(target_x_home,3), round(target_y_home,3)
    stdscr.move(status_start_line_y + 1, 1); stdscr.clrtoeol()
    stdscr.addstr(status_start_line_y + 1, 1, f"Retour HOME terminé. Pos: X={current_x_mm:.1f}, Y={current_y_mm:.1f}"[:w-2]); stdscr.refresh()
    time.sleep(1.5) 
    
    final_display_line = status_start_line_y + 3
    if final_display_line < h:
        stdscr.move(final_display_line, 1); stdscr.clrtoeol()
        stdscr.addstr(final_display_line, 1, final_scan_message[:w-2], curses.A_BOLD)
    else: 
        stdscr.move(h-1,1); stdscr.clrtoeol()
        stdscr.addstr(h-1,1, final_scan_message[:w-2], curses.A_BOLD)
    stdscr.refresh()
    curses.curs_set(previous_cursor_visibility); return final_scan_message

def draw_automatic_mode_ui(stdscr, title, prompts, current_values, status_message="", help_message=""):
    stdscr.clear(); h, w = stdscr.getmaxyx()
    stdscr.addstr(1, (w - len(title)) // 2, title, curses.A_BOLD)
    stdscr.addstr(2, (w - len(title)) // 2, "-" * len(title))
    line_y = 4
    for i, prompt_key in enumerate(prompts): 
        val_str = str(current_values.get(prompt_key, "..."))
        display_str = f"{prompt_key}: {val_str}" 
        stdscr.addstr(line_y + i, 2, display_str[:w-3])
    status_line_y = h - 4 
    if error_msg_line_explicit: status_line_y = h -5 
    if status_message:
        is_error = "erreur" in status_message.lower() or "impossible" in status_message.lower() or "échec" in status_message.lower()
        msg_y = h - 4 if is_error else h - 3
        stdscr.move(msg_y,1); stdscr.clrtoeol() 
        stdscr.addstr(msg_y, 1, status_message[:w-2], curses.A_REVERSE if is_error else curses.A_NORMAL)
    stdscr.move(h-2,1); stdscr.clrtoeol() 
    if help_message: stdscr.addstr(h - 2, 1, help_message[:w-2])
    else: stdscr.addstr(h - 2, 1, "ESC pour retourner au menu principal."[:w-2])
    stdscr.refresh()
error_msg_line_explicit = False 

def _automatic_mode_loop(stdscr):
    global XLIM_MM, YLIM_MM, POINT_MARGIN_MM 
    curses.curs_set(1); stdscr.keypad(True) 
    project_name = ""; project_path = ""
    num_x_points = None; num_y_points = None
    images_per_position = None; calculated_positions = [] 
    current_stage_idx = 0
    stages = ["GET_PROJECT_NAME", "GET_NUM_Y", "GET_NUM_X", 
              "GET_IMAGES_PER_POS", "CONFIRM_PARAMETERS", 
              "CALCULATE_POSITIONS", "READY_TO_EXECUTE"]
    config_values = {}; status_msg = "Initialisation du mode automatique..."; help_text_current = "Entrez les informations. ESC pour annuler."

    while True:
        current_stage = stages[current_stage_idx]
        prompt_keys_ordered = ["Nom du Projet", "Chemin du Projet", "Bandes (ny)", "Images par bande (nx)", "Images par Position", "Total Positions"]
        ui_values = {
            "Nom du Projet": project_name if project_name else "...",
            "Chemin du Projet": project_path if project_path else "...",
            "Bandes (ny)": str(num_y_points) if num_y_points is not None else "...", 
            "Images par bande (nx)": str(num_x_points) if num_x_points is not None else "...", 
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
            if not temp_name: status_msg = "Erreur: Le nom du projet ne peut pas être vide. Réessayez."; continue
            project_name = temp_name; timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            project_folder_name = f"{project_name.replace(' ', '_')}_{timestamp}" 
            project_path = os.path.join(".", project_folder_name) 
            try:
                os.makedirs(project_path, exist_ok=True)
                status_msg = f"Dossier projet créé: {project_path}"
                config_values["project_name"] = project_name; config_values["project_path"] = project_path
                current_stage_idx += 1
            except OSError as e: status_msg = f"Erreur création dossier {project_path}: {e}"; project_name = ""; project_path = ""; 

        elif current_stage == "GET_NUM_Y": 
            help_text_current = "Nombre de bandes de scan sur l'axe Y. ESC pour annuler." 
            status_msg = "Configuration de la grille de scan..."
            draw_automatic_mode_ui(stdscr, "-- Mode Automatique --", prompt_keys_ordered, ui_values, status_msg, help_text_current)
            temp_ny = _get_int_input_curses(stdscr, input_y_offset, 2, "Bandes (ny) (ex: 4): ", min_val=1) 
            if temp_ny is None: status_msg = "Annulation..."; time.sleep(0.5); break
            num_y_points = temp_ny; config_values["num_y_points"] = num_y_points; current_stage_idx += 1
            
        elif current_stage == "GET_NUM_X": 
            help_text_current = "Nombre d'images à prendre par bande (sur l'axe X). ESC pour annuler." 
            status_msg = "Configuration de la grille de scan..."
            draw_automatic_mode_ui(stdscr, "-- Mode Automatique --", prompt_keys_ordered, ui_values, status_msg, help_text_current)
            temp_nx = _get_int_input_curses(stdscr, input_y_offset, 2, "Images par bande (nx) (ex: 5): ", min_val=1) 
            if temp_nx is None: status_msg = "Annulation..."; time.sleep(0.5); break
            num_x_points = temp_nx; config_values["num_x_points"] = num_x_points; current_stage_idx += 1

        elif current_stage == "GET_IMAGES_PER_POS":
            help_text_current = "Nombre de captures d'images à chaque position. ESC pour annuler."
            status_msg = "Configuration de la capture..."
            draw_automatic_mode_ui(stdscr, "-- Mode Automatique --", prompt_keys_ordered, ui_values, status_msg, help_text_current)
            temp_imgs = _get_int_input_curses(stdscr, input_y_offset, 2, "Images par Position (ex: 1): ", min_val=1)
            if temp_imgs is None: status_msg = "Annulation..."; time.sleep(0.5); break
            images_per_position = temp_imgs; config_values["images_per_position"] = images_per_position; current_stage_idx += 1

        elif current_stage == "CONFIRM_PARAMETERS":
            total_pos_str = ui_values["Total Positions"]
            status_msg = f"Vérifiez les paramètres ({total_pos_str} positions au total)."
            help_text_current = "'Entrée' pour valider et calculer les positions. 'ESC' pour annuler."
            draw_automatic_mode_ui(stdscr, "-- Mode Automatique --", prompt_keys_ordered, ui_values, status_msg, help_text_current)
            stdscr.nodelay(False); key = stdscr.getch()
            if key == 27: status_msg = "Annulation..."; time.sleep(0.5); break
            elif key == curses.KEY_ENTER or key in [10, 13]:
                status_msg = "Paramètres confirmés. Calcul des positions..."; current_stage_idx += 1 
            
        elif current_stage == "CALCULATE_POSITIONS":
            status_msg = "Calcul en cours..."; help_text_current = "Veuillez patienter."
            draw_automatic_mode_ui(stdscr, "-- Mode Automatique --", prompt_keys_ordered, ui_values, status_msg, help_text_current)
            time.sleep(0.1) 
            calculated_positions, calc_msg = _calculate_scan_positions(num_x_points, num_y_points, XLIM_MM, YLIM_MM, POINT_MARGIN_MM)
            if calculated_positions is None:
                status_msg = calc_msg 
                help_text_current = "Erreur de calcul. Appuyez sur ESC et vérifiez les limites/points."
                draw_automatic_mode_ui(stdscr, "-- Mode Automatique --", prompt_keys_ordered, ui_values, status_msg, help_text_current)
                while True:
                    if stdscr.getch() == 27: break
                status_msg = "Annulation suite à erreur de calcul..."; time.sleep(0.5); break
            else:
                config_values["calculated_positions"] = calculated_positions; csv_feedback_msg = ""
                if project_path: 
                    csv_file_path = os.path.join(project_path, "position_list.csv")
                    try:
                        with open(csv_file_path, 'w', newline='') as csvfile:
                            csv_writer = csv.writer(csvfile)
                            csv_writer.writerow(['X_mm', 'Y_mm', 'n_img']) 
                            for pos_x, pos_y in calculated_positions:
                                csv_writer.writerow([pos_x, pos_y, images_per_position])
                        csv_feedback_msg = " position_list.csv sauvegardé."
                    except IOError as e: csv_feedback_msg = f" Erreur sauvegarde CSV: {e}."
                else: csv_feedback_msg = " Erreur: Chemin du projet non défini pour CSV."
                status_msg = calc_msg + csv_feedback_msg; current_stage_idx += 1 
        
        elif current_stage == "READY_TO_EXECUTE":
            help_text_current = "'Entrée' pour lancer la séquence. 'ESC' pour retourner au menu."
            draw_automatic_mode_ui(stdscr, "-- Mode Automatique --", prompt_keys_ordered, ui_values, status_msg, help_text_current)
            stdscr.nodelay(False); key = stdscr.getch()
            if key == 27: 
                status_msg = "Séquence annulée avant démarrage."; 
                draw_automatic_mode_ui(stdscr, "-- Mode Automatique --", prompt_keys_ordered, ui_values, status_msg, "Retour au menu...")
                time.sleep(1); break
            elif key == curses.KEY_ENTER or key in [10, 13]:
                status_msg = "Lancement de la séquence de scan..."
                draw_automatic_mode_ui(stdscr, "-- Mode Automatique --", prompt_keys_ordered, ui_values, status_msg, "Scan en cours... ESC pour interrompre.")
                time.sleep(0.5)
                scan_result_msg = _execute_automatic_scan(stdscr, project_path, calculated_positions, num_y_points, num_x_points, images_per_position, prompt_keys_ordered, ui_values)
                status_msg = scan_result_msg; help_text_current = "Appuyez sur une touche pour retourner au menu."
                draw_automatic_mode_ui(stdscr, "-- Mode Automatique --", prompt_keys_ordered, ui_values, status_msg, help_text_current)
                stdscr.nodelay(False); stdscr.getch(); break 
    curses.curs_set(0); stdscr.keypad(True)
    return_message = f"Mode Auto: {project_name}" if project_name else "Mode Auto quitté/annulé."
    if calculated_positions and project_name: return_message += f" ({len(calculated_positions)} positions)"
    return return_message


# --- Load File Mode Functions ---
def draw_load_file_ui(stdscr, title, info_dict, status_message="", help_message=""):
    """Draws the UI for the load file mode."""
    stdscr.clear()
    h, w = stdscr.getmaxyx()

    # Title
    stdscr.addstr(1, (w - len(title)) // 2, title, curses.A_BOLD)
    stdscr.addstr(2, (w - len(title)) // 2, "-" * len(title))

    # Info display
    line_y = 4
    for i, (key, value) in enumerate(info_dict.items()):
        display_str = f"{key}: {str(value)}"
        stdscr.addstr(line_y + i, 2, display_str[:w-3])

    # Status Message
    if status_message:
        is_error = "erreur" in status_message.lower() or \
                   "impossible" in status_message.lower() or \
                   "échec" in status_message.lower() or \
                   "non trouvé" in status_message.lower()
        msg_y = h - 4 if is_error else h - 3 # one line for help, one for status/error
        stdscr.move(msg_y, 1); stdscr.clrtoeol()
        stdscr.addstr(msg_y, 1, status_message[:w-2], curses.A_REVERSE if is_error else curses.A_NORMAL)
    
    # Help Message
    stdscr.move(h - 2, 1); stdscr.clrtoeol()
    if help_message:
        stdscr.addstr(h - 2, 1, help_message[:w-2])
    else: # Default help
        stdscr.addstr(h - 2, 1, "ESC pour retourner au menu principal."[:w-2])

    stdscr.refresh()

def _parse_csv_file(file_path_str):
    """
    Parses the CSV file expecting 'X_mm', 'Y_mm', 'n_img' headers.
    Returns (list_of_positions, error_message_or_none).
    Each item in list_of_positions is a dict: {'x': float, 'y': float, 'n_img': int}.
    """
    positions = []
    expected_headers = ['X_mm', 'Y_mm', 'n_img']
    try:
        with open(file_path_str, mode='r', newline='', encoding='utf-8') as csvfile:
            reader = csv.DictReader(csvfile)
            if not reader.fieldnames or not all(h in reader.fieldnames for h in expected_headers):
                return None, f"Erreur: En-têtes CSV incorrects. Attendu: {', '.join(expected_headers)}. Trouvé: {', '.join(reader.fieldnames or [])}"

            for i, row in enumerate(reader):
                try:
                    x_mm = float(row['X_mm'])
                    y_mm = float(row['Y_mm'])
                    n_img = int(row['n_img'])
                    if n_img <= 0:
                        return None, f"Erreur ligne {i+2}: n_img doit être positif (reçu {n_img})."
                    positions.append({'x': x_mm, 'y': y_mm, 'n_img': n_img})
                except ValueError as ve:
                    return None, f"Erreur conversion ligne {i+2}: {ve}. Vérifiez les valeurs numériques."
                except KeyError as ke:
                    return None, f"Erreur ligne {i+2}: Colonne manquante {ke}."
        if not positions:
            return None, "Erreur: Fichier CSV vide ou aucune donnée valide trouvée."
        return positions, f"{len(positions)} positions chargées depuis {os.path.basename(file_path_str)}."
    except FileNotFoundError:
        return None, f"Erreur: Fichier CSV '{file_path_str}' non trouvé."
    except Exception as e:
        return None, f"Erreur lecture CSV '{file_path_str}': {e}"

def _execute_csv_scan(stdscr, project_path, positions_data_list, base_ui_info, title_for_ui):
    """
    Executes the scan based on loaded CSV data: calibration, move to points, capture, return Home.
    base_ui_info is a dictionary of fixed info to display (like project name, file name).
    title_for_ui is the main title for the UI drawing function.
    """
    global current_x_mm, current_y_mm, TARGET_SPEED_MM_S, XLIM_MM, YLIM_MM, \
           MM_PER_MICROSTEP, MICROSTEPS_PER_MM, MINIMUM_PULSE_CYCLE_DELAY, \
           HOMING_SPEED_MM_S, MAX_SPEED_MM_S

    previous_cursor_visibility = curses.curs_set(0)
    h, w = stdscr.getmaxyx()
    
    # Determine where status messages start based on how much fixed info we have
    status_start_line_y = 4 + len(base_ui_info) + 1 # Title, separator, info items, separator for status

    def _clear_and_show_status(lines_to_display, is_progress_bar=False):
        # Clear previous status lines (e.g., 5 lines for status)
        for i in range(5): # Max 5 lines for status updates
            if status_start_line_y + i < h:
                stdscr.move(status_start_line_y + i, 1); stdscr.clrtoeol()
        
        # Display new status lines
        for i, line_text in enumerate(lines_to_display[:5]): # Show up to 5 lines
             if status_start_line_y + i < h:
                line_attr = curses.A_NORMAL
                if "Erreur" in line_text or "Échec" in line_text: line_attr = curses.A_REVERSE | curses.A_BOLD
                elif "Calibration" in line_text or "Terminé" in line_text or "Retour HOME" in line_text : line_attr = curses.A_BOLD
                stdscr.addstr(status_start_line_y + i, 1, line_text[:w-2], line_attr)
        stdscr.refresh()

    # --- 1. Initial UI Draw ---
    draw_load_file_ui(stdscr, title_for_ui, base_ui_info, "Préparation du scan...", "ESC pour interrompre à tout moment.")
    
    # --- 2. Calibration Initiale ---
    _clear_and_show_status(["Calibration initiale en cours..."])
    calibration_messages = perform_calibration_cycle() # Updates current_x_mm, current_y_mm

    _clear_and_show_status(["Calibration initiale en cours..."] + [f"  {m}" for m in calibration_messages[:3]]) # Show first few messages

    calibration_failed = any("Error" in msg or "Erreur" in msg or "failed" in msg for msg in calibration_messages)
    if calibration_failed:
        final_scan_message = "Échec de la calibration initiale. Scan annulé."
        _clear_and_show_status(calibration_messages + [final_scan_message])
        time.sleep(3)
        curses.curs_set(previous_cursor_visibility)
        return final_scan_message
    
    _clear_and_show_status(["Calibration terminée avec succès.", f"  Position: X={current_x_mm:.1f}, Y={current_y_mm:.1f}"])
    time.sleep(1.5)

    scan_aborted_by_user = False
    final_scan_message = "Scan par fichier CSV non complété." # Default

    for point_idx, point_data in enumerate(positions_data_list):
        target_x_csv = point_data['x']
        target_y_csv = point_data['y']
        num_images_at_pos = point_data['n_img']

        # --- Check for ESC press ---
        stdscr.nodelay(True); key_press = stdscr.getch(); stdscr.nodelay(False)
        if key_press == 27: # ESC
            final_scan_message = "Scan interrompu par l'utilisateur."
            scan_aborted_by_user = True; break
        
        _clear_and_show_status([
            f"Point {point_idx+1}/{len(positions_data_list)}: Cible CSV ({target_x_csv:.1f}, {target_y_csv:.1f})",
            f"  Position actuelle: ({current_x_mm:.1f}, {current_y_mm:.1f})"
        ])

        # --- Clamp Coordinates ---
        actual_target_x_mm = max(0.0, min(target_x_csv, XLIM_MM))
        actual_target_y_mm = max(0.0, min(target_y_csv, YLIM_MM))
        clamped_msg = ""
        if abs(actual_target_x_mm - target_x_csv) > 1e-9 or abs(actual_target_y_mm - target_y_csv) > 1e-9:
            clamped_msg = f"  Avertissement: Cible clampée à ({actual_target_x_mm:.1f}, {actual_target_y_mm:.1f})"
            _clear_and_show_status([
                f"Point {point_idx+1}/{len(positions_data_list)}: Cible CSV ({target_x_csv:.1f}, {target_y_csv:.1f})",
                clamped_msg,
                "Déplacement en cours..."
            ])
        else:
             _clear_and_show_status([
                f"Point {point_idx+1}/{len(positions_data_list)}: Vers ({actual_target_x_mm:.1f}, {actual_target_y_mm:.1f})",
                "Déplacement en cours..."
            ])


        # --- Move to Position ---
        delta_x_to_move = actual_target_x_mm - current_x_mm
        delta_y_to_move = actual_target_y_mm - current_y_mm
        path_length_mm = math.sqrt(delta_x_to_move**2 + delta_y_to_move**2)

        if path_length_mm >= (MM_PER_MICROSTEP / 2.0):
            current_move_speed = TARGET_SPEED_MM_S
            if current_move_speed <= 1e-6:
                final_scan_message = "Erreur: Vitesse de déplacement trop faible. Scan annulé."
                _clear_and_show_status([final_scan_message]); time.sleep(2); scan_aborted_by_user = True; break
            
            time_for_move_s = path_length_mm / current_move_speed
            # CoreXY conversion is inside move_corexy
            dx_steps_cart = round(-delta_x_to_move * MICROSTEPS_PER_MM)
            dy_steps_cart = round(delta_y_to_move * MICROSTEPS_PER_MM)
            num_iterations = max(abs(int(dx_steps_cart + dy_steps_cart)), abs(int(dx_steps_cart - dy_steps_cart)))

            if num_iterations > 0:
                pulse_delay = time_for_move_s / num_iterations
                actual_pulse_delay = max(MINIMUM_PULSE_CYCLE_DELAY, pulse_delay)
                move_corexy(delta_x_to_move, delta_y_to_move, actual_pulse_delay) # Updates current_x_mm, current_y_mm
        else: # Already at or very close to target, just update logical
            current_x_mm = round(actual_target_x_mm, 3)
            current_y_mm = round(actual_target_y_mm, 3)
        
        _clear_and_show_status([
            f"Point {point_idx+1}/{len(positions_data_list)}: Atteint ({current_x_mm:.1f}, {current_y_mm:.1f})",
            f"  Capture de {num_images_at_pos} image(s)..."
        ])
        
        # --- Capture Images ---
        # Naming: ProjectName/P000_Id00.tiff, P000_Id01.tiff ... P001_Id00.tiff
        filename_prefix_for_capture = f"P{point_idx:03d}" 
        base_path_for_this_capture = os.path.join(project_path, filename_prefix_for_capture)
        
        capture_output_msgs = capture_images(num_images_at_pos, base_path_for_this_capture)
        
        # Display first few capture messages
        capture_status_lines = [
            f"Point {point_idx+1}/{len(positions_data_list)}: Capture à ({current_x_mm:.1f}, {current_y_mm:.1f})",
        ] + [f"    {m}" for m in capture_output_msgs[:2]]
        
        critical_capture_error = False
        for msg in capture_output_msgs:
            if "Erreur" in msg and "Résolution" not in msg and "directory" not in msg and "Impossible d'ouvrir la webcam" not in msg:
                critical_capture_error = True; break
            if "Impossible d'ouvrir la webcam" in msg: critical_capture_error = True; break
        
        if critical_capture_error:
            final_scan_message = f"Erreur critique de capture au point {point_idx+1}. Scan interrompu."
            capture_status_lines.append(final_scan_message)
            _clear_and_show_status(capture_status_lines); time.sleep(2)
            scan_aborted_by_user = True; break # Treat as abortion
        
        _clear_and_show_status(capture_status_lines)
        time.sleep(0.2) # Brief pause after each point's capture messages

    if not scan_aborted_by_user:
        final_scan_message = "Scan par fichier CSV terminé avec succès."

    _clear_and_show_status([final_scan_message, "Retour au point HOME (0,0)..."])

    # --- Return to Home (0,0) ---
    target_x_home, target_y_home = 0.0, 0.0
    delta_x_to_home = target_x_home - current_x_mm
    delta_y_to_home = target_y_home - current_y_mm
    home_path_len = math.sqrt(delta_x_to_home**2 + delta_y_to_home**2)

    if home_path_len >= (MM_PER_MICROSTEP / 2.0) :
        homing_speed_actual = min(HOMING_SPEED_MM_S, MAX_SPEED_MM_S)
        if homing_speed_actual <= 1e-6:
             _clear_and_show_status([final_scan_message, "Erreur: Vitesse de retour HOME trop faible."])
        else:
            time_for_home_move = home_path_len / homing_speed_actual
            # Deltas are already cartesian for move_corexy
            dx_steps_cart_h = round(-delta_x_to_home * MICROSTEPS_PER_MM)
            dy_steps_cart_h = round(delta_y_to_home * MICROSTEPS_PER_MM)
            num_iter_h = max(abs(int(dx_steps_cart_h + dy_steps_cart_h)), abs(int(dx_steps_cart_h - dy_steps_cart_h)))
            if num_iter_h > 0:
                pulse_delay_h = time_for_home_move / num_iter_h
                actual_pulse_delay_h = max(MINIMUM_PULSE_CYCLE_DELAY, pulse_delay_h)
                move_corexy(delta_x_to_home, delta_y_to_home, actual_pulse_delay_h) # updates current_x/y
    else: # Already home or very close
        current_x_mm, current_y_mm = round(target_x_home,3), round(target_y_home,3)

    _clear_and_show_status([final_scan_message, f"Retour HOME terminé. Pos: X={current_x_mm:.1f}, Y={current_y_mm:.1f}"])
    time.sleep(1.5)
    
    curses.curs_set(previous_cursor_visibility)
    return final_scan_message


def _load_file_mode_loop(stdscr):
    """Main loop for the Load File Mode UI and logic."""
    global XLIM_MM, YLIM_MM # Access global machine limits

    curses.curs_set(1)
    stdscr.keypad(True)
    
    csv_file_path_str = ""
    project_name = ""
    project_path_created = "" # Actual path to the created project folder
    loaded_positions_data = []
    
    current_stage_idx = 0
    # Stages: Get CSV path, Parse CSV, Get Project Name, Confirm, Execute
    stages = ["GET_CSV_PATH", "PARSE_CSV", "GET_PROJECT_NAME", "CONFIRM_AND_EXECUTE"]
    
    status_msg = "Initialisation du mode Charger Fichier..."
    help_text_current = "Entrez le chemin du fichier CSV. ESC pour annuler."

    # This dictionary will hold info to display on the UI
    ui_info = {
        "Fichier CSV": "Non défini",
        "Nom du Projet": "Non défini",
        "Points Chargés": "0"
    }
    title = "-- Charger Fichier CSV --"

    while True:
        current_stage = stages[current_stage_idx]
        draw_load_file_ui(stdscr, title, ui_info, status_msg, help_text_current)
        
        # Input line y starts after title, separator, and ui_info items
        input_y_offset = 4 + len(ui_info) 

        if current_stage == "GET_CSV_PATH":
            help_text_current = "Chemin vers fichier CSV (ex: ./scan.csv). ESC pour annuler."
            status_msg = "Attente du chemin du fichier CSV..."
            draw_load_file_ui(stdscr, title, ui_info, status_msg, help_text_current) # Redraw with current messages

            temp_path = _get_string_input_curses(stdscr, input_y_offset, 2, "Chemin Fichier CSV: ", max_len=100)
            if temp_path is None: # ESC pressed
                status_msg = "Annulation..."; draw_load_file_ui(stdscr, title, ui_info, status_msg, "Retour au menu..."); time.sleep(0.5); break
            
            if not temp_path.lower().endswith(".csv"):
                status_msg = "Erreur: Le fichier doit avoir une extension .csv. Réessayez."
                continue
            if not os.path.exists(temp_path):
                status_msg = f"Erreur: Fichier '{temp_path}' non trouvé. Réessayez."
                continue
            
            csv_file_path_str = temp_path
            ui_info["Fichier CSV"] = os.path.basename(csv_file_path_str)
            current_stage_idx += 1 # Move to PARSE_CSV
            status_msg = f"Fichier '{os.path.basename(csv_file_path_str)}' sélectionné. Analyse..."

        elif current_stage == "PARSE_CSV":
            help_text_current = "Analyse du fichier CSV en cours..."
            draw_load_file_ui(stdscr, title, ui_info, status_msg, help_text_current) # Show parsing message
            time.sleep(0.1) # Brief pause for user to see message

            positions, parse_msg = _parse_csv_file(csv_file_path_str)
            status_msg = parse_msg # Update status with result of parsing
            
            if positions is None: # Parsing failed
                help_text_current = "Erreur de parsing. ESC pour annuler, ou entrez un nouveau chemin."
                # Stay in GET_CSV_PATH to allow re-entry or ESC
                current_stage_idx = stages.index("GET_CSV_PATH") 
                ui_info["Fichier CSV"] = "Non défini" # Reset UI
                csv_file_path_str = ""
                # status_msg already contains the error from _parse_csv_file
                draw_load_file_ui(stdscr, title, ui_info, status_msg, help_text_current)
                # Wait for ESC or new input (handled by GET_CSV_PATH)
                # We could add a getch here to pause on error if needed before auto-looping
                time.sleep(2) # Pause to show error
                continue 
            
            loaded_positions_data = positions
            ui_info["Points Chargés"] = str(len(loaded_positions_data))
            current_stage_idx += 1 # Move to GET_PROJECT_NAME
            status_msg = f"{parse_msg} Prêt pour nom de projet."


        elif current_stage == "GET_PROJECT_NAME":
            help_text_current = "Entrez le nom du projet. ESC pour annuler."
            status_msg = "Configuration du projet..." # status_msg might still have parsing success
            draw_load_file_ui(stdscr, title, ui_info, status_msg, help_text_current)

            temp_proj_name = _get_string_input_curses(stdscr, input_y_offset, 2, "Nom du Projet: ")
            if temp_proj_name is None: # ESC
                status_msg = "Annulation..."; draw_load_file_ui(stdscr, title, ui_info, status_msg, "Retour au menu..."); time.sleep(0.5); break
            
            if not temp_proj_name:
                status_msg = "Erreur: Le nom du projet ne peut pas être vide. Réessayez."
                continue # Stay in GET_PROJECT_NAME

            project_name = temp_proj_name
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            project_folder_name = f"{project_name.replace(' ', '_')}_{timestamp}"
            project_path_created = os.path.join(".", project_folder_name)
            
            try:
                os.makedirs(project_path_created, exist_ok=True)
                ui_info["Nom du Projet"] = project_name # project_folder_name for full path
                status_msg = f"Dossier projet: {project_path_created}"
                current_stage_idx += 1 # Move to CONFIRM_AND_EXECUTE
            except OSError as e:
                status_msg = f"Erreur création dossier {project_path_created}: {e}"
                project_name = ""; project_path_created = ""; ui_info["Nom du Projet"] = "Erreur";
                # Stay in GET_PROJECT_NAME or allow ESC
                continue


        elif current_stage == "CONFIRM_AND_EXECUTE":
            status_msg = f"{len(loaded_positions_data)} points prêts pour scan. Dossier: {os.path.basename(project_path_created)}"
            help_text_current = "'Entrée' pour lancer. 'ESC' pour annuler et retourner au menu."
            draw_load_file_ui(stdscr, title, ui_info, status_msg, help_text_current)
            
            stdscr.nodelay(False) # Wait for a key press
            key = stdscr.getch()

            if key == 27: # ESC
                status_msg = "Scan annulé par l'utilisateur."; 
                draw_load_file_ui(stdscr, title, ui_info, status_msg, "Retour au menu..."); time.sleep(1); break
            
            elif key == curses.KEY_ENTER or key in [10, 13]:
                status_msg = "Lancement de la séquence de scan depuis fichier..."
                draw_load_file_ui(stdscr, title, ui_info, status_msg, "Scan en cours... ESC pour interrompre.")
                time.sleep(0.5) # Show "Lancement" message

                # Pass ui_info as base_ui_info for _execute_csv_scan to display fixed info
                scan_execution_summary = _execute_csv_scan(stdscr, project_path_created, loaded_positions_data, ui_info, title)
                
                status_msg = scan_execution_summary
                help_text_current = "Appuyez sur une touche pour retourner au menu."
                draw_load_file_ui(stdscr, title, ui_info, status_msg, help_text_current)
                stdscr.nodelay(False); stdscr.getch() # Wait for key before exiting
                break # End of load file mode
            # Else, loop in CONFIRM_AND_EXECUTE stage waiting for Enter or ESC

    curses.curs_set(0) # Ensure cursor is hidden on exit
    stdscr.keypad(True) # Restore keypad state just in case
    return_message = f"Charger Fichier: {project_name}" if project_name and loaded_positions_data else "Charger Fichier quitté/annulé."
    if project_name and loaded_positions_data:
        return_message += f" ({len(loaded_positions_data)} pts)"
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
    
    status_line_y = h - 3 # Reserve two lines at the bottom for status and help
    if status_message: 
        stdscr.move(status_line_y, 1); stdscr.clrtoeol()
        stdscr.addstr(status_line_y, 1, status_message[:w-2])

    help_line_y = h-2
    stdscr.move(help_line_y, 1); stdscr.clrtoeol()
    stdscr.addstr(help_line_y, 1, "HAUT/BAS/ENTRÉE pour sélectionner. 'Q' pour quitter.")
    stdscr.refresh()

def _main_menu_loop(stdscr):
    global last_command_output, current_x_mm, current_y_mm 
    curses.curs_set(0); stdscr.keypad(True)
    menu_items = ["Mode automatique", "Mode manuel", "Charger un fichier", "Quitter SimuFly"]
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
            elif selected_option == "Mode automatique":
                status_msg = "Lancement Mode Automatique..."; draw_main_menu_ui(stdscr, menu_items, current_selection, status_msg); time.sleep(0.2)
                auto_mode_status = _automatic_mode_loop(stdscr) 
                curses.curs_set(0); stdscr.keypad(True) 
                status_msg = f"{auto_mode_status} Pos: X={current_x_mm:.1f}, Y={current_y_mm:.1f}"; last_command_output = []
            # --- Handle Load File Mode ---
            elif selected_option == "Charger un fichier":
                status_msg = "Lancement Mode Charger Fichier..."; draw_main_menu_ui(stdscr, menu_items, current_selection, status_msg); time.sleep(0.2)
                load_file_status = _load_file_mode_loop(stdscr) # Call the new load file mode loop
                curses.curs_set(0); stdscr.keypad(True) # Restore curses settings for main menu
                status_msg = f"{load_file_status} Pos: X={current_x_mm:.1f}, Y={current_y_mm:.1f}"
                last_command_output = [] # Clear any potential manual mode output
            elif selected_option == "Quitter SimuFly":
                status_msg = "Quitting SimuFly..."; draw_main_menu_ui(stdscr, menu_items, current_selection, status_msg); time.sleep(0.5); break 
            else: # For "Démonstration" or other unimplemented
                status_msg = f"Option '{selected_option}' non implémentée."
                draw_main_menu_ui(stdscr, menu_items, current_selection, status_msg); stdscr.timeout(1500); stdscr.getch(); stdscr.timeout(-1) # Show message for 1.5s
                status_msg = f"Pos: X={current_x_mm:.1f}, Y={current_y_mm:.1f}" # Reset status
        else: # Other key press, just update status if needed (e.g. if status showed a temp message)
            status_msg = f"GPIO {'OK' if pul_device_m1 else 'ERREUR'}. Pos: X={current_x_mm:.1f}, Y={current_y_mm:.1f}"


if __name__ == "__main__":
    gpio_ok = setup_gpio()
    if gpio_ok: print("GPIO initialisé. Lancement SimuFly Curses...")
    else: print("ERREUR: Init GPIO échouée. Interface Curses démarrera sans fonctions moteur.")
    time.sleep(0.1) # Give a moment for the print to show
    try:
        curses.wrapper(_main_menu_loop)
    except Exception as e: 
        # This catch is for errors outside the Curses wrapper or if wrapper fails badly
        print(f"Erreur critique hors boucle Curses principale: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # curses.endwin() # wrapper handles this
        print("Nettoyage GPIO avant de quitter SimuFly..."); 
        cleanup_gpio(); 
        print("SimuFly terminé.")