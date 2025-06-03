import gpiozero
import time
import math
import re # For command parsing
import curses

# GPIO Pin Configuration (BCM numbering) - As provided by user
# Motor 1 (often Motor A in CoreXY nomenclature)
PUL_PIN_M1 = 6
DIR_PIN_M1 = 5

# Motor 2 (often Motor B in CoreXY nomenclature)
PUL_PIN_M2 = 27
DIR_PIN_M2 = 17

# --- GPIOZero Device Objects (will be initialized in setup_gpio) ---
pul_device_m1 = None
dir_device_m1 = None
pul_device_m2 = None
dir_device_m2 = None

# --- Motor & System Parameters ---
MOTOR_NATIVE_STEPS_PER_REV = 400
DRIVER_MICROSTEP_DIVISOR = 2

MM_PER_MICROSTEP = 0.1
if MM_PER_MICROSTEP == 0:
    raise ValueError("MM_PER_MICROSTEP cannot be zero.")
MICROSTEPS_PER_MM = 1.0 / MM_PER_MICROSTEP

DEFAULT_XLIM_MM = 1000.0
DEFAULT_YLIM_MM = 1000.0
XLIM_MM = DEFAULT_XLIM_MM
YLIM_MM = DEFAULT_YLIM_MM

# --- Speed and Timing Parameters ---
DEFAULT_SPEED_MM_S = 300.0
MAX_SPEED_MM_S = 450.0
TARGET_SPEED_MM_S = min(DEFAULT_SPEED_MM_S, MAX_SPEED_MM_S)
HOMING_SPEED_MM_S = 200.0

MIN_START_SPEED_MM_S = 5.0 # <<< NOUVEAU >>> Speed at the beginning of accel and end of decel

MIN_PULSE_WIDTH = 0.000002      # 2 microseconds (minimum pulse high time)
MINIMUM_PULSE_CYCLE_DELAY = 0.0001 # 100µs -> max 10,000 step cycles/sec (effective max step rate)

# --- System State ---
current_x_mm = 0.0
current_y_mm = 0.0
absolute_mode = True
last_command_output = []

def setup_gpio():
    global pul_device_m1, dir_device_m1, pul_device_m2, dir_device_m2
    try:
        pul_device_m1 = gpiozero.OutputDevice(PUL_PIN_M1, active_high=True, initial_value=False)
        dir_device_m1 = gpiozero.OutputDevice(DIR_PIN_M1, active_high=True, initial_value=False)
        pul_device_m2 = gpiozero.OutputDevice(PUL_PIN_M2, active_high=True, initial_value=False)
        dir_device_m2 = gpiozero.OutputDevice(DIR_PIN_M2, active_high=True, initial_value=False)
        print("GPIO initialized successfully.")
        return True
    except Exception as e:
        print(f"Error during GPIO initialization: {e}")
        return False

def cleanup_gpio():
    if pul_device_m1: pul_device_m1.close()
    if dir_device_m1: dir_device_m1.close()
    if pul_device_m2: pul_device_m2.close()
    if dir_device_m2: dir_device_m2.close()
    print("GPIO cleaned up.")

def lerp(start, end, t):
    """Linear interpolation"""
    return start * (1 - t) + end * t

# <<< NOUVEAU / MODIFIÉ >>> : Remplacé move_motors_coordinated
def move_motors_coordinated_ramped(steps_m1_target, steps_m2_target,
                                   accel_iterations, cruise_iterations, decel_iterations,
                                   initial_delay, cruise_delay, final_delay):
    messages = []
    if not all([pul_device_m1, dir_device_m1, pul_device_m2, dir_device_m2]):
        messages.append("Error: GPIO devices not initialized.")
        return messages

    if steps_m1_target > 0: dir_device_m1.off()
    else: dir_device_m1.on()

    if steps_m2_target > 0: dir_device_m2.off()
    else: dir_device_m2.on()
    
    time.sleep(0.001) # Allow direction pins to settle

    abs_steps_m1 = abs(steps_m1_target)
    abs_steps_m2 = abs(steps_m2_target)
    
    total_iterations = accel_iterations + cruise_iterations + decel_iterations
    if total_iterations == 0:
        return messages # No steps to make

    # Ensure delays are not too small
    initial_delay = max(initial_delay, MINIMUM_PULSE_CYCLE_DELAY)
    cruise_delay = max(cruise_delay, MINIMUM_PULSE_CYCLE_DELAY)
    final_delay = max(final_delay, MINIMUM_PULSE_CYCLE_DELAY)

    # Ensure delays can accommodate MIN_PULSE_WIDTH
    initial_delay = max(initial_delay, MIN_PULSE_WIDTH + 1e-7) # Add small epsilon
    cruise_delay = max(cruise_delay, MIN_PULSE_WIDTH + 1e-7)
    final_delay = max(final_delay, MIN_PULSE_WIDTH + 1e-7)


    for i in range(total_iterations):
        current_calculated_delay = cruise_delay # Default

        if i < accel_iterations:
            if accel_iterations > 0:
                # Progress t from 0 (approx) to 1.0 over the accel phase
                t = (i + 1) / accel_iterations 
                current_calculated_delay = lerp(initial_delay, cruise_delay, t)
            else: # Should only happen if accel_iterations is 0
                 current_calculated_delay = cruise_delay
        elif i < accel_iterations + cruise_iterations:
            current_calculated_delay = cruise_delay
        else: # Deceleration phase
            if decel_iterations > 0:
                # Progress t from 0 (approx) to 1.0 over the decel phase
                decel_phase_iter = i - (accel_iterations + cruise_iterations)
                t = (decel_phase_iter + 1) / decel_iterations
                current_calculated_delay = lerp(cruise_delay, final_delay, t)
            else: # Should only happen if decel_iterations is 0
                current_calculated_delay = final_delay


        # Ensure current_delay respects global minimums
        current_calculated_delay = max(current_calculated_delay, MINIMUM_PULSE_CYCLE_DELAY)
        current_calculated_delay = max(current_calculated_delay, MIN_PULSE_WIDTH + 1e-7)


        perform_pulse_m1 = False
        if accel_iterations + cruise_iterations + decel_iterations > 0: # Avoid division by zero for step distribution
            # Distribute steps for M1 evenly across total_iterations
            if abs_steps_m1 > 0 and (i * abs_steps_m1) // total_iterations != ((i - 1) * abs_steps_m1) // total_iterations :
                 perform_pulse_m1 = True
            if i == 0 and abs_steps_m1 > 0 : # Ensure first step if any
                 perform_pulse_m1 = True


        perform_pulse_m2 = False
        if accel_iterations + cruise_iterations + decel_iterations > 0:
            # Distribute steps for M2 evenly across total_iterations
            if abs_steps_m2 > 0 and (i * abs_steps_m2) // total_iterations != ((i - 1) * abs_steps_m2) // total_iterations :
                perform_pulse_m2 = True
            if i == 0 and abs_steps_m2 > 0 : # Ensure first step if any
                perform_pulse_m2 = True
        
        # Bresenham-like step distribution for smoother multi-axis moves under ramp
        # This is a simplified placeholder. A true Bresenham or DDA for step timing
        # during ramps is more complex than simply checking if i < abs_steps_mX.
        # The provided logic above tries to distribute the sub-motor steps across the dominant axis's ramp iterations.
        # A simpler (but potentially less smooth for the non-dominant motor) way:
        # perform_pulse_m1 = (i < abs_steps_m1) # This was the old way, might cause issues with ramping if total_iterations is much larger
        # perform_pulse_m2 = (i < abs_steps_m2)
        # For now, using a slightly more advanced distribution based on dominant axis ramp iterations.
        # This part is tricky. Let's use a standard DDA approach for deciding when to step each motor based on dominant axis count.
        # The `total_iterations` is based on the dominant motor for the ramp profile.
        # We need to decide *which* of the `total_iterations` will include a step for M1 and M2.

        # Corrected DDA-like step decision for ramped move:
        # Each of the `total_iterations` represents a time slice dictated by the ramp.
        # We decide if a motor steps in a given time slice.
        # This is similar to how it was before, just that total_iterations comes from the ramp calculation.
        perform_pulse_m1 = (i < abs_steps_m1) if abs_steps_m1 <= total_iterations else ( (i * abs_steps_m1) // total_iterations > ((i-1)*abs_steps_m1)//total_iterations if i > 0 else (abs_steps_m1 > 0) )
        perform_pulse_m2 = (i < abs_steps_m2) if abs_steps_m2 <= total_iterations else ( (i * abs_steps_m2) // total_iterations > ((i-1)*abs_steps_m2)//total_iterations if i > 0 else (abs_steps_m2 > 0) )


        if perform_pulse_m1: pul_device_m1.on()
        if perform_pulse_m2: pul_device_m2.on()
        
        time.sleep(MIN_PULSE_WIDTH)

        if perform_pulse_m1: pul_device_m1.off()
        if perform_pulse_m2: pul_device_m2.off()
        
        inter_pulse_delay = current_calculated_delay - MIN_PULSE_WIDTH
        if inter_pulse_delay > 0:
            time.sleep(inter_pulse_delay)
        # else: if inter_pulse_delay is zero or negative, it means the step cycle is very short,
        # potentially limited by MIN_PULSE_WIDTH itself. No additional sleep needed.
            
    return messages

# <<< MODIFIÉ >>> : Remplacé move_corexy
def move_corexy_ramped(delta_x_mm, delta_y_mm, target_speed_mm_s):
    global current_x_mm, current_y_mm
    motor_messages = []

    delta_x_steps_cartesian = round(-delta_x_mm * MICROSTEPS_PER_MM)
    delta_y_steps_cartesian = round(delta_y_mm * MICROSTEPS_PER_MM)

    steps_m1 = delta_x_steps_cartesian + delta_y_steps_cartesian
    steps_m2 = delta_x_steps_cartesian - delta_y_steps_cartesian
    
    abs_steps_m1 = abs(int(steps_m1))
    abs_steps_m2 = abs(int(steps_m2))
    
    total_dominant_steps = max(abs_steps_m1, abs_steps_m2)

    if total_dominant_steps == 0:
        # No physical movement, but update logical position if deltas were non-zero (e.g. due to rounding to 0 steps)
        current_x_mm += delta_x_mm
        current_y_mm += delta_y_mm
        current_x_mm = round(current_x_mm, 3)
        current_y_mm = round(current_y_mm, 3)
        return motor_messages # No motor movement needed

    # --- Calculate ramp parameters ---
    accel_iterations = 0
    cruise_iterations = 0
    decel_iterations = 0

    if total_dominant_steps == 1:
        accel_iterations = 0
        cruise_iterations = 1
        decel_iterations = 0
    elif total_dominant_steps == 2:
        accel_iterations = 1
        cruise_iterations = 0
        decel_iterations = 1
    elif total_dominant_steps >=3: # 1/3, 1/3, 1/3 split
        accel_iterations = total_dominant_steps // 3
        decel_iterations = total_dominant_steps // 3
        cruise_iterations = total_dominant_steps - accel_iterations - decel_iterations
    else: # total_dominant_steps == 0, already handled
        pass


    # Calculate delays
    # Ensure speeds are positive for delay calculation
    safe_min_start_speed_mm_s = max(0.01, MIN_START_SPEED_MM_S) # Avoid zero division
    safe_target_speed_mm_s = max(0.01, target_speed_mm_s)      # Avoid zero division

    initial_delay = 1.0 / (safe_min_start_speed_mm_s * MICROSTEPS_PER_MM)
    cruise_delay = 1.0 / (safe_target_speed_mm_s * MICROSTEPS_PER_MM)
    final_delay = initial_delay
    
    # Ensure delays are not smaller than system minimum
    initial_delay = max(initial_delay, MINIMUM_PULSE_CYCLE_DELAY)
    cruise_delay = max(cruise_delay, MINIMUM_PULSE_CYCLE_DELAY)
    final_delay = max(final_delay, MINIMUM_PULSE_CYCLE_DELAY)

    # If target speed is slower than min_start_speed, adjust initial_delay to not be "faster"
    if safe_target_speed_mm_s < safe_min_start_speed_mm_s:
        initial_delay = cruise_delay # Start at cruise_delay if target is very slow

    motor_messages = move_motors_coordinated_ramped(
        int(steps_m1), int(steps_m2),
        accel_iterations, cruise_iterations, decel_iterations,
        initial_delay, cruise_delay, final_delay
    )

    current_x_mm += delta_x_mm
    current_y_mm += delta_y_mm
    current_x_mm = round(current_x_mm, 3) 
    current_y_mm = round(current_y_mm, 3)
    return motor_messages

def parse_command_and_execute(line):
    global current_x_mm, current_y_mm, absolute_mode, TARGET_SPEED_MM_S, XLIM_MM, YLIM_MM 
    
    output_messages = []
    command = line.strip().upper()
    parts = command.split()
    instruction = parts[0] if parts else ""

    output_messages.append(f"CMD: {command}")

    if instruction == "ABS":
        absolute_mode = True
        output_messages.append("  Mode: Absolute Positioning (ABS)")
    elif instruction == "REL":
        absolute_mode = False
        output_messages.append("  Mode: Relative Positioning (REL)")
    elif instruction == "HOME": # <<< MODIFIÉ >>>
        output_messages.append("  Homing (HOME):")
        target_x_abs = 0.0
        target_y_abs = 0.0
        
        dx_home = 0.0 
        dy_home = 0.0
        if absolute_mode:
            dx_home = target_x_abs - current_x_mm
            dy_home = target_y_abs - current_y_mm
        else: # REL mode
            # In REL mode, HOME just resets coordinates without physical move based on current interpretation
            current_x_mm = target_x_abs
            current_y_mm = target_y_abs
            output_messages.append("    Logical position reset to 0,0. No physical movement in REL for this HOME.")
            output_messages.append(f"  New position: X={current_x_mm:.3f} mm, Y={current_y_mm:.3f} mm")
            return output_messages, True # Exit early for REL HOME

        actual_homing_speed = min(HOMING_SPEED_MM_S, MAX_SPEED_MM_S)
        home_path_length_mm = math.sqrt(dx_home**2 + dy_home**2)

        if home_path_length_mm > (MM_PER_MICROSTEP / 2.0): # Only move if a significant distance
            output_messages.append(f"    Moving to 0,0 from {current_x_mm:.2f}, {current_y_mm:.2f}")
            output_messages.append(f"    Homing cruise speed: {actual_homing_speed:.2f} mm/s.")
            
            # move_corexy_ramped handles position updates internally
            motor_msgs = move_corexy_ramped(dx_home, dy_home, actual_homing_speed)
            output_messages.extend(motor_msgs)
        else: 
            current_x_mm = target_x_abs # Snap to 0,0 if already very close
            current_y_mm = target_y_abs
            output_messages.append("    Already at (or very close to) 0,0.")
        
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
        # ... (no changes to LIMITS logic itself)
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
            
    elif instruction == "MOVE" : # <<< MODIFIÉ >>>
        target_x_cmd = None
        target_y_cmd = None
        s_value_this_cmd_req = None
        
        for part in parts[1:]:
            if part.startswith('X'):
                try: target_x_cmd = float(part[1:])
                except ValueError: output_messages.append(f"  Invalid X value: {part}"); return output_messages, True
            elif part.startswith('Y'):
                try: target_y_cmd = float(part[1:])
                except ValueError: output_messages.append(f"  Invalid Y value: {part}"); return output_messages, True
            elif part.startswith('S'):
                 try: s_value_this_cmd_req = float(part[1:])
                 except ValueError: output_messages.append(f"  Invalid S value in MOVE: {part}")

        if target_x_cmd is None and target_y_cmd is None:
            output_messages.append("  No X or Y coordinate specified for MOVE.")
            return output_messages, True

        current_move_cruise_speed_mm_s = TARGET_SPEED_MM_S
        if s_value_this_cmd_req is not None:
            if s_value_this_cmd_req > 0:
                if s_value_this_cmd_req > MAX_SPEED_MM_S:
                    current_move_cruise_speed_mm_s = MAX_SPEED_MM_S
                    output_messages.append(f"  Requested MOVE speed {s_value_this_cmd_req:.2f} mm/s exceeds limit of {MAX_SPEED_MM_S:.2f} mm/s.")
                    output_messages.append(f"  MOVE cruise speed clamped to MAX: {current_move_cruise_speed_mm_s:.2f} mm/s.")
                else:
                    current_move_cruise_speed_mm_s = s_value_this_cmd_req
            else:
                output_messages.append(f"  Speed S in MOVE must be positive. Using global {TARGET_SPEED_MM_S:.2f} mm/s (max {MAX_SPEED_MM_S:.2f} mm/s).")
        
        current_move_cruise_speed_mm_s = min(current_move_cruise_speed_mm_s, MAX_SPEED_MM_S)

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
        if XLIM_MM != float('inf'):
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
            if current_move_cruise_speed_mm_s <= 1e-6: 
                output_messages.append(f"  Target cruise speed {current_move_cruise_speed_mm_s:.2e} mm/s is too low. No move executed.")
            else:
                output_messages.append(f"  Moving by dx={delta_x_to_move:.3f} mm, dy={delta_y_to_move:.3f} mm")
                output_messages.append(f"  To X={actual_target_x_mm:.3f}, Y={actual_target_y_mm:.3f}")
                output_messages.append(f"  Target cruise speed: {current_move_cruise_speed_mm_s:.2f} mm/s. Min start speed: {MIN_START_SPEED_MM_S:.2f} mm/s.")
                
                # move_corexy_ramped handles position updates internally
                motor_msgs = move_corexy_ramped(delta_x_to_move, delta_y_to_move, current_move_cruise_speed_mm_s)
                output_messages.extend(motor_msgs)
        
        output_messages.append(f"  New position: X={current_x_mm:.3f} mm, Y={current_y_mm:.3f} mm")

    elif instruction == "POS":
        x_lim_display = f"{XLIM_MM:.0f}" if XLIM_MM != float('inf') else "INF (Disabled)"
        y_lim_display = f"{YLIM_MM:.0f}" if YLIM_MM != float('inf') else "INF (Disabled)"
        output_messages.append(f"  Current Position: X={current_x_mm:.3f} mm, Y={current_y_mm:.3f} mm")
        output_messages.append(f"  Target Speed (Cruise): {TARGET_SPEED_MM_S:.2f} mm/s (Max settable: {MAX_SPEED_MM_S:.2f} mm/s)")
        output_messages.append(f"  Min Start/End Speed: {MIN_START_SPEED_MM_S:.2f} mm/s") # <<< NOUVEAU >>>
        output_messages.append(f"  Mode: {'Absolute' if absolute_mode else 'Relative'}")
        output_messages.append(f"  Effective Limits: X=[0, {x_lim_display}], Y=[0, {y_lim_display}]")

    elif instruction in ["EXIT", "QUIT"]:
        output_messages.append("  Exiting program.")
        return output_messages, False 
        
    else:
        if instruction:
            output_messages.append(f"  Unknown or unsupported command: {instruction}")
    
    return output_messages, True 

def draw_ui(stdscr, header_lines, status_lines, command_output_lines, input_prompt):
    stdscr.clear()
    max_y, max_x = stdscr.getmaxyx()
    current_y = 0

    for i, line in enumerate(header_lines):
        if current_y < max_y: stdscr.addstr(current_y, 0, line[:max_x-1]); current_y += 1
    if current_y < max_y: stdscr.addstr(current_y, 0, "-" * (max_x -1) ); current_y +=1

    for i, line in enumerate(status_lines):
        if current_y < max_y: stdscr.addstr(current_y, 0, line[:max_x-1]); current_y += 1
    if current_y < max_y: stdscr.addstr(current_y, 0, "-" * (max_x-1) ); current_y +=1
    
    output_start_y = current_y
    available_lines_for_output = max_y - output_start_y - 2 
    
    start_index = 0
    if len(command_output_lines) > available_lines_for_output and available_lines_for_output > 0 :
        start_index = len(command_output_lines) - available_lines_for_output
    
    for i, line in enumerate(command_output_lines[start_index:]):
        if output_start_y + i < max_y - 2: stdscr.addstr(output_start_y + i, 0, line[:max_x-1]) 
    
    current_y = max_y - 2 
    if current_y > 0 : stdscr.addstr(current_y, 0, "-" * (max_x-1) )
    
    input_line_y = max_y - 1
    if input_line_y > 0:
        stdscr.addstr(input_line_y, 0, input_prompt)
        stdscr.move(input_line_y, len(input_prompt))

    stdscr.refresh()

def _curses_main_loop(stdscr):
    global TARGET_SPEED_MM_S, current_x_mm, current_y_mm, absolute_mode, last_command_output, XLIM_MM, YLIM_MM

    curses.curs_set(1)
    stdscr.nodelay(False)

    running = True
    while running:
        x_limit_display_str = f"{XLIM_MM:.0f}" if XLIM_MM != float('inf') else "INF (Disabled)"
        y_limit_display_str = f"{YLIM_MM:.0f}" if YLIM_MM != float('inf') else "INF (Disabled)"

        header = [ # <<< MODIFIÉ >>>
            "--- CoreXY CLI Controller (Ramped Motion) ---", # MODIFIED
            f"Max Settable Speed: {MAX_SPEED_MM_S:.2f} mm/s, Min Start Speed: {MIN_START_SPEED_MM_S:.2f} mm/s",
            f"Optimal speed : 300 mm/s",
            f"Effective Limits: X=[0, {x_limit_display_str}], Y=[0, {y_limit_display_str}] mm",
            f"Resolution: {MM_PER_MICROSTEP} mm/microstep ({MICROSTEPS_PER_MM} microsteps/mm)",
            f"Motor Native Steps/Rev: {MOTOR_NATIVE_STEPS_PER_REV}, Driver Microstepping: 1/{DRIVER_MICROSTEP_DIVISOR}",
            f"Min Pulse Cycle Delay: {MINIMUM_PULSE_CYCLE_DELAY*1000:.3f} ms (Max ~{1.0/MINIMUM_PULSE_CYCLE_DELAY/MICROSTEPS_PER_MM if MICROSTEPS_PER_MM > 0 else 'N/A'} mm/s)",
            "Available commands:",
            "  MOVE X<v> Y<v> [S<v>]",
            "  ABS, REL, HOME, S<v>, LIMITS [ON|OFF], POS, EXIT/QUIT"
        ]
        status = [ # <<< MODIFIÉ >>>
            f"Current Position: X={current_x_mm:.3f} mm, Y={current_y_mm:.3f} mm",
            f"Target Cruise Speed: {TARGET_SPEED_MM_S:.2f} mm/s, Mode: {'ABS' if absolute_mode else 'REL'}"
        ]
        
        input_prompt_text = "CoreXY > "
        draw_ui(stdscr, header, status, last_command_output, input_prompt_text)
        
        input_line_y = stdscr.getmaxyx()[0] - 1
        
        try:
            if input_line_y > 0 and input_line_y > (len(header) + len(status) +2) :
                 stdscr.move(input_line_y, len(input_prompt_text))
                 curses.echo()
                 cmd_line_bytes = stdscr.getstr(input_line_y, len(input_prompt_text), 60)
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
            last_command_output = [f"An unexpected error occurred: {e}"]
            # import traceback # Uncomment for debugging
            # last_command_output.extend(traceback.format_exc().splitlines())
            
    status = [
        f"Current Position: X={current_x_mm:.3f} mm, Y={current_y_mm:.3f} mm",
        f"Target Speed: {TARGET_SPEED_MM_S:.2f} mm/s, Mode: {'ABS' if absolute_mode else 'REL'}"
    ]
    if not last_command_output or (last_command_output and "Exiting program." not in last_command_output[-1]):
        last_command_output.append("Exiting program.")
    
    x_limit_display_str = f"{XLIM_MM:.0f}" if XLIM_MM != float('inf') else "INF (Disabled)"
    y_limit_display_str = f"{YLIM_MM:.0f}" if YLIM_MM != float('inf') else "INF (Disabled)"
    header = [
        "--- CoreXY CLI Controller (Ramped Motion) ---",
        f"Max Settable Speed: {MAX_SPEED_MM_S:.2f} mm/s, Min Start Speed: {MIN_START_SPEED_MM_S:.2f} mm/s",
        f"Effective Limits: X=[0, {x_limit_display_str}], Y=[0, {y_limit_display_str}] mm",
        "Available commands: ... (EXIT/QUIT to close)"
    ]
    try:
        draw_ui(stdscr, header, status, last_command_output, "Exited. ")
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
            print("Program terminated.") 
    else:
        print("GPIO setup failed. Program will not start controller interface.")
        print("Program terminated.")