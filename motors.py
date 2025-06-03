import gpiozero
import time
import math
import re # For command parsing
import curses # <-- Ajout pour l'interface CLI améliorée

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
DEFAULT_SPEED_MM_S = 350.0       # Default speed in mm/s
MAX_SPEED_MM_S = 450.0           # <<< NEW >>> Maximum allowable speed by user
TARGET_SPEED_MM_S = min(DEFAULT_SPEED_MM_S, MAX_SPEED_MM_S) # Global target speed, ensure initial is not over max
HOMING_SPEED_MM_S = 350.0        # Speed for HOME moves in mm/s (ensure this is also sensible, <= MAX_SPEED_MM_S if desired)

MIN_PULSE_WIDTH = 0.000002      # 2 microseconds (minimum pulse high time)
MINIMUM_PULSE_CYCLE_DELAY = 0.0001 # 100µs -> max 10,000 step cycles/sec

# --- System State ---
current_x_mm = 0.0
current_y_mm = 0.0
absolute_mode = True # True for ABS (absolute), False for REL (relative)

# --- Curses UI state ---
last_command_output = []

def setup_gpio():
    """Initializes GPIO pins. Returns True on success, False on failure."""
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
        print("Ensure you are running the script with necessary permissions (e.g., sudo)")
        print("and that the GPIO pins are not already in use.")
        return False

def cleanup_gpio():
    """Cleans up GPIO resources."""
    if pul_device_m1: pul_device_m1.close()
    if dir_device_m1: dir_device_m1.close()
    if pul_device_m2: pul_device_m2.close()
    if dir_device_m2: dir_device_m2.close()
    print("GPIO cleaned up.")

def move_motors_coordinated(steps_m1_target, steps_m2_target, pulse_cycle_delay_for_move):
    """
    Coordinates the movement of both motors for CoreXY motion with a specific pulse delay.
    Returns a list of messages if an error occurs, otherwise an empty list.
    """
    messages = []
    if not all([pul_device_m1, dir_device_m1, pul_device_m2, dir_device_m2]):
        messages.append("Error: GPIO devices not initialized.")
        return messages

    if steps_m1_target > 0: dir_device_m1.off() # Assuming off is one direction
    else: dir_device_m1.on()                  # And on is the other

    if steps_m2_target > 0: dir_device_m2.off() # Assuming off is one direction
    else: dir_device_m2.on()                  # And on is the other
    
    time.sleep(0.001) # Allow direction pins to settle

    abs_steps_m1 = abs(steps_m1_target)
    abs_steps_m2 = abs(steps_m2_target)
    
    total_iterations = max(abs_steps_m1, abs_steps_m2)

    for i in range(total_iterations):
        perform_pulse_m1 = (i < abs_steps_m1)
        perform_pulse_m2 = (i < abs_steps_m2)

        if perform_pulse_m1: pul_device_m1.on()
        if perform_pulse_m2: pul_device_m2.on()
        
        time.sleep(MIN_PULSE_WIDTH)

        if perform_pulse_m1: pul_device_m1.off()
        if perform_pulse_m2: pul_device_m2.off()
        
        inter_pulse_delay = pulse_cycle_delay_for_move - MIN_PULSE_WIDTH
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

    # CoreXY kinematics:
    # MotorA_steps = -X_steps + Y_steps
    # MotorB_steps = -X_steps - Y_steps
    # BUT, my previous interpretation for CoreXY might have X inverted or a motor direction inverted.
    # The common CoreXY setup has:
    # M1 (or A) = X + Y
    # M2 (or B) = X - Y
    # Let's re-evaluate the signs based on what "delta_x_mm" and "delta_y_mm" mean
    # If positive delta_x_mm is move right, positive delta_y_mm is move "away" (further in Y)

    # If M1 positive moves +X and +Y cartesian
    # If M2 positive moves +X and -Y cartesian

    # To move +X (1,0): M1 = 1, M2 = 1  => (M1+M2)/2 = X
    # To move +Y (0,1): M1 = 1, M2 = -1 => (M1-M2)/2 = Y
    
    # So, X_cartesian = (M1+M2)/2 and Y_cartesian = (M1-M2)/2
    # We want M1 and M2 from X_cartesian, Y_cartesian
    # M1 = X_cartesian + Y_cartesian
    # M2 = X_cartesian - Y_cartesian

    # The code currently has:
    # delta_x_steps_cartesian = round(-delta_x_mm * MICROSTEPS_PER_MM) <--- Note the negative
    # delta_y_steps_cartesian = round(delta_y_mm * MICROSTEPS_PER_MM)
    # steps_m1 = delta_x_steps_cartesian + delta_y_steps_cartesian
    # steps_m2 = delta_x_steps_cartesian - delta_y_steps_cartesian
    # This implies that a positive delta_x_mm results in negative delta_x_steps_cartesian.
    # This might be intentional to map to a specific motor wiring or observed behavior.
    # Let's stick to the existing logic for now, as changing it might break current user's setup.
    
    delta_x_steps_cartesian = round(-delta_x_mm * MICROSTEPS_PER_MM)
    delta_y_steps_cartesian = round(delta_y_mm * MICROSTEPS_PER_MM)

    steps_m1 = delta_x_steps_cartesian + delta_y_steps_cartesian
    steps_m2 = delta_x_steps_cartesian - delta_y_steps_cartesian
    
    if steps_m1 == 0 and steps_m2 == 0:
        pass # No motor movement needed
    else:
        motor_messages = move_motors_coordinated(int(steps_m1), int(steps_m2), pulse_cycle_delay_for_move)

    current_x_mm += delta_x_mm
    current_y_mm += delta_y_mm
    
    current_x_mm = round(current_x_mm, 3) 
    current_y_mm = round(current_y_mm, 3)
    return motor_messages

def parse_command_and_execute(line):
    """
    Parses a custom command line and executes it.
    Returns a tuple: (list_of_output_messages, continue_running_boolean).
    """
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
    elif instruction == "HOME":
        output_messages.append("  Homing (HOME):")
        target_x_abs = 0.0
        target_y_abs = 0.0
        
        dx_home = 0.0 
        dy_home = 0.0
        if absolute_mode:
            dx_home = target_x_abs - current_x_mm
            dy_home = target_y_abs - current_y_mm
        
        # Effective homing speed should also respect MAX_SPEED_MM_S if it were variable
        # For now, HOMING_SPEED_MM_S is a constant, ensure it's <= MAX_SPEED_MM_S
        actual_homing_speed = min(HOMING_SPEED_MM_S, MAX_SPEED_MM_S)

        home_path_length_mm = math.sqrt(dx_home**2 + dy_home**2)

        if home_path_length_mm > (MM_PER_MICROSTEP / 2.0) and absolute_mode:
            output_messages.append(f"    Moving to 0,0 from {current_x_mm:.2f}, {current_y_mm:.2f} at {actual_homing_speed:.2f} mm/s")
            time_for_home_move_s = home_path_length_mm / actual_homing_speed
            
            delta_x_steps_cartesian_home = round(-dx_home * MICROSTEPS_PER_MM) # Using consistent cartesian step calculation
            delta_y_steps_cartesian_home = round(dy_home * MICROSTEPS_PER_MM)
            steps_m1_home = delta_x_steps_cartesian_home + delta_y_steps_cartesian_home
            steps_m2_home = delta_x_steps_cartesian_home - delta_y_steps_cartesian_home
            num_iterations_home = max(abs(int(steps_m1_home)), abs(int(steps_m2_home)))

            if num_iterations_home > 0:
                pulse_delay_for_home_move = time_for_home_move_s / num_iterations_home
                actual_pulse_delay_for_home_move = max(MINIMUM_PULSE_CYCLE_DELAY, pulse_delay_for_home_move)
                motor_msgs = move_corexy(dx_home, dy_home, actual_pulse_delay_for_home_move)
                output_messages.extend(motor_msgs)
            else: # Should not happen if path_length is significant, but good to cover
                current_x_mm = target_x_abs
                current_y_mm = target_y_abs
        else: # Already at home or in relative mode
            current_x_mm = target_x_abs
            current_y_mm = target_y_abs
            if absolute_mode:
                 output_messages.append("    Already at (or very close to) 0,0.")
            else: # In REL mode, HOME just resets coordinates without physical move
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
                if speed_val_mm_s_req > MAX_SPEED_MM_S: # <<< MODIFIED >>> Enforce max speed
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
            
    elif instruction == "MOVE" :
        target_x_cmd = None
        target_y_cmd = None
        s_value_this_cmd_req = None # Requested speed for this move
        
        for part in parts[1:]:
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

        current_move_speed_mm_s = TARGET_SPEED_MM_S # Start with global (already capped)
        if s_value_this_cmd_req is not None:
            if s_value_this_cmd_req > 0:
                if s_value_this_cmd_req > MAX_SPEED_MM_S: # <<< MODIFIED >>> Enforce max speed for MOVE S
                    current_move_speed_mm_s = MAX_SPEED_MM_S
                    output_messages.append(f"  Requested MOVE speed {s_value_this_cmd_req:.2f} mm/s exceeds limit of {MAX_SPEED_MM_S:.2f} mm/s.")
                    output_messages.append(f"  MOVE speed clamped to MAX: {current_move_speed_mm_s:.2f} mm/s.")
                else:
                    current_move_speed_mm_s = s_value_this_cmd_req
            else: # s_value_this_cmd_req <= 0
                output_messages.append(f"  Speed S in MOVE must be positive. Using global {TARGET_SPEED_MM_S:.2f} mm/s (max {MAX_SPEED_MM_S:.2f} mm/s).")
        
        # Ensure the final current_move_speed_mm_s is not above MAX_SPEED_MM_S
        # This is mostly redundant if TARGET_SPEED_MM_S is already capped and s_value_this_cmd_req is capped,
        # but acts as a final safeguard.
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
        if XLIM_MM != float('inf'): # Only show clamping if limits are ON
            if abs(actual_target_x_mm - final_target_x_mm) > 1e-9 or abs(actual_target_y_mm - final_target_y_mm) > 1e-9 : # check for float differences
                 clamped_output_needed = True
        elif final_target_x_mm < 0 or final_target_y_mm < 0 : # If limits are INF, only clamp if target < 0
             clamped_output_needed = True


        if clamped_output_needed:
            output_messages.append(f"  Warning: Target ({final_target_x_mm:.2f}, {final_target_y_mm:.2f}) is out of effective bounds.")
            output_messages.append(f"           Will be clamped to ({actual_target_x_mm:.2f}, {actual_target_y_mm:.2f}).")

        delta_x_to_move = actual_target_x_mm - current_x_mm
        delta_y_to_move = actual_target_y_mm - current_y_mm
        
        path_length_mm = math.sqrt(delta_x_to_move**2 + delta_y_to_move**2)

        if path_length_mm < (MM_PER_MICROSTEP / 2.0):
            output_messages.append(f"  Move too small or already at target. Current: ({current_x_mm:.3f}, {current_y_mm:.3f}), Target: ({actual_target_x_mm:.3f}, {actual_target_y_mm:.3f})")
            current_x_mm = actual_target_x_mm # Ensure position is updated even if no move
            current_y_mm = actual_target_y_mm
            current_x_mm = round(current_x_mm,3) 
            current_y_mm = round(current_y_mm,3)
        else:
            delta_x_steps_cartesian = round(-delta_x_to_move * MICROSTEPS_PER_MM) # Using consistent cartesian step calculation
            delta_y_steps_cartesian = round(delta_y_to_move * MICROSTEPS_PER_MM)
            steps_m1 = delta_x_steps_cartesian + delta_y_steps_cartesian
            steps_m2 = delta_x_steps_cartesian - delta_y_steps_cartesian
            num_iterations = max(abs(int(steps_m1)), abs(int(steps_m2)))

            if num_iterations == 0: # Should be caught by path_length_mm check, but good for safety
                output_messages.append("  No motor steps required for this move (delta too small).")
                current_x_mm = actual_target_x_mm # Update position
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
                    if (num_iterations * actual_pulse_delay_for_this_move) > 1e-9: # Avoid division by zero if delay is ~0
                        effective_speed_mm_s = path_length_mm / (num_iterations * actual_pulse_delay_for_this_move)
                    else: # effectively infinite speed if delay is zero, or no iterations (already handled)
                        effective_speed_mm_s = current_move_speed_mm_s # or some other indicator

                    output_messages.append(f"  Moving by dx={delta_x_to_move:.3f} mm, dy={delta_y_to_move:.3f} mm")
                    output_messages.append(f"  To X={actual_target_x_mm:.3f}, Y={actual_target_y_mm:.3f}")
                    output_messages.append(f"  Target speed: {current_move_speed_mm_s:.2f} mm/s. Effective speed: {effective_speed_mm_s:.2f} mm/s.")
                    output_messages.append(f"  Pulse cycle delay: {actual_pulse_delay_for_this_move*1000:.4f} ms.")
                    
                    motor_msgs = move_corexy(delta_x_to_move, delta_y_to_move, actual_pulse_delay_for_this_move)
                    output_messages.extend(motor_msgs)
        
        output_messages.append(f"  New position: X={current_x_mm:.3f} mm, Y={current_y_mm:.3f} mm")

    elif instruction == "POS":
        x_lim_display = f"{XLIM_MM:.0f}" if XLIM_MM != float('inf') else "INF (Disabled)"
        y_lim_display = f"{YLIM_MM:.0f}" if YLIM_MM != float('inf') else "INF (Disabled)"
        output_messages.append(f"  Current Position: X={current_x_mm:.3f} mm, Y={current_y_mm:.3f} mm")
        output_messages.append(f"  Target Speed: {TARGET_SPEED_MM_S:.2f} mm/s (Max settable: {MAX_SPEED_MM_S:.2f} mm/s)") # <<< MODIFIED >>>
        output_messages.append(f"  Mode: {'Absolute' if absolute_mode else 'Relative'}")
        output_messages.append(f"  Effective Limits: X=[0, {x_lim_display}], Y=[0, {y_lim_display}]")


    elif instruction in ["EXIT", "QUIT"]:
        output_messages.append("  Exiting program.")
        return output_messages, False 
        
    else:
        if instruction: # Only show error if there was an instruction
            output_messages.append(f"  Unknown or unsupported command: {instruction}")
    
    return output_messages, True 

def draw_ui(stdscr, header_lines, status_lines, command_output_lines, input_prompt):
    """Draws the entire UI in the curses window."""
    stdscr.clear()
    max_y, max_x = stdscr.getmaxyx()
    current_y = 0

    # Header
    for i, line in enumerate(header_lines):
        if current_y < max_y:
            stdscr.addstr(current_y, 0, line[:max_x-1]) # Ensure text fits
            current_y += 1
    if current_y < max_y: # Separator after header
        stdscr.addstr(current_y, 0, "-" * (max_x -1) )
        current_y +=1

    # Status
    for i, line in enumerate(status_lines):
        if current_y < max_y:
            stdscr.addstr(current_y, 0, line[:max_x-1])
            current_y += 1
    if current_y < max_y: # Separator after status
        stdscr.addstr(current_y, 0, "-" * (max_x-1) )
        current_y +=1
    
    # Command Output (scrollable part)
    output_start_y = current_y
    # Reserve 1 line for bottom separator, 1 for input prompt
    available_lines_for_output = max_y - output_start_y - 2 
    
    start_index = 0
    if len(command_output_lines) > available_lines_for_output and available_lines_for_output > 0 :
        start_index = len(command_output_lines) - available_lines_for_output
    
    for i, line in enumerate(command_output_lines[start_index:]):
        if output_start_y + i < max_y - 2: # Ensure we don't write over the input prompt area
            stdscr.addstr(output_start_y + i, 0, line[:max_x-1]) 
    
    current_y = max_y - 2 # Position for separator above input
    if current_y > 0 : # Check if there's space for the separator
         stdscr.addstr(current_y, 0, "-" * (max_x-1) )
    
    # Input Prompt
    input_line_y = max_y - 1
    if input_line_y > 0: # Check if there's space for the input line
        stdscr.addstr(input_line_y, 0, input_prompt)
        stdscr.move(input_line_y, len(input_prompt)) # Move cursor to end of prompt

    stdscr.refresh()

def _curses_main_loop(stdscr):
    """Main loop for the command-line interface, managed by curses."""
    global TARGET_SPEED_MM_S, current_x_mm, current_y_mm, absolute_mode, last_command_output, XLIM_MM, YLIM_MM

    curses.curs_set(1) # Show cursor
    stdscr.nodelay(False) # Blocking getstr

    running = True
    while running:
        x_limit_display_str = f"{XLIM_MM:.0f}" if XLIM_MM != float('inf') else "INF (Disabled)"
        y_limit_display_str = f"{YLIM_MM:.0f}" if YLIM_MM != float('inf') else "INF (Disabled)"

        header = [
            "--- CoreXY CLI Controller (Custom Commands) ---",
            f"Max Settable Speed: {MAX_SPEED_MM_S:.2f} mm/s",
            f"Optimal Speed : 350 mm/s",
            f"Effective Limits: X=[0, {x_limit_display_str}], Y=[0, {y_limit_display_str}] mm",
            f"Resolution: {MM_PER_MICROSTEP} mm/microstep ({MICROSTEPS_PER_MM} microsteps/mm)",
            f"Motor Native Steps/Rev: {MOTOR_NATIVE_STEPS_PER_REV}, Driver Microstepping: 1/{DRIVER_MICROSTEP_DIVISOR}",
            f"Min Pulse Cycle Delay: {MINIMUM_PULSE_CYCLE_DELAY*1000:.3f} ms",
            "Available commands:",
            "  MOVE X<v> Y<v> [S<v>]",
            "  ABS, REL",
            "  HOME",
            "  S<v> or S <v>", 
            "  LIMITS [ON|OFF]", 
            "  POS",
            "  EXIT/QUIT"
        ]
        status = [
            f"Current Position: X={current_x_mm:.3f} mm, Y={current_y_mm:.3f} mm",
            f"Target Speed: {TARGET_SPEED_MM_S:.2f} mm/s, Mode: {'ABS' if absolute_mode else 'REL'}"
        ]
        
        input_prompt_text = "CoreXY > "
        draw_ui(stdscr, header, status, last_command_output, input_prompt_text)
        
        input_line_y = stdscr.getmaxyx()[0] - 1 # Y-coordinate for input line
        
        try:
            # Ensure there's actually a line to get input from
            if input_line_y > 0 and input_line_y > (len(header) + len(status) +2) : # check if space exists
                 stdscr.move(input_line_y, len(input_prompt_text)) # Move cursor
                 curses.echo() # Echo characters typed by the user
                 cmd_line_bytes = stdscr.getstr(input_line_y, len(input_prompt_text), 60) # Get input
                 curses.noecho() # Disable echo
                 cmd_line = cmd_line_bytes.decode('utf-8').strip()
            else: # Terminal too small for input line
                cmd_line = "" # Cannot get input, treat as empty command
                last_command_output = ["Terminal too small. Resize or EXIT."]
                # Forcing an exit might be too aggressive, let user type exit if they can see previous output
                # running = False # Or provide a way to exit if this state persists


            if not cmd_line: # If user just presses Enter or line is empty
                if not last_command_output or last_command_output[-1] != "Terminal too small. Resize or EXIT.":
                    last_command_output = [] # Clear previous output if it wasn't the resize warning
                continue

            last_command_output, running = parse_command_and_execute(cmd_line)

        except curses.error as e:
            curses.noecho() # Ensure echo is off on error
            last_command_output = [f"Curses error: {e}", "Try resizing terminal or type 'exit'."]
        except KeyboardInterrupt:
            curses.noecho() 
            last_command_output = ["Keyboard interrupt detected. Exiting..."]
            running = False
        except Exception as e: # Catch any other unexpected error
            curses.noecho() 
            last_command_output = [
                f"An unexpected error occurred: {e}",
                "Check command syntax or internal logic."
            ]
            # For debugging:
            # import traceback
            # last_command_output.append("Traceback:")
            # last_command_output.extend(traceback.format_exc().splitlines())
            
    # Final screen paint before exiting the curses wrapper
    status = [ # Re-fetch status for final paint
        f"Current Position: X={current_x_mm:.3f} mm, Y={current_y_mm:.3f} mm",
        f"Target Speed: {TARGET_SPEED_MM_S:.2f} mm/s, Mode: {'ABS' if absolute_mode else 'REL'}"
    ]
    if not last_command_output or "Exiting program." not in last_command_output[-1]:
        last_command_output.append("Exiting program.")
    
    # Rebuild header one last time for final screen paint before exit
    x_limit_display_str = f"{XLIM_MM:.0f}" if XLIM_MM != float('inf') else "INF (Disabled)"
    y_limit_display_str = f"{YLIM_MM:.0f}" if YLIM_MM != float('inf') else "INF (Disabled)"
    header = [
        "--- CoreXY CLI Controller (Custom Commands) ---",
        f"Max Settable Speed: {MAX_SPEED_MM_S:.2f} mm/s",
        f"Effective Limits: X=[0, {x_limit_display_str}], Y=[0, {y_limit_display_str}] mm",
        f"Resolution: {MM_PER_MICROSTEP} mm/microstep ({MICROSTEPS_PER_MM} microsteps/mm)",
        f"Motor Native Steps/Rev: {MOTOR_NATIVE_STEPS_PER_REV}, Driver Microstepping: 1/{DRIVER_MICROSTEP_DIVISOR}",
        f"Min Pulse Cycle Delay: {MINIMUM_PULSE_CYCLE_DELAY*1000:.3f} ms",
        "Available commands: ... (EXIT/QUIT to close)" # Simplified for final message
    ]
    # Simplified call to draw_ui for the very last paint before curses ends.
    # Might not be fully visible if curses cleanup is too fast.
    try:
        draw_ui(stdscr, header, status, last_command_output, "Exited. ")
        stdscr.refresh()
        time.sleep(0.5) # Brief pause to see final message
    except curses.error:
        pass # Curses might already be in a bad state if error occurred

if __name__ == "__main__":
    if setup_gpio():
        try:
            curses.wrapper(_curses_main_loop)
        except Exception as e: # Catch errors that might occur outside the wrapper's try/except
            print(f"A critical error occurred outside curses main loop: {e}")
            import traceback
            traceback.print_exc()
        finally:
            # curses.endwin() # curses.wrapper handles this
            print("Cleaning up GPIO before exit...") # Will print after curses has ended
            cleanup_gpio() 
            print("Program terminated.") 
    else:
        print("GPIO setup failed. Program will not start controller interface.")
        print("Program terminated.")