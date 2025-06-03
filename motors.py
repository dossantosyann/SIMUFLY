import gpiozero
import time
import math
import re
import curses

# GPIO Pin Configuration (BCM numbering)
PUL_PIN_M1 = 6
DIR_PIN_M1 = 5

PUL_PIN_M2 = 27
DIR_PIN_M2 = 17

# Endstop Pin Configuration (BCM numbering)
ENDSTOP_X_PINS = [13, 26]
ENDSTOP_Y_PINS = [12, 16]

# GPIOZero Device Objects
pul_device_m1 = None
dir_device_m1 = None
pul_device_m2 = None
dir_device_m2 = None

# Endstop Device Objects
endstop_x = None
endstop_y = None

# Motor & System Parameters
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

# Speed and Timing Parameters
DEFAULT_SPEED_MM_S = 200.0
TARGET_SPEED_MM_S = DEFAULT_SPEED_MM_S
HOMING_SPEED_MM_S = 200.0
CALIBRATION_SPEED_MM_S = 100.0

MIN_PULSE_WIDTH = 0.000002
MINIMUM_PULSE_CYCLE_DELAY = 0.0001

# System State
current_x_mm = 0.0
current_y_mm = 0.0
absolute_mode = True

# Curses UI state
last_command_output = []
# Global stdscr for UI updates outside main loop
_stdscr_global = None 


def setup_gpio():
    """Initializes GPIO pins. Returns True on success, False on failure."""
    global pul_device_m1, dir_device_m1, pul_device_m2, dir_device_m2
    global endstop_x, endstop_y
    try:
        pul_device_m1 = gpiozero.OutputDevice(PUL_PIN_M1, active_high=True, initial_value=False)
        dir_device_m1 = gpiozero.OutputDevice(DIR_PIN_M1, active_high=True, initial_value=False)
        pul_device_m2 = gpiozero.OutputDevice(PUL_PIN_M2, active_high=True, initial_value=False)
        dir_device_m2 = gpiozero.OutputDevice(DIR_PIN_M2, active_high=True, initial_value=False)
        
        endstop_x = gpiozero.InputDevice(ENDSTOP_X_PINS[0], pull_up=True)
        endstop_y = gpiozero.InputDevice(ENDSTOP_Y_PINS[0], pull_up=True)

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
    if endstop_x: endstop_x.close()
    if endstop_y: endstop_y.close()
    print("GPIO cleaned up.")

def move_motors_coordinated(steps_m1_target, steps_m2_target, pulse_cycle_delay_for_move, check_endstops=False):
    """
    Coordinates the movement of both motors for CoreXY motion with a specific pulse delay.
    Returns a list of messages if an error occurs, otherwise an empty list.
    Also returns a tuple (x_stopped, y_stopped) indicating which axis hit an endstop.
    """
    messages = []
    x_stopped = False
    y_stopped = False

    if not all([pul_device_m1, dir_device_m1, pul_device_m2, dir_device_m2]):
        messages.append("Error: GPIO devices not initialized.")
        return messages, False, False

    if steps_m1_target > 0: dir_device_m1.off()
    else: dir_device_m1.on()

    if steps_m2_target > 0: dir_device_m2.off()
    else: dir_device_m2.on()
    
    time.sleep(0.001)

    abs_steps_m1 = abs(steps_m1_target)
    abs_steps_m2 = abs(steps_m2_target)
    
    total_iterations = max(abs_steps_m1, abs_steps_m2)

    for i in range(total_iterations):
        perform_pulse_m1 = (i < abs_steps_m1) and not x_stopped
        perform_pulse_m2 = (i < abs_steps_m2) and not y_stopped

        if perform_pulse_m1: pul_device_m1.on()
        if perform_pulse_m2: pul_device_m2.on()
        
        time.sleep(MIN_PULSE_WIDTH)

        if perform_pulse_m1: pul_device_m1.off()
        if perform_pulse_m2: pul_device_m2.off()
        
        inter_pulse_delay = pulse_cycle_delay_for_move - MIN_PULSE_WIDTH
        if inter_pulse_delay > 0:
            time.sleep(inter_pulse_delay)
            
    return messages, x_stopped, y_stopped

def move_corexy(delta_x_mm, delta_y_mm, pulse_cycle_delay_for_move, check_endstops=False):
    """
    Calculates steps and initiates CoreXY movement with a given pulse cycle delay.
    Returns a list of messages from motor coordination and endstop status (x_stopped, y_stopped).
    """
    global current_x_mm, current_y_mm
    motor_messages = []
    x_stopped_during_move = False
    y_stopped_during_move = False

    delta_x_steps_cartesian = round(-delta_x_mm * MICROSTEPS_PER_MM)
    delta_y_steps_cartesian = round(delta_y_mm * MICROSTEPS_PER_MM)

    steps_m1 = delta_x_steps_cartesian + delta_y_steps_cartesian
    steps_m2 = delta_x_steps_cartesian - delta_y_steps_cartesian
    
    if steps_m1 == 0 and steps_m2 == 0:
        pass
    else:
        motor_messages, x_stopped_during_move, y_stopped_during_move = move_motors_coordinated(
            int(steps_m1), int(steps_m2), pulse_cycle_delay_for_move, check_endstops
        )

    if not x_stopped_during_move:
        current_x_mm += delta_x_mm
    if not y_stopped_during_move:
        current_y_mm += delta_y_mm
    
    current_x_mm = round(current_x_mm, 3) 
    current_y_mm = round(current_y_mm, 3)
    return motor_messages, x_stopped_during_move, y_stopped_during_move


def calibrate_homing():
    """
    Performs calibration by moving both axes negatively until their respective endstops are hit.
    Sets the current position of the hit axis to 0.
    """
    global current_x_mm, current_y_mm, last_command_output, _stdscr_global
    output_messages = []
    
    output_messages.append("  Starting calibration (CALIBRATE)...")
    output_messages.append(f"  Moving at {CALIBRATION_SPEED_MM_S:.2f} mm/s towards negative X and Y.")
    last_command_output = output_messages[:] # Copy for display
    _refresh_curses_ui() # Refresh UI

    # Calculate pulse delay for calibration speed (approx)
    temp_dx = -1.0
    temp_dy = -1.0
    temp_path_length = math.sqrt(temp_dx**2 + temp_dy**2)
    
    temp_dx_steps_cartesian = round(-temp_dx * MICROSTEPS_PER_MM)
    temp_dy_steps_cartesian = round(temp_dy * MICROSTEPS_PER_MM)
    
    temp_steps_m1 = temp_dx_steps_cartesian + temp_dy_steps_cartesian
    temp_steps_m2 = temp_dx_steps_cartesian - temp_dy_steps_cartesian
    temp_num_iterations = max(abs(int(temp_steps_m1)), abs(int(temp_steps_m2)))

    if temp_num_iterations == 0:
        calculated_pulse_cycle_delay = MINIMUM_PULSE_CYCLE_DELAY
    else:
        calculated_pulse_cycle_delay = (temp_path_length / CALIBRATION_SPEED_MM_S) / temp_num_iterations
        calculated_pulse_cycle_delay = max(MINIMUM_PULSE_CYCLE_DELAY, calculated_pulse_cycle_delay)

    output_messages.append(f"  Calculated pulse delay for calibration: {calculated_pulse_cycle_delay*1000:.4f} ms.")
    last_command_output = output_messages[:]
    _refresh_curses_ui()
                
    MAX_HOMING_ITERATIONS = int(max(DEFAULT_XLIM_MM, DEFAULT_YLIM_MM) / MM_PER_MICROSTEP * 2)
    
    output_messages.append("  Moving individual motor microsteps towards endstops...")
    last_command_output = output_messages[:]
    _refresh_curses_ui()

    x_homed_finally = False
    y_homed_finally = False

    # Store initial position for delta calculation
    start_x_calibration_mm = current_x_mm
    start_y_calibration_mm = current_y_mm

    # Homing X-axis
    output_messages.append("  Homing X-axis...")
    dir_device_m1.off() # Adjust for your wiring for -X movement
    dir_device_m2.off() # Adjust for your wiring for -X movement
    time.sleep(0.001)

    for i in range(MAX_HOMING_ITERATIONS):
        if endstop_x.is_active:
            delta_x_traveled = start_x_calibration_mm - current_x_mm
            output_messages.append(f"    X endstop hit! Traveled: {delta_x_traveled:.3f} mm.")
            current_x_mm = 0.0
            x_homed_finally = True
            break

        pul_device_m1.on()
        pul_device_m2.on()
        time.sleep(MIN_PULSE_WIDTH)
        pul_device_m1.off()
        pul_device_m2.off()
        time.sleep(calculated_pulse_cycle_delay - MIN_PULSE_WIDTH)
        
        current_x_mm -= MM_PER_MICROSTEP # Approximation for X- movement
        current_x_mm = round(current_x_mm, 3)
        
        if i % 50 == 0: # Update UI every 50 steps
            delta_x_traveled = start_x_calibration_mm - current_x_mm
            output_messages[-1] = f"    X Homing: {current_x_mm:.3f} mm (Delta: {delta_x_traveled:.3f} mm)..."
            last_command_output = output_messages[:]
            _refresh_curses_ui()
        
        if current_x_mm < -(DEFAULT_XLIM_MM + 100):
            output_messages.append("    X-axis homing timeout: exceeded expected travel range.")
            break

    # Re-copy output messages for Y-axis homing phase start
    last_command_output = output_messages[:]
    _refresh_curses_ui()

    # Homing Y-axis
    output_messages.append("  Homing Y-axis...")
    dir_device_m1.on()  # Adjust for your wiring for -Y movement
    dir_device_m2.off() # Adjust for your wiring for -Y movement
    time.sleep(0.001)

    for i in range(MAX_HOMING_ITERATIONS):
        if endstop_y.is_active:
            delta_y_traveled = start_y_calibration_mm - current_y_mm
            output_messages.append(f"    Y endstop hit! Traveled: {delta_y_traveled:.3f} mm.")
            current_y_mm = 0.0
            y_homed_finally = True
            break

        pul_device_m1.on()
        pul_device_m2.on()
        time.sleep(MIN_PULSE_WIDTH)
        pul_device_m1.off()
        pul_device_m2.off()
        time.sleep(calculated_pulse_cycle_delay - MIN_PULSE_WIDTH)

        current_y_mm -= MM_PER_MICROSTEP # Approximation for Y- movement
        current_y_mm = round(current_y_mm, 3)

        if i % 50 == 0: # Update UI every 50 steps
            delta_y_traveled = start_y_calibration_mm - current_y_mm
            output_messages[-1] = f"    Y Homing: {current_y_mm:.3f} mm (Delta: {delta_y_traveled:.3f} mm)..."
            last_command_output = output_messages[:]
            _refresh_curses_ui()
            
        if current_y_mm < -(DEFAULT_YLIM_MM + 100):
            output_messages.append("    Y-axis homing timeout: exceeded expected travel range.")
            break
            
    if x_homed_finally and y_homed_finally:
        output_messages.append("  Calibration complete: X and Y homed to (0.000, 0.000) mm.")
    else:
        output_messages.append("  Calibration incomplete: One or both axes failed to home.")
        output_messages.append(f"    X homed: {x_homed_finally}, Y homed: {y_homed_finally}")

    output_messages.append(f"  Final Position: X={current_x_mm:.3f} mm, Y={current_y_mm:.3f} mm")
    last_command_output = output_messages[:] # Final update
    _refresh_curses_ui()

    return output_messages


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
        
        home_path_length_mm = math.sqrt(dx_home**2 + dy_home**2)

        if home_path_length_mm > (MM_PER_MICROSTEP / 2.0) and absolute_mode:
            output_messages.append(f"    Moving to 0,0 from {current_x_mm:.2f}, {current_y_mm:.2f} at {HOMING_SPEED_MM_S:.2f} mm/s")
            time_for_home_move_s = home_path_length_mm / HOMING_SPEED_MM_S
            
            delta_x_steps_cartesian_home = round(-dx_home * MICROSTEPS_PER_MM)
            delta_y_steps_cartesian_home = round(dy_home * MICROSTEPS_PER_MM)
            steps_m1_home = delta_x_steps_cartesian_home + delta_y_steps_cartesian_home
            steps_m2_home = delta_x_steps_cartesian_home - delta_y_steps_cartesian_home
            num_iterations_home = max(abs(int(steps_m1_home)), abs(int(steps_m2_home)))

            if num_iterations_home > 0:
                pulse_delay_for_home_move = time_for_home_move_s / num_iterations_home
                actual_pulse_delay_for_home_move = max(MINIMUM_PULSE_CYCLE_DELAY, pulse_delay_for_home_move)
                motor_msgs, _, _ = move_corexy(dx_home, dy_home, actual_pulse_delay_for_home_move)
                output_messages.extend(motor_msgs)
            else:
                current_x_mm = target_x_abs
                current_y_mm = target_y_abs
        else:
            current_x_mm = target_x_abs
            current_y_mm = target_y_abs
            if absolute_mode:
                 output_messages.append("    Already at (or very close to) 0,0.")
            else:
                output_messages.append("    Logical position reset to 0,0. No physical movement in REL for this HOME.")
        output_messages.append(f"  New position: X={current_x_mm:.3f} mm, Y={current_y_mm:.3f} mm")
    
    elif instruction == "CALIBRATE":
        output_messages.extend(calibrate_homing())

    elif instruction.startswith("S") and instruction != "MOVE":
        try:
            if len(instruction) > 1 and instruction[1:].replace('.', '', 1).isdigit():
                 speed_val_mm_s = float(instruction[1:])
            elif len(parts) > 1 and parts[1].replace('.', '', 1).isdigit():
                 speed_val_mm_s = float(parts[1])
            else:
                output_messages.append(f"  Invalid S command format. Use S<value> or S <value>.")
                return output_messages, True

            if speed_val_mm_s > 0:
                TARGET_SPEED_MM_S = speed_val_mm_s
                output_messages.append(f"  Global target speed set to: {TARGET_SPEED_MM_S:.2f} mm/s")
            else:
                output_messages.append("  Speed S must be positive.")
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
        s_value_this_cmd = None
        
        for part in parts[1:]:
            if part.startswith('X'):
                try: target_x_cmd = float(part[1:])
                except ValueError: output_messages.append(f"  Invalid X value: {part}"); return output_messages, True
            elif part.startswith('Y'):
                try: target_y_cmd = float(part[1:])
                except ValueError: output_messages.append(f"  Invalid Y value: {part}"); return output_messages, True
            elif part.startswith('S'):
                 try: 
                     s_value_this_cmd = float(part[1:])
                 except ValueError: output_messages.append(f"  Invalid S value in MOVE: {part}")

        if target_x_cmd is None and target_y_cmd is None:
            output_messages.append("  No X or Y coordinate specified for MOVE.")
            return output_messages, True

        current_move_speed_mm_s = TARGET_SPEED_MM_S
        if s_value_this_cmd is not None:
            if s_value_this_cmd > 0:
                current_move_speed_mm_s = s_value_this_cmd
            else:
                output_messages.append(f"  Speed S in MOVE must be positive. Using global {TARGET_SPEED_MM_S:.2f} mm/s.")
        
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

        if XLIM_MM != float('inf') and (actual_target_x_mm != final_target_x_mm or actual_target_y_mm != final_target_y_mm):
            output_messages.append(f"  Warning: Target ({final_target_x_mm:.2f}, {final_target_y_mm:.2f}) is out of enabled bounds.")
            output_messages.append(f"           Will be clamped to ({actual_target_x_mm:.2f}, {actual_target_y_mm:.2f}).")
        elif XLIM_MM == float('inf') and (actual_target_x_mm != final_target_x_mm or actual_target_y_mm != final_target_y_mm):
             output_messages.append(f"  Warning: Target ({final_target_x_mm:.2f}, {final_target_y_mm:.2f}) is out of [0, inf) bounds.")
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
                output_messages.append("  No motor steps required for this move.")
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
                    
                    if (num_iterations * actual_pulse_delay_for_this_move) > 0: 
                        effective_speed_mm_s = path_length_mm / (num_iterations * actual_pulse_delay_for_this_move)
                    else:
                        effective_speed_mm_s = 0 

                    output_messages.append(f"  Moving by dx={delta_x_to_move:.3f} mm, dy={delta_y_to_move:.3f} mm")
                    output_messages.append(f"  To X={actual_target_x_mm:.3f}, Y={actual_target_y_mm:.3f}")
                    output_messages.append(f"  Target speed: {current_move_speed_mm_s:.2f} mm/s. Effective speed: {effective_speed_mm_s:.2f} mm/s.")
                    output_messages.append(f"  Pulse cycle delay: {actual_pulse_delay_for_this_move*1000:.4f} ms.")
                    
                    motor_msgs, _, _ = move_corexy(delta_x_to_move, delta_y_to_move, actual_pulse_delay_for_this_move)
                    output_messages.extend(motor_msgs)
        
        output_messages.append(f"  New position: X={current_x_mm:.3f} mm, Y={current_y_mm:.3f} mm")

    elif instruction == "POS":
        x_lim_display = f"{XLIM_MM:.0f}" if XLIM_MM != float('inf') else "INF (Disabled)"
        y_lim_display = f"{YLIM_MM:.0f}" if YLIM_MM != float('inf') else "INF (Disabled)"
        output_messages.append(f"  Current Position: X={current_x_mm:.3f} mm, Y={current_y_mm:.3f} mm")
        output_messages.append(f"  Target Speed: {TARGET_SPEED_MM_S:.2f} mm/s")
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
    if len(command_output_lines) > available_lines_for_output:
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

def _refresh_curses_ui():
    """Helper function to refresh the UI immediately."""
    global _stdscr_global
    if _stdscr_global:
        x_limit_display_str = f"{XLIM_MM:.0f}" if XLIM_MM != float('inf') else "INF (Disabled)"
        y_limit_display_str = f"{YLIM_MM:.0f}" if YLIM_MM != float('inf') else "INF (Disabled)"

        header = [
            "--- CoreXY CLI Controller (Custom Commands) ---",
            f"Effective Limits: X=[0, {x_limit_display_str}], Y=[0, {y_limit_display_str}] mm",
            f"Resolution: {MM_PER_MICROSTEP} mm/microstep ({MOTOR_NATIVE_STEPS_PER_REV * DRIVER_MICROSTEP_DIVISOR:.0f} microsteps/rev)",            f"Motor Native Steps/Rev: {MOTOR_NATIVE_STEPS_PER_REV}, Driver Microstepping: 1/{DRIVER_MICROSTEP_DIVISOR}",
            f"Min Pulse Cycle Delay: {MINIMUM_PULSE_CYCLE_DELAY*1000:.3f} ms",
            "Available commands:",
            "  MOVE X<v> Y<v> [S<v>]",
            "  ABS, REL",
            "  HOME",
            "  CALIBRATE",
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
        draw_ui(_stdscr_global, header, status, last_command_output, input_prompt_text)


def _curses_main_loop(stdscr):
    """Main loop for the command-line interface, managed by curses."""
    global TARGET_SPEED_MM_S, current_x_mm, current_y_mm, absolute_mode, last_command_output, XLIM_MM, YLIM_MM, _stdscr_global

    _stdscr_global = stdscr # Store stdscr globally for refresh calls
    curses.curs_set(1) 
    stdscr.nodelay(False)

    running = True
    while running:
        _refresh_curses_ui() # Initial draw or redraw after command

        input_line_y = stdscr.getmaxyx()[0] - 1
        
        try:
            if input_line_y > 0: 
                 stdscr.move(input_line_y, len(input_prompt_text))
                 curses.echo() 
                 cmd_line_bytes = stdscr.getstr(input_line_y, len(input_prompt_text), 60) 
                 curses.noecho() 
                 cmd_line = cmd_line_bytes.decode('utf-8').strip()
            else: 
                cmd_line = "" 

            if not cmd_line:
                last_command_output = [] 
                continue

            last_command_output, running = parse_command_and_execute(cmd_line)
            # After command execution, refresh UI to show results
            _refresh_curses_ui()

        except curses.error as e:
            curses.noecho() 
            last_command_output = [f"Curses error: {e}", "Try resizing terminal or type 'exit'."]
            _refresh_curses_ui()
        except KeyboardInterrupt:
            curses.noecho() 
            last_command_output = ["Keyboard interrupt detected. Exiting..."]
            running = False
            _refresh_curses_ui()
        except Exception as e:
            curses.noecho() 
            last_command_output = [
                f"An unexpected error occurred: {e}",
                "Check command syntax or internal logic."
            ]
            import traceback
            traceback.print_exc()
            last_command_output.append("Traceback:")
            last_command_output.extend(traceback.format_exc().splitlines())
            _refresh_curses_ui()
            
    status = [ 
        f"Current Position: X={current_x_mm:.3f} mm, Y={current_y_mm:.3f} mm",
        f"Target Speed: {TARGET_SPEED_MM_S:.2f} mm/s, Mode: {'ABS' if absolute_mode else 'REL'}"
    ]
    if not last_command_output or "Exiting program." not in last_command_output[-1]:
        last_command_output.append("Exiting program.")
    
    x_limit_display_str = f"{XLIM_MM:.0f}" if XLIM_MM != float('inf') else "INF (Disabled)"
    y_limit_display_str = f"{YLIM_MM:.0f}" if YLIM_MM != float('inf') else "INF (Disabled)"
    header = [
        "--- CoreXY CLI Controller (Custom Commands) ---",
        f"Effective Limits: X=[0, {x_limit_display_str}], Y=[0, {y_limit_display_str}] mm",
        f"Resolution: {MM_PER_MICROSTEP} mm/microstep ({MICROSTEPS_PER_MM} microsteps/mm)",
        f"Motor Native Steps/Rev: {MOTOR_NATIVE_STEPS_PER_REV}, Driver Microstepping: 1/{DRIVER_MICROSTEP_DIVISOR}",
        f"Min Pulse Cycle Delay: {MINIMUM_PULSE_CYCLE_DELAY*1000:.3f} ms",
        "Available commands:",
        "  MOVE X<v> Y<v> [S<v>]",
        "  ABS, REL",
        "  HOME",
        "  CALIBRATE",
        "  S<v> or S <v>",
        "  LIMITS [ON|OFF]",
        "  POS",
        "  EXIT/QUIT"
    ]
    draw_ui(stdscr, header, status, last_command_output, input_prompt_text)
    stdscr.refresh()
    time.sleep(1) 

if __name__ == "__main__":
    if setup_gpio():
        try:
            curses.wrapper(_curses_main_loop)
        except Exception as e:
            print(f"A critical error occurred: {e}")
            import traceback
            traceback.print_exc()
        finally:
            print("Cleaning up before exit...")
            cleanup_gpio() 
            print("Program terminated.") 
    else:
        print("GPIO setup failed. Program will not start controller interface.")
        print("Program terminated.")