#!/usr/bin/env python3

import gpiozero
import time
import math
import re # For command parsing

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

MM_PER_MICROSTEP = 0.5
if MM_PER_MICROSTEP == 0:
    raise ValueError("MM_PER_MICROSTEP cannot be zero.")
MICROSTEPS_PER_MM = 1.0 / MM_PER_MICROSTEP

XLIM_MM = 1000.0
YLIM_MM = 1000.0

# --- Speed and Timing Parameters ---
DEFAULT_SPEED_MM_S = 20.0       # Default speed in mm/s
TARGET_SPEED_MM_S = DEFAULT_SPEED_MM_S # Global target speed, set by S command
HOMING_SPEED_MM_S = 10.0        # Speed for HOME moves in mm/s

MIN_PULSE_WIDTH = 0.000002      # 2 microseconds (minimum pulse high time)
# Minimum total time for a pulse cycle (on-wait-off-wait).
MINIMUM_PULSE_CYCLE_DELAY = 0.0001 # 100µs -> max 10,000 step cycles/sec

# --- System State ---
current_x_mm = 0.0
current_y_mm = 0.0
absolute_mode = True # True for ABS (absolute), False for REL (relative)

def setup_gpio():
    """Initializes GPIO pins."""
    global pul_device_m1, dir_device_m1, pul_device_m2, dir_device_m2
    try:
        pul_device_m1 = gpiozero.OutputDevice(PUL_PIN_M1, active_high=True, initial_value=False)
        dir_device_m1 = gpiozero.OutputDevice(DIR_PIN_M1, active_high=True, initial_value=False)
        pul_device_m2 = gpiozero.OutputDevice(PUL_PIN_M2, active_high=True, initial_value=False)
        dir_device_m2 = gpiozero.OutputDevice(DIR_PIN_M2, active_high=True, initial_value=False)
        print("GPIO initialized successfully.")
    except Exception as e:
        print(f"Error during GPIO initialization: {e}")
        print("Ensure you are running the script with necessary permissions (e.g., sudo)")
        print("and that the GPIO pins are not already in use.")
        exit(1)

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
    """
    if not all([pul_device_m1, dir_device_m1, pul_device_m2, dir_device_m2]):
        print("Error: GPIO devices not initialized.")
        return

    if steps_m1_target > 0: dir_device_m1.on()
    else: dir_device_m1.off()

    if steps_m2_target > 0: dir_device_m2.on()
    else: dir_device_m2.off()
    
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
        
        # Delay between step rising edges (controls speed)
        inter_pulse_delay = pulse_cycle_delay_for_move - MIN_PULSE_WIDTH
        if inter_pulse_delay > 0:
            time.sleep(inter_pulse_delay)

def move_corexy(delta_x_mm, delta_y_mm, pulse_cycle_delay_for_move):
    """
    Calculates steps and initiates CoreXY movement with a given pulse cycle delay.
    """
    global current_x_mm, current_y_mm

    delta_x_steps_cartesian = round(delta_x_mm * MICROSTEPS_PER_MM)
    delta_y_steps_cartesian = round(delta_y_mm * MICROSTEPS_PER_MM)

    steps_m1 = delta_x_steps_cartesian + delta_y_steps_cartesian
    steps_m2 = delta_x_steps_cartesian - delta_y_steps_cartesian
    
    # Debug prints if needed, but speed info is now shown in parse_command_and_execute
    # print(f"  Cartesian displacement (steps): dX={delta_x_steps_cartesian}, dY={delta_y_steps_cartesian}")
    # print(f"  Motor steps: M1={steps_m1}, M2={steps_m2}")

    if steps_m1 == 0 and steps_m2 == 0:
        # This case should ideally be caught before calling move_corexy if path_length is tiny
        pass # Fall through to update position based on intended small move
    else:
        move_motors_coordinated(int(steps_m1), int(steps_m2), pulse_cycle_delay_for_move)

    current_x_mm += delta_x_mm
    current_y_mm += delta_y_mm
    
    current_x_mm = round(current_x_mm, 3) 
    current_y_mm = round(current_y_mm, 3)

def parse_command_and_execute(line):
    """Parses a custom command line and executes it."""
    global current_x_mm, current_y_mm, absolute_mode, TARGET_SPEED_MM_S
    
    command = line.strip().upper()
    parts = command.split()
    instruction = parts[0] if parts else ""

    print(f"CMD: {command}")

    if instruction == "ABS":
        absolute_mode = True
        print("  Mode: Absolute Positioning (ABS)")
    elif instruction == "REL":
        absolute_mode = False
        print("  Mode: Relative Positioning (REL)")
    elif instruction == "HOME":
        print("  Homing (HOME):")
        target_x_abs = 0.0
        target_y_abs = 0.0
        
        dx_home = 0
        dy_home = 0
        if absolute_mode:
            dx_home = target_x_abs - current_x_mm
            dy_home = target_y_abs - current_y_mm
        
        home_path_length_mm = math.sqrt(dx_home**2 + dy_home**2)

        if home_path_length_mm > (MM_PER_MICROSTEP / 2.0) and absolute_mode:
            print(f"    Moving to 0,0 from {current_x_mm:.2f}, {current_y_mm:.2f} at {HOMING_SPEED_MM_S:.2f} mm/s")
            time_for_home_move_s = home_path_length_mm / HOMING_SPEED_MM_S
            
            delta_x_steps_cartesian_home = round(dx_home * MICROSTEPS_PER_MM)
            delta_y_steps_cartesian_home = round(dy_home * MICROSTEPS_PER_MM)
            steps_m1_home = delta_x_steps_cartesian_home + delta_y_steps_cartesian_home
            steps_m2_home = delta_x_steps_cartesian_home - delta_y_steps_cartesian_home
            num_iterations_home = max(abs(int(steps_m1_home)), abs(int(steps_m2_home)))

            if num_iterations_home > 0:
                pulse_delay_for_home_move = time_for_home_move_s / num_iterations_home
                actual_pulse_delay_for_home_move = max(MINIMUM_PULSE_CYCLE_DELAY, pulse_delay_for_home_move)
                move_corexy(dx_home, dy_home, actual_pulse_delay_for_home_move)
            else:
                current_x_mm = target_x_abs
                current_y_mm = target_y_abs
        else:
            current_x_mm = target_x_abs
            current_y_mm = target_y_abs
            if absolute_mode:
                 print("    Already at (or very close to) 0,0.")
            else:
                print("    Logical position reset to 0,0. No physical movement in REL for this HOME.")
        print(f"  New position: X={current_x_mm:.3f} mm, Y={current_y_mm:.3f} mm")
            
    elif instruction.startswith("S") and instruction != "MOVE": # Standalone Speed command (e.g., S50)
        try:
            # Check if it's just "S" or "S<number>"
            if len(instruction) > 1 and instruction[1:].replace('.', '', 1).isdigit(): # S<number>
                 speed_val_mm_s = float(instruction[1:])
            elif len(parts) > 1 and parts[1].replace('.', '', 1).isdigit(): # S <number>
                 speed_val_mm_s = float(parts[1])
            else:
                print(f"  Invalid S command format. Use S<value> or S <value>.")
                return True

            if speed_val_mm_s > 0:
                TARGET_SPEED_MM_S = speed_val_mm_s
                print(f"  Global target speed set to: {TARGET_SPEED_MM_S:.2f} mm/s")
            else:
                print("  Speed S must be positive.")
        except ValueError:
            print(f"  Invalid speed value in S command: {parts[1] if len(parts) > 1 else instruction[1:]}")
            
    elif instruction == "MOVE" :
        target_x_cmd = None
        target_y_cmd = None
        s_value_this_cmd = None # Speed for this specific MOVE command
        
        for part in parts[1:]:
            if part.startswith('X'):
                try: target_x_cmd = float(part[1:])
                except ValueError: print(f"  Invalid X value: {part}"); return True
            elif part.startswith('Y'):
                try: target_y_cmd = float(part[1:])
                except ValueError: print(f"  Invalid Y value: {part}"); return True
            elif part.startswith('S'): # Speed parameter for this MOVE e.g. S30
                 try: 
                     s_value_this_cmd = float(part[1:])
                 except ValueError: print(f"  Invalid S value in MOVE: {part}")

        if target_x_cmd is None and target_y_cmd is None:
            print("  No X or Y coordinate specified for MOVE.")
            return True

        current_move_speed_mm_s = TARGET_SPEED_MM_S # Default to global
        if s_value_this_cmd is not None:
            if s_value_this_cmd > 0:
                current_move_speed_mm_s = s_value_this_cmd
                TARGET_SPEED_MM_S = current_move_speed_mm_s # Update global if S is in this MOVE line
                # print(f"  Move speed set to {current_move_speed_mm_s:.2f} mm/s (global updated).") # Optional verbose
            else:
                print(f"  Speed S in MOVE must be positive. Using global {TARGET_SPEED_MM_S:.2f} mm/s.")
        
        final_target_x_mm = current_x_mm
        final_target_y_mm = current_y_mm
        if absolute_mode:
            if target_x_cmd is not None: final_target_x_mm = target_x_cmd
            if target_y_cmd is not None: final_target_y_mm = target_y_cmd
        else: # Relative mode
            if target_x_cmd is not None: final_target_x_mm = current_x_mm + target_x_cmd
            if target_y_cmd is not None: final_target_y_mm = current_y_mm + target_y_cmd
        
        actual_target_x_mm = max(0.0, min(final_target_x_mm, XLIM_MM))
        actual_target_y_mm = max(0.0, min(final_target_y_mm, YLIM_MM))

        if actual_target_x_mm != final_target_x_mm or actual_target_y_mm != final_target_y_mm:
            print(f"  Warning: Target ({final_target_x_mm:.2f}, {final_target_y_mm:.2f}) is out of bounds.")
            print(f"           Will be clamped to ({actual_target_x_mm:.2f}, {actual_target_y_mm:.2f}).")

        delta_x_to_move = actual_target_x_mm - current_x_mm
        delta_y_to_move = actual_target_y_mm - current_y_mm
        
        path_length_mm = math.sqrt(delta_x_to_move**2 + delta_y_to_move**2)

        if path_length_mm < (MM_PER_MICROSTEP / 2.0):
            print(f"  Move too small or already at target. Current: ({current_x_mm:.3f}, {current_y_mm:.3f}), Target: ({actual_target_x_mm:.3f}, {actual_target_y_mm:.3f})")
            current_x_mm = actual_target_x_mm
            current_y_mm = actual_target_y_mm
            round(current_x_mm,3)
            round(current_y_mm,3)
        else:
            delta_x_steps_cartesian = round(delta_x_to_move * MICROSTEPS_PER_MM)
            delta_y_steps_cartesian = round(delta_y_to_move * MICROSTEPS_PER_MM)
            steps_m1 = delta_x_steps_cartesian + delta_y_steps_cartesian
            steps_m2 = delta_x_steps_cartesian - delta_y_steps_cartesian
            num_iterations = max(abs(int(steps_m1)), abs(int(steps_m2)))

            if num_iterations == 0:
                print("  No motor steps required for this move.")
                current_x_mm = actual_target_x_mm
                current_y_mm = actual_target_y_mm
                round(current_x_mm,3)
                round(current_y_mm,3)
            else:
                if current_move_speed_mm_s <= 1e-6:
                    print(f"  Target speed {current_move_speed_mm_s:.2f} mm/s is too low. No move executed.")
                else:
                    time_for_move_s = path_length_mm / current_move_speed_mm_s
                    pulse_delay_for_this_move = time_for_move_s / num_iterations
                    actual_pulse_delay_for_this_move = max(MINIMUM_PULSE_CYCLE_DELAY, pulse_delay_for_this_move)
                    effective_speed_mm_s = path_length_mm / (num_iterations * actual_pulse_delay_for_this_move)

                    print(f"  Moving by dx={delta_x_to_move:.3f} mm, dy={delta_y_to_move:.3f} mm")
                    print(f"  To X={actual_target_x_mm:.3f}, Y={actual_target_y_mm:.3f}")
                    print(f"  Target speed: {current_move_speed_mm_s:.2f} mm/s. Effective speed: {effective_speed_mm_s:.2f} mm/s.")
                    print(f"  Pulse cycle delay: {actual_pulse_delay_for_this_move*1000:.4f} ms.")
                    
                    move_corexy(delta_x_to_move, delta_y_to_move, actual_pulse_delay_for_this_move)
        
        print(f"  New position: X={current_x_mm:.3f} mm, Y={current_y_mm:.3f} mm")

    elif instruction == "POS":
        print(f"  Current Position: X={current_x_mm:.3f} mm, Y={current_y_mm:.3f} mm")
        print(f"  Target Speed: {TARGET_SPEED_MM_S:.2f} mm/s")
    
    elif instruction in ["EXIT", "QUIT"]:
        return False
        
    else:
        if instruction:
            print(f"  Unknown or unsupported command: {instruction}")
    
    print("") # Add a blank line for better readability between commands
    return True

def main_cli():
    """Main loop for the command-line interface."""
    global TARGET_SPEED_MM_S
    print("\n--- CoreXY CLI Controller (Custom Commands) ---")
    print(f"Limits: X=[0, {XLIM_MM:.0f}], Y=[0, {YLIM_MM:.0f}] mm")
    print(f"Resolution: {MM_PER_MICROSTEP} mm/microstep ({MICROSTEPS_PER_MM} microsteps/mm)")
    print(f"Motor Native Steps/Rev: {MOTOR_NATIVE_STEPS_PER_REV}, Driver Microstepping: 1/{DRIVER_MICROSTEP_DIVISOR}")
    print(f"Min Pulse Cycle Delay: {MINIMUM_PULSE_CYCLE_DELAY*1000:.3f} ms (Max ~{1/(MINIMUM_PULSE_CYCLE_DELAY*1000):.1f} kHz iter/sec)")
    print(f"Initial (assumed) position: X={current_x_mm:.2f}, Y={current_y_mm:.2f} mm")
    print(f"Initial Target Speed: {TARGET_SPEED_MM_S:.2f} mm/s")
    print("Available commands:")
    print("  MOVE X<val> Y<val> [S<speed_mm_s>] - Move to coordinates (e.g., MOVE X100 Y50 S50)")
    print("  ABS                              - Absolute positioning mode")
    print("  REL                              - Relative positioning mode")
    print("  HOME                             - Home (moves to 0,0 at homing speed if ABS)")
    print("  S <speed_mm_s> or S<speed_mm_s>  - Set global target speed (e.g., S75 or S 75)")
    print("  POS                              - Display current position and target speed")
    print("  EXIT / QUIT                      - Exit program")
    print("-----------------------------")

    running = True
    while running:
        try:
            cmd_line = input("CoreXY > ").strip()
            if not cmd_line:
                continue
            running = parse_command_and_execute(cmd_line)
        except KeyboardInterrupt:
            print("\nKeyboard interrupt detected. Shutting down...")
            running = False
        except Exception as e:
            print(f"An unexpected error occurred: {e}")
            import traceback
            traceback.print_exc()

if __name__ == "__main__":
    try:
        setup_gpio()
        main_cli()
    finally:
        print("Cleaning up before exit...")
        cleanup_gpio()
        print("Program terminated.")