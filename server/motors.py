#!/usr/bin/env python3

import gpiozero
import time
import math
import re # For G-code command parsing

# GPIO Pin Configuration (BCM numbering) - As provided by user
# Motor 1 (often Motor A in CoreXY nomenclature)
PUL_PIN_M1 = 6    # Pulse pin (PUL+)
DIR_PIN_M1 = 5    # Direction pin (DIR+)

# Motor 2 (often Motor B in CoreXY nomenclature)
PUL_PIN_M2 = 27   # Pulse pin (PUL+)
DIR_PIN_M2 = 17   # Direction pin (DIR+)

# --- GPIOZero Device Objects (will be initialized in setup_gpio) ---
pul_device_m1 = None
dir_device_m1 = None
pul_device_m2 = None
dir_device_m2 = None

# --- Motor & System Parameters ---
# These are for documentation or if MM_PER_MICROSTEP needs to be calculated
# based on mechanical properties (pulley size, etc.)
MOTOR_NATIVE_STEPS_PER_REV = 400  # Native steps per full revolution of the motor (e.g., 200 for 1.8deg, 400 for 0.9deg)
DRIVER_MICROSTEP_DIVISOR = 2      # Microstepping factor set on the driver (e.g., 1, 2, 4, 8, 16)
                                  # Total microsteps per motor revolution = MOTOR_NATIVE_STEPS_PER_REV * DRIVER_MICROSTEP_DIVISOR

# User-defined direct conversion: 1 microstep pulse from RPi = 0.5 mm displacement
# This is the primary value used for calculations in this script.
MM_PER_MICROSTEP = 0.5
if MM_PER_MICROSTEP == 0:
    raise ValueError("MM_PER_MICROSTEP cannot be zero.")
MICROSTEPS_PER_MM = 1.0 / MM_PER_MICROSTEP

XLIM_MM = 1000.0  # X-axis limit in mm
YLIM_MM = 1000.0  # Y-axis limit in mm

# Pulse timing
PULSE_DELAY_FAST = 0.0005 # Delay for fast speed (in seconds per step cycle)
PULSE_DELAY_SLOW = 0.005  # Delay for slow speed (in seconds per step cycle)
current_pulse_delay = PULSE_DELAY_SLOW # Active delay, modifiable by S command

# --- System State ---
current_x_mm = 0.0  # Current X position in mm
current_y_mm = 0.0  # Current Y position in mm
absolute_mode = True # True for ABS (absolute), False for REL (relative)

# Minimum pulse width (in seconds)
# Some drivers require a minimum pulse duration.
# Adjust if needed; gpiozero's on()/off() are usually fast enough.
MIN_PULSE_WIDTH = 0.000002  # 2 microseconds (example)


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

def move_motors_coordinated(steps_m1_target, steps_m2_target):
    """
    Coordinates the movement of both motors for CoreXY motion.
    steps_m1_target: number of steps (positive or negative) for motor 1.
    steps_m2_target: number of steps (positive or negative) for motor 2.
    """
    if not all([pul_device_m1, dir_device_m1, pul_device_m2, dir_device_m2]):
        print("Error: GPIO devices not initialized.")
        return

    # Determine direction for M1
    # NOTE: If M1 turns the wrong way for a "positive" contribution
    # (i.e., when steps_m1_target > 0), invert .on() and .off() here.
    if steps_m1_target > 0:
        dir_device_m1.on()  # Assume .on() (HIGH) is M1's "positive" direction
    else:
        dir_device_m1.off() # Assume .off() (LOW) is M1's "negative" direction

    # Determine direction for M2
    # NOTE: If M2 turns the wrong way for a "positive" contribution
    # (i.e., when steps_m2_target > 0), invert .on() and .off() here.
    if steps_m2_target > 0:
        dir_device_m2.on()  # Assume .on() (HIGH) is M2's "positive" direction
    else:
        dir_device_m2.off() # Assume .off() (LOW) is M2's "negative" direction
    
    # Short delay to allow direction pins to settle before pulsing
    time.sleep(0.001) 

    abs_steps_m1 = abs(steps_m1_target)
    abs_steps_m2 = abs(steps_m2_target)
    
    total_iterations = max(abs_steps_m1, abs_steps_m2)

    for i in range(total_iterations):
        perform_pulse_m1 = (i < abs_steps_m1)
        perform_pulse_m2 = (i < abs_steps_m2)

        if perform_pulse_m1:
            pul_device_m1.on()
        if perform_pulse_m2:
            pul_device_m2.on()
        
        time.sleep(MIN_PULSE_WIDTH) # Hold pulse HIGH

        if perform_pulse_m1:
            pul_device_m1.off()
        if perform_pulse_m2:
            pul_device_m2.off()
        
        # Delay between step rising edges (controls speed)
        # Ensures the total step cycle time is current_pulse_delay
        delay_this_step = current_pulse_delay - MIN_PULSE_WIDTH
        if delay_this_step > 0:
            time.sleep(delay_this_step)

def move_corexy(delta_x_mm, delta_y_mm):
    """
    Calculates the required steps for each motor for CoreXY displacement
    and initiates the movement.
    delta_x_mm: desired relative X displacement (mm).
    delta_y_mm: desired relative Y displacement (mm).
    """
    global current_x_mm, current_y_mm

    # Convert displacements in mm to number of Cartesian microsteps
    delta_x_steps_cartesian = round(delta_x_mm * MICROSTEPS_PER_MM)
    delta_y_steps_cartesian = round(delta_y_mm * MICROSTEPS_PER_MM)

    # CoreXY Kinematics:
    # Motor1_Steps = Cartesian_X_Steps + Cartesian_Y_Steps
    # Motor2_Steps = Cartesian_X_Steps - Cartesian_Y_Steps
    steps_m1 = delta_x_steps_cartesian + delta_y_steps_cartesian
    steps_m2 = delta_x_steps_cartesian - delta_y_steps_cartesian
    
    print(f"  Cartesian displacement (steps): dX={delta_x_steps_cartesian}, dY={delta_y_steps_cartesian}")
    print(f"  Motor steps: M1={steps_m1}, M2={steps_m2}")

    if steps_m1 == 0 and steps_m2 == 0:
        print("  No movement required (displacement too small or zero).")
        return

    move_motors_coordinated(int(steps_m1), int(steps_m2))

    # Update current position
    # Use original delta_mm to avoid accumulating 'round' errors
    current_x_mm += delta_x_mm
    current_y_mm += delta_y_mm
    
    # Round for display and to avoid tiny floating point inaccuracies
    current_x_mm = round(current_x_mm, 3) 
    current_y_mm = round(current_y_mm, 3)

def parse_gcode_and_execute(line):
    """Parses a simple G-code line and executes the command."""
    global current_x_mm, current_y_mm, absolute_mode, current_pulse_delay
    
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
    elif instruction == "HOME": # Simplified "Homing" command
        print("  Homing (HOME):")
        if absolute_mode:
            print(f"    Moving to 0,0 from {current_x_mm:.2f}, {current_y_mm:.2f}")
            target_x_abs = 0.0
            target_y_abs = 0.0
            
            dx = target_x_abs - current_x_mm
            dy = target_y_abs - current_y_mm
            move_corexy(dx, dy)
        else: # In relative mode, HOME often means "go to limit switches and set zero"
              # Here, we'll just reset logical coordinates without physical movement.
            current_x_mm = 0.0
            current_y_mm = 0.0
            print("    Logical position reset to 0,0. No physical movement in REL for this HOME.")
        print(f"  New position: X={current_x_mm:.3f} mm, Y={current_y_mm:.3f} mm")

    elif instruction.startswith("S"): # Feedrate command
        try:
            speed_val = float(instruction[1:]) # e.g., F3000
            # Simple interpretation: high value = fast, low value = slow
            if speed_val > 1500: # Arbitrary threshold (e.g., mm/min, but here just a number)
                current_pulse_delay = PULSE_DELAY_FAST
                print(f"  Speed set to FAST (delay: {current_pulse_delay*1000:.3f} ms/pulse)")
            else:
                current_pulse_delay = PULSE_DELAY_SLOW
                print(f"  Speed set to SLOW (delay: {current_pulse_delay*1000:.3f} ms/pulse)")
        except ValueError:
            print(f"  Invalid speed value: {instruction}")
            
    elif instruction == "MOVE" : # Linear/Rapid move
        target_x_cmd = None
        target_y_cmd = None
        
        for part in parts[1:]:
            if part.startswith('X'):
                try: target_x_cmd = float(part[1:])
                except ValueError: print(f"  Invalid X value: {part}"); return
            elif part.startswith('Y'):
                try: target_y_cmd = float(part[1:])
                except ValueError: print(f"  Invalid Y value: {part}"); return
            elif part.startswith('F'): # Allow F in MOVE/G1 as well
                 try:
                    speed_val = float(part[1:])
                    if speed_val > 1500: current_pulse_delay = PULSE_DELAY_FAST
                    else: current_pulse_delay = PULSE_DELAY_SLOW
                    print(f"  Speed updated (delay: {current_pulse_delay*1000:.3f} ms/pulse)")
                 except ValueError: print(f"  Invalid F value in MOVE: {part}")


        if target_x_cmd is None and target_y_cmd is None:
            print("  No X or Y coordinate specified for MOVE.")
            return

        final_target_x_mm = current_x_mm
        final_target_y_mm = current_y_mm

        if absolute_mode:
            if target_x_cmd is not None: final_target_x_mm = target_x_cmd
            if target_y_cmd is not None: final_target_y_mm = target_y_cmd
        else: # Relative mode
            if target_x_cmd is not None: final_target_x_mm = current_x_mm + target_x_cmd
            if target_y_cmd is not None: final_target_y_mm = current_y_mm + target_y_cmd
        
        # Boundary checks
        actual_target_x_mm = max(0.0, min(final_target_x_mm, XLIM_MM))
        actual_target_y_mm = max(0.0, min(final_target_y_mm, YLIM_MM))

        if actual_target_x_mm != final_target_x_mm or actual_target_y_mm != final_target_y_mm:
            print(f"  Warning: Target ({final_target_x_mm:.2f}, {final_target_y_mm:.2f}) is out of bounds.")
            print(f"           Will be clamped to ({actual_target_x_mm:.2f}, {actual_target_y_mm:.2f}).")

        delta_x_to_move = actual_target_x_mm - current_x_mm
        delta_y_to_move = actual_target_y_mm - current_y_mm

        # Check if move is too small (less than half a microstep)
        if abs(delta_x_to_move) < (MM_PER_MICROSTEP / 2.0) and abs(delta_y_to_move) < (MM_PER_MICROSTEP / 2.0):
            print(f"  Move too small or already at target. Current: ({current_x_mm:.3f}, {current_y_mm:.3f}), Target: ({actual_target_x_mm:.3f}, {actual_target_y_mm:.3f})")
        else:
            print(f"  Moving by dx={delta_x_to_move:.3f} mm, dy={delta_y_to_move:.3f} mm")
            print(f"  To X={actual_target_x_mm:.3f} mm, Y={actual_target_y_mm:.3f} mm")
            move_corexy(delta_x_to_move, delta_y_to_move)
        
        print(f"  New position: X={current_x_mm:.3f} mm, Y={current_y_mm:.3f} mm")

    elif instruction == "POS":
        print(f"  Current Position: X={current_x_mm:.3f} mm, Y={current_y_mm:.3f} mm")
    
    elif instruction in ["EXIT", "QUIT"]:
        return False # Signal to exit main loop
        
    else:
        if instruction: # Don't show error for an empty line
            print(f"  Unknown or unsupported command: {instruction}")
            
    print("")
    return True # Continue main loop

def main_cli():
    """Main loop for the command-line interface."""
    print("\n--- CoreXY CLI Controller ---")
    print(f"Limits: X=[0, {XLIM_MM:.0f}], Y=[0, {YLIM_MM:.0f}] mm")
    print(f"Resolution: {MM_PER_MICROSTEP} mm/microstep ({MICROSTEPS_PER_MM} microsteps/mm)")
    print(f"Motor Native Steps/Rev: {MOTOR_NATIVE_STEPS_PER_REV}, Driver Microstepping: 1/{DRIVER_MICROSTEP_DIVISOR}")
    print(f"Initial (assumed) position: X={current_x_mm:.2f}, Y={current_y_mm:.2f} mm")
    print("Available commands:")
    print("  MOVE X<val> Y<val> [S<value>]  - Rapid/Linear move (e.g., MOVE X100 Y50 F3000)")
    print("  ABS                            - Absolute positioning mode")
    print("  REL                            - Relative positioning mode")
    print("  HOME                           - Home (moves to 0,0 if in ABS, else resets 0,0)")
    print("  S<value>                       - Set speed (e.g., F1000 for slow, F3000 for fast)")
    print("  POS                            - Display current position")
    print("  EXIT / QUIT                    - Exit program")
    print("-----------------------------")

    running = True
    while running:
        try:
            cmd_line = input("CoreXY > ").strip()
            if not cmd_line: # If user just presses Enter
                continue
            running = parse_gcode_and_execute(cmd_line)
        except KeyboardInterrupt:
            print("\nKeyboard interrupt detected. Shutting down...")
            running = False
        except Exception as e:
            print(f"An unexpected error occurred: {e}")
            # You might want to stop the program here or allow continuation
            # running = False 

if __name__ == "__main__":
    try:
        setup_gpio()
        main_cli()
    finally:
        print("Cleaning up before exit...")
        cleanup_gpio()
        print("Program terminated.")