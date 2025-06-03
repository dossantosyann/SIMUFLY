import gpiozero
import time
import math
import re # For command parsing
import curses

# GPIO Pin Configuration (BCM numbering)
PUL_PIN_M1 = 6
DIR_PIN_M1 = 5
PUL_PIN_M2 = 27
DIR_PIN_M2 = 17

# GPIOZero Device Objects
pul_device_m1 = None
dir_device_m1 = None
pul_device_m2 = None
dir_device_m2 = None

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
DEFAULT_SPEED_MM_S = 300.0
MAX_SPEED_MM_S = 450.0
TARGET_SPEED_MM_S = min(DEFAULT_SPEED_MM_S, MAX_SPEED_MM_S)
HOMING_SPEED_MM_S = 50.0 # Reduced for safety with new profile potentially
MIN_START_SPEED_MM_S = 5.0

MIN_PULSE_WIDTH = 0.000002
MINIMUM_PULSE_CYCLE_DELAY = 0.0001

# System State
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
    return start * (1 - t) + end * t

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
    
    time.sleep(0.001)

    abs_steps_m1 = abs(steps_m1_target)
    abs_steps_m2 = abs(steps_m2_target)
    
    total_profile_iterations = accel_iterations + cruise_iterations + decel_iterations
    if total_profile_iterations == 0:
        return messages

    initial_delay = max(initial_delay, MINIMUM_PULSE_CYCLE_DELAY, MIN_PULSE_WIDTH + 1e-7)
    cruise_delay = max(cruise_delay, MINIMUM_PULSE_CYCLE_DELAY, MIN_PULSE_WIDTH + 1e-7)
    final_delay = max(final_delay, MINIMUM_PULSE_CYCLE_DELAY, MIN_PULSE_WIDTH + 1e-7)

    for i in range(total_profile_iterations):
        current_calculated_delay = cruise_delay

        if i < accel_iterations:
            if accel_iterations > 0:
                t = (i + 1) / accel_iterations 
                current_calculated_delay = lerp(initial_delay, cruise_delay, t)
            else:
                 current_calculated_delay = cruise_delay
        elif i < accel_iterations + cruise_iterations:
            current_calculated_delay = cruise_delay
        else: 
            if decel_iterations > 0:
                decel_phase_iter = i - (accel_iterations + cruise_iterations)
                t = (decel_phase_iter + 1) / decel_iterations
                current_calculated_delay = lerp(cruise_delay, final_delay, t)
            else:
                current_calculated_delay = final_delay
        
        current_calculated_delay = max(current_calculated_delay, MINIMUM_PULSE_CYCLE_DELAY, MIN_PULSE_WIDTH + 1e-7)

        # Distribute actual motor steps over the total_profile_iterations
        perform_pulse_m1 = False
        if abs_steps_m1 > 0 and total_profile_iterations > 0 :
            # Step if the cumulative fraction of steps for this iteration crosses an integer
             if (i * abs_steps_m1) // total_profile_iterations != ((i - 1) * abs_steps_m1) // total_profile_iterations :
                 perform_pulse_m1 = True
             if i == 0 : # Ensure first step if any steps for M1
                 perform_pulse_m1 = True


        perform_pulse_m2 = False
        if abs_steps_m2 > 0 and total_profile_iterations > 0 :
             if (i * abs_steps_m2) // total_profile_iterations != ((i - 1) * abs_steps_m2) // total_profile_iterations :
                 perform_pulse_m2 = True
             if i == 0 : # Ensure first step if any steps for M2
                 perform_pulse_m2 = True
        
        if perform_pulse_m1: pul_device_m1.on()
        if perform_pulse_m2: pul_device_m2.on()
        
        time.sleep(MIN_PULSE_WIDTH)

        if perform_pulse_m1: pul_device_m1.off()
        if perform_pulse_m2: pul_device_m2.off()
        
        inter_pulse_delay = current_calculated_delay - MIN_PULSE_WIDTH
        if inter_pulse_delay > 0:
            time.sleep(inter_pulse_delay)
            
    return messages

# <<< MODIFIÉ LOURDEMENT pour profil temporel 10-80-10 >>>
def move_corexy_ramped(delta_x_mm, delta_y_mm, target_cruise_speed_mm_s):
    global current_x_mm, current_y_mm
    motor_messages = []
    output_messages_move = [] # For detailed output

    path_length_mm = math.sqrt(delta_x_mm**2 + delta_y_mm**2)

    # Calcul des pas totaux pour chaque moteur (ne change pas)
    delta_x_steps_cartesian = round(-delta_x_mm * MICROSTEPS_PER_MM)
    delta_y_steps_cartesian = round(delta_y_mm * MICROSTEPS_PER_MM)
    steps_m1_total_actual = int(delta_x_steps_cartesian + delta_y_steps_cartesian)
    steps_m2_total_actual = int(delta_x_steps_cartesian - delta_y_steps_cartesian)

    if path_length_mm < (MM_PER_MICROSTEP / 2.0) or (steps_m1_total_actual == 0 and steps_m2_total_actual == 0) :
        # Pas de mouvement physique significatif, mettre à jour la position logique
        current_x_mm += delta_x_mm
        current_y_mm += delta_y_mm
        current_x_mm = round(current_x_mm, 3)
        current_y_mm = round(current_y_mm, 3)
        output_messages_move.append(f"  Note: Move too small or zero steps. Position updated logically.")
        return motor_messages, output_messages_move # Retourne aussi les messages pour affichage

    # Vitesses de sécurité pour calculs
    safe_min_start_speed = max(0.01, MIN_START_SPEED_MM_S)
    safe_target_cruise_speed = max(0.01, target_cruise_speed_mm_s)

    # Calcul des délais cibles pour l'interpolation
    initial_delay = 1.0 / (safe_min_start_speed * MICROSTEPS_PER_MM)
    final_delay = initial_delay
    effective_cruise_delay = 1.0 / (safe_target_cruise_speed * MICROSTEPS_PER_MM)
    
    # Assurer que les délais respectent les minimums système
    initial_delay = max(initial_delay, MINIMUM_PULSE_CYCLE_DELAY)
    effective_cruise_delay = max(effective_cruise_delay, MINIMUM_PULSE_CYCLE_DELAY)
    final_delay = max(final_delay, MINIMUM_PULSE_CYCLE_DELAY)

    T_total = 0.0
    # Cas où la vitesse de croisière est inférieure ou égale à la vitesse de démarrage
    if safe_target_cruise_speed <= safe_min_start_speed:
        if safe_target_cruise_speed > 1e-6: # Éviter la division par zéro
            T_total = path_length_mm / safe_target_cruise_speed
        # Pour un mouvement à vitesse constante (ou décélération pour atteindre la croisière)
        # les délais initial et final devraient être égaux au délai de croisière pour la LERP
        initial_delay = effective_cruise_delay
        final_delay = effective_cruise_delay
        output_messages_move.append(f"  Note: Target speed <= start speed. Will move at {safe_target_cruise_speed:.2f} mm/s.")
    else:
        # Calcul standard de T_total pour profil 10-80-10
        v_avg_profile = (0.1 * safe_min_start_speed) + (0.9 * safe_target_cruise_speed)
        if v_avg_profile > 1e-6: # Éviter la division par zéro
            T_total = path_length_mm / v_avg_profile
        else: # v_avg_profile est trop petit, T_total serait énorme ou indéfini
            T_total = 0 # Mènera à un fallback ou pas de mouvement si path_length > 0

    if T_total <= 1e-6 and path_length_mm > (MM_PER_MICROSTEP / 2.0) :
        output_messages_move.append(f"  Warning: Calculated total move time is near zero ({T_total:.2e}s) for a non-zero path. Check speeds.")
        # Fallback: Mouvement très court à vitesse de démarrage minimale
        if safe_min_start_speed > 1e-6:
            T_total = path_length_mm / safe_min_start_speed
            initial_delay = 1.0 / (safe_min_start_speed * MICROSTEPS_PER_MM)
            effective_cruise_delay = initial_delay
            final_delay = initial_delay
            output_messages_move.append(f"  Fallback to move at min_start_speed: {safe_min_start_speed:.2f} mm/s.")
        else: # Ne peut même pas utiliser min_start_speed
            current_x_mm += delta_x_mm; current_y_mm += delta_y_mm # Mise à jour logique
            # ... rounding ...
            output_messages_move.append(f"  Cannot execute move due to zero effective speed.")
            return [], output_messages_move


    # Temps pour chaque phase
    t_accel = 0.1 * T_total
    t_cruise = 0.8 * T_total
    t_decel = 0.1 * T_total
    output_messages_move.append(f"  Calculated times: Total={T_total:.3f}s, Accel={t_accel:.3f}s, Cruise={t_cruise:.3f}s, Decel={t_decel:.3f}s")


    # Calcul des pas (itérations du profil) pour chaque phase
    steps_accel_float = 0.0
    steps_cruise_float = 0.0
    steps_decel_float = 0.0

    if safe_target_cruise_speed <= safe_min_start_speed: # Mouvement à vitesse constante effective
        steps_accel_float = safe_target_cruise_speed * MICROSTEPS_PER_MM * t_accel
        steps_cruise_float = safe_target_cruise_speed * MICROSTEPS_PER_MM * t_cruise
        steps_decel_float = safe_target_cruise_speed * MICROSTEPS_PER_MM * t_decel
    else: # Mouvement avec rampe standard
        steps_accel_float = ((safe_min_start_speed + safe_target_cruise_speed) / 2.0) * MICROSTEPS_PER_MM * t_accel
        steps_cruise_float = safe_target_cruise_speed * MICROSTEPS_PER_MM * t_cruise
        steps_decel_float = ((safe_target_cruise_speed + safe_min_start_speed) / 2.0) * MICROSTEPS_PER_MM * t_decel

    accel_iterations = int(round(steps_accel_float))
    cruise_iterations = int(round(steps_cruise_float))
    decel_iterations = int(round(steps_decel_float))
    
    total_profile_iterations = accel_iterations + cruise_iterations + decel_iterations
    output_messages_move.append(f"  Profile iterations: Accel={accel_iterations}, Cruise={cruise_iterations}, Decel={decel_iterations} (Total: {total_profile_iterations})")


    # Fallback si le nombre total d'itérations du profil est 0 pour un mouvement non nul
    # Cela peut arriver si les temps calculés sont si courts que tous les pas arrondis sont 0.
    # Dans ce cas, on force au moins quelques itérations pour effectuer les pas moteurs.
    min_total_actual_steps = max(1, max(abs(steps_m1_total_actual), abs(steps_m2_total_actual))) # Au moins 1 si un moteur doit bouger

    if total_profile_iterations == 0 and (abs(steps_m1_total_actual) > 0 or abs(steps_m2_total_actual) > 0):
        output_messages_move.append(f"  Warning: Zero profile iterations for non-zero motor steps. Forcing minimal iterations.")
        # Forcer un mouvement court à la vitesse la plus lente (initial_delay)
        accel_iterations = 0
        decel_iterations = 0
        cruise_iterations = min_total_actual_steps # Le nombre de pas du moteur dominant
        total_profile_iterations = cruise_iterations
        
        # S'assurer que les délais correspondent à un mouvement lent constant
        slow_const_delay = max(initial_delay, effective_cruise_delay, final_delay) # Prendre le plus lent
        initial_delay = slow_const_delay
        effective_cruise_delay = slow_const_delay
        final_delay = slow_const_delay
        output_messages_move.append(f"  Fallback profile: {cruise_iterations} cruise iterations at delay {effective_cruise_delay*1000:.3f}ms.")


    if total_profile_iterations > 0 :
        motor_messages = move_motors_coordinated_ramped(
            steps_m1_total_actual, steps_m2_total_actual,
            accel_iterations, cruise_iterations, decel_iterations,
            initial_delay, effective_cruise_delay, final_delay
        )
    else:
        output_messages_move.append("  Skipping motor coordination due to zero profile iterations despite motor steps.")
        # Normalement, le fallback ci-dessus devrait empêcher cela, mais sécurité.

    current_x_mm += delta_x_mm
    current_y_mm += delta_y_mm
    current_x_mm = round(current_x_mm, 3) 
    current_y_mm = round(current_y_mm, 3)
    return motor_messages, output_messages_move


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
    elif instruction == "HOME":
        output_messages.append("  Homing (HOME):")
        target_x_abs = 0.0
        target_y_abs = 0.0
        
        dx_home = 0.0 
        dy_home = 0.0
        if absolute_mode:
            dx_home = target_x_abs - current_x_mm
            dy_home = target_y_abs - current_y_mm
        else: 
            current_x_mm = target_x_abs
            current_y_mm = target_y_abs
            output_messages.append("    Logical position reset to 0,0. No physical movement in REL for HOME.")
            output_messages.append(f"  New position: X={current_x_mm:.3f} mm, Y={current_y_mm:.3f} mm")
            return output_messages, True

        actual_homing_cruise_speed = min(HOMING_SPEED_MM_S, MAX_SPEED_MM_S)
        
        output_messages.append(f"    Moving to 0,0 from {current_x_mm:.2f}, {current_y_mm:.2f}")
        output_messages.append(f"    Homing cruise speed: {actual_homing_cruise_speed:.2f} mm/s (10-80-10 time profile).")
        
        motor_msgs, move_details = move_corexy_ramped(dx_home, dy_home, actual_homing_cruise_speed)
        output_messages.extend(move_details)
        output_messages.extend(motor_msgs)
        
        output_messages.append(f"  New position: X={current_x_mm:.3f} mm, Y={current_y_mm:.3f} mm")
            
    elif instruction.startswith("S") and instruction != "MOVE": 
        try:
            speed_val_mm_s_req = 0.0
            # ... (parsing de la vitesse S inchangé) ...
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
            output_messages.append(f"  Invalid speed value in S command.") # Simplified error
    
    elif instruction == "LIMITS":
        # ... (inchangé) ...
        if len(parts) > 1:
            sub_command = parts[1].upper()
            if sub_command == "OFF":
                XLIM_MM = float('inf'); YLIM_MM = float('inf')
                output_messages.append("  Coordinate limits DISABLED.")
            elif sub_command == "ON":
                XLIM_MM = DEFAULT_XLIM_MM; YLIM_MM = DEFAULT_YLIM_MM
                output_messages.append(f"  Coordinate limits ENABLED (X: [0, {XLIM_MM:.0f}], Y: [0, {YLIM_MM:.0f}]).")
            else: output_messages.append("  Usage: LIMITS [ON|OFF]")
        else:
            x_lim_display = f"{XLIM_MM:.0f}" if XLIM_MM != float('inf') else "INF"
            y_lim_display = f"{YLIM_MM:.0f}" if YLIM_MM != float('inf') else "INF"
            output_messages.append(f"  Current limits: X=[0, {x_lim_display}], Y=[0, {y_lim_display}]")

    elif instruction == "MOVE" :
        target_x_cmd = None
        target_y_cmd = None
        s_value_this_cmd_req = None
        # ... (parsing X, Y, S inchangé) ...
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
            # ... (logique de clamp de la vitesse S inchangée) ...
            if s_value_this_cmd_req > 0:
                if s_value_this_cmd_req > MAX_SPEED_MM_S:
                    current_move_cruise_speed_mm_s = MAX_SPEED_MM_S
                    output_messages.append(f"  Requested MOVE speed {s_value_this_cmd_req:.2f} mm/s exceeds limit.")
                    output_messages.append(f"  MOVE cruise speed clamped to MAX: {current_move_cruise_speed_mm_s:.2f} mm/s.")
                else:
                    current_move_cruise_speed_mm_s = s_value_this_cmd_req
            else:
                output_messages.append(f"  Speed S in MOVE must be positive. Using global {TARGET_SPEED_MM_S:.2f} mm/s.")
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

        # ... (Warning de clamp inchangé) ...
        clamped_output_needed = False
        if XLIM_MM != float('inf'):
            if abs(actual_target_x_mm - final_target_x_mm) > 1e-9 or abs(actual_target_y_mm - final_target_y_mm) > 1e-9 :
                 clamped_output_needed = True
        elif final_target_x_mm < 0 or final_target_y_mm < 0 :
             clamped_output_needed = True
        if clamped_output_needed:
            output_messages.append(f"  Warning: Target ({final_target_x_mm:.2f}, {final_target_y_mm:.2f}) clamped to ({actual_target_x_mm:.2f}, {actual_target_y_mm:.2f}).")


        delta_x_to_move = actual_target_x_mm - current_x_mm
        delta_y_to_move = actual_target_y_mm - current_y_mm
        
        output_messages.append(f"  Preparing MOVE by dx={delta_x_to_move:.3f} mm, dy={delta_y_to_move:.3f} mm")
        output_messages.append(f"  To X={actual_target_x_mm:.3f}, Y={actual_target_y_mm:.3f}")
        output_messages.append(f"  Target cruise speed: {current_move_cruise_speed_mm_s:.2f} mm/s (10-80-10 time profile).")
        output_messages.append(f"  Min start/end speed: {MIN_START_SPEED_MM_S:.2f} mm/s.")
        
        motor_msgs, move_details = move_corexy_ramped(delta_x_to_move, delta_y_to_move, current_move_cruise_speed_mm_s)
        output_messages.extend(move_details) # Afficher les détails de calcul du profil
        output_messages.extend(motor_msgs)
        
        output_messages.append(f"  New position: X={current_x_mm:.3f} mm, Y={current_y_mm:.3f} mm")

    elif instruction == "POS":
        # ... (inchangé, déjà bon) ...
        x_lim_display = f"{XLIM_MM:.0f}" if XLIM_MM != float('inf') else "INF"
        y_lim_display = f"{YLIM_MM:.0f}" if YLIM_MM != float('inf') else "INF"
        output_messages.append(f"  Pos: X={current_x_mm:.3f}, Y={current_y_mm:.3f} mm")
        output_messages.append(f"  Cruise Speed: {TARGET_SPEED_MM_S:.2f} mm/s (Max: {MAX_SPEED_MM_S:.2f})")
        output_messages.append(f"  Min Start/End Speed: {MIN_START_SPEED_MM_S:.2f} mm/s")
        output_messages.append(f"  Mode: {'ABS' if absolute_mode else 'REL'}. Limits: X=[0, {x_lim_display}], Y=[0, {y_lim_display}]")


    elif instruction in ["EXIT", "QUIT"]:
        output_messages.append("  Exiting program.")
        return output_messages, False 
        
    else:
        if instruction:
            output_messages.append(f"  Unknown command: {instruction}")
    
    return output_messages, True 

# --- Curses UI (inchangé, sauf peut-être les textes d'aide si besoin) ---
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
    curses.curs_set(1); stdscr.nodelay(False)
    running = True
    while running:
        x_limit_display_str = f"{XLIM_MM:.0f}" if XLIM_MM != float('inf') else "INF"
        y_limit_display_str = f"{YLIM_MM:.0f}" if YLIM_MM != float('inf') else "INF"
        header = [
            "--- CoreXY CLI Controller (10-80-10 Time Profile) ---",
            f"Max Cruise: {MAX_SPEED_MM_S:.2f} mm/s, Min Start/End: {MIN_START_SPEED_MM_S:.2f} mm/s",
            f"Optimal speed : 300 mm/s", # User specific
            f"Limits: X=[0, {x_limit_display_str}], Y=[0, {y_limit_display_str}] mm",
            f"Res: {MM_PER_MICROSTEP} mm/µstep ({MICROSTEPS_PER_MM} µsteps/mm)",
            f"Min Pulse Cycle: {MINIMUM_PULSE_CYCLE_DELAY*1000:.3f} ms",
            "Cmds: MOVE X<v> Y<v> [S<v>], ABS, REL, HOME, S<v>, LIMITS, POS, EXIT"
        ]
        status = [
            f"Pos: X={current_x_mm:.3f} Y={current_y_mm:.3f} mm",
            f"Set Cruise: {TARGET_SPEED_MM_S:.2f} mm/s, Mode: {'ABS' if absolute_mode else 'REL'}"
        ]
        input_prompt_text = "CoreXY > "
        draw_ui(stdscr, header, status, last_command_output, input_prompt_text)
        input_line_y = stdscr.getmaxyx()[0] - 1
        try:
            if input_line_y > 0 and input_line_y > (len(header) + len(status) +2) :
                 stdscr.move(input_line_y, len(input_prompt_text)); curses.echo()
                 cmd_line_bytes = stdscr.getstr(input_line_y, len(input_prompt_text), 60)
                 curses.noecho(); cmd_line = cmd_line_bytes.decode('utf-8').strip()
            else: cmd_line = ""; last_command_output = ["Terminal too small."]
            if not cmd_line:
                if not last_command_output or (last_command_output and last_command_output[-1] != "Terminal too small."):
                    last_command_output = [] 
                continue
            last_command_output, running = parse_command_and_execute(cmd_line)
        except curses.error as e: curses.noecho(); last_command_output = [f"Curses error: {e}"]
        except KeyboardInterrupt: curses.noecho(); last_command_output = ["Keyboard interrupt. Exiting..."]; running = False
        except Exception as e: curses.noecho(); last_command_output = [f"Unexpected error: {e}"]
            # import traceback; last_command_output.extend(traceback.format_exc().splitlines()) # For debug
    # Final paint before exit
    # ... (inchangé)
    status = [f"Pos: X={current_x_mm:.3f} Y={current_y_mm:.3f}", f"Speed: {TARGET_SPEED_MM_S:.2f} Mode: {'ABS' if absolute_mode else 'REL'}"]
    if not last_command_output or (last_command_output and "Exiting program." not in last_command_output[-1]): last_command_output.append("Exiting program.")
    header = ["--- CoreXY CLI Controller (10-80-10 Time Profile) ---", "Exited."]
    try: draw_ui(stdscr, header, status, last_command_output, " "); stdscr.refresh(); time.sleep(0.5) 
    except: pass


if __name__ == "__main__":
    if setup_gpio():
        try:
            curses.wrapper(_curses_main_loop)
        except Exception as e:
            print(f"Critical error outside curses: {e}")
            import traceback; traceback.print_exc()
        finally:
            print("Cleaning up GPIO..."); cleanup_gpio(); print("Terminated.") 
    else:
        print("GPIO setup failed. Program will not start."); print("Terminated.")