import curses
import re
import time
import sys

# Largeur fixe pour l'interface CLI et indentation pour le texte aligné à gauche
CLI_WIDTH = 70
LEFT_INDENT_STR = ""  # Pas d'indentation à gauche

# --- Curses based display functions (inchangées) ---

def display_separator(stdscr, y, width=CLI_WIDTH):
    """Displays a separator line at given y coordinate."""
    try:
        stdscr.addstr(y, 0, '-' * width)
    except curses.error: # pragma: no cover
        pass
    return y + 1

def display_title(stdscr, y, text, width=CLI_WIDTH):
    """Displays a centered title."""
    line_content = f" {text} ".center(width, '-')
    try:
        stdscr.addstr(y, 0, line_content)
    except curses.error: # pragma: no cover
        pass
    return y + 1

def display_section_header(stdscr, y, text, width=CLI_WIDTH):
    """Displays a left-aligned section header."""
    content = f"{text} :"
    try:
        stdscr.addstr(y, 0, f"{LEFT_INDENT_STR}{content}".ljust(width))
    except curses.error: # pragma: no cover
        pass
    return y + 1

def display_option(stdscr, y, text, width=CLI_WIDTH):
    """Displays a left-aligned menu option."""
    try:
        stdscr.addstr(y, 0, f"{LEFT_INDENT_STR}{text}".ljust(width))
    except curses.error: # pragma: no cover
        pass
    return y + 1

def display_left_aligned_message(stdscr, y, message, width=CLI_WIDTH):
    """Displays a left-aligned informational message, clearing the line first."""
    try:
        stdscr.addstr(y, 0, " " * width)
        stdscr.addstr(y, 0, f"{LEFT_INDENT_STR}{message}".ljust(width))
    except curses.error: # pragma: no cover
        pass
    return y + 1

def get_left_aligned_input_curses(stdscr, y, prompt_text, width=CLI_WIDTH):
    """Displays a prompt and gets input using curses."""
    prompt_display_text = f"{LEFT_INDENT_STR}{prompt_text}"
    input_start_x = len(prompt_display_text)
    
    try:
        stdscr.addstr(y, 0, prompt_display_text)
        curses.echo()
        curses.curs_set(1)
        stdscr.move(y, input_start_x)
        
        max_input_chars = width - input_start_x -1
        if max_input_chars < 0: max_input_chars = 0

        input_bytes = stdscr.getstr(y, input_start_x, max_input_chars)
        input_str = input_bytes.decode('utf-8', 'replace')

        curses.noecho()
        curses.curs_set(0)
        return input_str.strip()
    except curses.error: # pragma: no cover
        curses.noecho()
        curses.curs_set(0)
        return ""

def display_left_aligned_animation_char_curses(stdscr, y, char_to_display, width=CLI_WIDTH): # Inchangée
    """Displays an animation character on the line and refreshes."""
    try:
        stdscr.addstr(y, 0, f"{LEFT_INDENT_STR}{char_to_display}".ljust(width))
        stdscr.refresh()
    except curses.error: # pragma: no cover
        pass

# --- Fonctions d'interprétation des commandes ---

def interpreter_commande_capture_multi(commande_str): # Inchangée (pour mode "Automatique")
    updates = {}
    stripped_cmd_upper = commande_str.strip().upper()
    if stripped_cmd_upper == "START":
        updates["COMMAND"] = "START"
        return updates
    if stripped_cmd_upper == "BACK" or stripped_cmd_upper == "B":
        updates["COMMAND"] = "BACK"
        return updates

    pos_match = re.search(r"POS\s+(\d+)", commande_str, re.IGNORECASE)
    if pos_match:
        try:
            updates["POS"] = int(pos_match.group(1))
        except ValueError: # pragma: no cover
            pass

    n_match = re.search(r"N\s+(\d+)", commande_str, re.IGNORECASE)
    if n_match:
        try:
            updates["N"] = int(n_match.group(1))
        except ValueError: # pragma: no cover
            pass

    name_match = re.search(r"NAME\s+((?:(?!\b(?:POS|N|START|BACK)\b).)+?)(?:\s+(?:POS|N|START|BACK)|$)", commande_str, re.IGNORECASE)
    if name_match:
        name_value = name_match.group(1).strip()
        if name_value:
            updates["NAME"] = name_value
    return updates

def interpreter_commande_pas_a_pas(commande_str, drone_state): # Inchangée
    """Interprète les commandes pour le mode 'Manuel' et met à jour drone_state."""
    cmd_upper = commande_str.strip().upper()
    original_cmd = commande_str.strip()
    action = None
    message = ""

    if cmd_upper == "BACK" or cmd_upper == "B":
        action = "BACK"
        return action, message

    if cmd_upper == "ABS":
        if drone_state["mode"] != "ABS":
            drone_state["mode"] = "ABS"
            message = "Mode de déplacement réglé sur ABSOLU."
        else:
            message = "Mode de déplacement déjà en ABSOLU."
        action = "STATE_UPDATED"
        return action, message

    if cmd_upper == "REL":
        if drone_state["mode"] != "REL":
            drone_state["mode"] = "REL"
            message = "Mode de déplacement réglé sur RELATIF."
        else:
            message = "Mode de déplacement déjà en RELATIF."
        action = "STATE_UPDATED"
        return action, message

    if cmd_upper == "HOME":
        drone_state["x"] = 0.0
        drone_state["y"] = 0.0
        message = f"Retour à la position HOME (0.0, 0.0)."
        action = "STATE_UPDATED"
        return action, message

    speed_match = re.match(r"SPEED\s+(\d+(?:\.\d*)?)", original_cmd, re.IGNORECASE)
    if speed_match:
        try:
            val = float(speed_match.group(1))
            if val > 0:
                drone_state["speed"] = val
                message = f"Vitesse par défaut réglée à {val}."
                action = "STATE_UPDATED"
            else:
                message = "La vitesse doit être un nombre positif."
                action = "ERROR"
        except ValueError: # pragma: no cover
            message = "Valeur de vitesse invalide."
            action = "ERROR"
        return action, message

    capture_match = re.match(r"CAPTURE\s+N\s+(\d+)", original_cmd, re.IGNORECASE)
    if capture_match:
        try:
            num_images = int(capture_match.group(1))
            if num_images > 0:
                message = f"Capture de {num_images} images à ({drone_state['x']:.1f}, {drone_state['y']:.1f}) simulée."
                action = "CAPTURE_SIMULATED"
            else:
                message = "Le nombre d'images doit être positif."
                action = "ERROR"
        except ValueError: # pragma: no cover
            message = "Nombre d'images invalide."
            action = "ERROR"
        return action, message

    move_match = re.match(r"MOVE\s+X\s+(-?\d+(?:\.\d*)?)\s+Y\s+(-?\d+(?:\.\d*)?)(?:\s+S\s+(\d+(?:\.\d*)?))?", original_cmd, re.IGNORECASE)
    if move_match:
        try:
            target_x = float(move_match.group(1))
            target_y = float(move_match.group(2))
            move_speed = drone_state["speed"]
            if move_match.group(3):
                custom_speed = float(move_match.group(3))
                if custom_speed > 0:
                    move_speed = custom_speed
                else:
                    action = "ERROR"
                    message = "La vitesse S spécifiée doit être positive."
                    return action, message
            
            old_x, old_y = drone_state["x"], drone_state["y"]
            if drone_state["mode"] == "ABS":
                drone_state["x"] = target_x
                drone_state["y"] = target_y
            else: 
                drone_state["x"] += target_x
                drone_state["y"] += target_y
            
            message = (f"Déplacement de ({old_x:.1f}, {old_y:.1f}) vers ({drone_state['x']:.1f}, {drone_state['y']:.1f}) "
                       f"à la vitesse {move_speed:.1f} (Mode: {drone_state['mode']}).")
            action = "MOVE_SIMULATED"
        except ValueError: # pragma: no cover
            message = "Coordonnées X, Y ou vitesse S invalides pour MOVE."
            action = "ERROR"
        return action, message
        
    if original_cmd:
        message = f"Commande non reconnue: '{original_cmd}'"
        action = "ERROR"
        
    return action, message

def interpreter_commande_fichier_json(commande_str, fichier_state):
    """Interprète les commandes pour le mode 'Fichier JSON'."""
    cmd_upper = commande_str.strip().upper()
    original_cmd = commande_str.strip()
    action = None
    message = ""

    if cmd_upper == "BACK" or cmd_upper == "B":
        action = "BACK"
        return action, message

    load_match = re.match(r"LOAD\s+(.+)", original_cmd, re.IGNORECASE) # Accept any filename after LOAD
    if load_match:
        filename = load_match.group(1).strip()
        if not filename: # pragma: no cover
            message = "Nom de fichier manquant après LOAD."
            action = "ERROR"
        elif not filename.lower().endswith(".json"):
            message = f"Le fichier '{filename}' doit avoir une extension .json."
            action = "ERROR"
        else:
            # Ici, on pourrait ajouter os.path.exists(filename) si on voulait vérifier l'existence
            fichier_state["loaded_filename"] = filename
            message = f"Fichier '{filename}' prêt à être utilisé."
            action = "FILE_LOADED"
        return action, message

    if cmd_upper == "START":
        if fichier_state["loaded_filename"]:
            message = f"Démarrage de la simulation avec le fichier '{fichier_state['loaded_filename']}'."
            # En théorie, ici on lirait le JSON et on configurerait la simulation.
            # Pour l'instant, on simule juste le démarrage.
            action = "JSON_SIMULATION_START"
        else:
            message = "Aucun fichier JSON chargé. Utilisez LOAD <nom_fichier.json>."
            action = "ERROR"
        return action, message
        
    if original_cmd:
        message = f"Commande non reconnue: '{original_cmd}'"
        action = "ERROR"
        
    return action, message

# --- Fonctions d'affichage des menus ---

def afficher_menu_principal_curses(stdscr, width=CLI_WIDTH): # Inchangée
    stdscr.clear()
    y = 0
    y = display_separator(stdscr, y, width)
    y = display_title(stdscr, y, "Drone Simulator", width)
    y = display_separator(stdscr, y, width)
    y += 1
    y = display_section_header(stdscr, y, "Sélection du mode de déplacement", width)
    y = display_option(stdscr, y, "0. Calibrage", width)
    y = display_option(stdscr, y, "1. Automatique", width)
    y = display_option(stdscr, y, "2. Manuel", width)
    y = display_option(stdscr, y, "3. À partir d'un fichier (JSON)", width)
    y = display_separator(stdscr, y, width)
    
    input_line_y = y 
    choix = get_left_aligned_input_curses(stdscr, input_line_y, "Votre choix (0-3, ou 'q' pour quitter) : ", width)
    stdscr.refresh()
    return choix, input_line_y

def afficher_sous_menu_capture_curses(stdscr, mode_deplacement_choisi, current_params, width=CLI_WIDTH): # Inchangée (pour mode "Automatique")
    stdscr.clear()
    y = 0
    y = display_separator(stdscr, y, width)
    y = display_title(stdscr, y, "Drone Simulator", width)
    y = display_separator(stdscr, y, width)
    y += 1

    pos_str = str(current_params['POS']) if current_params['POS'] is not None else "Non défini"
    n_str = str(current_params['N']) if current_params['N'] is not None else "Non défini"
    name_str = f"'{current_params['NAME']}'" if current_params['NAME'] is not None else "Non défini"
    info_text = f"Paramètres: POS={pos_str}, N={n_str}, NAME={name_str}"
    y = display_option(stdscr, y, info_text, width)
    y += 1

    mode_text = f"Mode de déplacement choisi : {mode_deplacement_choisi}"
    y = display_option(stdscr, y, mode_text, width)
    y += 1

    y = display_section_header(stdscr, y, "Sélection du mode de capture", width)
    y = display_option(stdscr, y, "  POS <nombre> : positions de capture", width)
    y = display_option(stdscr, y, "  N <nombre>   : images par position", width)
    y = display_option(stdscr, y, "  NAME <texte> : nom du chantier", width)
    y = display_option(stdscr, y, "  START        : lance la prise de vue", width)
    y = display_option(stdscr, y, "  B / BACK     : Retour au menu principal", width)
    y = display_option(stdscr, y, "Ex: POS 10 N 5 NAME MonProjet", width)
    y = display_separator(stdscr, y, width)
    
    input_line_y = y
    commande = get_left_aligned_input_curses(stdscr, input_line_y, "Votre commande : ", width)
    stdscr.refresh()
    return commande, input_line_y

def afficher_menu_calibration_curses(stdscr, width=CLI_WIDTH): # Inchangée
    stdscr.clear()
    current_y = 0
    current_y = display_separator(stdscr, current_y, width)
    current_y = display_title(stdscr, current_y, "Drone Simulator", width)
    current_y = display_separator(stdscr, current_y, width)
    current_y += 1 
    
    current_y = display_section_header(stdscr, current_y, "Calibration en cours", width)
    animation_line_y = current_y
    
    animation_chars = ["|", "/", "-", "\\"]
    for i in range(20):
        anim_char = animation_chars[i % len(animation_chars)]
        display_left_aligned_animation_char_curses(stdscr, animation_line_y, anim_char, width)
        time.sleep(0.15)
    
    current_y = display_option(stdscr, animation_line_y, "Calibration terminée.", width)
    current_y = display_separator(stdscr, current_y, width)
    
    get_left_aligned_input_curses(stdscr, current_y, "Appuyez sur Entrée pour continuer...", width)
    stdscr.refresh()

def afficher_menu_pas_a_pas_curses(stdscr, mode_name, drone_state, width=CLI_WIDTH): # Inchangée
    """Affiche le menu et l'état pour le mode 'Manuek'."""
    stdscr.clear()
    y = 0
    y = display_separator(stdscr, y, width)
    y = display_title(stdscr, y, "Drone Simulator - Mode Manuel", width)
    y = display_separator(stdscr, y, width)
    y += 1

    state_info1 = f"Position actuelle: X={drone_state['x']:.1f}, Y={drone_state['y']:.1f}"
    state_info2 = f"Vitesse par défaut: {drone_state['speed']:.1f} | Mode déplacement: {drone_state['mode']}"
    y = display_option(stdscr, y, state_info1, width)
    y = display_option(stdscr, y, state_info2, width)
    y += 1
    
    y = display_section_header(stdscr, y, "Commandes disponibles", width)
    y = display_option(stdscr, y, "  MOVE X <val> Y <val> [S <val>] : Déplacer le drone", width)
    y = display_option(stdscr, y, "     (S <val> est la vitesse pour CE mouvement, optionnel)", width)
    y = display_option(stdscr, y, "  ABS / REL                      : Changer mode (Absolu/Relatif)", width)
    y = display_option(stdscr, y, "  HOME                           : Retourner à X=0, Y=0", width)
    y = display_option(stdscr, y, "  SPEED <val>                    : Changer vitesse par défaut", width)
    y = display_option(stdscr, y, "  CAPTURE N <nombre>             : Simuler N captures d'images", width)
    y = display_option(stdscr, y, "  B / BACK                       : Retour au menu principal", width)
    y = display_separator(stdscr, y, width)

    input_line_y = y
    commande = get_left_aligned_input_curses(stdscr, input_line_y, "Votre commande : ", width)
    stdscr.refresh()
    return commande, input_line_y

def afficher_menu_fichier_json_curses(stdscr, fichier_state, width=CLI_WIDTH):
    """Affiche le menu pour le mode 'Fichier JSON'."""
    stdscr.clear()
    y = 0
    y = display_separator(stdscr, y, width)
    y = display_title(stdscr, y, "Drone Simulator - Mode Fichier JSON", width)
    y = display_separator(stdscr, y, width)
    y += 1

    # Affichage du fichier chargé
    fichier_charge_text = f"Fichier chargé : {fichier_state['loaded_filename'] if fichier_state['loaded_filename'] else 'Aucun'}"
    y = display_option(stdscr, y, fichier_charge_text, width)
    y += 1
    
    y = display_section_header(stdscr, y, "Commandes disponibles", width)
    y = display_option(stdscr, y, "  LOAD <nom_fichier.json>        : Charger un fichier de mission", width)
    y = display_option(stdscr, y, "  START                          : Démarrer la mission du fichier", width)
    y = display_option(stdscr, y, "  B / BACK                       : Retour au menu principal", width)
    y = display_separator(stdscr, y, width)

    input_line_y = y
    commande = get_left_aligned_input_curses(stdscr, input_line_y, "Votre commande : ", width)
    stdscr.refresh()
    return commande, input_line_y

# --- Logique principale du simulateur ---

def simulateur_drone_curses(stdscr):
    curses.curs_set(0)
    curses.noecho()
    stdscr.keypad(True)

    # État pour le mode "Automatique" (mode 1)
    parametres_capture_auto = {"POS": None, "N": None, "NAME": None}
    
    message_display_duration = 1.5

    while True:
        choix_principal, main_menu_input_line_y = afficher_menu_principal_curses(stdscr, CLI_WIDTH)
        main_menu_message_y = main_menu_input_line_y + 1

        if choix_principal == '0': # Calibrage
            afficher_menu_calibration_curses(stdscr, CLI_WIDTH)

        elif choix_principal == '1': # Mode "Automatique"
            mode_actuel = "Automatique" 
            # parametres_capture_auto est réutilisé/réinitialisé à chaque entrée dans ce mode si nécessaire
            while True:
                commande_brute, sub_menu_input_line_y = afficher_sous_menu_capture_curses(
                    stdscr, mode_actuel, parametres_capture_auto, CLI_WIDTH
                )
                message_y_pos_submenu = sub_menu_input_line_y + 1
                parsed_updates = interpreter_commande_capture_multi(commande_brute)
                messages_to_show = []

                if "POS" in parsed_updates:
                    parametres_capture_auto["POS"] = parsed_updates["POS"]
                    messages_to_show.append(f"Nombre de positions défini à : {parametres_capture_auto['POS']}")
                if "N" in parsed_updates:
                    parametres_capture_auto["N"] = parsed_updates["N"]
                    messages_to_show.append(f"Nombre d'images/pos défini à : {parametres_capture_auto['N']}")
                if "NAME" in parsed_updates:
                    parametres_capture_auto["NAME"] = parsed_updates["NAME"]
                    messages_to_show.append(f"Nom du chantier défini à : '{parametres_capture_auto['NAME']}'")

                if messages_to_show:
                    current_msg_line = message_y_pos_submenu
                    for msg_line in messages_to_show:
                        display_left_aligned_message(stdscr, current_msg_line, msg_line, CLI_WIDTH)
                        current_msg_line +=1
                    stdscr.refresh()
                    time.sleep(message_display_duration)

                commande_speciale = parsed_updates.get("COMMAND")
                if commande_speciale == "START":
                    if all(p is not None for p in parametres_capture_auto.values()):
                        stdscr.clear()
                        y = 0
                        y = display_left_aligned_message(stdscr, y+1, f"Lancement: mode '{mode_actuel}'", CLI_WIDTH)
                        y = display_left_aligned_message(stdscr, y, f"  Positions : {parametres_capture_auto['POS']}", CLI_WIDTH)
                        y = display_left_aligned_message(stdscr, y, f"  Images/pos: {parametres_capture_auto['N']}", CLI_WIDTH)
                        y = display_left_aligned_message(stdscr, y, f"  Chantier  : '{parametres_capture_auto['NAME']}'", CLI_WIDTH)
                        y += 1
                        y = display_left_aligned_message(stdscr, y, "Simulation de la prise de vue terminée.", CLI_WIDTH)
                        y = display_separator(stdscr, y, CLI_WIDTH)
                        get_left_aligned_input_curses(stdscr, y, "Appuyez sur Entrée pour retourner au menu principal...", CLI_WIDTH)
                        break 
                    else:
                        display_left_aligned_message(stdscr, message_y_pos_submenu, "Veuillez définir POS, N, et NAME avant START.", CLI_WIDTH)
                        stdscr.refresh()
                        time.sleep(message_display_duration)
                elif commande_speciale == "BACK":
                    break 
                elif not parsed_updates and commande_brute.strip():
                    display_left_aligned_message(stdscr, message_y_pos_submenu, f"Commande non reconnue : '{commande_brute}'.", CLI_WIDTH)
                    stdscr.refresh()
                    time.sleep(message_display_duration)

        elif choix_principal == '2': # Mode "Manuel"
            drone_state_pas_a_pas = { "x": 0.0, "y": 0.0, "speed": 10.0, "mode": "ABS" }
            while True:
                commande_brute, sub_menu_input_line_y = afficher_menu_pas_a_pas_curses(
                    stdscr, "Manuel", drone_state_pas_a_pas, CLI_WIDTH
                )
                message_y_pos_submenu = sub_menu_input_line_y + 1
                action, message = interpreter_commande_pas_a_pas(commande_brute, drone_state_pas_a_pas)

                if message:
                    display_left_aligned_message(stdscr, message_y_pos_submenu, message, CLI_WIDTH)
                    stdscr.refresh()
                    time.sleep(message_display_duration if action != "ERROR" else message_display_duration + 0.5)
                
                if action == "BACK":
                    break

        elif choix_principal == '3': # Mode "À partir d'un fichier (JSON)"
            fichier_json_state = {"loaded_filename": None}
            while True:
                commande_brute, sub_menu_input_line_y = afficher_menu_fichier_json_curses(
                    stdscr, fichier_json_state, CLI_WIDTH
                )
                message_y_pos_submenu = sub_menu_input_line_y + 1
                action, message = interpreter_commande_fichier_json(commande_brute, fichier_json_state)

                if message:
                    display_left_aligned_message(stdscr, message_y_pos_submenu, message, CLI_WIDTH)
                    stdscr.refresh()
                    time.sleep(message_display_duration if action != "ERROR" else message_display_duration + 0.5)

                if action == "BACK":
                    break
                elif action == "JSON_SIMULATION_START":
                    # Afficher un écran de simulation similaire aux autres modes START
                    stdscr.clear()
                    y = 0
                    y = display_left_aligned_message(stdscr, y+1, f"Lancement: Mode Fichier JSON", CLI_WIDTH)
                    y = display_left_aligned_message(stdscr, y, f"  Fichier   : '{fichier_json_state['loaded_filename']}'", CLI_WIDTH)
                    # Ici, on pourrait afficher des paramètres lus du JSON si c'était implémenté
                    y += 1
                    y = display_left_aligned_message(stdscr, y, "Simulation de la mission à partir du fichier terminée.", CLI_WIDTH)
                    y = display_separator(stdscr, y, CLI_WIDTH)
                    get_left_aligned_input_curses(stdscr, y, "Appuyez sur Entrée pour retourner au menu principal...", CLI_WIDTH)
                    break # Retourne au menu principal après la simulation

        elif choix_principal.lower() == 'q': # Quitter
            stdscr.clear()
            y = 0
            y = display_left_aligned_message(stdscr, y+1, "Drone Simulator arrêté. Au revoir !", CLI_WIDTH)
            y = display_separator(stdscr, y, CLI_WIDTH)
            stdscr.refresh()
            time.sleep(1)
            break
        else: # Choix invalide au menu principal
            display_left_aligned_message(stdscr, main_menu_message_y, "Choix invalide. Veuillez réessayer.", CLI_WIDTH)
            stdscr.refresh()
            time.sleep(message_display_duration)

if __name__ == "__main__":
    try:
        curses.wrapper(simulateur_drone_curses)
    except KeyboardInterrupt: # pragma: no cover
        print("Interruption clavier détectée. Fermeture du programme.")
    except curses.error as e: # pragma: no cover
        print(f"Une erreur Curses est survenue: {e}")
        print("Le terminal pourrait être dans un état instable. Essayez de taper 'reset'.")
    finally:
        pass