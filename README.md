# Mode manuel

## Liste des commandes et leur description

1. **MOVE X Y [S]** : Se déplace en XY à vitesse S en mm/s. Cette commande peut être utilisée en mode absolu ou relatif. La vitesse mise en argument n'est utilisée que pour un déplacement.
   
   Exemples :
   - MOVE X200 Y350
   - MOVE X500
   - MOVE X300 Y300 S250
   - MOVE X-150 Y-200 (mode relatif uniquement)

2. **ABS** : Définit le mode de déplacement en coordonnées absolues.

3. **REL** : Définit le mode de déplacement en coordonnées relatives.

4. **HOME** : Déplace la caméra à l'origine au point (0,0).

5. **SETHOME** : Cette commande définit la position physique actuelle de la caméra comme l'origine (0,0). Attention, les limites de déplacement de la nacelle ne sont pas mises à jour après l'utilisation de cette commande. Il est donc impératif de veiller à ne pas endommager les moteurs en leur demandant de se déplacer en dehors des zones permises par le cadre.

6. **CAL** : Calibre l'origine (0,0) à l'aide des capteurs fin de course.

7. **Capture [PATH]** : Capture et enregistre l'image au chemin indiqué.

8. **S** : Définit la vitesse pour tous les prochains déplacements en mm/s.

9. **LIMITS [ON/OFF]** : Active ou désactive les limites de position de la caméra. ATTENTION à ne pas endommager le système en désactivant les limites.

10. **POS** : Affiche les paramètres actuels du système.

11. **MENU/EXIT** : Retourne au menu principal.



# Marche à suivre pour se connecter au Raspberry Pi

Pour se connecter au Raspberry Pi, il existe plusieurs méthodes.

## Connexion physique

La première, et la plus simple, est de connecter un écran, un clavier et une souris à l'ordinateur et de le contrôler de cette façon.

## Connexion via SSH

La deuxième, est de se connecter en SSH au Raspberry Pi. Pour ce faire, une fois que ce dernier est alimenté, il va générer un point d'accès Wi-Fi appelé **pi5hotspot** dont le mot de passe est **drone2025**. Une fois connecté à ce réseau, à l'aide d'un terminal, scannez les différents appareils connectés au réseau et identifiez l'adresse IP du Raspberry Pi.

Avec ces informations, nous pouvons maintenant nous connecter en SSH avec la commande :

```bash
ssh pi5@<IP ADDRESS>
```

Cette connexion nous permet de contrôler le Raspberry Pi via le terminal et de lancer notre programme qui est entièrement géré en lignes de commandes, c'est parfait !

## Connexion via VNC

Alternativement, il est également possible de se connecter graphiquement au Raspberry Pi, c'est-à-dire d'utiliser n'importe quel PC comme écran pour le Raspberry, avec le protocole VNC. Pour ce faire, si ce n'est pas déjà activé, il faut activer le VNC du Raspberry. Pour cela, connectez-vous en SSH comme expliqué précédemment et suivez les instructions suivantes :

1. Ouvrez l'outil de configuration du Raspberry Pi :

   ```bash
   sudo raspi-config
   ```

2. Naviguez dans le menu avec les flèches du clavier :
   - Sélectionnez **3 Interface Options**
   - Puis **I3 VNC**
   - Choisissez **Yes** pour activer le serveur VNC
   - Confirmez avec **OK**

3. Redémarrez le Raspberry Pi pour que les changements prennent effet :

   ```bash
   sudo reboot
   ```

4. Une fois redémarré, vous pouvez vérifier que le service VNC est actif :

   ```bash
   sudo systemctl status vncserver-x11-serviced
   ```

Maintenant, il nous faut nous connecter via VNC au Raspberry Pi :

1. Téléchargez et installez un client VNC sur votre ordinateur (par exemple VNC Viewer de RealVNC)

2. Ouvrez le client VNC et créez une nouvelle connexion avec l'adresse IP du Raspberry Pi (la même que pour SSH)

3. Connectez-vous avec les identifiants du Raspberry Pi :
   - **Nom d'utilisateur** : pi5
   - **Mot de passe** : drone2025

Vous devriez maintenant voir l'interface graphique du Raspberry Pi sur votre écran !


###

# Marche à suivre pour utiliser le programme

Le programme est en principe déjà présent sur le Raspberry Pi au chemin _/SIMUFLY/simufly.py_ et peut être exécuté avec la commande :

```bash
python3 simufly.py
```

Cependant, si le programme n'est pas présent, il suffit de cloner le répertoire GitHub avec la commande : 

```bash
git clone https://github.com/dossantosyann/SIMUFLY.git
```

Si des dépendances sont manquantes, installez-les.

Le programme se contrôle à l'aide de lignes de commandes et des flèches du clavier pour naviguer les menus.
