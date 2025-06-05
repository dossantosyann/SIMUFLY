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
