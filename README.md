# Simulateur-de-prise-de-vue-par-drone

# Notes perso
Ce sont les notes que j'ai (Yann) pris lors de mes recherches pour la création du logicielle. Les étapes présentes sont écrites dans l'ordre chonologique où je les ai exécutées

## Sources
[Création du projet Flask + React à l'aide de Vite](https://www.youtube.com/watch?v=ctQMqqEo4G8)

[Build du projet Flask + React pour déploiement](https://www.youtube.com/watch?v=tvcWCQqLegM)

[Contrôle de la webcam avec OpenCV](https://www.youtube.com/watch?v=sd25t4HmFdU)

# Création du projet Flask + React

## Création frontend (client), projet React via Vite :
```bash
npx create-vite 
-> React
-> Javascript
cd "dossier"
npm install
npm run dev
```

## Création backend (server), python Flask :
```bash
python3 -m venv venv
source venv/bin/activate
pip3 install flask
-> créer main.py
```

## Installer flask-cors :
```bash
pip3 install Flask-CORS
```

## Importer cors dans backend :
Dans le fichier main.py ajouter : 
```python 
from flask_cors import CORS
```

## Installer axios :

```bash
npm install axios
```

## Importer useEffect dans frontend :
Dans le fichier client/src/app, modifier la première ligne pour ajouter la librairie useEffect : 
```jsx
import { useState, useEffect } from 'react'
```
# Build du projet pour déploiement

### Frontend :
```bash
npm run build
```
### Backend :
Dans le fichier main.py :
```python
from flask import Flask, jsonify, send_from_directory


@app.route("/")
def home():
    return send_from_directory(app.static_folder, "index.html")

```

### Lancer le serveur
```bash
python3 main.py
```

# Installation de Tailwindcss
Tailwind sera utilisé plus tard pour l'interface, mais l'installation a déjà été effectuée.

[Guide officiel](https://tailwindcss.com/docs/installation/using-vite)



# Gestion de la webcam

## Installation d'OpenCV en Python
```bash
pip3 install opencv-python
```





# Installation Windows fraîche
WSL ne permet pas d'utiliser une Webcam, il faut programmer du python directement dans Windows.

J'ai refait une installation fraîche de Windows, voici tout ce que j'ai du installer dans l'ordre pour que ça fonctionne :
```bash
wsl --install
sudo apt install python3-venv
pip install opencv-python
```
À partir de là, la webcam ne semblait pas fonctionner à travers WSL. J'ai donc installé usbipd-win pour pouvoir accorder l'accès à mon port USB à WSL (je n'ai pas eu ce soucis sur MacOS).

https://github.com/dorssel/usbipd-win

```bash
winget install usbipd (dans CMD Windows)


usbipd list
usbipd bind --busid=<BUSID>
usbipd attach --wsl --busid=X-X
```

J'ai ensuite utilisé lsusb de usb-utils pour afficher la liste des périphériques USB connectés