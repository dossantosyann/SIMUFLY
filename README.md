# Simulateur-de-prise-de-vue-par-drone

# Notes perso
Ce sont les notes que j'ai (Yann) pris lors de mes recherches pour la création du logicielle. Les étapes présentes sont écrites dans l'ordre chonologique où je les ai exécutées

## Sources
[Vidéo YouTube](https://www.youtube.com/watch?v=ctQMqqEo4G8)


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

# Gestion de la webcam

## Installation d'OpenCV en Python
```bash
pip3 install opencv-python
```