# 🛡️ Sentinel
### Système de surveillance périmétrique intelligente basé sur ESP32-S3-CAM et Intelligence Artificielle

Sentinel est un système embarqué de surveillance périmétrique développé dans le cadre d'un projet de Licence en Électronique et Informatique Industrielle (EII).

Le système combine des capteurs de mouvement, une caméra intelligente et un modèle d'intelligence artificielle afin de détecter, analyser et confirmer automatiquement une intrusion humaine ou un drone avant de déclencher une alerte.

---

# Objectifs

- Détection fiable des intrusions
- Réduction des fausses alertes
- Analyse d'images embarquée
- Fonctionnement autonome
- Notification distante via Blynk
- Faible coût de réalisation

---

# Architecture du système

Le système utilise :

- ESP32-S3-CAM
- Caméra OV3660
- Radar RCWL-0516
- Capteur PIR HC-SR501
- Deux servomoteurs SG90
- Pointeur laser KY-008
- Buzzer
- Plateforme Blynk
- Modèle IA MobileNetV1 entraîné avec Edge Impulse

---

# Fonctionnement

1. Initialisation du système
2. Surveillance continue
3. Double détection (Radar + PIR)
4. Capture d'image
5. Analyse par Intelligence Artificielle
6. Confirmation de la menace
7. Déclenchement de l'alerte
8. Retour automatique en surveillance

---

# Intelligence Artificielle

Le modèle embarqué a été entraîné avec **Edge Impulse**.

Classification :

- Humain
- Drone
- Négatif

Architecture :

- MobileNetV1
- Image : 96 × 96 pixels
- Quantification INT8

---

# Outils utilisés

- Arduino IDE
- Edge Impulse
- Blynk IoT
- Proteus
- Wokwi
- Fritzing
- Draw.io

---

# Structure du dépôt

```
Firmware/
Documentation/
Simulation/
PCB/
Dataset/
Images/
```

---

# Auteur

**TONDE Wendbenedo Daniel**

Étudiant en Licence 3

Électronique et Informatique Industrielle

IST Ouaga 2000

Burkina Faso

---

# Licence

Projet académique réalisé dans le cadre d'un mémoire de Licence.
