# Arduino Sonar Turret – 360° Ultrasonic Surveillance :

![Project Banner](docs/banner_placeholder.png)

## Overview

The **Sonar Turret** is a **360° real-time ultrasonic surveillance system** built with:
- **Arduino UNO + HC-SR04 ultrasonic sensors (x3)**
- **DHT22 sensor** (for environmental calibration)
- **Python backend** for data processing, object tracking, clustering, and visualization

## Key Features :

- Dynamic **speed of sound correction** using temperature + humidity  
- **Object clustering & tracking** (velocity, direction) with DBSCAN  
- **Real-time visualization** (2D or 3D) using Matplotlib  
- Modular design: Arduino firmware + Python data pipeline  


## Setup :

###Hardware Requirements :
- Arduino UNO (or compatible board)  
- 3x HC-SR04 Ultrasonic Sensors  
- DHT22 Temperature & Humidity Sensor  
- PC (Windows/Linux/Mac) with Python 3.8+ installed  

### Software Requirements :
- Arduino IDE  
- Python 3.8+ with dependencies  

### Install Python dependencies :
pip install -r python/requirements.txt

### Upload Arduino Code :
- Open arduino/sonar_turret.ino in Arduino IDE
-Select correct Board & Port
- Upload (ctrl + r)
- Run the turret from terminal/command prompt:

### python run_turret.py --port COMXX --baud 115200 --viz 3D


## Flowchart LR :

    A[Arduino UNO] -->|JSON via Serial| B[Python Processing]
    B --> C[Object Tracking + Clustering]
    C --> D[Matplotlib Visualization (2D/3D)]

