#!/usr/bin/env python3

import time
import pybullet as p

input_stl = "/home/gauss/projects/personal_ws/src/birmingham_cell/birmingham_cell_description/meshes/container1.STL"   # Percorso al tuo modello OBJ
output_convex_obj = "/home/gauss/projects/personal_ws/src/birmingham_cell/birmingham_cell_description/meshes/container1_convex_model.obj"  # Percorso di output per il modello convesso
log_file = "/home/gauss/projects/personal_ws/src/birmingham_cell/birmingham_cell_description/meshes/vhacd_log.txt"  # File di log per il processo VHACD
urdf_path = "/home/gauss/projects/personal_ws/src/birmingham_cell/birmingham_cell_description/urdf/container1.xacro"
# Parametri per la decomposizione
params = {
    'concavity': 0.0025,
    'alpha': 0.04,
    'beta': 0.05,
    'resolution': 100000,
    'depth': 20,
    'planeDownsampling': 4,
    'convexhullDownsampling': 4,
    'maxNumVerticesPerCH': 64,
    'minVolumePerCH': 0.0001
}

# Esecuzione di VHACD
p.vhacd(input_stl, output_convex_obj, log_file)

exit()
# Connessione a PyBullet
physicsClient = p.connect(p.GUI)

# Caricamento del modello convesso
model_id = p.loadURDF(output_convex_obj)

num = 0
while num < 60:
    num += 1
    time.sleep(1)

p.disconnect()