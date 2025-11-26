import serial
import matplotlib.pyplot as plt
import time
import csv
import re
from collections import deque

# ---------------------------
# CONFIGURATION
# ---------------------------
PORT = "COM8"         # <-- Mets ton port Arduino
BAUD = 115200
CSV_FILE = "sht41_angles_log.csv"

WINDOW = 300  # nombre de points visibles en temps réel

# ---------------------------
# REGEX pour extraire les données
# ---------------------------
pattern = re.compile(r"([A-Za-z0-9]+):([-+]?[0-9]*\.?[0-9]+)")

# Les capteurs qu'on attend
expected_fields = [
    "T0","H0","T1","H1","T2","H2","T3","H3","Tenv","Henv",
    "A0","A1","A2","A3", 
    "scale"   # anglesScaled des 4 servos
]

# ---------------------------
# INITIALISATION
# ---------------------------
ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(2)

# buffers
timestamps = deque(maxlen=WINDOW)
T = [deque(maxlen=WINDOW) for _ in range(5)]
H = [deque(maxlen=WINDOW) for _ in range(5)]
A = [deque(maxlen=WINDOW) for _ in range(4)]
SCALE = deque(maxlen=WINDOW)

# création du CSV
with open(CSV_FILE, "a", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["time"] + expected_fields)

# ---------------------------
# GRAPHIQUES
# ---------------------------
plt.ion()
fig, axs = plt.subplots(4, 1, figsize=(11,10))

# Températures
lines_T = [axs[0].plot([], [], label=f"T{i}")[0] for i in range(5)]
axs[0].set_title("Températures SHT41")
axs[0].set_ylabel("Temp (°C)")
axs[0].legend()

# Humidité
lines_H = [axs[1].plot([], [], label=f"H{i}")[0] for i in range(5)]
axs[1].set_title("Humidité SHT41")
axs[1].set_ylabel("Humidité (%)")
axs[1].legend()

# Angles servo
lines_A = [axs[2].plot([], [], label=f"AllAngleScaled{i}")[0] for i in range(4)]
axs[2].set_title("Angle des servos (angleScaled)")
axs[2].set_ylabel("Angle (°)")
axs[2].set_xlabel("Temps (s)")
axs[2].legend()

line_scale, = axs[3].plot([], [], label="scale")
axs[3].set_title("Facteur externe scale")
axs[3].set_ylabel("scale")
axs[3].set_xlabel("Temps (s)")
axs[3].legend()
# ---------------------------
# BOUCLE PRINCIPALE
# ---------------------------
start = time.time()

while True:
    raw = ser.readline().decode(errors="ignore").strip()
    if not raw:
        continue

    matches = dict(pattern.findall(raw))

    # Vérif qu’on a *au moins* T0 et A0
    if len(matches) == 0:
        continue

    # Génère un tableau ordonné de données
    try:
        vals = [float(matches.get(field, "nan")) for field in expected_fields]
    except:
        continue

    t = time.time() - start

    # Sauvegarde CSV
    with open(CSV_FILE, "a", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([t] + vals)

    # Push dans les graphiques
    timestamps.append(t)

    for i in range(5):
        T[i].append(vals[2*i])        # T0,T1,T2,T3,Tenv
        H[i].append(vals[2*i+1])      # H0,H1,H2,H3,Henv

    for i in range(4):
        A[i].append(160-vals[10 + i])     # A0,A1,A2,A3
    
    scale_value = vals[-1]
    SCALE.append(scale_value)

    # Mise à jour des courbes
    for i in range(5):
        lines_T[i].set_xdata(timestamps)
        lines_T[i].set_ydata(T[i])

        lines_H[i].set_xdata(timestamps)
        lines_H[i].set_ydata(H[i])

    for i in range(4):
        lines_A[i].set_xdata(timestamps)
        lines_A[i].set_ydata(A[i])
    
    line_scale.set_xdata(timestamps)
    line_scale.set_ydata(SCALE)

    # autoscale
    for ax in axs:
        ax.relim()
        ax.autoscale_view()

    plt.pause(0.01)