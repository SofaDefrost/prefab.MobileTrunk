import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import make_interp_spline
from scipy import signal

# Lecture du fichier de données
with open("/home/fabrice/Documents/CRIStAL/prefab.MobileTrunk/test/join_states_data_processing/record_join_states/read_digital_twin_joint_states.txt", "r") as f:
    lines = f.readlines()

summit_xl_joint_state_wheels_dic= {"vitesse 2" :"digital_twin_front_right_wheel",
                                   "vitesse 1" :" digital_twin_front_left_wheel ", 
                                   "vitesse 3" :" digital_twin_back_left_wheel ", 
                                   "vitesse 4" :"digital_twin_back_right_wheel "}

temps = []
vitesses = [[] for _ in range(4)]

for line in lines:
    elements = line.strip().split(',')
    temps.append(float(elements[0]))
    for i in range(4):
        vitesses[i].append(float(elements[i+1].strip("[]' ")))

# trier les valeurs de temps
sorted_temps = np.sort(temps)



# Supprimer les doublons dans temps et extraire les indices uniques
unique_indices = np.unique(temps, return_index=True)[1]

# Interpolation avec splines cubiques
xnew = np.linspace(min(temps), max(temps), num=500)
vitesses_smooth = []
for i in range(4):
    # Extraire les valeurs correspondantes dans vitesses[i]
    unique_vitesses = np.array(vitesses[i])[unique_indices]
    # Créer le spline avec les temps uniques et les vitesses correspondantes
    spl = make_interp_spline(np.array(temps)[unique_indices], unique_vitesses, k=3)
    vitesses_smooth.append(spl(xnew))

# Filtrage passe-bas avec la fréquence de coupure modifiée
fc = 0.3  # fréquence de coupure initiale
n = len(xnew) # nombre total de points de données
duree = xnew[-1] - xnew[0] # durée totale de l'enregistrement
fs = n / duree # fréquence d'échantillonnage
fc_modifiee = 1 # nouvelle fréquence de coupure
w = fc_modifiee / (fs / 2) # normalisation de la nouvelle fréquence de coupure
b, a = signal.butter(1, w, 'lowpass')
vitesses_filtre = []
for i in range(4):
    vitesses_filtre.append(signal.filtfilt(b, a, vitesses_smooth[i]))

# # Plot des données brutes et des courbes interpolées
# for i in range(4):
#     fig, ax = plt.subplots()
#     ax.plot(temps, vitesses[i], label=f"Vitesse {i+1} - brutes")
#     #ax.plot(xnew, vitesses_smooth[i], label=f"Vitesse {i+1} - interp")
#     ax.plot(xnew, vitesses_filtre[i], label=f"Vitesse {i+1} - filtre")
#     ax.set_xlabel("Temps (s)")
#     ax.set_ylabel(f"Vitesse de rotation(rad/s)")
#     ax.text(0,4,"pour une vitesse de 0.3 m/s et une vitesse de rotation de 90°")
#     ax.legend()
#     ax.grid()
# plt.show()
