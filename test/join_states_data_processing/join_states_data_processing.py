import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import make_interp_spline
from scipy import signal

with open("/home/fabrice/Documents/CRIStAL/prefab.MobileTrunk/test/join_states_data_processing/record_join_states/read_digital_twin_joint_states_0.5_m_par_s.txt", "r") as f:
    digital_twin_lines = f.readlines()


with open("/home/fabrice/Documents/CRIStAL/prefab.MobileTrunk/test/join_states_data_processing/record_join_states/read_summit_xl_joint_states_0.5_m_par_s.txt", "r") as f:
    summit_xl_lines = f.readlines()

summit_xl_dt = []
summit_xl_speed = [[] for _ in range(4)]

digital_twin_dt = []
digital_twin_speed= [[] for _ in range(4)]




for line in  summit_xl_lines:
    elements = line.strip().split(',')
    summit_xl_dt.append(float(elements[0]))
    for i in range(4):
        summit_xl_speed[i].append(float(elements[i+1].strip("[]' ")))

for line in digital_twin_lines:
    elements = line.strip().split(',')
    digital_twin_dt.append(float(elements[0]))
    for i in range(4):
        digital_twin_speed[i].append(float(elements[i+1].strip("[]' ")))  



# # trier les valeurs de temps du jumeau numérique
# digital_twin_sorted_dt = np.sort(digital_twin_dt)
# # trier les valeurs de temps robot réel
# summit_xl_sorted_dt = np.sort(summit_xl_dt)

# # Supprimer les doublons dans temps et extraire les indices uniques
# digital_twin_unique_indices = np.unique(digital_twin_dt, return_index=True)[1]
# summit_xl_unique_indices = np.unique(summit_xl_dt, return_index=True)[1]


# # Interpolation avec splines cubiques

# # Pour le jumeau numérique
# digital_twin_xnew = np.linspace(min(digital_twin_dt), max(digital_twin_dt), num=500)
# digital_twin_smooth_speed = []
# for i in range(4):
#     # Extraire les valeurs correspondantes dans vitesses[i]
#     digital_twin_unique_speed = np.array(digital_twin_speed[i])[digital_twin_unique_indices]
#     # Créer le spline avec les temps uniques et les vitesses correspondantes
#     spl = make_interp_spline(np.array(digital_twin_dt)[digital_twin_unique_indices], digital_twin_unique_speed, k=3)
#     digital_twin_smooth_speed.append(spl(digital_twin_xnew))


# # Pour le robot réel
# summit_xl_xnew = np.linspace(min(summit_xl_dt), max(summit_xl_dt), num=500)
# summit_xl_smooth_speed = []
# for i in range(4):
#     # Extraire les valeurs correspondantes dans vitesses[i]
#     summit_xl_unique_speed = np.array(summit_xl_speed[i])[summit_xl_unique_indices]
#     # Créer le spline avec les temps uniques et les vitesses correspondantes
#     spl = make_interp_spline(np.array(summit_xl_dt)[summit_xl_unique_indices], summit_xl_unique_speed, k=3)
#     summit_xl_smooth_speed.append(spl(summit_xl_xnew))


# #Pour le jumeau numérique
# # Filtrage passe-bas avec la fréquence de coupure modifiée
# fc = 0.3  # fréquence de coupure initiale
# n = len(digital_twin_xnew) # nombre total de points de données
# duree = digital_twin_xnew[-1] - digital_twin_xnew[0] # durée totale de l'enregistrement
# fs = n / duree # fréquence d'échantillonnage
# fc_modifiee = 1# nouvelle fréquence de coupure
# w = fc_modifiee / (fs / 2) # normalisation de la nouvelle fréquence de coupure
# b, a = signal.butter(1, w, 'lowpass')
# digital_twin_xnew_filtred_speed = []
# for i in range(4):
#     digital_twin_xnew_filtred_speed.append(signal.filtfilt(b, a, digital_twin_smooth_speed[i]))

# #Pour le summit_xl
# #Pour le jumeau numérique
# # Filtrage passe-bas avec la fréquence de coupure modifiée
# fc = 1  # fréquence de coupure initiale
# n = len(summit_xl_xnew) # nombre total de points de données
# duree = summit_xl_xnew[-1] - summit_xl_xnew[0] # durée totale de l'enregistrement
# fs = n / duree # fréquence d'échantillonnage
# fc_modifiee = 1 # nouvelle fréquence de coupure
# w = fc_modifiee/ (fs / 2) # normalisation de la nouvelle fréquence de coupure
# b, a = signal.butter(1, w, 'lowpass')
# summit_xl_xnew_filtred_speed = []
# for i in range(4):
#     summit_xl_xnew_filtred_speed.append(signal.filtfilt(b, a, summit_xl_smooth_speed[i]))

# Plot des données 
for i in range(4):
    fig, ax = plt.subplots()
    ax.plot(digital_twin_dt, digital_twin_speed[i],  label=f"Vitesse {i+1}")
    #ax.plot(digital_twin_xnew , digital_twin_xnew_filtred_speed[i], label=f"digital_twin angular_speed{i+1} - filtre")
    ax.plot(summit_xl_dt ,  summit_xl_speed[i], label=f"Summit_xl angular_speed{i+1}")
    ax.set_xlabel("Temps (s)")
    ax.set_ylabel(f"Vitesse de rotation(rad/s)")
    #ax.text(0,4,"pour une vitesse de 0.3 m/s et une vitesse de rotation de 90°")
    ax.legend()
    ax.grid()
plt.show()
