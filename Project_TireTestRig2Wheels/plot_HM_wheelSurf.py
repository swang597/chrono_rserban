import numpy as np
import matplotlib.pyplot as plt

data_heightmap = np.loadtxt("/home/swang597/Documents/Research/chrono_fork_rserban/Project_TireTestRig2Wheels/heightmap.txt")
data_wheel_surf = np.loadtxt("/home/swang597/Documents/Research/chrono_fork_rserban/Project_TireTestRig2Wheels/wheel_surface_height.txt")
data_wheel_surf *=  1e-2
fig, ax = plt.subplots(1,2, figsize=(10,5))
ax[0].imshow(data_heightmap, cmap='terrain')
ax[0].set_title("Heightmap")
ax[1].imshow(data_wheel_surf, cmap='terrain')
ax[1].set_title("Wheel surface height")
plt.show()
