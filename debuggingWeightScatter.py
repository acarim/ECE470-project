import matplotlib.pyplot as plt
import numpy as np
fig, ax = plt.subplots()
ax.scatter(self.particles[:,0], self.particles[:,1], c = np.ravel(self.weights), cmap='gray', label='Particles')
plt.show()