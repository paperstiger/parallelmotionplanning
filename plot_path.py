import numpy as np
import matplotlib.pyplot as plt

mat = np.loadtxt('path.txt')
diff = mat[1:] - mat[:-1]
dist = np.sum(np.linalg.norm(diff, axis=1))
print('length  = ', dist)
fig, ax = plt.subplots()
ax.plot(mat[:, 0], mat[:, 1])
ax.plot([0.5, 0.5], [0.2, 0.8], color='k')
plt.show()