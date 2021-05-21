# import numpy
import numpy as np
import matplotlib.pyplot as plt
# Using uniform() method
gfg = np.random.uniform(2.1, 5.5, 1000)
plt.hist(gfg, bins = 100, density = True)
plt.show()