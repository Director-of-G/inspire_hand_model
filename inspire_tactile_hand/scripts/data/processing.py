from matplotlib import pyplot as plt
import numpy as np

FINGER = "4-1"

finger_to_actuate_index_map = {
    "0": 1,
    "6": 1,
    "7": 1,
    "8": 1,
    "4-0": 1,
    "4-1": 2,
}

data = np.load("./finger{}.npy".format(FINGER))

index_actuate = finger_to_actuate_index_map[FINGER]
data_active = data[:, index_actuate]
data_passive = np.delete(data, index_actuate, axis=1)

for i in range(data_passive.shape[1]):
    plt.plot(data_active, data_passive[:, i], label="passive{}".format(i))

plt.legend()
plt.show()
