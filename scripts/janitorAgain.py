import numpy as np
import re
import matplotlib.pyplot as plt

pat_acc = re.compile(r'^accel[ \-0-9].*$')
pat_bias = re.compile(r'^bias.*$')
with open('data.txt') as f:
    lines = f.readlines()
    
data_acc = np.array([[float(y) for y in re.sub(' +', ' ',x[5:].strip()).split(' ')] for x in lines if pat_acc.match(x)])
# data_bias = np.array([[float(y) for y in re.sub(' +', ' ',x[4:].strip()).split(' ')[:3]] for x in lines if pat_bias.match(x)])

# data_mag_acc_bias = np.sqrt(data_bias[:,0]**2 + data_bias[:,1]**2 + data_bias[:,2]**2)
# data_mag_acc_imu = np.sqrt(data_acc[:,0]**2 + data_acc[:,1]**2 + data_acc[:,2]**2)

# data_real_acc_imu = np.add(data_acc, data_bias)
# data_mag_real_acc_imu = np.sqrt(data_real_acc_imu[:,0]**2 + data_real_acc_imu[:,1]**2 + data_real_acc_imu[:,2]**2)

plt.plot(data_acc)
# plt.plot(data_mag_real_acc_imu)
# plt.plot(data_mag_acc_imu)
# plt.legend(["bias_mag", "acc_mag"])
plt.legend(["x", "y", "z"])
# plt.legend(["ax", "ay", "az", "bax","bay","baz"])
plt.show()