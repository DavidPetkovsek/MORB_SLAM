import numpy as np
import re
import matplotlib.pyplot as plt

pat_acc = re.compile(r'^accel[ \-0-9].*$')
pat_bias = re.compile(r'^bias.*$')
with open('data.txt') as f:
    lines = f.readlines()
    
# data_acc = np.array([[float(y) for y in re.sub(' +', ' ',x[5:].strip()).split(' ')] for x in lines if pat_acc.match(x)])
data_bias = np.array([[float(y) for y in re.sub(' +', ' ',x[4:].strip()).split(' ')[:3]] for x in lines if pat_bias.match(x)])

# data_mag_acc_bias = np.sqrt(data_bias[:,0]**2 + data_bias[:,1]**2 + data_bias[:,2]**2)
# data_mag_acc_imu = np.sqrt(data_acc[:,0]**2 + data_acc[:,1]**2 + data_acc[:,2]**2)

# diff = np.subtract(data_bias, data_acc)
# mag = np.sqrt(diff[:,0]**2 + diff[:,1]**2 + diff[:,2]**2)

# data_mag_real_acc_imu = np.sqrt(data_real_acc_imu[:,0]**2 + data_real_acc_imu[:,1]**2 + data_real_acc_imu[:,2]**2)

### FOR LOST DETECTION TESTING
# last_val = 0
# longest_streak = 0
# longest_streak_index = 0
# active_streak = 0
# index = -1
# threshold = 0.01
# for i in data_bias[:,2]:
#     if i*last_val > 0:
#         if abs(i) > threshold:
#             active_streak += 1
#     else:
#         if active_streak > longest_streak:
#             longest_streak = active_streak
#             longest_streak_index = index
#         if active_streak >= 10:
#             print("Streak of", active_streak, "@ index", index)
#         active_streak = 0
        
#     last_val = i
#     index += 1
# print("Longest Streak is", longest_streak, "@ index", longest_streak_index)

# positions = np.zeros(len(data_bias[:,2]))
# positions[0] = data_bias[0,2]
# for i in range(1,len(data_bias[:,2])):
#     positions[i] = positions[i-1] + data_bias[i,2]
plt.plot(data_bias[:,:2])

# smooth_x = np.zeros(len(data_bias[:,0]))
# smooth_x[0] = data_bias[0,0]
# smooth_x[1] = data_bias[1,0]
# for i in range(2,len(smooth_x)):
#     smooth_x[i] = (data_bias[i,0] + smooth_x[i-1] + smooth_x[i-2])/3

# # plt.plot(data_acc)
# plt.plot(smooth_x)
# plt.plot(positions)


# plt.plot(diff)
# plt.plot(data_mag_acc_imu)
# plt.legend(["bias_mag", "acc_mag"])
plt.legend(["x", "y"])
# plt.legend(["ax", "ay", "az", "bax","bay","baz"])
plt.show()