import numpy as np
import re
import matplotlib.pyplot as plt

pat_acc = re.compile(r'^accel[ \-0-9].*$')
pat_bias = re.compile(r'^bias.*$')
with open('data.txt') as f:
    lines = f.readlines()
    
# data_acc = np.array([[float(y) for y in re.sub(' +', ' ',x[5:].strip()).split(' ')] for x in lines if pat_acc.match(x)])
data_bias = np.array([[float(y) for y in re.sub(' +', ' ',x[4:].strip()).split(' ')[:3]] for x in lines if pat_bias.match(x)])

plt.plot(data_bias)

plt.legend(["x", "y", "z"])
plt.show()