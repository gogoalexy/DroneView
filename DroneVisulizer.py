import csv

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

with open("log_65_2020-2-11-17-22-24_vehicle_local_position_0.csv", 'r', newline='') as log:
    reader = csv.DictReader(log, delimiter=',')
    #row1 = next(reader)
    #prvstime = int(row1['timestamp'])
    #prvs = np.array([float(row1['x']), float(row1['y']), float(row1['z'])])
    timestep = []
    path = []
    num=0
    for row in reader:
        currtime = int(row['timestamp'])
        curr = np.array([float(row['x']), float(row['y']), float(row['z'])])
        #dt = currtime - prvstime
        #vector = curr - prvs
        #timestep.append(dt)
        path.append(curr)

        #prvstime = currtime
        #prvs = curr
        num += 1
        if num == 100:
            #break
            pass

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot([i[0] for i in path], [i[1] for i in path], [i[2] for i in path], 'green')
    plt.show()
