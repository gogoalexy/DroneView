import argparse
import os
from tempfile import TemporaryDirectory

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import pandas as pd
import pyulog

parser = argparse.ArgumentParser()
parser.add_argument("input")
parser.add_argument("-v", "--visualize", action="store_true")
parser.add_argument("-o", "--output", action="store_true")
args = parser.parse_args()

infile = args.input
infileName = os.path.splitext(infile)[0]

try:
    px4log = pyulog.px4.PX4ULog(pyulog.ULog(infile))
    print(px4log)
except:
    pass

with TemporaryDirectory() as csvtmp:

    os.system(f"ulog2csv -m 'vehicle_attitude,vehicle_local_position' -o {csvtmp} {infile}")

    csvreader = pd.read_csv(f"{csvtmp}/{infileName}_vehicle_local_position_0.csv")
    time_seq = np.array( [ microsec/1e6 for microsec in csvreader['timestamp']] )
    horizontal_std = np.array(csvreader['eph'])
    vertical_std = np.array(csvreader['epv'])
    horizontal_velocity_std = np.array(csvreader['evh'])
    virtical_velocity_std = np.array(csvreader['evv'])
    x = np.array(csvreader['x'])
    y = np.array(csvreader['y'])
    z = np.array(csvreader['z'])
    vx = np.array(csvreader['vx'])
    vy = np.array(csvreader['vy'])
    vz = np.array(csvreader['vz'])

    fig = plt.figure(0)
    ax = fig.add_subplot(111, projection='3d')
    ax.invert_zaxis()
    ax.plot(x, y, z, 'green')
    ax.plot([x[0]], [y[0]], [z[0]], 'ro')
    ax.plot([x[-1]], [y[-1]], [z[-1]], 'bo')
    fig, axs = plt.subplots(3, sharex=True)
    axs[0].plot(time_seq, x, color='g')
    axs[0].fill_between(time_seq, x-horizontal_std, x+horizontal_std, color='y', alpha=0.3)
    axs[1].plot(time_seq, y, color='g')
    axs[1].fill_between(time_seq, y-horizontal_std, y+horizontal_std, color='y', alpha=0.3)
    axs[2].invert_yaxis()
    axs[2].plot(time_seq, z, color='g')
    axs[2].fill_between(time_seq, z-vertical_std, z+vertical_std, color='y', alpha=0.3)
    fig, axs = plt.subplots(3, sharex=True)
    axs[0].plot(time_seq, vx)
    axs[0].fill_between(time_seq, vx-horizontal_velocity_std, vx+horizontal_velocity_std, color='r', alpha=0.3)
    axs[1].plot(time_seq, vy)
    axs[1].fill_between(time_seq, vy-horizontal_velocity_std, vy+horizontal_velocity_std, color='r', alpha=0.3)
    axs[2].invert_yaxis()
    axs[2].plot(time_seq, vz)
    axs[2].fill_between(time_seq, vz-virtical_velocity_std, vz+virtical_velocity_std, color='r', alpha=0.3)
    plt.show()
