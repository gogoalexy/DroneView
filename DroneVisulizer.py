import argparse
import csv
import os

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import pyulog

parser = argparse.ArgumentParser()
parser.add_argument("input")
parser.add_argument("-o", "--output", action="store_true")
args = parser.parse_args()

infile = args.input
infileName = os.path.splitext(infile)[0]

def load_log_file(filename):
    useful_messages = ['vehicle_attitude', 'vehicle_local_position']
    try:
        log = pyulog.ULog(filename, message_name_filter_list=useful_messages)
    except FileNotFoundError:
        print(f"File {filename} does not exist.")
    except:
        print("Error while reading or parsing the ulog file.")
    else:
        return log
    
def calculate_roll_yaw_pitch(ulog_object):
    px4log = pyulog.px4.PX4ULog(ulog_object)
    px4log.add_roll_pitch_yaw()
    
def cumulate_rieman_sum(x, y, x0=0., y0=0.):
    xi_1, y_cumulate = x0, y0
    for xi, yi in zip(x, y):
        y_cumulate = y_cumulate + (xi - xi_1)*yi
        xi_1 = xi
        yield y_cumulate
    
drone_log = load_log_file(infile)
calculate_roll_yaw_pitch(drone_log)

attitude = drone_log.get_dataset('vehicle_attitude')
time = attitude.data['timestamp']
roll = attitude.data['roll']
yaw = attitude.data['yaw']
pitch = attitude.data['pitch']

local_position = drone_log.get_dataset('vehicle_local_position')
time_seq = np.array( [ microsec/1e6 for microsec in local_position.data['timestamp'] ] )
horizontal_std = np.array(local_position.data['eph'])
vertical_std = np.array(local_position.data['epv'])
horizontal_velocity_std = np.array(local_position.data['evh'])
virtical_velocity_std = np.array(local_position.data['evv'])
x = np.array(local_position.data['x'])
y = np.array(local_position.data['y'])
z = np.array(local_position.data['z'])
vx = np.array(local_position.data['vx'])
vy = np.array(local_position.data['vy'])
vz = np.array(local_position.data['vz'])

if args.output:
    with open(f"{infileName}_basic.csv", 'w', newline='') as csvfile:
        writer = csv.writer(csvfile, delimiter=',', quoting=csv.QUOTE_MINIMAL)
        writer.writerow(["t", "x", "y", "z", "vx", "vy", "vz", "eph", "epv", "evh", "evv"])
        for ti, xi, yi, zi, vxi, vyi, vzi, ephi, epvi, evhi, evvi in zip(time_seq, x, y, z, vx, vy, vz, horizontal_std, vertical_std, horizontal_velocity_std, virtical_velocity_std):
            writer.writerow([ti, xi, yi, zi, vxi, vyi, vzi, ephi, epvi, evhi, evvi])
    
sum_x = list(cumulate_rieman_sum(time_seq, vx, time_seq[0], x[0]))
sum_y = list(cumulate_rieman_sum(time_seq, vy, time_seq[0], y[0]))
sum_z = list(cumulate_rieman_sum(time_seq, vz, time_seq[0], z[0]))

fig = plt.figure(0)
ax = fig.add_subplot(111, projection='3d')
ax.invert_zaxis()
ax.plot(x, y, z, 'green')
ax.plot([x[0]], [y[0]], [z[0]], 'ro')
ax.plot([x[-1]], [y[-1]], [z[-1]], 'bo')
fig, axs = plt.subplots(3, sharex=True)
axs[0].plot(time_seq, x, color='g')
axs[0].fill_between(time_seq, x-horizontal_std, x+horizontal_std, color='y', alpha=0.3)
axs[0].plot(time_seq, sum_x, color='c')
axs[1].plot(time_seq, y, color='g')
axs[1].fill_between(time_seq, y-horizontal_std, y+horizontal_std, color='y', alpha=0.3)
axs[1].plot(time_seq, sum_y, color='c')
axs[2].invert_yaxis()
axs[2].plot(time_seq, z, color='g')
axs[2].fill_between(time_seq, z-vertical_std, z+vertical_std, color='y', alpha=0.3)
axs[2].plot(time_seq, sum_z, color='c')
fig, axs = plt.subplots(3, sharex=True)
axs[0].plot(time_seq, vx)
axs[0].fill_between(time_seq, vx-horizontal_velocity_std, vx+horizontal_velocity_std, color='r', alpha=0.3)
axs[1].plot(time_seq, vy)
axs[1].fill_between(time_seq, vy-horizontal_velocity_std, vy+horizontal_velocity_std, color='r', alpha=0.3)
axs[2].invert_yaxis()
axs[2].plot(time_seq, vz)
axs[2].fill_between(time_seq, vz-virtical_velocity_std, vz+virtical_velocity_std, color='r', alpha=0.3)
fig, axs = plt.subplots(3, sharex=True)
axs[0].plot(time, roll, color='k')
axs[1].plot(time, yaw, color='k')
axs[2].plot(time, pitch, color='k')
plt.show()
