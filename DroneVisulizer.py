import argparse
import csv
import os

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import pyulog

from helper import ULogHelper
from configurations import flight_modes_table, arming_status_table
from Diagnosis import DiagnoseFailure
from tracker import PositionTracker

parser = argparse.ArgumentParser()
parser.add_argument("input")
parser.add_argument("-o", "--output", action="store_true")
parser.add_argument("-0", "--start_from_zero", action="store_true")
args = parser.parse_args()

infile = args.input
infileName = os.path.splitext(infile)[0]

def load_log_file(filename):
    useful_messages = ['estimator_status', 'vehicle_attitude', 'vehicle_local_position', 'vehicle_status']
    try:
        ulog = pyulog.ULog(filename, message_name_filter_list=useful_messages)
    except FileNotFoundError:
        print(f"File {filename} does not exist.")
    except:
        print("Error while reading or parsing the ulog file.")
    else:
        return ulog

def getCatagory(ulog, message):
    catagory = ulog.get_dataset(message)
    return catagory

def getColumns(catogory, column_names, column_alias=None, time_in_second=False):
    if column_alias:
        data_collection = {alias: catogory.data[name] for name, alias in zip(column_names, column_alias)}
    else:
        data_collection = {name: catogory.data[name] for name in column_names}
    if time_in_second:
        time_key = 'timestamp' if 'timestamp' in data_collection else 'time'
        normalized_time = [time/1e6 for time in data_collection[time_key]]
        data_collection[time_key] = normalized_time
    return data_collection

def calculate_roll_yaw_pitch(ulog):
    px4log = pyulog.px4.PX4ULog(ulog)
    px4log.add_roll_pitch_yaw()

def get_flight_modes(ulog):
    vehicle_status = ulog.get_dataset('vehicle_status')
    modes = vehicle_status.list_value_changes('nav_state')
    for element in modes:
        time, mode = element
        if mode in flight_modes_table:
            mode_name = flight_modes_table[mode]
            yield (time, mode_name)

def get_arming_states(ulog):
    vehicle_status = ulog.get_dataset('vehicle_status')
    arming = vehicle_status.list_value_changes('arming_state')
    for element in arming:
        time, state = element
        if state in arming_status_table:
            status_name = arming_status_table[state]
            yield (time, status_name)

def cumulate_rieman_sum(x, y, x0=0., y0=0.):
    xi_1, y_cumulate = x0, y0
    for xi, yi in zip(x, y):
        y_cumulate = y_cumulate + (xi - xi_1)*yi
        xi_1 = xi
        yield y_cumulate

drone_log = load_log_file(infile)
calculate_roll_yaw_pitch(drone_log)
failuredetect = DiagnoseFailure(drone_log)
pos = PositionTracker(drone_log)
print(list(get_flight_modes(drone_log)))
print(list(get_arming_states(drone_log)))

vehicle_status = getCatagory(drone_log, 'vehicle_status')
vehicle_attitude = getCatagory(drone_log, 'vehicle_attitude')
estimator_status = getCatagory(drone_log, 'estimator_status')

attitude = getColumns(vehicle_attitude, ['timestamp', 'roll', 'pitch', 'yaw'], ['time', 'roll', 'pitch', 'yaw'], time_in_second=True)
deg_attitude = {key: np.rad2deg(values) for key, values in attitude.items()}
deg_attitude['time'] = attitude['time']
vibration = getColumns(estimator_status, ['timestamp', 'vibe[0]', 'vibe[1]', 'vibe[2]'], ['time', 'GyroDeltaAngleConing', 'GyroHighFreq', 'AccelHighFreq'], time_in_second=True)
watchdog = getColumns(estimator_status, ['timestamp', 'innovation_check_flags'], time_in_second=True)
failure = getColumns(vehicle_status, ['timestamp', 'failure_detector_status'], time_in_second=True)

if args.start_from_zero:
    time_seq = np.array( pos.normalized_time )
else:
    time_seq = np.array( pos.time )
horizontal_std = np.array(pos.std_horizontal_pos_err)
vertical_std = np.array(pos.std_vertical_pos_err)
horizontal_velocity_std = np.array(pos.std_horizontal_vel_err)
virtical_velocity_std = np.array(pos.std_vertical_vel_err)
x = np.array(pos.x)
y = np.array(pos.y)
z = np.array(pos.z)
vx = np.array(pos.vx)
vy = np.array(pos.vy)
vz = np.array(pos.vz)

if args.output:
    with open(f"{infileName}_velocity.csv", 'w', newline='') as csvfile:
        writer = csv.writer(csvfile, delimiter=',', quoting=csv.QUOTE_MINIMAL)
        writer.writerow(["t", "vx", "vy", "vz"])
        for ti, vxi, vyi, vzi in zip(time_seq, vx, vy, vz):
            writer.writerow([ti, vxi, vyi, vzi])

sum_x = list(cumulate_rieman_sum(time_seq, vx, time_seq[0], x[0]))
sum_y = list(cumulate_rieman_sum(time_seq, vy, time_seq[0], y[0]))
sum_z = list(cumulate_rieman_sum(time_seq, vz, time_seq[0], z[0]))

fig = plt.figure(0)
ax = fig.add_subplot(111, projection='3d')
ax.invert_zaxis()
ax.plot(x, y, z, 'green')
ax.plot([x[0]], [y[0]], [z[0]], 'bo', label="start")
ax.plot([x[-1]], [y[-1]], [z[-1]], 'ro', label="end")
ax.legend()
fig, axs = plt.subplots(3, sharex=True)
fig.suptitle("Local Position")
axs[0].set_title("x")
axs[0].plot(time_seq, x, color='g')
axs[0].fill_between(time_seq, x-horizontal_std, x+horizontal_std, color='y', alpha=0.3)
#axs[0].plot(time_seq, sum_x, color='c')
axs[1].set_title("y")
axs[1].plot(time_seq, y, color='g')
axs[1].fill_between(time_seq, y-horizontal_std, y+horizontal_std, color='y', alpha=0.3)
axs[1].set_ylabel("distance to reference point (m)")
#axs[1].plot(time_seq, sum_y, color='c')
axs[2].set_title("z")
axs[2].invert_yaxis()
axs[2].plot(time_seq, z, color='g')
axs[2].fill_between(time_seq, z-vertical_std, z+vertical_std, color='y', alpha=0.3)
#axs[2].plot(time_seq, sum_z, color='c')
axs[2].set_xlabel("time (s)")
fig, axs = plt.subplots(3, sharex=True)
fig.suptitle("Local Velocity")
axs[0].set_title("x")
axs[0].plot(time_seq, vx)
axs[0].fill_between(time_seq, vx-horizontal_velocity_std, vx+horizontal_velocity_std, color='r', alpha=0.3)
axs[1].set_title("y")
axs[1].plot(time_seq, vy)
axs[1].fill_between(time_seq, vy-horizontal_velocity_std, vy+horizontal_velocity_std, color='r', alpha=0.3)
axs[1].set_ylabel("velocity (m/s)")
axs[2].set_title("z")
axs[2].invert_yaxis()
axs[2].plot(time_seq, vz)
axs[2].fill_between(time_seq, vz-virtical_velocity_std, vz+virtical_velocity_std, color='r', alpha=0.3)
axs[2].set_xlabel("time (s)")
fig = plt.figure(4)
plt.plot(attitude['time'], deg_attitude['roll'], color='r')
plt.plot(attitude['time'], deg_attitude['pitch'], color='g')
plt.plot(attitude['time'], deg_attitude['yaw'], color='b')
fig = plt.figure(5)
plt.plot(vibration['time'], vibration['GyroDeltaAngleConing'], color='c')
plt.plot(vibration['time'], vibration['GyroHighFreq'], color='m')
plt.plot(vibration['time'], vibration['AccelHighFreq'], color='y')
fig = plt.figure(6)
plt.plot(watchdog['timestamp'], watchdog['innovation_check_flags'], color='k')
plt.plot(failure['timestamp'], failure['failure_detector_status'], color='r')
diagnose = failuredetect.change_diagnose(watchdog['timestamp'], watchdog['innovation_check_flags'], 'innovation_check_flags')
diagnose1 = failuredetect.change_diagnose(failure['timestamp'], failure['failure_detector_status'], 'failure_detector_status')
for item, event in enumerate(diagnose):
    for index, label in enumerate(event[1]):
        plt.text(event[0], item*5+index*20+5, label)
for item, event in enumerate(diagnose1):
    for index, label in enumerate(event[1]):
        plt.text(event[0], item*5+index*20+10, label)
plt.show()
