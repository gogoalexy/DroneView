import argparse
import csv
import os
from tempfile import TemporaryDirectory
import time

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

parser = argparse.ArgumentParser()
parser.add_argument("input")
parser.add_argument("-pos", "--poition", action="store_true")
parser.add_argument("-vel", "--velocity", action="store_true")
parser.add_argument("-acc", "--acceleration", action="store_true")
parser.add_argument("-att", "--attitude", action="store_true")
parser.add_argument("-v", "--visualize", action="store_true")
args = parser.parse_args()

infile = args.input
infileName = os.path.splitext(infile)[0]
mode = "position"

with TemporaryDirectory() as csvtmp:
    try:
        os.system(f"ulog2csv -o {csvtmp} {infile}")
        os.system(f"ls {csvtmp}")
    except:
        pass

    with open(f"{csvtmp}/{infileName}_vehicle_local_position_0.csv", 'r', newline='') as log, open(f"{infileName}_{mode}.csv", 'w', newline='') as out:
        reader = csv.DictReader(log, delimiter=',')
        fieldnames = ["time", "x", "y", "z", "horizontal_error", "vertical_error"]
        writer = csv.writer(out, delimiter=',', quoting=csv.QUOTE_MINIMAL)
        writer.writerow(fieldnames)
        timestep = []
        path = []
        for row in reader:
            time = [int(row['timestamp'])]
            pos = [ float(row['x']), float(row['y']), float(row['z']), float(row['eph']), float(row['epv']) ]
            writer.writerow(time + pos)
            path.append(pos)


        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.plot([i[0] for i in path], [i[1] for i in path], [i[2] for i in path], 'green')
        plt.show()
