import numpy as np

from helper import ULogHelper

class PositionTracker:
    
    def __init__(self, ulog):
        data_parser = ULogHelper(ulog)
        data_parser.extractRequiredMessages(['vehicle_local_position'])
        self.time = data_parser.getTimeSeries('vehicle_local_position')
        self.normalized_time = data_parser.getTimeSeries('vehicle_local_position', start_from_zero=True)
        self.x = data_parser.getMessage('vehicle_local_position', 'x')
        self.y = data_parser.getMessage('vehicle_local_position', 'y')
        self.z = data_parser.getMessage('vehicle_local_position', 'z')
        self.std_horizontal_pos_err = data_parser.getMessage('vehicle_local_position', 'eph')
        self.std_vertical_pos_err = data_parser.getMessage('vehicle_local_position', 'epv')
        self.vx = data_parser.getMessage('vehicle_local_position', 'vx')
        self.vy = data_parser.getMessage('vehicle_local_position', 'vy')
        self.vz = data_parser.getMessage('vehicle_local_position', 'vz')
        self.std_horizontal_vel_err = data_parser.getMessage('vehicle_local_position', 'evh')
        self.std_vertical_vel_err = data_parser.getMessage('vehicle_local_position', 'evv')
        self.ax = data_parser.getMessage('vehicle_local_position', 'ax')
        self.ay = data_parser.getMessage('vehicle_local_position', 'ay')
        self.az = data_parser.getMessage('vehicle_local_position', 'az')
        self.valid_xy = data_parser.getMessage('vehicle_local_position', 'xy_valid')
        self.valid_z = data_parser.getMessage('vehicle_local_position', 'z_valid')
        self.valid_vxvy = data_parser.getMessage('vehicle_local_position', 'v_xy_valid')
        self.valid_vz = data_parser.getMessage('vehicle_local_position', 'v_z_valid')
        
    def positionAtTime(self, timepoint, useNormalizedTime=False):
        time_series = self.normalized_time if useNormalizedTime else self.time
        xp = np.interp(timepoint, time_series, self.x)
        yp = np.interp(timepoint, time_series, self.y)
        zp = np.interp(timepoint, time_series, self.z)
        return (xp, yp, zp)
    
    def velocityAtTime(self, timepoint, useNormalizedTime=False):
        time_series = self.normalized_time if useNormalizedTime else self.time
        vxp = np.interp(timepoint, time_series, self.vx)
        vyp = np.interp(timepoint, time_series, self.vy)
        vzp = np.interp(timepoint, time_series, self.vz)
        return (vxp, vyp, vzp)
        
    def accelerationAtTime(self, timepoint, useNormalizedTime=False):
        time_series = self.normalized_time if useNormalizedTime else self.time
        axp = np.interp(timepoint, time_series, self.ax)
        ayp = np.interp(timepoint, time_series, self.ay)
        azp = np.interp(timepoint, time_series, self.az)
        return (axp, ayp, azp)
        
    def getPositionStdBoundary(self, std=1):
        boundary = {'x_upper': [], 'x_lower': [], 'y_upper': [], 'y_lower': [], 'z_upper': [], 'z_lower': []}
        for x, y, hstd, z, vstd in zip(self.x, self.y, self.std_horizontal_pos_err, self.z, self.std_vertical_pos_err):
            boundary['x_upper'].append(x+hstd*std)
            boundary['x_lower'].append(x-hstd*std)
            boundary['y_upper'].append(y+hstd*std)
            boundary['y_lower'].append(y-hstd*std)
            boundary['z_upper'].append(z+vstd*std)
            boundary['z_lower'].append(z-vstd*std)
        return boundary
        
    def getVelocityStdBoundary(self, std=1):
        boundary = {'x_upper': [], 'x_lower': [], 'y_upper': [], 'y_lower': [], 'z_upper': [], 'z_lower': []}
        for vx, vy, hvstd, vz, vvstd in zip(self.vx, self.vy, self.std_horizontal_vel_err, self.vz, self.std_vertical_vel_err):
            boundary['x_upper'].append(vx+hvstd*std)
            boundary['x_lower'].append(vx-hvstd*std)
            boundary['y_upper'].append(vy+hvstd*std)
            boundary['y_lower'].append(vy-hvstd*std)
            boundary['z_upper'].append(vz+vvstd*std)
            boundary['z_lower'].append(vz-vvstd*std)
        return boundary
