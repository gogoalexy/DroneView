from scipy.signal import butter

from helper import ULogHelper

class DiagnoseFailure:

    def __init__(self, ulog):
        data_parser = ULogHelper(ulog)
        data_parser.extractRequiredMessages(['estimator_status', 'vehicle_status'])
    
    def change_diagnose(self, timestamps, flags, flag_type):
        if flag_type == 'innovation_check_flags':
            return self.innovation_check(timestamps, flags)
        elif flag_type == 'failure_detector_status':
            return self.failure_detector(timestamps, flags)
        elif flag_type == 'gps_check_fail_flags':
            return self.gps_check_fail(timestamps, flags)

    def detect_change(rule_check):
        def routine(self, timestamps, flags):
            prvsflag = 0
            for time, flag in zip(timestamps, flags):
                if flag == prvsflag:
                    continue
                else:
                    prvsflag = flag
                    yield rule_check(time, flag)
        return routine

    @detect_change
    def innovation_check(time, flag):
        outcome = []
        if flag & 0x01:
            outcome.append("vel")
        if flag>>1 & 0x01:
            outcome.append("hpos")
        if flag>>2 & 0x01:
            outcome.append("vpos")
        if flag>>3 & 0x07:
            outcome.append("mag")
        if flag>>6 & 0x01:
            outcome.append("yaw")
        if flag>>7 & 0x01:
            outcome.append("airspeed")
        if flag>>8 & 0x01:
            outcome.append("syn sideslip")
        if flag>>9 & 0x01:
            outcome.append("height above ground")
        if flag>>10 & 0x03:
            outcome.append("OF")
        return (time, outcome)

    @detect_change
    def failure_detector(time, flag):
        outcome = []
        if flag & 0x01:
            outcome.append("roll")
        if flag>>1 & 0x01:
            outcome.append("pitch")
        if flag>>2 & 0x01:
            outcome.append("yaw")
        if flag>>3 & 0x07:
            outcome.append("ext")
        return (time, outcome)

    @detect_change
    def gps_check_fail(time, flag):
        outcome = []
        if flag & 0x01:
            outcome.append("gps_fix")
        if flag>>1 & 0x01:
            outcome.append("min_sat_count")
        if flag>>2 & 0x01:
            outcome.append("min_pdop")
        if flag>>3 & 0x01:
            outcome.append("max_horz_err")
        if flag>>4 & 0x01:
            outcome.append("max_vert_err")
        if flag>>5 & 0x01:
            outcome.append("max_spd_err")
        if flag>>6 & 0x01:
            outcome.append("max_horz_drift")
        if flag>>7 & 0x01:
            outcome.append("max_vert_drift")
        if flag>>8 & 0x01:
            outcome.append("max_horz_spd_err")
        if flag>>9 & 0x01:
            outcome.append("max_vert_spd_err")
        return (time, outcome)

    def failsafe(self, ulog):
        vehicle_status = ulog.get_dataset('vehicle_status')
        failsafe_status = vehicle_status.list_value_changes('failsafe')
        for element in failsafe_status:
            time, status = element
            if status:
                yield (time, 'trigger')
                
class Vibration:

    def __init__(self, ulog):
        data_parser = ULogHelper(ulog)
        data_parser.extractRequiredMessages(['estimator_status', 'sensor_combined'])
        self.time = data_parser.getTimeSeries('estimator_status')
        self.normalized_time = data_parser.getTimeSeries('estimator_status', start_from_zero=True)
        self.gyro_delta_angle_coning = data_parser.getMessage('estimator_status', 'vibe[0]')
        self.gyro_high_freq = data_parser.getMessage('estimator_status', 'vibe[1]')
        self.accel_high_freq = data_parser.getMessage('estimator_status', 'vibe[2]')
        self.sensor_time = data_parser.getTimeSeries('sensor_combined')
        self.raw_accel_x = data.parser.getMessage('accelerometer_m_s2[0]')
        self.raw_accel_y = data.parser.getMessage('accelerometer_m_s2[1]')
        self.raw_accel_z = data.parser.getMessage('accelerometer_m_s2[2]')
        
    def calcIMUClipping():
        pass
    
    def calcVibration():
        pass
    '''
    // calculate vibration levels and check for accelerometer clipping (called by a backends)
void AP_InertialSensor::calc_vibration_and_clipping(uint8_t instance, const Vector3f &accel, float dt)
{
    // check for clipping
    if (_backends[instance] == nullptr) {
        return;
    }
    if (fabsf(accel.x) >  _backends[instance]->get_clip_limit() ||
        fabsf(accel.y) >  _backends[instance]->get_clip_limit() ||
        fabsf(accel.z) > _backends[instance]->get_clip_limit()) {
        _accel_clip_count[instance]++;
    }

    // calculate vibration levels
    if (instance < INS_VIBRATION_CHECK_INSTANCES) {
        // filter accel at 5hz
        Vector3f accel_filt = _accel_vibe_floor_filter[instance].apply(accel, dt);

        // calc difference from this sample and 5hz filtered value, square and filter at 2hz
        Vector3f accel_diff = (accel - accel_filt);
        accel_diff.x *= accel_diff.x;
        accel_diff.y *= accel_diff.y;
        accel_diff.z *= accel_diff.z;
        _accel_vibe_filter[instance].apply(accel_diff, dt);
    }
}

    The algorithm for calculating the vibration levels can be seen in the AP_InertialSensor.cpp’s calc_vibration_and_clipping() method 
    but in short it involves calculating the standard deviation of the accelerometer readings like this:
    Capture the raw x, y and z accelerometer values from the primary IMU
    High-pass filter the raw values at 5hz to remove the vehicle’s movement and create a “accel_vibe_floor” for x,y and z axis.
    Calculate the difference between the latest accel values and the accel_vibe_floor.
    Square the above differences, filter at 2hz and then calculate the square root (for x, y and z).
    These final three values are what appear in the VIBE msg’s VibeX, Y and Z fields.
    '''
