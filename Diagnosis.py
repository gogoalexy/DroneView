from helper import ULogHelper

class DiagnoseFailure():

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
