
class ULogHelper:

    def __init__(self, ulog):
        self._log = ulog
        self._message_catagories = None

    def extractRequiredMessages(self, catagory_labels):
        self._message_catagories = {label: self._log.get_dataset(label) for label in catagory_labels}

    def getMessage(self, catagory, message_label):
        try:
            return self._message_catagories[catagory].data[message_label]
        except:
            return None

    def getTimeSeries(self, catagory, start_from_zero=False):
        utime_series = self.getMessage(catagory, 'timestamp')
        if start_from_zero:
            start_utime = utime_series[0]
            return [(utime-start_utime)/1e6 for utime in utime_series]
        else:
            return [utime/1e6 for utime in utime_series]

