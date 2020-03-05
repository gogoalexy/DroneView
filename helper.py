
class ULogHelper:

    def __init__(self, ulog):
        self.log = ulog
        self.message_catagories = None

    def extractRequiredMessages(self, catagory_labels):
        self.message_catagories = {label: self.log.get_dataset(label) for label in catagory_labels}

    def getMessage(self, catagory, message_label):
        try:
            return self.message_catagories[catagory].data[message_label]
        except:
            return None

    # def getNormalizeMessage(self, zero_point=None):
