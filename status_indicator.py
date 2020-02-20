from time import sleep

from gpiozero import LED

class StatusIndicator:
    
    def __init__(self):
        self.standby_indicator = LED(23)
        self.record_indicator = LED(24)
        self.standby_indicator.off()
        self.record_indicator.off()
        
    def ready(self):
        self.standby_indicator.on()     
    
    def connecting(self):
        self.standby_indicator.off()
        self.record_indicator.off()
        for event in range(0, 2):
            self.standby_indicator.on()
            sleep(0.3)
            self.standby_indicator.off()
            self.record_indicator.on()
            sleep(0.3)
            self.record_indicator.off()
        self.standby_indicator.off()
        self.record_indicator.off()
    
    def connected(self):
        self.standby_indicator.blink()
    
    def error(self):
        self.standby_indicator.off()
        self.record_indicator.on()
        sleep(3)
    
    def start_recording(self):
        self.record_indicator.blink()
    
    def end_recording(self):
        self.record_indicator.off()
    
    def test(self):
        self.standby_indicator.on()
        self.record_indicator.on()
        sleep(1)
        self.standby_indicator.off()
        self.record_indicator.off()
    
    def off(self):
        self.standby_indicator.off()
        self.record_indicator.off()
        
