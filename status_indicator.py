from time import sleep

from gpiozero import LED

class StatusIndicator:
    
    def __init__(self):
        self.red = LED(23)
        self.green = LED(24)
        self.blue = LED(25)
        
    def ready(self):
        self.green.on()
    
    def pressed(self):
        self.blue.on()
        sleep(0.3)
        self.blue.off()
    
    def connecting(self):
        self.green.on()
        self.red.on()

    def connected(self):
        self.red.off()
        self.green.blink()
    
    def error(self):
        self.off()
        self.red.on()
        sleep(3)
    
    def start_recording(self):
        self.green.off()
        self.red.blink()
    
    def end_recording(self):
        self.red.off()
        self.connected()
    
    def test(self):
        self.red.on()
        sleep(0.5)
        self.red.off()
        self.green.on()
        sleep(0.5)
        self.green.off()
        self.blue.on()
        sleep(0.5)
        self.blue.off()
    
    def off(self):
        self.red.off()
        self.green.off()
        self.blue.off()
        
