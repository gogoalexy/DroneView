import gpiozero
import picamera
from time import sleep

button = gpiozero.Button(17)
indicator = gpiozero.LED(27)
camera = picamera.PiCamera()
camera.resolution = (640, 480)

button.wait_for_press()
indicator.blink()
camera.start_recording("droneview.h264")
camera.wait_recording(1)

while True:
    if button.is_pressed:
        break
    else:
        pass
    
camera.stop_recording()
indicator.off()
