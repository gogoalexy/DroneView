import gpiozero
import picamera
import time

button = gpiozero.Button(17)
indicator = gpiozero.LED(27)
camera = picamera.PiCamera()
camera.resolution = (1280, 720)
camera.framerate = 60
time.sleep(1)

button.wait_for_press()
t = time.localtime()
indicator.blink()
camera.start_recording(time.strftime("%m%d%Y-%H%M", t) + ".h264")
camera.wait_recording(1)

while True:
    if button.is_pressed:
        break
    else:
        pass
    
camera.stop_recording()
indicator.off()
