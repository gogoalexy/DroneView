import sys
import time

import picamera
from gpiozero import Button
from dronekit import connect

from status_indicator import StatusIndicator

def exit_signal():
    sys.exit()

camera = picamera.PiCamera()
camera.resolution = (1280, 720)
camera.framerate = 30
camera.brightness = 40
camera.awb_mode = 'sunlight'
button = Button(25)
status = StatusIndicator()

status.test()
time.sleep(1)

status.ready()
button.wait_for_press()
status.pressed()
try:
    vehicle = connect("/dev/ttyS0", wait_ready=True, baud=921600)
    status.connected()
except:
    status.error()
    time.sleep(5)
    sys.exit(1)

button.when_pressed = exit_signal

while True:
    
    time_string = time.strftime("%b%d%y-%H%M", time.localtime())
    
    try:
        while not vehicle.armed:
            time.sleep(0.2)
        log = open(time_string + ".log", 'w')
    except:
        status.pressed()
        vehicle.close()
        log.close()
        status.off()
        time.sleep(2)
        break
    else:
        camera.start_recording(time_string + ".h264")
        status.start_recording()
        while vehicle.armed:
            log.write(str(vehicle.last_heartbeat) + ';')
            log.write(str(vehicle.location.local_frame) + ';')
            log.write('\n')
            camera.wait_recording(0.5)
        
        camera.stop_recording()
        status.end_recording()

    log.close()
