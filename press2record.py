import time

import picamera
from gpiozero import Button
from dronekit import connect

from status_indicator import StatusIndicator

camera = picamera.PiCamera()
camera.resolution = (1280, 720)
camera.framerate = 30
button = Button(25)
status = StatusIndicator()
status.test()
time.sleep(1)

while True:
    status.ready()
    button.wait_for_press()
    
    time_string = time.strftime("%m%d%Y-%H%M", time.localtime())
    with open(time_string + ".log", 'w') as log:
        try:
            vehicle = connect("/dev/ttyS0", wait_ready=True, baud=921600)
            log.write("Vehicle connected.\n")
            status.connected()
        except:
            log.write("Vehicle connection failed. Abort!\n")
            status.error()

        while not vehicle.armed:
            time.sleep(0.2)

        camera.start_recording(tstring + ".h264")
        status.start_recording()
        while vehicle.armed:
            log.write(str(vehicle.last_heartbeat) + ';')
            log.write(str(vehicle.mode) + ';')
            log.write(str(vehicle.attitude) + ';')
            log.write(str(vehicle.heading) + ';')
            log.write(str(vehicle.location.local_frame) + ';')
            log.write('\n')
            camera.wait_recording(0.5)
        
        camera.stop_recording()
        status.end_recording()
        log.close()

    vehicle.close()
    status.off()
