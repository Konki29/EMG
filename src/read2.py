import machine
import utime

sensor_myoware = machine.ADC(0)

while True:
    print(sensor_myoware.read_u16())
    utime.sleep(50)