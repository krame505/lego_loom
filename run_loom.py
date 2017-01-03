#!/usr/bin/env python3

from BrickPi import *

DEFAULT_MAX_ERROR = 10
def motorRotateDegreesPID(max_powers, angles, ports, k_p=0.5, k_i=0.4, k_d=0.1, max_errors=None, sampling_time=0):
    if len(angles) != len(ports):
        raise ArgumentError("Must be same number of angles and ports specified")
    if len(max_powers) != len(ports):
        raise ArgumentError("Must be same number of max powers and ports specified")
    if max_errors == None:
        max_errors = [DEFAULT_MAX_ERROR] * len(ports)
    else:
        if len(max_errors) != len(ports):
            raise ArgumentError("Must be same number of max powers and ports specified")

    max_power = {ports[i]: max_powers[i] for i in range(len(ports))}
    BrickPiUpdateValues()
    encoder = {port: BrickPi.Encoder[port] for port in ports}
    #target = {ports[i]: encoder[ports[i]] + angles[i] * 2 for i in range(len(ports))}
    target = {ports[i]: angles[i] * 2 for i in range(len(ports))}
    max_error = {ports[i]: max_errors[i] for i in range(len(ports))}
    
    errors = {port: encoder[port] - target[port] for port in ports}
    current_time = time.time()
    prev_errors = errors
    prev_time = current_time
    
    integral = {port: 0 for port in ports}
    derivative = {port: 0 for port in ports}
    
    while any((abs(errors[port]) > max_error[port] * 2 or derivative[port] != 0 for port in ports)):
        time.sleep(sampling_time)
        
        BrickPiUpdateValues()
        prev_errors = errors
        prev_time = current_time
        errors = {port: BrickPi.Encoder[port] - target[port] for port in ports}
        current_time = time.time()

        dt = current_time - prev_time
        
        for port in ports:
            p = errors[port]

            integral[port] += p * dt
            i = integral[port]
            
            derivative[port] = (p - prev_errors[port]) / dt
            d = derivative[port]

            speed = -k_p * p - k_i * i - k_d * d
            #print("%7.2f %7.2f %7.2f: %7.2f" % (p, i, d, speed))
            if max_power[port] > 255:
                max_power[port] = 255
            elif max_power[port] < 0:
                max_power[port] = 0
            if speed > max_power[port]:
                speed = max_power[port]
            elif speed < -max_power[port]:
                speed = -max_power[port]
                
            BrickPi.MotorSpeed[port] = int(speed)

    for port in ports:
        BrickPi.MotorSpeed[port] = 0

    return [errors[port] / 2 for port in ports]
    

WARP1 = PORT_A
WARP2 = PORT_B
BEATER = PORT_C
SHUTTLE = PORT_D

SHUTTLE_LEFT = PORT_1
SHUTTLE_RIGHT = PORT_2

SHUTTLE_POWER = 200
BEATER_POWER = 200

BEATER_ANGLE = 300

def pass_shuttle_left():
    BrickPi.MotorSpeed[SHUTTLE] = SHUTTLE_POWER
    while not BrickPi.Sensor[SHUTTLE_LEFT]:
        BrickPiUpdateValues()
        time.sleep(.01)
    BrickPi.MotorSpeed[SHUTTLE] = 0
    
def pass_shuttle_right():
    BrickPi.MotorSpeed[SHUTTLE] = -SHUTTLE_POWER
    while not BrickPi.Sensor[SHUTTLE_RIGHT]:
        BrickPiUpdateValues()
        time.sleep(.01)
    BrickPi.MotorSpeed[SHUTTLE] = 0

beater_init = None
def move_beater():
    global beater_init
    
    if beater_init is None:
        BrickPiUpdateValues()
        beater_init = BrickPi.Encoder[BEATER] / 2
        
    motorRotateDegreesPID([BEATER_POWER], [beater_init + BEATER_ANGLE], [BEATER], k_p=1, k_i=0.1, k_d=0.15)
    time.sleep(0.2)
    motorRotateDegreesPID([BEATER_POWER], [beater_init], [BEATER], k_p=1, k_i=0.1, k_d=0.2)

if __name__ == "__main__":
    BrickPiSetup()  # setup the serial port for communication
    
    BrickPi.MotorEnable[PORT_A] = 1 #Enable the Motor A
    BrickPi.MotorEnable[PORT_B] = 1 #Enable the Motor B
    BrickPi.MotorEnable[PORT_C] = 1 #Enable the Motor C
    BrickPi.MotorEnable[PORT_D] = 1 #Enable the Motor D

    BrickPi.SensorType[SHUTTLE_LEFT] = TYPE_SENSOR_TOUCH
    BrickPi.SensorType[SHUTTLE_RIGHT] = TYPE_SENSOR_TOUCH
    
    BrickPiSetupSensors()   #Send the properties of sensors to BrickPi

    while True:
        pass_shuttle_left()
        move_beater()
        pass_shuttle_right()
        move_beater()
        
