

# ---------- Imports

# Imports for hardware PWM on RPi
# Install with: sudo pip3 install rpi-hardware-pwm --break-system-packages
# Test: lsmod | grep pwm
#       pinctrl get 19 -> 19: a5    pd | lo // GPIO19 = PWM0_1
#       cat /sys/kernel/debug/pwm
# GPIO.setup(pin, GPIO.OUT) breaks the hardware PWM output states until next reboot!!!
ROT_HW_PWM=True
LIN_HW_PWM=True
SOUND_DC=True
ENABLE_WEI=True
ENABLE_CAM=True
ENABLE_GPAD=True
ENABLE_IR=True
ENABLE_SHT=True
ENABLE_INA=True
ENABLE_VIFCON=False



import time
from threading import Thread
import sys
import logging   
import os
import RPi.GPIO as GPIO
import board
import digitalio
import busio


if ROT_HW_PWM == True or LIN_HW_PWM == True :
    from rpi_hardware_pwm import HardwarePWM
if ENABLE_WEI == True :
    from hx711v0_5_1 import HX711 # 2-wire interface: Clock, Data 
if ENABLE_CAM == True :
    from picamera2 import Picamera2
if ENABLE_GPAD == True :
    import gamepad as gp
if ENABLE_IR == True :
    import adafruit_mlx90640 # 2-wire I2C
if ENABLE_SHT == True :
    import adafruit_sht31d # 2-wire I2C
if ENABLE_INA == True :
    import adafruit_ina219 # 2-wire I2C
if ENABLE_VIFCON == True :
    import serial    
    

import adafruit_max31865 # 4-wire SPI: SDI/MOSI, SDA/MISO, CLK, CS
import adafruit_max31856 # 4-wire SPI

import democz_recipe as rec

from tkinter import *
from PIL import ImageTk, Image
import numpy as np
import matplotlib as mpl
from matplotlib.colors import ListedColormap, LinearSegmentedColormap
from matplotlib.widgets import CheckButtons
import matplotlib.pyplot as plt
import matplotlib.backends.backend_tkagg as tkagg
import matplotlib.colors as mcolors




# ---------- Global constants

HW_VERSION=2 # 0 = home, 1 = Paul, 2 = Platine

# Pins
GPIO.setmode(GPIO.BCM)
MYLED_PIN=24 #  Why 4 did not work here??
SSR_PIN=23
PT1_CS_PIN=board.D17
TC1_CS_PIN=board.D27
TC2_CS_PIN=board.D22
WC_CLK_PIN=6
WC_DAT_PIN=5
ROT_PWM_PIN=18 # PWM0 HW
ROT_DIR_PIN=16
FAN_PWM_PIN=25
LIN_PWM_PIN=19 # PWM1 HW
LIN_DIR_PIN=26
LIN_ON_PIN=20
if HW_VERSION == 2 :
    SOUND_PIN=13
else :
    SOUND_PIN=24 

LIN_Z_START=100
LIN_Z_MIN=0
LIN_Z_MAX=220
LIN_VAL_MIN=-100
LIN_VAL_MAX=100
ROT_VAL_MIN=-12
ROT_VAL_MAX=12
FAN_VAL_MIN=0
FAN_VAL_MAX=80
PID_SET_MIN=0
PID_SET_MAX=250 # 250 C
HEATER_POWER=1.500 # 1500 W
SENSOR_SAMPLETIME=2000 # 2 sec
CAMERA_SAMPLETIME=30000 # 60 sec
MOTORS_SAMPLETIME=100 # 100 millisec
HEATER_SAMPLETIME=5 # 5 millisec
PID_SAMPLETIME=50 # 50 millisec
PID_OUT_MIN=250 # 250 millisec
PID_OUT_MAX=5000 # 5 sec
PID_OUT_PERIOD=10000 # 10 sec
PID_KP=200
PID_KI=0.3
PID_KD=0

# ---------- Global variables

# Sensors
global pt1, tc1, tc2, hx, mlx, sht, ina, pt1_val, tc1_val, tc2_val, hx_val, sht_val, mlx_val, ina_val, kwh_val, picam2
# PID
global pid_set, pid_in, pid_in_old, pid_out_p, pid_out_i, pid_out_d, pid_out, pid_mode, pid_out_man, pid_sel
global pid_kp_norm, pid_ki_norm, pid_kd_norm, pid_out_sum
# Motors, alarm
global pwm_rot, pwm_fan, pwm_lin, pwm_sound, fan_val, fan_val_old, rot_val, rot_val_old, lin_val, lin_val_old, lin_z
global sound_freq, sound_freq_old, sound_duty, sound_duty_old, sound_on, led_on
# Time
global tt, time_start, ttalarm, time_startalarm, time_startalarmtimer
global tt_startctrt, tt_startctrp, tt_startmotl, tt_startmotr, tt_startmotf, tt_startal
global lasttime_sensors, lasttime_cam, lasttime_pid, lasttime_heat, lasttime_sound

global stop_threads, start, planalarm
global fdatanamestr
global serport

# ---------- Sensors

def init_sensors() :
    
    global pt1, tc1, tc2, hx, mlx, sht, ina, pt1_val, tc1_val, tc2_val, hx_val, sht_val, mlx_val, ina_val, lasttime_sensors

    lasttime_sensors = 0
    spi = board.SPI()
    
    i2c = busio.I2C(board.SCL, board.SDA, frequency=100000)

    timestart = time.time_ns()
    pt1_cs = digitalio.DigitalInOut(PT1_CS_PIN) 
    pt1 = adafruit_max31865.MAX31865(spi, pt1_cs)
    pt1_val = pt1.temperature
    timeend = time.time_ns()
    logging.info('PT_cruc = '+str( round(pt1_val,3) )+'°C / '+str( round((timeend-timestart)/1000) )+' micros')

    timestart = time.time_ns()
    tc1_cs = digitalio.DigitalInOut(TC1_CS_PIN)  
    tc1 = adafruit_max31856.MAX31856(spi, tc1_cs)
    tc1_val = 0 # tc1.temperature
    timeend = time.time_ns()
    logging.info('TC_cruc = '+str( round(tc1_val, 3) )+'°C / '+str( round((timeend-timestart)/1000) )+' micros')

    timestart = time.time_ns()
    tc2_cs = digitalio.DigitalInOut(TC2_CS_PIN)  
    tc2 = adafruit_max31856.MAX31856(spi, tc2_cs)
    tc2_val = 0 # tc2.temperature
    timeend = time.time_ns()
    logging.info('TC_air = '+str(round(tc2_val, 3) )+'°C / '+str( round((timeend-timestart)/1000) )+' micros')
    
    hx_val = 0.0
    if ENABLE_WEI == True :
        timestart = time.time_ns()
        hx = HX711(WC_DAT_PIN, WC_CLK_PIN)
        hx.setReadingFormat("MSB", "MSB")
        hx.autosetOffset()
        #hx.setReferenceUnit(3000)
        hx.setReferenceUnit(3083)
        hx_val = hx.getWeight()
        timeend = time.time_ns()
        logging.info('Weight = '+str(round(hx_val , 3) )+'g / '+str( round((timeend-timestart)/1000) )+' micros'+'\n')
    else :
        hx_val = 0.0

    mlx_val = np.zeros((24*32,))
    if ENABLE_IR == True :
        timestart = time.time_ns()
        mlx = adafruit_mlx90640.MLX90640(i2c)
        logging.info("MLX addr detected on I2C")
        logging.info([hex(i) for i in mlx.serial_number])
        mlx.refresh_rate = adafruit_mlx90640.RefreshRate.REFRESH_2_HZ
        mlx.getFrame(mlx_val)
        timeend = time.time_ns()
        logging.info('IR (0) = '+str(round(mlx_val[0], 2) )+'°C / '+str( round((timeend-timestart)/1000) )+' micros')

    sht_val = [0, 0]
    if ENABLE_SHT == True :
        timestart = time.time_ns()
        sht = adafruit_sht31d.SHT31D(i2c, 68) # default address = 0x44
        logging.info("SHT31-D addr detected on I2C")
        # logging.info(sht.serial_number)
        sht.mode = adafruit_sht31d.MODE_SINGLE
        sht.repeatability = adafruit_sht31d.REP_HIGH
        # sht.clock_stretching = True
        sht_val = [ sht.temperature, sht.relative_humidity ]
        timeend = time.time_ns()
        logging.info('T_air = '+str(round(sht_val[0], 2) )+'°C / '+str( round((timeend-timestart)/1000) )+' micros')

    ina_val = [0, 0, 0, 0]
    if ENABLE_INA == True :
        timestart = time.time_ns()
        ina = adafruit_ina219.INA219(i2c) # default address = 0x40
        logging.info("INA219 addr detected on I2C")
        # optional : change configuration to use 32 samples averaging for both bus voltage and shunt voltage
        ina.bus_adc_resolution = adafruit_ina219.ADCResolution.ADCRES_12BIT_32S
        ina.shunt_adc_resolution = adafruit_ina219.ADCResolution.ADCRES_12BIT_32S
        # optional : change voltage range to 16V
        ina.bus_voltage_range = adafruit_ina219.BusVoltageRange.RANGE_16V        
        logging.info("Config register:")
        logging.info("  bus_voltage_range:    0x%1X" % ina.bus_voltage_range)
        logging.info("  gain:                 0x%1X" % ina.gain)
        logging.info("  bus_adc_resolution:   0x%1X" % ina.bus_adc_resolution)
        logging.info("  shunt_adc_resolution: 0x%1X" % ina.shunt_adc_resolution)
        logging.info("  mode:                 0x%1X" % ina.mode)
        ina_val[0] = ina.bus_voltage # voltage on V- (load side)
        ina_val[1] = ina.shunt_voltage  # voltage between V+ and V- across the shunt
        ina_val[2] = ina.current  # current in mA
        ina_val[3] = ina.power  # power in watts
        timeend = time.time_ns()
        logging.info(ina_val)
        logging.info('INA / '+str( round((timeend-timestart)/1000) )+' micros')
        

def update_sensors() :

    global pt1_val, tc1_val, tc2_val, hx_val, sht_val, ina_val, lasttime_sensors
    # global pt1, tc1, tc2, hx

    pt1_val_temp = pt1.temperature
    tc1_val_temp = tc1.temperature
    tc2_val_temp = tc2.temperature


    if pt1_val_temp > -50 and pt1_val_temp < 1000 :
        pt1_val = pt1_val_temp
    else :
        logging.error('Sensor out of range: PT1 = '+str(pt1_val_temp)+'\n')


    if tc1_val_temp > -50 and tc1_val_temp < 1000 :
        tc1_val = tc1_val_temp
    else :
        logging.error('Sensor out of range: TC1 = '+str(tc1_val_temp)+'\n')


    if tc2_val_temp > -50 and tc2_val_temp < 1000 :
        tc2_val = tc2_val_temp
    else :
        logging.error('Sensor out of range: TC2 = '+str(tc2_val_temp)+'\n')


    if ENABLE_WEI == True :
        hx_val_temp = hx.getWeight()
        if hx_val_temp > -1000 and hx_val_temp < 1000 :  # filter all values larger than 1000 g
            hx_val = hx_val_temp
        else :
            logging.error('Sensor out of range: HX = '+str(hx_val_temp)+'\n')


    # if ENABLE_IR == True :
        
        # frame = [0] * 768
        
        # stamp = time.monotonic()
        # try:
            # mlx.getFrame(frame)
        # except ValueError:
            # # these happen, no biggie - retry
            # print('Error!')
        # else :    
            # print("Read 2 frames in %0.2f s" % (time.monotonic() - stamp))
            # for h in range(24):
                # for w in range(32):
                    # t = frame[h * 32 + w]
                    # print("%0.1f, " % t, end="")        
                # print()
            # print()        


    if ENABLE_SHT == True :
        
        sht_val_temp_t = sht.temperature
        if sht_val_temp_t >= -40 and sht_val_temp_t <= 125 : 
            sht_val[0] = sht_val_temp_t
        else :
            logging.error('Sensor out of range: SHT_T = '+str(sht_val_temp_t)+'\n')
        
        sht_val_temp_h = sht.relative_humidity
        if sht_val_temp_h >= 0 and sht_val_temp_h <= 100 : 
            sht_val[1] = sht_val_temp_h
        else :
            logging.error('Sensor out of range: SHT_H = '+str(sht_val_temp_h)+'\n')

        # print(sht_val)


    if ENABLE_INA == True :
        ina_val[0] = ina.bus_voltage 
        ina_val[1] = ina.shunt_voltage  
        ina_val[2] = ina.current  
        ina_val[3] = ina.power  
        # logging.info(ina_val)


    lasttime_sensors = time.time_ns()


def update_sensors_thread() :

    global stop_threads
    global plot_params

    logging.info("Sensor thread started\n")

    while stop_threads == False :
        update_sensors()
        time.sleep(SENSOR_SAMPLETIME/1000)
        
    logging.info("Sensor thread finished\n")
    
# ---------- PID control

def init_pid() :
    
    global pid_kp_norm, pid_ki_norm, pid_kd_norm, pid_out_i, lasttime_pid, pid_in, pid_in_old, pid_out, pid_mode, pid_out_man, pid_set, pid_sel
        
    pid_sampletime_s = PID_SAMPLETIME / 1000
    pid_kp_norm = PID_KP
    pid_ki_norm = PID_KI * pid_sampletime_s 
    pid_kd_norm = PID_KD / pid_sampletime_s

    pid_in = pt1_val
    pid_in_old = pt1_val
    pid_out = PID_OUT_MIN
    pid_out_i = pid_out
    lasttime_pid = 0
    pid_mode = 1
    pid_out_man = PID_OUT_MIN
    pid_set = 0
    pid_sel = 0

# ---

def update_pid() :
    
    global lasttime_pid, pid_in, pid_in_old, pid_out_p, pid_out_i, pid_out_d, pid_out
    # global pid_kp_norm, pid_ki_norm, pid_kd_norm, pid_set, pid_out, pid_out_man, pid_sel
    
    now = time.time_ns()
    
    if now - lasttime_pid > PID_SAMPLETIME * 1000000 and pid_mode == 1 :
        
        pid_in = pt1_val
        if pid_sel == 1 : pid_in = tc1_val
        if pid_sel == 2 : pid_in = tc2_val
        
        pid_out_p = pid_kp_norm*(pid_set - pid_in)
        
        pid_out_i = pid_out_i + pid_ki_norm*(pid_set - pid_in)
        if pid_out_i > PID_OUT_MAX : 
            pid_out_i = PID_OUT_MAX
        elif pid_out_i < PID_OUT_MIN : 
            pid_out_i = PID_OUT_MIN 
        
        pid_out_d = -pid_kd_norm*(pid_in - pid_in_old)

        pid_out = pid_out_p + pid_out_i + pid_out_d

        if pid_out > PID_OUT_MAX : 
            pid_out = PID_OUT_MAX
        elif pid_out < PID_OUT_MIN : 
            pid_out = PID_OUT_MIN         
        
        lasttime_pid = now
        pid_in_old = pid_in
        
    if pid_mode == 0 :
        pid_out = pid_out_man

# ---

def update_pid_thread() :

    global stop_threads

    logging.info("PID thread started\n")

    while stop_threads == False :
        update_pid()
        time.sleep(PID_SAMPLETIME/1000)

    logging.info("PID thread finished\n")

# ---

def init_heating():
    
    global lasttime_heat, kwh_val

    GPIO.setup(SSR_PIN, GPIO.OUT)
    GPIO.output(SSR_PIN, GPIO.LOW)
    
    lasttime_heat = time.time_ns()
    kwh_val = 0
    
# ---

def update_heating():
    
    global lasttime_heat, kwh_val
    # global pid_out
    
    # Create slow PWM signal with pid_out/PID_OUT_PERIOD  duty
    
    now = time.time_ns()
    if now - lasttime_heat > PID_OUT_PERIOD * 1000000 : 
        lasttime_heat = lasttime_heat + PID_OUT_PERIOD * 1000000
        if pid_out > PID_OUT_MIN : 
            kwh_val += pid_out/1000/3600 * HEATER_POWER # Calculate consumed energy as time[h] * power[kW]
    
    if pid_out > PID_OUT_MIN and pid_out * 1000000 > now - lasttime_heat :
        GPIO.output(SSR_PIN, GPIO.HIGH)
    else :
        GPIO.output(SSR_PIN, GPIO.LOW)

# ---

def update_heating_thread() :

    global stop_threads, ssr

    logging.info("Heating thread started\n")

    while stop_threads == False :
        update_heating()
        time.sleep(HEATER_SAMPLETIME/1000) 

    GPIO.setmode(GPIO.BCM) # Why error otherwise?
    GPIO.setup(SSR_PIN, GPIO.OUT)
    GPIO.output(SSR_PIN, GPIO.LOW)

    logging.info("Heating thread finished\n")


# ---------- Motor control

def init_motors():
    
    global pwm_rot, pwm_fan, pwm_lin, pwm_sound
    global fan_val_old, rot_val_old, lin_val_old, fan_val, rot_val, lin_val, lin_z
    global sound_freq, sound_freq_old, sound_duty, sound_duty_old, sound_on, led_on, lasttime_sound
    # global ssr, 
    
    fan_val_old = 0
    rot_val_old = 0
    lin_val_old = 0
    fan_val = 0
    rot_val = 0
    lin_val = 0
    lin_z = LIN_Z_START
    sound_freq = 250
    sound_freq_old = 250
    sound_duty = 0
    sound_duty_old = 50
    sound_on = False
    led_on = False
    lasttime_sound = 0
    
    if ROT_HW_PWM==False : GPIO.setup(ROT_PWM_PIN, GPIO.OUT)
    GPIO.setup(ROT_DIR_PIN, GPIO.OUT)
    GPIO.setup(FAN_PWM_PIN, GPIO.OUT)
    if LIN_HW_PWM==False : GPIO.setup(LIN_PWM_PIN, GPIO.OUT)
    GPIO.setup(LIN_DIR_PIN, GPIO.OUT)
    GPIO.setup(LIN_ON_PIN, GPIO.OUT)
    GPIO.setup(SOUND_PIN, GPIO.OUT)
    GPIO.setup(MYLED_PIN, GPIO.OUT)
    
    GPIO.output(ROT_DIR_PIN, GPIO.LOW)
    GPIO.output(LIN_DIR_PIN, GPIO.LOW)
    GPIO.output(LIN_ON_PIN, GPIO.LOW) # Motor off (using 100 mA otherwise)
    GPIO.output(MYLED_PIN, GPIO.LOW)
    
    if ROT_HW_PWM == True :
        pwm_rot = HardwarePWM(pwm_channel=0, hz=20000, chip=0)
        pwm_rot.start(0)
        # pwm_rot.change_frequency(25_000)
    else :
        # PWM will also stop if the instance variable goes out of scope!
        # Pulse precision bad at high frequencies, especially above 1000 Hz!!!
        pwm_rot = GPIO.PWM(ROT_PWM_PIN, 250) # 490 Hz on Arduino; vibration at low frequency!
        pwm_rot.start(0) # 0% duty cycle
        # pwm_rot.ChangeFrequency(800)
        # pwm_rot.ChangeDutyCycle(20)   


    if LIN_HW_PWM == True :
        pwm_lin = HardwarePWM(pwm_channel=1, hz=100, chip=0)
        pwm_lin.start(0)
    else :
        pwm_lin = GPIO.PWM(LIN_PWM_PIN, 100)
        pwm_lin.start(0)


    pwm_fan = GPIO.PWM(FAN_PWM_PIN, 250) 
    pwm_fan.start(0)
    
    if SOUND_DC == False :
        pwm_sound = GPIO.PWM(SOUND_PIN, 250) 
        pwm_sound.start(50)


def update_motors():
    
    global fan_val_old, rot_val_old, lin_val_old, sound_freq_old, sound_duty, sound_duty_old, lasttime_sound
    # global pwm_rot, pwm_fan, pwm_lin, fan_val, rot_val, lin_val, sound_freq, sound_on
    
    # rotation: with software PWM not really accurate and const speed!
    if rot_val != rot_val_old :
        if rot_val == 0:
            if ROT_HW_PWM == True :
                pwm_rot.change_duty_cycle(0)
            else :
                pwm_rot.ChangeDutyCycle(0)
        else :
            # motRotDuty = 3.178 * abs(rot_val) + 1.130 # rpm to %
            motRotDuty = 0.83 * 11.0 * abs(rot_val)
            #if abs(rot_val) < 3 : motRotDuty = 0 # does not rotate smoothly
            if motRotDuty > 100 : motRotDuty = 100
            if ROT_HW_PWM == True :
                pwm_rot.change_duty_cycle(motRotDuty)
            else :
                pwm_rot.ChangeDutyCycle(motRotDuty)
            logging.debug("rot_val={:.2f} motRotDuty={:.2f}\n".format(rot_val, motRotDuty))
        if rot_val>0 :
            GPIO.output(ROT_DIR_PIN, GPIO.LOW)
        else :
            GPIO.output(ROT_DIR_PIN, GPIO.HIGH)
            
        rot_val_old = rot_val

    # linear: below 200 Hz works also with (slow) software PWM
    if lin_val != lin_val_old :
        if lin_val == 0:
            GPIO.output(LIN_ON_PIN, GPIO.LOW) 
            if LIN_HW_PWM == True :
                pwm_lin.change_duty_cycle(0)
            else :
                pwm_lin.ChangeDutyCycle(0)
        else :
            # motLinFreq = 2.86824 * abs(lin_val) # mm/min to step frequency
            # motLinFreq = 5.0 * abs(lin_val) * 8.33 # with HW PWM
            motLinFreq = 2.5 * abs(lin_val) * 8.33 # Correction Frankfurt
            if HW_VERSION == 1 :
                motLinFreq = 0.87 * 1.12 * 20 * 0.541 * abs(lin_val) * 8.33 # Vorfakor: 8.33
            # if motLinFreq < 0.5 : motLinFreq = 0.5
            if motLinFreq > 1500 : motLinFreq = 1500
            GPIO.output(LIN_ON_PIN, GPIO.HIGH)
            if LIN_HW_PWM == True :
                pwm_lin.change_duty_cycle(50)
                pwm_lin.change_frequency(motLinFreq)
            else :
                pwm_lin.ChangeDutyCycle(50)
                pwm_lin.ChangeFrequency(motLinFreq)
            logging.debug("lin_val={:.2f} motLinFreq={:.2f}\n".format(lin_val, motLinFreq))
        #if lin_val>0 :
        if lin_val<0 : # Correction Frankfurt
            GPIO.output(LIN_DIR_PIN, GPIO.HIGH)
        else :
            GPIO.output(LIN_DIR_PIN, GPIO.LOW)
            
        lin_val_old = lin_val
        
    # fan: not much disturbance also with software PWM
    if fan_val != fan_val_old :
        # motFanDuty = 0.000068874 * fan_val * fan_val + 5.02 # rpm to %
        if fan_val == 0 : motFanDuty = 0
        # else : motFanDuty = 96 + abs(fan_val)*4/100 
        else : motFanDuty = abs(fan_val) 
        if motFanDuty > 100 : motFanDuty = 100

        pwm_fan.ChangeDutyCycle(motFanDuty)
        logging.debug("fan_val={:.2f} motFanDuty={:.2f}\n".format(fan_val, motFanDuty))
        
        fan_val_old = fan_val

    # beeping with 1s period
    if sound_on == True :
        now = time.time_ns() 
        time_sound = now - lasttime_sound
        if time_sound > 0 and time_sound < 1000000000 : sound_duty = 50
        if time_sound > 1000000000 and time_sound < 2000000000 : sound_duty = 0
        if time_sound > 2000000000 : lasttime_sound = now
    else :
        sound_duty = 0

    # sound
    if sound_freq != sound_freq_old or sound_duty != sound_duty_old :
        if SOUND_DC == False :
            pwm_sound.ChangeFrequency(sound_freq)
            pwm_sound.ChangeDutyCycle(sound_duty)
        else :
            if sound_duty == 0 : GPIO.output(SOUND_PIN, GPIO.LOW)
            else : GPIO.output(SOUND_PIN, GPIO.HIGH)
        logging.debug("sound_freq={:.2f} sound_duty={:.2f}\n".format(sound_freq, sound_duty))
        sound_freq_old = sound_freq
        sound_duty_old  = sound_duty


def stop_motors():

    # global pwm_rot, pwm_fan, pwm_lin

    pwm_lin.stop()
    pwm_rot.stop()
    pwm_fan.stop()
    if SOUND_DC == False : pwm_sound.stop()
    GPIO.output(LIN_ON_PIN, GPIO.LOW)
    GPIO.output(MYLED_PIN, GPIO.LOW)


def update_motors_thread() :

    global stop_threads, lin_z

    logging.info("Motor thread started\n")

    while stop_threads == False :
        lin_z += lin_val * MOTORS_SAMPLETIME/1000/60 # calculate Z position, but how accurate it is here
        update_motors()
        time.sleep(MOTORS_SAMPLETIME/1000)  
    
    stop_motors()
        
    logging.info("Motor thread finished\n")

# ---------- Camera

def init_camera() :
    
    global lasttime_cam, picam2
    
    lasttime_cam = 0.0
    # gp.setdevice()
    picam2 = Picamera2()
    # picam2.set_logging(Picamera2.INFO, output=sys.stdout)
    picam2.set_logging(Picamera2.ERROR)
    config = picam2.create_still_configuration({"size": (1014, 760)}, raw=picam2.sensor_modes[2]) # Full sensor mode
    picam2.configure(config)
    os.environ["LIBCAMERA_LOG_LEVELS"] = "3" # 1 Info
    #logger2 = logging.getLogger('picamera2')
    #logger2.setLevel(logging.INFO)
    
    
def get_photo(exposure_time, filename) :
    
    # global picam2

    # picam2.start_and_capture_file("test.jpg")
    picam2.controls.ExposureTime = exposure_time
    picam2.start(show_preview=False)
    time.sleep(0.2)
    picam2.capture_file(filename)


# ---------- Serial interface

def init_vifcon() :
    
    global serport

    portName = '/dev/ttyUSB0'
    try:
        serial.Serial(port=portName)
    except serial.SerialException:
        logging.error('Port ' + portName + ' not present')
        serport = None
        # exit(1)
    else :
        serport = serial.Serial(
                port = portName,
                baudrate = int(115200),
                stopbits = serial.STOPBITS_ONE,
                bytesize = serial.EIGHTBITS,
                parity = serial.PARITY_NONE,
                timeout = float(0.2) )


# ---------- Make GUI
# All widget functions and update_gui() and get_controls() are defined INSIDE gui() so that all variables are accessible

def gui() :

    global stop_threads, pid_mode, pid_sel
    # global tt, tc1_val, tc2_val, pt1_val, hx_val, kwh_val, pid_out, pid_in, pid_set, pid_out_p, pid_out_i, pid_out_d, 
    # global lin_z, lin_val, rot_val, fan_val

    # root window
    root = Tk()
    root.title("EduCrys Control")
    # root.resizable(width=False, height=False)

    # Set cells with non-zero weight, so that they expand with sticky
    # root.rowconfigure(tuple(range(4)),weight=1)
    root.rowconfigure(1,weight=1)
    root.columnconfigure(0,weight=1) # pictures expand horizontally

    # ---------- Canvas for camera

    cnv_cam = Canvas(root, width=400, height=300)
    cnv_cam.configure(bg='grey70')
    cnv_cam.grid(column=0, row=0, sticky='w')

    if ENABLE_IR == False :
        cnv_img = Canvas(root, width=400, height=300)
        cnv_img.configure(bg='grey70')
        cnv_img.grid(column=1, row=0, sticky='w')

    # ---------- Plot

    # Label, visibility, y axis, scale factor, linestyle, line/labelcolor, if updated in loop
    # Axes: x1, y1, y2 or none 
    plot_params = [
        ['Time[s]', True, 'x1', 1.0, 'solid', 'black', True],
        ['RecipeTime[s]', False, 'none', 1.0, 'solid', 'black', True],
        ['T_TC_1[C]', True, 'y1', 1.0, 'solid', 'fuchsia', True],
        ['T_TC_2[C]', True, 'y1', 1.0, 'solid', 'sienna', True],
        ['T_PT_1[C]', True, 'y1', 1.0, 'solid', 'gold', True],
        ['T_air[C]', True, 'y1', 1.0, 'solid', 'yellow', True],
        ['H_air[%]', True, 'y1', 1.0, 'solid', 'skyblue', True],
        ['Weight[g]', True, 'y2', 1.0, 'dashed', 'green', True],
        ['Energy[kWh]', True, 'y2', 1.0, 'dashed', 'teal', True],
        ['5V-I[mA]', False, 'y2', 1.0, 'dashed', 'darkgreen', True],
        ['PID_Out[%]', True, 'y1', 1.0, 'solid', 'lime', True],
        ['PID_Inp[C]', False, 'y1', 1.0, 'solid', 'tomato', True],
        ['PID_Set[C]', True, 'y1', 1.0, 'solid', 'red', True],
        ['PID_P[ms]', False, 'y2', 1.0, 'dashed', 'dimgrey', True],
        ['PID_I[ms]', False, 'y2', 1.0, 'dashed', 'silver', True],
        ['PID_D[ms]', False, 'y2', 1.0, 'dashed', 'lightgrey', True],
        ['z[mm]', True, 'y1', 1.0, 'solid', 'black', True],
        ['Mot_Lin[mm/s]', True, 'y2', 1.0, 'dashed', 'blue', True],
        ['Mot_Rot[rpm]', True, 'y2', 1.0, 'dashed', 'cyan', True],
        ['Mot_Fan[%]', True, 'y2', 1.0, 'dashed', 'steelblue', True],
        ['PID_Set_R', False, 'y1', 1.0, 'dotted', 'red', False],
        ['PID_Out_R', False, 'y1', 1.0, 'dotted', 'lime', False],
        ['Mot_Lin_R', False, 'y2', 1.0, 'dotted', 'blue', False],
        ['Mot_Rot_R', False, 'y2', 1.0, 'dotted', 'cyan', False],
        ['Mot_Fan_R', False, 'y2', 1.0, 'dotted', 'steelblue', False],
        ]

    plot_list = []
    for i in range(len(plot_params)):
        newlist = []
        plot_list.append(newlist)
        
    plot_xaxis = 0 
    plot_ylist = []
    for i in range(len(plot_params)):
        if plot_params[i][2] == 'x1': plot_xaxis = i  
        if plot_params[i][2] == 'y1': plot_ylist.append(i)
        if plot_params[i][2] == 'y2': plot_ylist.append(i)

    my_dpi=96
    figDAT, axDAT = plt.subplots(sharey=True, figsize=(800/my_dpi, 400/my_dpi), dpi=my_dpi)
    figDAT.subplots_adjust(left=0.27, bottom=0.1, right=0.92, top=0.95, wspace=0, hspace=0)
    axDAT.grid(True, 'major', 'both', ls='--', lw=.5, c='k', alpha=.3)
    axDAT.set_xlabel('Time, s')

    # cm = plt.get_cmap('tab20')
    # plot_colors=[cm(1.*i/len(plot_ylist)) for i in range(len(plot_ylist))]    
    
    axDAT2 = axDAT.twinx()
    
    # Display cursor coordinates for both y axes
    def make_format(cur, new):
        def format_coord(x, y):
            display_coord = cur.transData.transform((x,y))
            inv = new.transData.inverted()
            ax_coord = inv.transform(display_coord)
            coords = [ax_coord, (x, y)]
            return ('Left: {:<20}    Right: {:<}'.format(*['({:.1f}, {:.2f})'.format(x, y) for x,y in coords]))
        return format_coord

    axDAT2.format_coord = make_format(axDAT2, axDAT)
            
    figDATi = []
    for j in range(len(plot_ylist)):
        i = plot_ylist[j]
        if (plot_params[i][2]=='y1'):
            li, =  axDAT.plot(plot_list[plot_xaxis], plot_list[i], visible=plot_params[i][1], label=plot_params[i][0], color=plot_params[i][5], linestyle=plot_params[i][4])
            figDATi.append( li )
        if (plot_params[i][2]=='y2'):
            li, =  axDAT2.plot(plot_list[plot_xaxis], plot_list[i], visible=plot_params[i][1], label=plot_params[i][0], color=plot_params[i][5], linestyle=plot_params[i][4])
            figDATi.append( li )

    # Make checkbuttons 
        
    rax = plt.axes([0.02, 0.02, 0.17, 0.98]) # left, bottom, width, height for Buttons
    labels = [str(line.get_label()) for line in figDATi]
    visibility = [line.get_visible() for line in figDATi]
    check = CheckButtons(rax, labels, visibility)     
    
    #for i, c in enumerate(plot_colors):
    #    check.labels[i].set_color(c)
        
    for j in range( len(figDATi) ):
        pl = figDATi[j]
        check.labels[j].set_color( pl.get_color() )
        
    # adjust checkbutton size
    # l1 = Line2D([x, x + w], [y + h, y], **lineparams)
    # l2 = Line2D([x, x + w], [y, y + h], **lineparams)    
    scx = 3.0 
    scy = 1.2 
    for rect in check.rectangles: 
        rect.set_width(rect.get_width()*scx)
        rect.set_height(rect.get_height()*scy)
    for l in check.lines:
        l[0].set_xdata([ l[0].get_xdata()[0], l[0].get_xdata()[0]+(l[0].get_xdata()[1]-l[0].get_xdata()[0])*scx ])
        l[1].set_xdata([ l[1].get_xdata()[0], l[1].get_xdata()[0]+(l[1].get_xdata()[1]-l[1].get_xdata()[0])*scx ]) 
        l[0].set_ydata([ l[0].get_ydata()[1]+(l[0].get_ydata()[0]-l[0].get_ydata()[1])*scy, l[0].get_ydata()[1] ])
        l[1].set_ydata([ l[1].get_ydata()[0], l[1].get_ydata()[0]+(l[1].get_ydata()[1]-l[1].get_ydata()[0])*scy ])

    def func(label):
        index = labels.index(label)
        figDATi[index].set_visible(not figDATi[index].get_visible())
        # Add and show new points, but no autoscale
        figDAT.canvas.draw_idle()
        figDAT.canvas.flush_events()
        # set_autoscale(True)

    check.on_clicked(func)

    plot = tkagg.FigureCanvasTkAgg(figDAT, master=root) # FigureCanvasTkAgg object
    plot.get_tk_widget().grid(column=0, columnspan=2, row=2, sticky='news')

    frm_bot = Frame(root)
    frm_bot.grid(column=0, columnspan=2, row=3, sticky='news')
    toolbar = tkagg.NavigationToolbar2Tk(plot, frm_bot)
    toolbar.update()
    toolbar.pack(side='left', padx=10)


    def set_autoscale(auto) :
        axDAT.autoscale(enable=auto, axis='both', tight=None) 
        axDAT.relim(visible_only=auto)
        axDAT2.autoscale(enable=auto, axis='both', tight=None)
        axDAT2.relim(visible_only=auto)
        toolbar.update() # Reset home state to current state
        if auto == True :
            axDAT.autoscale_view()
            axDAT2.autoscale_view()
            figDAT.canvas.draw_idle()
            figDAT.canvas.flush_events()

    def reset_plots() :

        for i in range(len(plot_params)):
            plot_list[i].clear()
                
        for i in range(len(figDATi)):
            pl = figDATi[i]
            pl.set_data(plot_list[plot_xaxis], plot_list[plot_ylist[i]])
       
        figDAT.canvas.draw_idle()


    def btnauto_run():
        set_autoscale(True)
    btnauto = Button(frm_bot, text="Autoscale", command=btnauto_run)
    btnauto.pack(side='right', padx=10)

    # ---------- Buttons and labels

    # All buttons and inputs are on a new frame!!
    frame1=Frame(root) # bg='green')
    frame1.grid(column=2, row=0, rowspan=4, padx=10, pady=10, sticky='news')

    rowi = 0
    
    # --- Start time

    lbltt = Label(frame1, text='Log time [s]:')
    lbltt.grid(column=0, row=rowi, sticky='w')

    lblttv = Label(frame1, text='0')
    lblttv.grid(column=2, row=rowi, sticky='w')
    
    def btnstart_run() :

        global start, time_start, fdatanamestr
        
        if start == False :
            logging.info('Start time\n')
            time_start = time.time_ns()
            start = True
            btnstart.config(background='lime', text='Stop')
            
            fname = time.strftime("%Y%m%d-%H%M%S")
            fdatanamestr = "datafile_{:s}.txt".format(fname)
            
            with open(fdatanamestr, "a") as fileDAT:
                fileDAT.write('Time[s] RecipeTime[s] T_TC_1[C] T_TC_2[C] T_PT_1[C] T_air[C] H_air[%] Weight[g] Energy[kWh] I_5V[mA] PID_Out[%] PID_Inp[C] PID_Set[C] PID_P[ms] PID_I[ms] PID_D[ms] z[mm] Mot_Lin[mm/min] Mot_Rot[rpm] Mot_Fan[%]\n')

            reset_plots()
            
            chkctrt.config(state='normal')
            chkctrp.config(state='normal')
            chkmotl.config(state='normal')
            chkmotr.config(state='normal')
            chkmotf.config(state='normal')
            chkal.config(state='normal')
            
        else :
            logging.info('Stop time\n')
            start = False
            tt = 0
            btnstart.config(background='lightgrey', text='Start')
            
            chkctrtv.set(False);  chkctrt_change();  chkctrt.config(state='disabled')
            chkctrpv.set(False);  chkctrp_change();  chkctrp.config(state='disabled')
            chkmotlv.set(False);  chkmotl_change();  chkmotl.config(state='disabled')
            chkmotrv.set(False);  chkmotr_change();  chkmotr.config(state='disabled')
            chkmotfv.set(False);  chkmotf_change();  chkmotf.config(state='disabled')
            chkalv.set(False);    chkal_change();    chkal.config(state='disabled')            
            
            
    btnstart = Button(frame1, text="Start", width=4, command=btnstart_run)
    btnstart.grid(column=4, row=rowi, sticky='w')

    # --- Measurements incl. PID input selection
    
    rbselv = IntVar()
    rbselv.set(0)
    def rbsel_change() :
        global pid_sel
        pid_sel = rbselv.get()
        logging.info('Switch PID Input: pid_sel = '+str(pid_sel)+'\n')

    rowi += 1
    lblsens = Label(frame1, text='MEASUREMENTS')
    lblsens.grid(column=0, columnspan=6, row=rowi, sticky='s')
    
    rowi += 1
    chkselv = BooleanVar()
    chksel = False
    def chksel_change() :
        if chkselv.get() == True :
            rbselpt1.config(state='normal')
            rbseltc1.config(state='normal')
            rbseltc2.config(state='normal')
        else :
            rbselpt1.config(state='disabled')
            rbseltc1.config(state='disabled')
            rbseltc2.config(state='disabled')
    chksel = Checkbutton(frame1, text='Switch PID Input :', variable=chkselv, command=chksel_change)
    chksel.grid(column=4, columnspan=2, row=rowi, sticky='e')    

    rowi += 1
    lblpt1 = Label(frame1, text='T_PT_1 [°C]:')
    lblpt1.grid(column=0, row=rowi, sticky='w')

    lblpt1v = Label(frame1, text='0')
    lblpt1v.grid(column=2, row=rowi, sticky='w')

    rbselpt1 = Radiobutton(frame1, text='', variable=rbselv, value=0, command=rbsel_change)
    rbselpt1.grid(column=5, row=rowi, sticky='w')
    rbselpt1.config(state='disabled')

    rowi += 1
    lbltc1 = Label(frame1, text='T_TC_1 [°C]:')
    lbltc1.grid(column=0, row=rowi, sticky='w')

    lbltc1v = Label(frame1, text='0')
    lbltc1v.grid(column=2, row=rowi, sticky='w')
    
    rbseltc1 = Radiobutton(frame1, text='', variable=rbselv, value=1, command=rbsel_change)
    rbseltc1.grid(column=5, row=rowi, sticky='w')
    rbseltc1.config(state='disabled')

    rowi += 1
    lbltc2 = Label(frame1, text='T_TC_2 [°C]:')
    lbltc2.grid(column=0, row=rowi, sticky='w')

    lbltc2v = Label(frame1, text='0')
    lbltc2v.grid(column=2, row=rowi, sticky='w')
    
    rbseltc2 = Radiobutton(frame1, text='', variable=rbselv, value=2, command=rbsel_change)
    rbseltc2.grid(column=5, row=rowi, sticky='w')
    rbseltc2.config(state='disabled')

    rowi += 1
    lblshtt = Label(frame1, text='T_air [°C]:')
    lblshtt.grid(column=0, row=rowi, sticky='w')

    lblshttv = Label(frame1, text='0')
    lblshttv.grid(column=2, row=rowi, sticky='w')
    
    rowi += 1
    lblshth = Label(frame1, text='H_air [%]:')
    lblshth.grid(column=0, row=rowi, sticky='w')

    lblshthv = Label(frame1, text='0')
    lblshthv.grid(column=2, row=rowi, sticky='w')

    rowi += 1
    lblpow = Label(frame1, text='Energy [kWh]:')
    lblpow.grid(column=0, row=rowi, sticky='w')

    lblpowv = Label(frame1, text='0')
    lblpowv.grid(column=2, row=rowi, sticky='w')

    def btnpow_run():
        global kwh_val
        kwh_val = 0
        logging.info('Reset: kwh_val = '+str(kwh_val)+'\n')

    btnpow = Button(frame1, text='Reset', command=btnpow_run)
    btnpow.grid(column=4, row=rowi, sticky='w')

    rowi += 1
    lblina = Label(frame1, text='5V Current [mA]:')
    lblina.grid(column=0, row=rowi, sticky='w')

    lblinav = Label(frame1, text='0')
    lblinav.grid(column=2, row=rowi, sticky='w')

    rowi += 1
    lblwei = Label(frame1, text='Weight [g]:')
    lblwei.grid(column=0, row=rowi, sticky='w')

    lblweiv = Label(frame1, text='0')
    lblweiv.grid(column=2, row=rowi, sticky='w')

    def btnwei_run():
        # global hx
        hx.autosetOffset()
        logging.info('Tare'+'\n')
    btnwei = Button(frame1, text='Tare', command=btnwei_run)
    btnwei.grid(column=4, row=rowi, sticky='w')
    if ENABLE_WEI == False : btnwei.config(state='disabled')

    # ---- 

    rowi += 1
    lblheat = Label(frame1, text='HEATING CONTROL')
    lblheat.grid(column=0, columnspan=6, row=rowi, sticky='s')

    # --- Heating control radiobutton

    rowi += 1
    lblctr = Label(frame1, text='Control:')
    lblctr.grid(column=0, row=rowi, sticky='w')

    rbctrv = IntVar()
    rbctrv.set(1)
    def rbctr_change() :
        global pid_mode
        pid_mode = rbctrv.get()
        if pid_mode == 0 :
            if chkctrpv.get() == False : inpctrp.config(state='normal')
            if chkctrpv.get() == False : btnctrp.config(state='normal')
            inpctrt.config(state='readonly')
            btnctrt.config(state='disabled')
        else :
            inpctrp.config(state='readonly')
            btnctrp.config(state='disabled')
            if chkctrtv.get() == False : inpctrt.config(state='normal')
            if chkctrtv.get() == False : btnctrt.config(state='normal')            
        if chkctrtv.get() == False and chkctrpv.get() == False : 
            logging.info('Heating control: pid_mode = '+str(pid_mode)+'\n')
            
    rbctr0 = Radiobutton(frame1, text='PID', variable=rbctrv, value=1, command=rbctr_change)
    rbctr0.grid(column=2, row=rowi, sticky='w')

    rowi += 1
    rbctr1 = Radiobutton(frame1, text='Manual', variable=rbctrv, value=0, command=rbctr_change)
    rbctr1.grid(column=2, row=rowi, sticky='w')
    
    rowi += 1
    lblrec = Label(frame1, text='Start/Stop recipe :')
    lblrec.grid(column=4, columnspan=2, row=rowi, sticky='e')

    # --- Target temperature

    rowi += 1
    lblctrt = Label(frame1, text='Target T [°C]:')
    lblctrt.grid(column=0, row=rowi, sticky='w')

    inpctrtv = DoubleVar()
    inpctrtv.set('0')
    inpctrt = Entry(frame1, textvariable=inpctrtv, width=5)
    inpctrt.grid(column=2, row=rowi, sticky='w')
    
    chkctrtv = BooleanVar()

    def btnctrt_run():
        global pid_set
        if inpctrtv.get() >= PID_SET_MIN and inpctrtv.get() <= PID_SET_MAX :        
            pid_set = inpctrtv.get()
            inpctrt.config(background='white')
            inpctrt.config(readonlybackground='lightgrey')
        else :
            inpctrt.config(background='tomato')
            inpctrt.config(readonlybackground='coral')
        if chkctrtv.get() == False: logging.info('Set: pid_set = '+str(pid_set)+'\n')
        
    btnctrt = Button(frame1, text='Set', command=btnctrt_run)
    btnctrt.grid(column=4, row=rowi, sticky='w')

    def chkctrt_change() :
        global tt_startctrt
        # global tt
        if chkctrtv.get() == True :
            inpctrt.config(state='readonly')
            btnctrt.config(state='disabled')
            rbctr0.config(state='disabled')
            rbctr1.config(state='disabled')
            chkctrt.config(background='lime')
            tt_startctrt = tt
            rec.set_sp(tt)
            if len(rec.sptime2)>0 : figDATi[15].set_data(rec.sptime2, rec.spvalu2)
            logging.info('Start recipe: '+'tt_startctrt = '+str(tt_startctrt)+'\n')
        else :
            rbctr_change() 
            if chkctrpv.get() == False : rbctr0.config(state='normal')
            if chkctrpv.get() == False : rbctr1.config(state='normal')
            chkctrt.config(background='grey', text='0 s')
            logging.info('Stop recipe: ctrt'+'\n')
    chkctrt = Checkbutton(frame1, text='0 s', width=6, anchor='w', background='grey', variable=chkctrtv, command=chkctrt_change)
    chkctrt.grid(column=5, row=rowi, sticky='w')
    chkctrt.config(state='disabled')


    # Manual power

    rowi += 1
    lblctrp = Label(frame1, text='Target P [%]:')
    lblctrp.grid(column=0, row=rowi, sticky='w')

    inpctrpv = DoubleVar()
    inpctrpv.set(0)
    inpctrp = Entry(frame1, textvariable=inpctrpv, width=5)
    inpctrp.config(state='readonly')
    inpctrp.grid(column=2, row=rowi, sticky='w')

    chkctrpv = BooleanVar()

    def btnctrp_run():
        global pid_out_man
        val_ms = PID_OUT_PERIOD * inpctrpv.get() / 100 # % to ms
        # PID_OUT_MIN/PID_OUT_PERIOD = 2.5%
        # PID_OUT_MAX/PID_OUT_PERIOD = 50%      
        if val_ms >= 0 and val_ms <= PID_OUT_MAX :
            pid_out_man = val_ms
            inpctrp.config(background='white')
            inpctrp.config(readonlybackground='lightgrey')
        else :
            inpctrp.config(background='tomato')
            inpctrp.config(readonlybackground='coral')           
        if chkctrpv.get() == False : logging.info('Set: pid_out_man = '+str(pid_out_man)+'\n')
    btnctrp = Button(frame1, text='Set', command=btnctrp_run)
    btnctrp.config(state='disabled')
    btnctrp.grid(column=4, row=rowi, sticky='w')

    def chkctrp_change() :
        global tt_startctrp
        # global tt
        if chkctrpv.get() == True :
            inpctrp.config(state='readonly')
            btnctrp.config(state='disabled')
            rbctr0.config(state='disabled')
            rbctr1.config(state='disabled')
            chkctrp.config(background='lime')
            tt_startctrp = tt
            rec.set_pp(tt)
            if len(rec.pptime2)>0 : figDATi[16].set_data(rec.pptime2, rec.ppvalu2)
            logging.info('Start recipe: '+'tt_startctrp = '+str(tt_startctrp)+'\n')
        else :
            rbctr_change() 
            if chkctrtv.get() == False : rbctr0.config(state='normal')
            if chkctrtv.get() == False : rbctr1.config(state='normal')
            chkctrp.config(background='grey', text='0 s')  
            logging.info('Stop recipe: ctrp'+'\n')
    chkctrp = Checkbutton(frame1, text='0 s', width=6, anchor='w', background='grey', variable=chkctrpv, command=chkctrp_change)
    chkctrp.grid(column=5, row=rowi, sticky='w')
    chkctrp.config(state='disabled')

    # ----

    rowi += 1
    lblmot = Label(frame1, text='MOTOR CONTROL')
    lblmot.grid(column=0, columnspan=6, row=rowi, sticky='s')
    
    # --- Linear velocity and coordinate

    rowi += 1
    lblmotl = Label(frame1, text='Lin. vel. [mm/min]:')
    lblmotl.grid(column=0, row=rowi, sticky='w')

    inpmotlv = DoubleVar()
    inpmotlv.set(0)
    inpmotl = Entry(frame1, textvariable=inpmotlv, width=5)
    inpmotl.grid(column=2, row=rowi, sticky='w')

    # Define before btnmotl
    inpmotzv = DoubleVar()
    inpmotzv.set(LIN_Z_START)
    inpmotz = Entry(frame1, textvariable=inpmotzv, width=5)
    
    chkmotlv = BooleanVar()

    def btnmotl_run():
        global lin_val
        if inpmotlv.get() >= LIN_VAL_MIN and inpmotlv.get() <= LIN_VAL_MAX :        
            lin_val = inpmotlv.get()
            inpmotl.config(background='white')
            inpmotl.config(readonlybackground='lightgrey')
        else :
            inpmotl.config(background='tomato')
            inpmotl.config(readonlybackground='coral')

        if lin_val == 0 :
            inpmotz.config(state='normal')
            btnmotz.config(state='normal')
        else :
            inpmotz.config(state='readonly')
            btnmotz.config(state='disabled')
        if chkmotlv.get() == False : logging.info('Set: lin_val = '+str(lin_val)+'\n')

    btnmotl = Button(frame1, text='Set', command=btnmotl_run)
    btnmotl.grid(column=4, row=rowi, sticky='w')
    
    def chkmotl_change() :
        global tt_startmotl
        # global tt
        if chkmotlv.get() == True :
            inpmotl.config(state='readonly')
            btnmotl.config(state='disabled')
            chkmotl.config(background='lime')
            tt_startmotl = tt
            rec.set_ml(tt)
            if len(rec.mltime2)>0 : figDATi[17].set_data(rec.mltime2, rec.mlvalu2)
            logging.info('Start recipe: '+'tt_startmotl = '+str(tt_startmotl)+'\n')
        else :
            inpmotl.config(state='normal')
            btnmotl.config(state='normal')
            chkmotl.config(background='grey', text='0 s')
            logging.info('Stop recipe: motl'+'\n')
    chkmotl = Checkbutton(frame1, text='0 s', width=6, anchor='w', background='grey', variable=chkmotlv, command=chkmotl_change)
    chkmotl.grid(column=5, row=rowi, sticky='w')
    chkmotl.config(state='disabled')

    rowi += 1
    lblmotz = Label(frame1, text='Coord. Z [mm]:')
    lblmotz.grid(column=0, row=rowi, sticky='w')
    
    inpmotz.grid(column=2, row=rowi, sticky='w')

    # Add update, allow reset while motor not running
    def btnmotz_run():
        global lin_z
        if abs(inpmotzv.get()) >= LIN_Z_MIN and abs(inpmotzv.get()) <= LIN_Z_MAX :        
            lin_z = inpmotzv.get()
            inpmotz.config(background='white')
            inpmotz.config(readonlybackground='lightgrey')
        else :
            inpmotz.config(background='tomato')
            inpmotz.config(readonlybackground='coral')
        logging.info('Set: lin_z = '+str(lin_z)+'\n')
    btnmotz = Button(frame1, text='Set', command=btnmotz_run)
    btnmotz.grid(column=4, row=rowi, sticky='w')
    

    # --- Rotation velocity

    rowi += 1
    lblmotr = Label(frame1, text='Rot. vel. [rpm]:')
    lblmotr.grid(column=0, row=rowi, sticky='w')

    inpmotrv = DoubleVar()
    inpmotrv.set(0)
    inpmotr = Entry(frame1, textvariable=inpmotrv, width=5)
    inpmotr.grid(column=2, row=rowi, sticky='w')
    
    chkmotrv = BooleanVar()

    def btnmotr_run():
        global rot_val
        if abs(inpmotrv.get()) >= ROT_VAL_MIN and abs(inpmotrv.get()) <= ROT_VAL_MAX :        
            rot_val = inpmotrv.get()
            inpmotr.config(background='white')
            inpmotr.config(readonlybackground='lightgrey')
        else :
            inpmotr.config(background='tomato')
            inpmotr.config(readonlybackground='coral')
        if chkmotrv.get() == False : logging.info('Set: rot_val = '+str(rot_val)+'\n')
    btnmotr = Button(frame1, text='Set', command=btnmotr_run)
    btnmotr.grid(column=4, row=rowi, sticky='w')
    
    def chkmotr_change() :
        global tt_startmotr
        # global tt
        if chkmotrv.get() == True :
            inpmotr.config(state='readonly')
            btnmotr.config(state='disabled')
            chkmotr.config(background='lime')
            tt_startmotr = tt
            rec.set_mr(tt)
            if len(rec.mrtime2)>0 : figDATi[18].set_data(rec.mrtime2, rec.mrvalu2)
            logging.info('Start recipe: '+'tt_startmotr = '+str(tt_startmotr)+'\n')
        else :
            inpmotr.config(state='normal')
            btnmotr.config(state='normal')  
            chkmotr.config(background='grey', text='0 s')
            logging.info('Stop recipe: motr'+'\n')
    chkmotr = Checkbutton(frame1, text='0 s', width=6, anchor='w', background='grey', variable=chkmotrv, command=chkmotr_change)
    chkmotr.grid(column=5, row=rowi, sticky='w')
    chkmotr.config(state='disabled')

    # --- Fan velocity

    rowi += 1
    lblmotf = Label(frame1, text='Fan vel. [%]:')
    lblmotf.grid(column=0, row=rowi, sticky='w')

    inpmotfv = DoubleVar()
    inpmotfv.set(0)
    inpmotf = Entry(frame1, textvariable=inpmotfv, width=5)
    inpmotf.grid(column=2, row=rowi, sticky='w')
    
    chkmotfv = BooleanVar()

    def btnmotf_run():
        global fan_val
        if abs(inpmotfv.get()) >= FAN_VAL_MIN and abs(inpmotfv.get()) <= FAN_VAL_MAX :        
            fan_val = inpmotfv.get()
            inpmotf.config(background='white')
            inpmotf.config(readonlybackground='lightgrey')
        else :
            inpmotf.config(background='tomato')
            inpmotf.config(readonlybackground='coral')
        if chkmotfv.get() == False : logging.info('Set: fan_val = '+str(fan_val)+'\n')
    btnmotf = Button(frame1, text='Set', command=btnmotf_run)
    btnmotf.grid(column=4, row=rowi, sticky='w')
    
    def chkmotf_change() :
        global tt_startmotf
        # global tt
        if chkmotfv.get() == True :
            inpmotf.config(state='readonly')
            btnmotf.config(state='disabled')
            chkmotf.config(background='lime')
            tt_startmotf = tt
            rec.set_mf(tt)
            if len(rec.mftime2)>0 : figDATi[19].set_data(rec.mftime2, rec.mfvalu2)
            logging.info('Start recipe: '+'tt_startmotf = '+str(tt_startmotf)+'\n')
        else :
            inpmotf.config(state='normal')
            btnmotf.config(state='normal') 
            chkmotf.config(background='grey', text='0 s')
            logging.info('Stop recipe: motf'+'\n')
    chkmotf = Checkbutton(frame1, text='0 s', width=6, anchor='w', background='grey', variable=chkmotfv, command=chkmotf_change)
    chkmotf.grid(column=5, row=rowi, sticky='w')
    chkmotf.config(state='disabled')
    
    # --- Alarm countdown

    rowi += 1
    lblal = Label(frame1, text='Alarm [s]:')
    lblal.grid(column=0, row=rowi, sticky='w')

    inpalv = IntVar()
    inpalv.set(0)
    inpal = Entry(frame1, textvariable=inpalv, width=5)
    inpal.grid(column=2, row=rowi, sticky='w')
    
    chkalv = BooleanVar()

    def btnal_run():
        global planalarm, time_startalarm, time_startalarmtimer, ttalarm, sound_on
        if inpalv.get()>0 and planalarm == False :
            time_startalarmtimer = time.time_ns()
            time_startalarm = time_startalarmtimer + inpalv.get()*1000000000
            planalarm = True
            inpal.config(state='readonly')
            logging.info('Set alarm'+'\n')
            btnal.config(background='lime', text='Reset')
        else :
            planalarm = False
            time_startalarmtimer = 0
            time_startalarm = 0
            ttalarm = 0
            sound_on = False
            inpal.config(state='normal')
            logging.info('Reset alarm'+'\n')
            btnal.config(background='lightgrey', text='Set')
    btnal = Button(frame1, text='Set', width=4, command=btnal_run)
    btnal.grid(column=4, row=rowi, sticky='w')
    # btnal.config(state='disabled')
    
    def chkal_change() :
        global tt_startal
        # global tt
        if chkalv.get() == True :
            inpal.config(state='readonly')
            btnal.config(state='disabled')
            chkal.config(background='lime')
            tt_startal = tt
            rec.set_al(tt)
            logging.info('Start recipe: '+'tt_startal = '+str(tt_startal)+'\n')
        else :
            inpal.config(state='normal')
            btnal.config(state='normal') 
            chkal.config(background='grey', text='0 s')
            logging.info('Stop recipe: al'+'\n')
    chkal = Checkbutton(frame1, text='0 s', width=6, anchor='w', background='grey', variable=chkalv, command=chkal_change )
    chkal.grid(column=5, row=rowi, sticky='w')
    chkal.config(state='disabled')

    # ---

    lblspace1 = Label(frame1, text=' ')
    lblspace1.grid(column=1, row=rowi)

    lblspace2 = Label(frame1, text=' ')
    lblspace2.grid(column=3, row=rowi)

    frame1.rowconfigure(tuple(range(rowi)),weight=1)
    
    # ---------- Take camera picture and LED on/off
    
    frm_exp = Frame(root) # bg='green')
    frm_exp.grid(column=0, row=1, padx=5, pady=2)

    lblexp = Label(frm_exp, text='Cam. exp. [us]:')
    lblexp.pack(side='left', padx=3)

    inpexpv = IntVar()
    inpexpv.set(100000)
    inpexp = Entry(frm_exp, textvariable=inpexpv, width=8)
    inpexp.pack(side='left', padx=3)

    def btnexp_run():
        global expimg
        fname = time.strftime("%Y%m%d-%H%M%S")
        fnamestr = "PI_{:s}.jpg".format(fname)
        get_photo(inpexpv.get(), fnamestr)
        # rect1 = canvas1.create_rectangle(0, 300, 50, 0, width=0, fill="white") 
        expimg = ImageTk.PhotoImage(Image.open(fnamestr).resize((400,300), Image.LANCZOS))
        cnv_cam.create_image(400, 300, image=expimg, anchor='se') # x, y
        if start == False : logging.info('Set/Photo: inpexpv = '+str(inpexpv.get())+'\n')
    btnexp = Button(frm_exp, text='Set/Photo', command=btnexp_run)
    btnexp.pack(side='left', padx=3)
    if ENABLE_CAM == False : btnexp.config(state='disabled')


    def btnled_run():
        global led_on
        if led_on == False :
            led_on = True
            GPIO.output(MYLED_PIN, GPIO.HIGH)
            logging.info('LED On'+'\n')
            btnled.config(background='lime', text='LED off')
        else :
            led_on = False
            GPIO.output(MYLED_PIN, GPIO.LOW)
            logging.info('LED Off'+'\n')
            btnled.config(background='lightgrey', text='LED on')
    btnled = Button(frm_exp, text='LED on', width=4, command=btnled_run)
    btnled.pack(side='left', padx=3)
            
    # ---------- Load previous camera picture

    frm_img = Frame(root) 
    frm_img.grid(column=1, row=1, padx=5, pady=2)


    def btnimg_run():
        global imgimg
        fnamestr = inpimg.get()
        try :
            imgimg = ImageTk.PhotoImage(Image.open(fnamestr).resize((400,300), Image.LANCZOS))
            cnv_img.create_image(400, 300, image=imgimg, anchor='se') # x, y
        except :
            logging.error('Not found: fnamestr = '+fnamestr+'\n')
        else :
            logging.info('Load: fnamestr = '+fnamestr+'\n')

    if ENABLE_IR == False :

        inpimg = Entry(frm_img, width=20)
        inpimg.insert(END, 'PI_20240608-134739.jpg')
        inpimg.pack(side='left', padx=10)

        btnimg = Button(frm_img, text='Load', command=btnimg_run)
        btnimg.pack(side='left', padx=10)

    # ---------- Show IR image
    
    if ENABLE_IR == True :
        
        mlx_shape = (24,32)

        my_dpi=96
        figIR, axIR = plt.subplots(figsize=(400/my_dpi, 300/my_dpi), dpi=my_dpi)

        pltIR = axIR.imshow(np.zeros(mlx_shape),vmin=20,vmax=30) #start plot with zeros, cmap='Purples', 
        cbar = figIR.colorbar(pltIR) # setup colorbar for temps
        cbar.set_label('Temperature [$^{\circ}$C]',fontsize=10) # colorbar label
        axIR.axis("off")
        figIR.tight_layout()
        #figIR.subplots_adjust(left=0.05, bottom=0.05, right=0.95, top=0.95, wspace=0, hspace=0)
        lblmin = figIR.text(0.68, 0.06, '20.00' )
        lblmax = figIR.text(0.68, 0.91,  '30.00' )

        plotIR = tkagg.FigureCanvasTkAgg(figIR, master=root) # FigureCanvasTkAgg object
        plotIR.get_tk_widget().grid(column=1, row=0, sticky='w')

    
    def btnir_run() :
        global mlx_val
        # global mlx
        
        mlx_val = np.zeros((24*32,)) # setup array for storing all 768 temperatures
        try:
            mlx.getFrame(mlx_val) # read MLX temperatures 
        except ValueError:
            logging.info('Error reading IR image!\n')
        else :
            data_array = (np.reshape(mlx_val,mlx_shape)) # reshape to 24x32
            data_min = np.min(data_array)
            data_max = np.max(data_array)
            pltIR.set_data(np.fliplr(data_array)) # flip left to right
            pltIR.set_clim(vmin=data_min,vmax=data_max) # set bounds
            cbar.mappable.set_clim(vmin=data_min,vmax=data_max)
            lblmin.set_text( str(round(data_min, 2)) )
            lblmax.set_text( str(round(data_max, 2)) )

            figIR.canvas.draw_idle()
            figIR.canvas.flush_events() 
                      
            fname = time.strftime("%Y%m%d-%H%M%S")
            fnamestr = "IR_{:s}.jpg".format(fname)
            figIR.savefig(fnamestr, bbox_inches='tight')
            
            #for h in range(24):
            #    for w in range(32):
            #        t = frame[h * 32 + w]
            #        print("%0.1f, " % t, end="")        
            #    print()
            #print()      
              
        if start == False : logging.info('Get IR'+'\n') 
    
    
    if ENABLE_IR == True :

        btnir = Button(frm_img, text='Get IR', command=btnir_run)
        btnir.pack(side='left', padx=10)


    # ---------- GUI Loop

    def update_gui() :
        
        global tt, ttalarm, start, sound_on, lasttime_cam
        # global time_start, stop_threads, lin_val, rot_val, fan_val, pid_mode, pid_set, pid_sel
        # global tt_startctrt, tt_startctrp, tt_startmotl, tt_startmotr, tt_startmotf, tt_startal
        
        now = time.time_ns()
        
        if start == True :
            tt = (now - time_start)/1000000000
        else :
            tt = 0

        # --- Update params from recipes, use GUI change functions, DO NOT set global variables directly
        # TODO: Add red color if limits exceeded

        if start == True :
            
            if chkctrtv.get() == True : 
                pid_set_rec = rec.interpol(rec.sptime, rec.spvalu, tt, 1)
                inpctrtv.set(pid_set_rec)
                btnctrt_run() # pid_set
                chkctrt.config(text=str(int(tt - tt_startctrt)) + ' s')
                
            if chkctrpv.get() == True : 
                pid_out_man_rec = rec.interpol(rec.pptime, rec.ppvalu, tt, 1)
                if pid_out_man_rec >= 0 : # Set manual power
                    inpctrpv.set(pid_out_man_rec)
                    btnctrp_run() # pid_out_man
                    rbctrv.set(0)
                    rbctr_change() # pid_mode
                else : # Use T control
                    rbctrv.set(1)
                    rbctr_change() # pid_mode
                chkctrp.config(text=str(int(tt - tt_startctrp)) + ' s')
            
            if chkmotlv.get() == True : 
                lin_val_rec = rec.interpol(rec.mltime, rec.mlvalu, tt, 0)
                inpmotlv.set(lin_val_rec)
                btnmotl_run() # lin_val
                chkmotl.config(text=str(int(tt - tt_startmotl)) + ' s')
            
            if chkmotrv.get() == True : 
                rot_val_rec = rec.interpol(rec.mrtime, rec.mrvalu, tt, 0)
                inpmotrv.set(rot_val_rec)
                btnmotr_run() # rot_val
                chkmotr.config(text=str(int(tt - tt_startmotr)) + ' s')
            
            if chkmotfv.get() == True : 
                fan_val_rec = rec.interpol(rec.mftime, rec.mfvalu, tt, 0)
                inpmotfv.set(fan_val_rec)
                btnmotf_run() # fan_val
                chkmotf.config(text=str(int(tt - tt_startmotf)) + ' s')
            
            if chkalv.get() == True : 
                alarm_rec = rec.interpol(rec.altime, rec.alvalu, tt, 0)
                if alarm_rec > 0 : sound_on = True
                else : sound_on = False
                chkal.config(text=str(int(tt - tt_startal)) + ' s')




        # --- Update sensor data in GUI
        
        lblttv.config(text = str(round(tt,1)))
        lblpt1v.config(text = str(round(pt1_val,1)))
        lbltc1v.config(text = str(round(tc1_val,1)))
        lbltc2v.config(text = str(round(tc2_val,1)))
        lblshttv.config(text = str(round(sht_val[0],1)))
        lblshthv.config(text = str(round(sht_val[1],1)))        
        lblpowv.config(text = str(round(kwh_val,2)))
        lblinav.config(text = str(round(ina_val[2],2)))
        lblweiv.config(text = str(round(hx_val,1)))
                
        # --- Update z coordinate
        if lin_val != 0 :
            #btnmotz.config(state='normal')
            #inpmotz.delete(0,END)
            #inpmotz.insert(0,str(round(lin_z,2)))
            inpmotz.config(state='readonly')
            btnmotz.config(state='disabled')
            inpmotzv.set(lin_z) # just display current value
            
            if lin_z < LIN_Z_MIN or lin_z > LIN_Z_MAX :
                inpmotlv.set(0)
                btnmotl_run()
                inpmotz.config(background='tomato')
                inpmotz.config(readonlybackground='coral')
            else :
                inpmotz.config(background='white')
                inpmotz.config(readonlybackground='lightgrey')

        # --- Update PID IST values
        if pid_mode == 0 :
            inpctrtv.set(pid_set) # just display current value
        else :
            inpctrpv.set(100*pid_out/PID_OUT_PERIOD) # just display current value          

        # --- Update alarm countdown
        if planalarm == True and now < time_startalarm :
            ttalarm = (time_startalarm - now)/1000000000
            inpalv.set(ttalarm) # just display current value  
            logging.debug('ttalarm = ' + str(ttalarm)+ ' now = ' + str(now) + ' time_startalarm  ' +  str(time_startalarm) )
        
        if planalarm == True and now > time_startalarm : 
            sound_on = True
        
        # --- Logging data
        
        if start == True :
                    
            if ENABLE_CAM == True and now - lasttime_cam > CAMERA_SAMPLETIME * 1000000 :
                lasttime_cam = now
                btnexp_run() # save photo
                
                if ENABLE_IR == True :
                    btnir_run() # save IR image

            pid_out_percent = 100*pid_out/PID_OUT_PERIOD
            ttrecipe = 0.0
            serdata = [tt, ttrecipe, tc1_val, tc2_val, pt1_val, sht_val[0], sht_val[1], hx_val, kwh_val, ina_val[2], pid_out_percent, pid_in, pid_set, pid_out_p, pid_out_i, pid_out_d, 
                      lin_z, lin_val, rot_val, fan_val]
                      
            logging.debug(serdata)
                        
            for i in range(len(plot_params)):
                if plot_params[i][6] == True : # Skip recipe plots
                    plot_list[i].append(float(serdata[i])*plot_params[i][3])
                    
            for i in range(len(figDATi)):
                pl = figDATi[i]
                # print(pl.get_linestyle())
                if pl.get_linestyle() != ':' : # Skip recipe plots
                    pl.set_data(plot_list[plot_xaxis], plot_list[plot_ylist[i]])
            
            # TODO: If recipe is running, add a vertical line for current time
            
            # No autoscale here. Use the button
            figDAT.canvas.draw_idle()
            # figDAT.canvas.flush_events()
        
            with open(fdatanamestr, "a") as fileDAT:
                for i in serdata:
                    fileDAT.write(str(i))
                    fileDAT.write(' ')
                fileDAT.write('\n')
        
    
        now2 = time.time_ns() - now
        logging.debug('GUI update time [ms] = ' + str(now2/1000000) )
        # 1 ms without plot, 400 ms with plot & autoscale
        
        if stop_threads == False :
            root.after(SENSOR_SAMPLETIME, update_gui)

    # ---------- Loop for gamepad input

    def getcontrols():
        # global lin_val
        # global stop_threads
        res = gp.getevent()
        if res == 'right1' : 
            if inpmotlv.get() < LIN_VAL_MAX : inpmotlv.set( inpmotlv.get() + 1 )
            # inpmotl.delete(0,END)
            # inpmotl.insert(0,str(round( inpmotlv.get(),1) ))
        if res == 'left1' :
            if inpmotlv.get() > LIN_VAL_MIN : inpmotlv.set( inpmotlv.get() - 1 )
        if res == 'up1':            
            btnmotl.config(relief=SUNKEN)
            if inpmotlv.get() > 0 : btnmotl_run()  
            if inpmotlv.get() < 0 : 
                inpmotlv.set(0)
                btnmotl_run()
        if res == 'up0':      
            btnmotl.config(relief=RAISED) 
        if res == 'down1':            
            btnmotl.config(relief=SUNKEN) 
            if inpmotlv.get() < 0 : btnmotl_run()  
            if inpmotlv.get() > 0 : 
                inpmotlv.set(0)
                btnmotl_run()
            btnmotl_run()            
        if res == 'down0':      
            btnmotl.config(relief=RAISED) 
            
        if stop_threads == False :
            root.after(50, getcontrols)

    # ---------- Loop for VIFCON control

    def vifcon_control():
        
        # global serport
        # global stop_threads

        if serport != None and serport.in_waiting > 0 :
            
            inp = ''
            inp2 = ''
            inp = serport.read(1).decode() # 0.2 timeout set in init!
            logging.info('Serial data received: ' + str(inp))
        
            if inp == '!' : 
                serport.write(('*').encode())
                pid_out_percent = 100*pid_out/PID_OUT_PERIOD
                serdata = [tt, 0.0, tc1_val, tc2_val, pt1_val, 0.0, 0.0, hx_val, sht_val[0], sht_val[1], 
                           kwh_val, ina_val[2], 0, 0, 0, pid_out_percent, pid_set, pid_in, 0, 0,
                           pid_out_p, pid_out_p, pid_out_d, lin_z, lin_val, rot_val, fan_val, 0, 0]
                for i in serdata :
                    serport.write((str(i)+' ').encode())
                serport.write(('#').encode())
            
            try :    
                
                if inp == '1' :
                    time.sleep(0.01)
                    inp2 = serport.read(1).decode()
                    
                    if inp2=='m' or inp2=='M' : 
                        vali = int(serport.readline().decode()) 
                        if vali>=0 and vali<=2 : 
                            serport.write(("  m:="+str(vali)).encode())
                            if vali == 2 : 
                                rbctrv.set(0)
                                rbctr_change()
                            if vali == 1 : 
                                rbctrv.set(1)
                                rbctr_change()
                    if inp2=='s' or inp2=='S' : 
                        val = float(serport.readline().decode())   
                        if val>=0 and val<=400 : 
                            serport.write(("  s:="+str(val)).encode())
                            inpctrtv.set(val)
                            btnctrt_run() 
                    if inp2=='o' or inp2=='O' :  
                        val = float(serport.readline().decode())
                        if val>=0 and val<=100 : 
                            serport.write(("  o:="+str(val)).encode())
                            inpctrpv.set(val)
                            btnctrp_run()
                    if inp2=='p' or inp2=='P' : 
                        val = float(serport.readline().decode()) 
                        if val>=0 and val<=1e4 :  
                            serport.write(("  p:="+str(val)).encode()) 
                            # TODO PID_KP
                    if inp2=='i' or inp2=='I' : 
                        val = float(serport.readline().decode()) 
                        if val>=0 and val<=1e2 :  
                            serport.write(("  i:="+str(val)).encode()) 
                            # TODO PID_KI
                    if inp2=='d' or inp2=='D' :
                        val = float(serport.readline().decode())  
                        if val>=0 and val<=1e2 : 
                            serport.write(("  d:="+str(val)).encode())
                            # TODO PID_KD
              
                if inp == '2' :
                    time.sleep(0.01)
                    inp2 = serport.read(1).decode()

                    if inp2=='l' or inp2=='L' : 
                        val = float(serport.readline().decode())  
                        if val>=-600 and val<=600 : 
                            serport.write(("  l:="+str(val)).encode())
                            inpmotlv.set(val)
                            btnmotl_run()
                    if inp2=='z' or inp2=='Z' : 
                        val = float(serport.readline().decode())   
                        if val>=-500 and val<=500 : 
                            serport.write(("  z:="+str(val)).encode())
                            inpmotzv.set(val)
                            btnmotz_run()
                    if inp2=='t' or inp2=='T' : 
                        val = float(serport.readline().decode())  
                        if val>=-500 and val<=500 : 
                            serport.write(("  t:="+str(val)).encode())
                            # TODO LIN_Z_MAX
                    if inp2=='b' or inp2=='B' : 
                        val = float(serport.readline().decode())   
                        if val>=-500 and val<=500 : 
                            serport.write(("  b:="+str(val)).encode()) 
                            # TODO LIN_Z_MIN
                    if inp2=='r' or inp2=='R' : 
                        val = float(serport.readline().decode())  
                        if val>=-20 and val<=20 : 
                            serport.write(("  r:="+str(val)).encode()) 
                            inpmotrv.set(val)
                            btnmotr_run()
                    if inp2=='f' or inp2=='F' : 
                        val = float(serport.readline().decode())   
                        if val>=0 and val<=1600 : 
                            serport.write(("  f:="+str(val)).encode())
                            inpmotfv.set(val)
                            btnmotf_run()     
            
            except ValueError :
                logging.info('Could not convert number in command: '+inp+inp2)
                
        if stop_threads == False :
            root.after(100, vifcon_control)

    # ----------
    
    def close_event() :
        global stop_threads # set global variable here!!!
        stop_threads = True
        time.sleep(SENSOR_SAMPLETIME/1000)
        GPIO.cleanup()
        exit()
    
    root.protocol("WM_DELETE_WINDOW", close_event)  

    # ----------
    
    update_gui() # call first time, then loop
    
    if ENABLE_GPAD == True :
        getcontrols() # call first time, then loop

    if ENABLE_VIFCON == True :
        vifcon_control() # call first time, then loop

    root.mainloop()
    
    # End gui()

# ---------- Main

tt = 0.0
ttalarm = 0
time_startalarm = 0
time_startalarmtimer = 0
start = False
stop_threads = False
planalarm = False

fname = time.strftime("%Y%m%d-%H%M%S")
flognamestr = "logfile_{:s}.txt".format(fname)

# logging.StreamHandler().setLevel(logging.ERROR)

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    #format="%(message)s",
    handlers=[
        logging.StreamHandler(sys.stdout),
        logging.FileHandler(flognamestr, 'w+'),
    ]
)

# fh = logging.FileHandler(flognamestr, 'w+') #w+ for rewrite
# logging.getLogger().addHandler(fh)

logging.info('Starting at ' + time.strftime("%y-%m-%d-%H:%M:%S") + '\n')


init_sensors() 
if ENABLE_CAM == True : init_camera()
if ENABLE_GPAD == True :  gp.setdevice()
init_pid()
init_heating()
init_motors()
if ENABLE_VIFCON == True : init_vifcon()


# can pt1_val be unreachable while being updated in another thread?
thread_sensors = Thread(target=update_sensors_thread)
thread_sensors.start()

thread_pid = Thread(target=update_pid_thread)
thread_pid.start()

thread_heating = Thread(target=update_heating_thread)
thread_heating.start()

thread_motors = Thread(target=update_motors_thread)
thread_motors.start()

# ----------

gui()






