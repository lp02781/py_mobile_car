import threading
import serial
import time
import RPi.GPIO as GPIO

bleSerial = serial.Serial("/dev/ttyS0", baudrate=9600, timeout=1.0)

gData = ""

SW1 = 5
SW2 = 6
SW3 = 13
SW4 = 19

PWMA = 18
AIN1 = 22
AIN2 = 27

PWMB = 23
BIN1 = 25
BIN2 = 24

tones = {
"B0": 31,
"C1": 33,
"CS1": 35,
"D1": 37,
"DS1": 39,
"E1": 41,
"F1": 44,
"FS1": 46,
"G1": 49,
"GS1": 52,
"A1": 55,
"AS1": 58,
"B1": 62,
"C2": 65,
"CS2": 69,
"D2": 73,
"DS2": 78,
"E2": 82,
"F2": 87,
"FS2": 93,
"G2": 98,
"GS2": 104,
"A2": 110,
"AS2": 117,
"B2": 123,
"C3": 131,
"CS3": 139,
"D3": 147,
"DS3": 156,
"E3": 165,
"F3": 175,
"FS3": 185,
"G3": 196,
"GS3": 208,
"A3": 220,
"AS3": 233,
"B3": 247,
"C4": 262,
"CS4": 277,
"D4": 294,
"DS4": 311,
"E4": 330,
"F4": 349,
"FS4": 370,
"G4": 392,
"GS4": 415,
"A4": 440,
"AS4": 466,
"B4": 494,
"C5": 523,
"CS5": 554,
"D5": 587,
"DS5": 622,
"E5": 659,
"F5": 698,
"FS5": 740,
"G5": 784,
"GS5": 831,
"A5": 880,
"AS5": 932,
"B5": 988,
"C6": 1047,
"CS6": 1109,
"D6": 1175,
"DS6": 1245,
"E6": 1319,
"F6": 1397,
"FS6": 1480,
"G6": 1568,
"GS6": 1661,
"A6": 1760,
"AS6": 1865,
"B6": 1976,
"C7": 2093,
"CS7": 2217,
"D7": 2349,
"DS7": 2489,
"E7": 2637,
"F7": 2794,
"FS7": 2960,
"G7": 3136,
"GS7": 3322,
"A7": 3520,
"AS7": 3729,
"B7": 3951,
"C8": 4186,
"CS8": 4435,
"D8": 4699,
"DS8": 4978
}

song2=["D4","P","P","E4","D4","E4","G4","P","P","A4","G4","A4","B4","P","P","A4","B4",
       "G4","E4","D4","P","P","E4","D4","E4","G4","P","P","A4","G4","A4","B4","P","A4",
       "G4","E4","D4","P","E4","G4","P","P","A4","G4","P","P","D5","P","P","P","D5","P","P","D5","P","B4","P","A4","P","B4","P","A4","B4",
       "G4","E4","D4","P","P","E4","D4","E4","G4","P","P","A4","G4","A4","B4","A4","G4","E4","D4","E4","G4","P","P","A4","G4","P","P","G4"]

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(SW1,GPIO.IN,pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(SW2,GPIO.IN,pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(SW3,GPIO.IN,pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(SW4,GPIO.IN,pull_up_down=GPIO.PUD_DOWN)

GPIO.setup(PWMA,GPIO.OUT)
GPIO.setup(AIN1,GPIO.OUT)
GPIO.setup(AIN2,GPIO.OUT)

GPIO.setup(PWMB,GPIO.OUT)
GPIO.setup(BIN1,GPIO.OUT)
GPIO.setup(BIN2,GPIO.OUT)

L_Motor = GPIO.PWM(PWMA,500)
L_Motor.start(0)

R_Motor = GPIO.PWM(PWMB,500)
R_Motor.start(0)

SW1 = 5
SW2 = 6
SW3 = 13
SW4 = 19

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(SW1,GPIO.IN,pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(SW2,GPIO.IN,pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(SW3,GPIO.IN,pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(SW4,GPIO.IN,pull_up_down=GPIO.PUD_DOWN)

LED1 = 26
LED2 = 16
LED3 = 20
LED4 = 21

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(LED1,GPIO.OUT)
GPIO.setup(LED2,GPIO.OUT)
GPIO.setup(LED3,GPIO.OUT)
GPIO.setup(LED4,GPIO.OUT)

BUZZER = 12

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(BUZZER,GPIO.OUT)

p = GPIO.PWM(BUZZER,391)
p.stop()

def play(freq):
    p.ChangeFrequency(freq)
    p.start(25)
    
def stop():
    p.start(0)

def playsong(song):
    for i in range(len(song)):
        if(song[i]=="P"):
            stop()
        else:
            play(tones[song[i]])        
        time.sleep(0.3)
    stop()

def led_off():
    GPIO.output(LED1,GPIO.LOW)
    GPIO.output(LED2,GPIO.LOW)
    GPIO.output(LED3,GPIO.LOW)
    GPIO.output(LED4,GPIO.LOW)

def led_front():
    GPIO.output(LED1,GPIO.HIGH)
    GPIO.output(LED2,GPIO.HIGH)
    GPIO.output(LED3,GPIO.LOW)
    GPIO.output(LED4,GPIO.LOW)
    time.sleep(0.5)
    led_off()
    time.sleep(0.5)

def led_back():
    GPIO.output(LED1,GPIO.LOW)
    GPIO.output(LED2,GPIO.LOW)
    GPIO.output(LED3,GPIO.HIGH)
    GPIO.output(LED4,GPIO.HIGH)
    time.sleep(0.5)
    led_off()
    time.sleep(0.5)

def led_left():
    GPIO.output(LED1,GPIO.HIGH)
    GPIO.output(LED2,GPIO.LOW)
    GPIO.output(LED3,GPIO.HIGH)
    GPIO.output(LED4,GPIO.LOW)
    time.sleep(0.5)
    led_off()
    time.sleep(0.5)

def led_right():
    GPIO.output(LED1,GPIO.LOW)
    GPIO.output(LED2,GPIO.HIGH)
    GPIO.output(LED3,GPIO.LOW)
    GPIO.output(LED4,GPIO.HIGH)
    time.sleep(0.5)
    led_off()
    time.sleep(0.5)

def led_stop():
    GPIO.output(LED1,GPIO.LOW)
    GPIO.output(LED2,GPIO.LOW)
    GPIO.output(LED3,GPIO.LOW)
    GPIO.output(LED4,GPIO.LOW)

def motor_go(speed):
    GPIO.output(AIN1,0)
    GPIO.output(AIN2,1)
    L_Motor.ChangeDutyCycle(speed)
    GPIO.output(BIN1,0)
    GPIO.output(BIN2,1)
    R_Motor.ChangeDutyCycle(speed)

def motor_back(speed):
    GPIO.output(AIN1,1)
    GPIO.output(AIN2,0)
    L_Motor.ChangeDutyCycle(speed)
    GPIO.output(BIN1,1)
    GPIO.output(BIN2,0)
    R_Motor.ChangeDutyCycle(speed)
    
def motor_left(speed):
    GPIO.output(AIN1,1)
    GPIO.output(AIN2,0)
    L_Motor.ChangeDutyCycle(speed)
    GPIO.output(BIN1,0)
    GPIO.output(BIN2,1)
    R_Motor.ChangeDutyCycle(speed)
    
def motor_right(speed):
    GPIO.output(AIN1,0)
    GPIO.output(AIN2,1)
    L_Motor.ChangeDutyCycle(speed)
    GPIO.output(BIN1,1)
    GPIO.output(BIN2,0)
    R_Motor.ChangeDutyCycle(speed)

def motor_stop():
    GPIO.output(AIN1,0)
    GPIO.output(AIN2,1)
    L_Motor.ChangeDutyCycle(0)
    GPIO.output(BIN1,0)
    GPIO.output(BIN2,1)
    R_Motor.ChangeDutyCycle(0)

def serial_thread():
    global gData
    while True:
        data = bleSerial.readline()
        data = data.decode()
        gData = data

def main():
    global gData
    try:
        while True:
            if gData.find("go") >= 0:
                gData = ""
                print("ok go")
                motor_go(50)
                led_front()
                led_front()
               
                print("ok left")
                motor_left(50)
                led_left()
                led_left()

                print("ok right")
                motor_right(50)
                led_right()
                led_right()

                print("ok back")
                motor_back(50)
                led_back()
                led_back()

                print("ok stop")
                motor_stop()
                led_off()

    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    task1 = threading.Thread(target = serial_thread)
    task2 = threading.Thread(target = playsong(song2))
    task1.start()
    task2.start()
    main()
    bleSerial.close()
    GPIO.cleanup()
