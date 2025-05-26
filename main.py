import RPi.GPIO as GPIO
import subprocess
import time
import os

from RPLCD.i2c import CharLCD
# import board
# import busio
# import adafruit_character_lcd.character_lcd_i2c as character_lcd

#from facial_recognition import run_facial_recognition
from facial_recognition import face_recognizer

# GPIO setup
GPIO.setmode(GPIO.BCM)
MOTION_SENSOR_PIN = 16
MAGNETIC_SENSOR_PIN = 26  # KY-025 magnetic sensor
RED_PIN = 25   # GPIO 31 (Red)
GREEN_PIN = 23  # GPIO 29 (Green)
BLUE_PIN = 24   # GPIO 33 (Blue)

BUTTON_PIN = 13  # KY-004 connected to GPIO 13

#LCD screen init
lcd = CharLCD('PCF8574', 0x27)

# lcd = LCD(2, 0x3F, True)
# i2c = busio.I2C(board.SCL, board.SDA)
# cols = 16
# rows = 2
# lcd = character_lcd.Character_LCD_I2C(i2c, cols, rows, address = 0x3F)


# Set up pins
GPIO.setup(MOTION_SENSOR_PIN, GPIO.IN)
GPIO.setup(MAGNETIC_SENSOR_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # Magnetic sensor with pull-up
GPIO.setup(RED_PIN, GPIO.OUT)
GPIO.setup(GREEN_PIN, GPIO.OUT)
GPIO.setup(BLUE_PIN, GPIO.OUT)
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # Enable internal pull-up resistor

# Initialize PWM for smooth color transitions
red_pwm = GPIO.PWM(RED_PIN, 100)    # 100Hz frequency
green_pwm = GPIO.PWM(GREEN_PIN, 100)
blue_pwm = GPIO.PWM(BLUE_PIN, 100)
red_pwm.start(0)
green_pwm.start(0)
blue_pwm.start(0)

door_closed = True # INITIAL STATE OF THE DOOR TODO: SET UP

# Path to motor scripts
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MOTOR_SCRIPT = os.path.join(SCRIPT_DIR, "motor.py")
MOTOR_SCRIPT_II = os.path.join(SCRIPT_DIR, "motorII.py")

#Create a button logic variables

# Variables for double-click detection
LONG_PRESS_TIME = 1.5  # 1 second for long press
DEBOUNCE_TIME = 0.03   # 50ms debounce


def set_led_color(red, green, blue):
    """Set RGB LED color (0-100 duty cycle for each color)"""
    red_pwm.ChangeDutyCycle(red)
    green_pwm.ChangeDutyCycle(green)
    blue_pwm.ChangeDutyCycle(blue)

def run_motor_open():
    
    global door_closed
    """Run motor control script to open"""
    try:
        set_led_color(50, 0, 100) # YELLOW for wait
        subprocess.run(["python3", MOTOR_SCRIPT], check=True)
        door_closed = False
                    print("opening the door")
    except FileNotFoundError:
        print(f"Error: {MOTOR_SCRIPT} not found.")
    except subprocess.CalledProcessError as e:
        print(f"Error running motor.py: {e}")
        
def run_motor_close():
    """Run motor control script to close"""
    global door_closed
    try:
        subprocess.run(["python3", MOTOR_SCRIPT_II], check=True)
        door_closed = True
        print('door is closed')
    except FileNotFoundError:
        print(f"Error: {MOTOR_SCRIPT_II} not found.")
    except subprocess.CalledProcessError as e:
        print(f"Error running motorII.py: {e}")
#TODO NOVII

def check_magnet():
    global door_closed
    if not door_closed:
#         print("""Check if magnet is detected and run close motor if so""")
        if GPIO.input(MAGNETIC_SENSOR_PIN) == GPIO.HIGH:
            print("Magnet detected! Running close motor...")
            set_led_color(50, 0, 100) # YELLOW for wait
            lcd.clear()
            lcd.write_string("Door Closing")
            run_motor_close()
            lcd.clear()
            # Small delay to prevent multiple triggers
            time.sleep(2)
            return True
        return False
 

try:
    while True:
        set_led_color(100, 100, 100)
        check_magnet()

        # Wait for button press
        if GPIO.input(BUTTON_PIN) == GPIO.LOW and door_closed:  # Button pressed (KY-004 pulls to GND when pressed)
            lcd.clear()
            lcd.write_string("Move out the way")
            run_motor_open()
            lcd.clear()
            
#         set_led_color(100, 0, 100)  # Update LED after button action
        time.sleep(0.1)  # Cooldown to avoid repeats
               
           # Check magnet first         
        if GPIO.input(MOTION_SENSOR_PIN) and door_closed:
            print("Motion detected! Starting facial recognition...")
            
            set_led_color(100, 100, 0) # BLUE LED during recognition (Red + Green)
            lcd.clear()
            lcd.write_string("Identify \r\nYourself...")
            time.sleep(2)
            lcd.clear()
            lcd.write_string("Watch in the\r\nCamera")

            if face_recognizer.run_facial_recognition():
                print("Authorized face recognized! Running motor script...")
                
                name = face_recognizer.getName()
                lcd.clear()
                lcd.write_string("Please wait...\r\n" + name)

                run_motor_open()
                set_led_color(100, 0, 100) # Green LED for
                
                lcd.clear()
                lcd.write_string("Welcome Home \r\n"+ name)

                time.sleep(5)
                lcd.clear()
            else:                
                set_led_color(50, 100, 0) # Purple LED for failure
                print("No authorized face detected or recognition timed out.")
                lcd.clear()
                lcd.write_string("No Authorized\r\nFace Detected")
                time.sleep(2)
                lcd.clear()

            # Add REMOVE cooldown period
            print("Resuming motion monitoring in 5 seconds...")
            
            #set_led_color(50, 0, 100) # YELLOW for wait
            #time.sleep(2)
            
        # Small delay to reduce CPU usage
        time.sleep(0.5)
        
except KeyboardInterrupt:
    print("Script terminated by user.")
    # Clean up
    lcd.clear()
    red_pwm.stop()
    green_pwm.stop()
    blue_pwm.stop()
    GPIO.cleanup()
    print("GPIO cleaned up.")

finally:
    # Clean up
    lcd.clear()
    red_pwm.stop()
    green_pwm.stop()
    blue_pwm.stop()
    GPIO.cleanup()
    print("GPIO cleaned up.")
