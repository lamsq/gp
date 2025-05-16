import RPi.GPIO as GPIO
import subprocess
import time
import os
#from facial_recognition import run_facial_recognition
from facial_recognition import face_recognizer

# GPIO setup
GPIO.setmode(GPIO.BCM)
MOTION_SENSOR_PIN = 16
RED_PIN = 25   # GPIO 31 (Red)
GREEN_PIN = 23  # GPIO 29 (Green)
BLUE_PIN = 24   # GPIO 33 (Blue)

# Set up pins
GPIO.setup(MOTION_SENSOR_PIN, GPIO.IN)
GPIO.setup(RED_PIN, GPIO.OUT)
GPIO.setup(GREEN_PIN, GPIO.OUT)
GPIO.setup(BLUE_PIN, GPIO.OUT)

# Initialize PWM for smooth color transitions
red_pwm = GPIO.PWM(RED_PIN, 100)    # 100Hz frequency
green_pwm = GPIO.PWM(GREEN_PIN, 100)
blue_pwm = GPIO.PWM(BLUE_PIN, 100)
red_pwm.start(0)
green_pwm.start(0)
blue_pwm.start(0)

# Path to motor.py
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MOTOR_SCRIPT = os.path.join(SCRIPT_DIR, "motor.py")
MOTOR_SCRIPT_II = os.path.join(SCRIPT_DIR, "motorII.py")

def set_led_color(red, green, blue):
    """Set RGB LED color (0-100 duty cycle for each color)"""
    red_pwm.ChangeDutyCycle(red)
    green_pwm.ChangeDutyCycle(green)
    blue_pwm.ChangeDutyCycle(blue)

def run_motor_open():
    """Run motor control script"""
    try:
        subprocess.run(["python3", MOTOR_SCRIPT], check=True)
        print("motor.py executed successfully.")
    except FileNotFoundError:
        print(f"Error: {MOTOR_SCRIPT} not found.")
    except subprocess.CalledProcessError as e:
        print(f"Error running motor.py: {e}")
        
        
def run_motor_close():
    """Run motor control script"""
    try:
        subprocess.run(["python3", MOTOR_SCRIPT_II], check=True)
        print("motorII.py executed successfully.")
    except FileNotFoundError:
        print(f"Error: {MOTOR_SCRIPT} not found.")
    except subprocess.CalledProcessError as e:
        print(f"Error running motor.py: {e}")
        
        

try:
    # Initial state: Red LED
    set_led_color(100, 100, 0)
    print("System ready. Waiting for motion detection on GPIO 16...")
    
    while True:
        if GPIO.input(MOTION_SENSOR_PIN):
            print("\nMotion detected! Starting facial recognition...")
            
            
            set_led_color(100, 0, 0) # Yellow LED during recognition (Red + Green)
            
            if face_recognizer.run_facial_recognition():
                set_led_color(100, 0, 100) # Green LED for success
                print("Authorized face recognized! Running motor script...")
                run_motor_open()
                
            else:                
                set_led_color(0, 100, 100) # Red LED for failure
                print("No authorized face detected or recognition timed out.")
            
            # Add cooldown period
            print("Resuming motion monitoring in 5 seconds...")
            time.sleep(5)
            
            # Return to default red state
            set_led_color(100, 0, 0)
        
        # Small delay to reduce CPU usage
        time.sleep(0.1)
        

except KeyboardInterrupt:
    print("\nScript terminated by user.")
    #GPIO.cleanup()

finally:
    # Clean up
    #red_pwm.stop()
    #green_pwm.stop()
    #blue_pwm.stop()
    #GPIO.cleanup()
    print("GPIO cleaned up.")