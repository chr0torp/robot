import RPi.GPIO as GPIO
import time

def start_servo():
    print("Starting servo movement...")
    GPIO.setmode(GPIO.BOARD)

    GPIO.setup(11, GPIO.OUT)
    servol = GPIO.PWM(11, 50)  # GPIO 11 for PWM with 50Hz frequency

    servol.start(0)  # Initialize with 0% duty cycle (0 degrees)
    time.sleep(2)  # Wait before stopping the servo

    return servol

def open(servol):
    print("Opening servo")
    servol.ChangeDutyCycle(6)  # Set duty cycle to 6% (approximately 90 degrees)
    time.sleep(2)  # Wait for the servo to move
    print("Servo opened.")

def close(servol):
    print("Closing servo")
    servol.ChangeDutyCycle(3.5)  # Set duty cycle to 3.5% (approximately 0 degrees)
    time.sleep(2)  # Wait for the servo to move
    print("Servo closed.")

def stop(servol):
    print("Stopping servo")
    servol.ChangeDutyCycle(0)  # Stop the servo
    servol.stop()  # Stop the servo
    GPIO.cleanup()  # Clean up GPIO settings
    print("Servo movement completed and GPIO cleaned up.")
    time.sleep(1)  # Wait for a moment before stopping completely

if __name__ == "__main__":
    try:
        servol = start_servo()

        open(servol)
        close(servol)
        time.sleep(4)  # Wait before re-opening

        open(servol)  # Open again to demonstrate the servo can be reused
        time.sleep(2)  # Wait before closing again
        close(servol)
        time.sleep(4)  # Wait before re-opening again

        open(servol)  # Final open to show the servo can be controlled multiple times

    except KeyboardInterrupt:
        print("Program interrupted by user.")
    finally:
        stop(servol)  # Ensure the servo is stopped and GPIO is cleaned up
