import RPi.GPIO as GPIO # Import Raspberry Pi GPIO library
import time

GPIO.setwarnings(False) # Ignore warning for now
GPIO.setmode(GPIO.BOARD) # Use physical pin numbering
GPIO.setup(12, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(11, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(13, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(15, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

def get_contact_states():
	return (GPIO.input(11) == GPIO.HIGH,GPIO.input(13) == GPIO.HIGH,GPIO.input(15) == GPIO.HIGH,GPIO.input(12) == GPIO.HIGH)

if __name__ == '__main__':
	while True: # Run forever
		msg = ""
		if GPIO.input(11) == GPIO.HIGH:
			msg+=" 11,"
		if GPIO.input(13) == GPIO.HIGH:
			msg+=" 13,"
		if GPIO.input(15) == GPIO.HIGH:
			msg+=" 15,"
		if GPIO.input(12) == GPIO.HIGH:
			msg+=" 12,"
		print(msg)
		time.sleep(0.1)
