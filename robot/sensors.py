import robot.mpu6050 as imu
import RPi.GPIO as GPIO
import robot.camera

class Sensor_Interface():
	def __init__(self):
		## IMU
		self.imu = imu.mpu6050(0x68)
		## Contact Sensors
		GPIO.setwarnings(False) # Ignore warning for now
		GPIO.setmode(GPIO.BOARD) # Use physical pin numbering
		GPIO.setup(12, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
		GPIO.setup(11, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
		GPIO.setup(13, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
		GPIO.setup(15, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
		## Camera

	def get_imu_data(self):
		return self.imu.get_all_data()

	def get_contact_states():
		return (GPIO.input(11) == GPIO.HIGH,GPIO.input(13) == GPIO.HIGH,GPIO.input(15) == GPIO.HIGH,GPIO.input(12) == GPIO.HIGH)

	def camera_data():
		pass
