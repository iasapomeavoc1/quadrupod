import robot.mpu6050
import RPi.GPIO as GPIO
import robot.camera

class Sensor_Interface():
	def __init__(self):
		## IMU
		self.bus = smbus.SMBus(1) # bus = smbus.SMBus(0) fuer Revision 1
		self.address = 0x68       # via i2cdetect
		self.power_mgmt_1 = 0x6b
		self.power_mgmt_2 = 0x6c
		## Contact Sensors
		GPIO.setwarnings(False) # Ignore warning for now
		GPIO.setmode(GPIO.BOARD) # Use physical pin numbering
		GPIO.setup(12, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
		GPIO.setup(11, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
		GPIO.setup(13, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
		GPIO.setup(15, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
		## Camera

	def get_imu_data(self):
		self.bus.write_byte_data(self.address, self.power_mgmt_1, 0)
		gyro_xout = mpu6050.read_word_2c(0x43)
		gyro_yout = mpu6050.read_word_2c(0x45)
		gyro_zout = mpu6050.read_word_2c(0x47)
		acc_xout = mpu6050.read_word_2c(0x3b)
		acc_yout = mpu6050.read_word_2c(0x3d)
		acc_zout = mpu6050.read_word_2c(0x3f)
		return [gyro_xout,gyro_yout,gyro_zout,acc_xout,acc_yout,acc_zout]

	def get_contact_states():
		return (GPIO.input(11) == GPIO.HIGH,GPIO.input(13) == GPIO.HIGH,GPIO.input(15) == GPIO.HIGH,GPIO.input(12) == GPIO.HIGH)

	def camera_data():
		pass