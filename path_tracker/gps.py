import time
import time, serial


def get_coordinates():
	gps = serial.Serial("/dev/ttyACM0", baudrate=9600)
	line = gps.readline()
	decoded_line = line.decode('utf-8')
	data = decoded_line.split(",")
	try:
		if data[0] == "$GPRMC":
			lat_nmea = str(data[3])
			dd = float(lat_nmea[:2])
			mmm = float(lat_nmea[2:])/60
			latitude = dd + mmm

			long_nmea = str(data[5])
			dd = float(long_nmea[:3])
			mmm = float(long_nmea[3:])/60
			longitude = dd + mmm
			return [longitude, latitude]
		else:
			return 0,0

	except TypeError:
		get_coordinates()
