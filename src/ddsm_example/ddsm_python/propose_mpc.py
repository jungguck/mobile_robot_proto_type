import time
import math

from motor_driver import md

def main():

	robot = md(port='/dev/ttyACM0')

	print(" connecting !! ")
	time.sleep(0.001)


	try:
		start_time = time.time()

		while True:

			#
			#
			#

			elapsed = time.time() = start_time

			v_cmd = ?
			w_cmd = ?

			#2. 계산된 값을 모터 드라이브에게 전달 

			robot.drive(v_cmd, w_cmd)

			#3. 제어 주기 

			time.sleep(0.05)

	except KeyboardInterrupt:

		print("\n stop")

	finally:

		robot.stop()
		robot.close()

if __name__ == "__main__"
	main()