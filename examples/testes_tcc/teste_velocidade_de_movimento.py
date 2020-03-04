##########DEPENDENCIES#############

from dronekit import connect, VehicleMode,LocationGlobalRelative,APIException
import time
import socket
#import exceptions
import math
import argparse
from pymavlink import mavutil
#########FUNCTIONS#################

def connectMyCopter():

	parser = argparse.ArgumentParser(description='commands')
	parser.add_argument('--connect')
	args = parser.parse_args()

	connection_string = args.connect

	if not connection_string:
		import dronekit_sitl
		sitl = dronekit_sitl.start_default()
		connection_string = sitl.connection_string()

	vehicle = connect(connection_string,wait_ready=True)

	return vehicle

def arm_and_takeoff(targetHeight):
	while vehicle.is_armable!=True:
		print("Esperando o veiculo se armar")
		time.sleep(1)
	print("Veiculo armado")

	vehicle.mode = VehicleMode("GUIDED")

	while vehicle.mode!='GUIDED':
		print("Aguardando entrar em modo GUIDED")
		time.sleep(1)
	print("Veiculo em modo GUIDED")

	vehicle.armed = True
	while vehicle.armed==False:
		print("Esperando o veiculo se armar")
		time.sleep(1)
	print("Cuidado as helices virtuais estao em funcionamento")

	vehicle.simple_takeoff(targetHeight) ##meters


	while True:
		print("Current Altitude: %d"%vehicle.location.global_relative_frame.alt, targetHeight)

		if vehicle.location.global_relative_frame.alt>=.92*targetHeight:
			break
		time.sleep(1)
	print("Target altitude reached!!")
	return None

def send_local_ned_velocity(vx, vy, vz):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        vx, vy, vz, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    # send command to vehicle on 1 Hz cycle
    vehicle.send_mavlink(msg)
    vehicle.flush()


def send_global_ned_velocity(vx, vy, vz):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        vx, vy, vz, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    # send command to vehicle on 1 Hz cycle
    vehicle.send_mavlink(msg)
    vehicle.flush()
##########MAIN EXECUTABLE###########
if __name__ == "__main__":
    # altitude = 10
    vehicle = connectMyCopter()
    # print("\nGet all vehicle attribute values:")
    # print(" Autopilot Firmware version: %s" % vehicle.version)
    # print("   Major version number: %s" % vehicle.version.major)
    # print("   Minor version number: %s" % vehicle.version.minor)
    # print("   Patch version number: %s" % vehicle.version.patch)
    # print("   Release type: %s" % vehicle.version.release_type())
    # print("   Release version: %s" % vehicle.version.release_version())
    # print("   Stable release?: %s" % vehicle.version.is_stable())
    # print(" Autopilot capabilities")
    # print("   Supports MISSION_FLOAT message type: %s" % vehicle.capabilities.mission_float)
    # print("   Supports PARAM_FLOAT message type: %s" % vehicle.capabilities.param_float)
    # print("   Supports MISSION_INT message type: %s" % vehicle.capabilities.mission_int)
    # print("   Supports COMMAND_INT message type: %s" % vehicle.capabilities.command_int)
    # print("   Supports PARAM_UNION message type: %s" % vehicle.capabilities.param_union)
    # print("   Supports ftp for file transfers: %s" % vehicle.capabilities.ftp)
    # print("   Supports commanding attitude offboard: %s" % vehicle.capabilities.set_attitude_target)
    # print("   Supports commanding position and velocity targets in local NED frame: %s" % vehicle.capabilities.set_attitude_target_local_ned)
    # print("   Supports set position + velocity targets in global scaled integers: %s" % vehicle.capabilities.set_altitude_target_global_int)
    # print("   Supports terrain protocol / data handling: %s" % vehicle.capabilities.terrain)
    # print("   Supports direct actuator control: %s" % vehicle.capabilities.set_actuator_target)
    # print("   Supports the flight termination command: %s" % vehicle.capabilities.flight_termination)
    # print("   Supports mission_float message type: %s" % vehicle.capabilities.mission_float)
    # print("   Supports onboard compass calibration: %s" % vehicle.capabilities.compass_calibration)
    # print(" Global Location: %s" % vehicle.location.global_frame)
    # print(" Global Location (relative altitude): %s" % vehicle.location.global_relative_frame)
    # print(" Local Location: %s" % vehicle.location.local_frame)
    # print(" Attitude: %s" % vehicle.attitude)
    # print(" Velocity: %s" % vehicle.velocity)
    # print(" GPS: %s" % vehicle.gps_0)
    # print(" Gimbal status: %s" % vehicle.gimbal)
    # print(" Battery: %s" % vehicle.battery)
    # print(" EKF OK?: %s" % vehicle.ekf_ok)
    # print(" Last Heartbeat: %s" % vehicle.last_heartbeat)
    # print(" Rangefinder: %s" % vehicle.rangefinder)
    # print(" Rangefinder distance: %s" % vehicle.rangefinder.distance)
    # print(" Rangefinder voltage: %s" % vehicle.rangefinder.voltage)
    # print(" Heading: %s" % vehicle.heading)
    # print(" Is Armable?: %s" % vehicle.is_armable)
    # print(" System status: %s" % vehicle.system_status.state)
    # print(" Groundspeed: %s" % vehicle.groundspeed)    # settable
    # print(" Airspeed: %s" % vehicle.airspeed)    # settable
    # print(" Mode: %s" % vehicle.mode.name)    # settable
    # print(" Armed: %s" % vehicle.armed)    # settable
    # arm_and_takeoff(altitude)
    # time.sleep(5)

    # while counter<2:
    # 	send_local_ned_velocity(1,0,0)
    # 	time.sleep(1)
    # 	print("Moving NORTH relative to front of drone")
    # 	counter=counter+1

    # time.sleep(2)
    # counter=0
    # vel1=0
    # vel2=0
    # vel3=0
    # while counter <= 2:
    #     counter=counter+1
    #     vel1= vel1+1           # 0x   0y
    #     send_local_ned_velocity(vel1,vel2,vel3)
    #     print("NORTE")
    #     time.sleep(1)
    #     if counter == 2:               #  NORTE
    #         while counter >= 0:        # +x   0y
    #             send_local_ned_velocity(vel1,vel2,vel3)
    #             print("OESTE")
    #             counter=counter-1
    #             vel1=vel1-1# x
    #             vel2=vel2+1# y
    #             time.sleep(1)
    #             if counter == 0:               # OESTE
    #                 while counter <= 2:        # 0x    +y
    #                     send_local_ned_velocity(vel1,vel2,vel3)
    #                     print("SUL")
    #                     counter=counter+1
    #                     vel1=vel1-1# x
    #                     vel2=vel2-1# y
    #                     time.sleep(1)
    #                     if counter == 2:               # SUL
    #                         while counter >= 0:        # -x    0y
    #                             send_local_ned_velocity(vel1,vel2,vel3)
    #                             print("LESTE")
    #                             counter = counter-1
    #                             vel1 = vel1+1# x
    #                             vel2 = vel2-1# y
    #                             time.sleep(1)
    #                             if counter == 0:               # LESTE
    #                                 while counter <= 2:        # 0x    -y
    #                                     send_local_ned_velocity(vel1,vel2,vel3)
    #                                     print("NORTE")
    #                                     counter = counter+1
    #                                     vel1 = vel1+1# x
    #                                     vel2 = vel2+1# y
    #                                     time.sleep(1)
    #                                     if counter == 2:
    #                                         print("TESTE")
    #                                         send_local_ned_velocity(0,0,0)

    #                                 else:
    #                                     break
    #                         else:
    #                             break
    #                 else:
    #                     break
    #         else:
    #             break
    # else:
    #     pass

    i = 0
    while i < 100:
        print("teste")
        if i <= 5:
            print(i)
            send_local_ned_velocity(1,0,0)
            time.sleep(0.2)
        elif i > 5 and i <= 10:
            print(i)
            send_local_ned_velocity(-0.2,0,0)
            time.sleep(0.2)
        elif i > 10 and i <= 15:
            print(i)
            send_local_ned_velocity(-0.7,0,0)
            time.sleep(0.2)
        elif i > 15 and i <= 20:
            print(i)
            send_local_ned_velocity(0.1,0,0)
            time.sleep(0.2)
        elif i > 20 and i <= 25:
            print(i)
            send_local_ned_velocity(1,0,0)
            time.sleep(0.2)
        elif i > 25 and i <= 30:
            print(i)
            send_local_ned_velocity(0,-1,0)
            time.sleep(0.2)
        elif i > 30 and i <= 35:
            print(i)
            send_local_ned_velocity(0,0.5,0)
            time.sleep(0.2)
        elif i > 35 and i <= 40:
            print(i)
            send_local_ned_velocity(0,0.9,0)
            time.sleep(0.2)
        elif i > 40 and i <= 45:
            print(i)
            send_local_ned_velocity(0,1,0)
            time.sleep(0.2)
        elif i > 45 and i <= 50:
            print(i)
            send_local_ned_velocity(0,-0.6,0)
            time.sleep(0.2)
        elif i > 50 and i <= 55:
            print(i)
            send_local_ned_velocity(1,0,0)
            time.sleep(0.2)
        elif i > 55 and i <= 60:
            print(i)
            send_local_ned_velocity(0,-0.4,0)
            time.sleep(0.2)
        elif i > 60 and i <= 65:
            print(i)
            send_local_ned_velocity(-0.9,0,0)
            time.sleep(0.2)
        elif i > 65 and i <= 70:
            print(i)
            send_local_ned_velocity(0,0,0)
            time.sleep(0.2)
        elif i > 70 and i <= 75:
            print(i)
            send_local_ned_velocity(0,1,0)
            time.sleep(0.2)
        elif i > 75 and i <= 80:
            print(i)
            send_local_ned_velocity(0,-1,0)
            time.sleep(0.2)
        elif i > 80 and i <= 85:
            print(i)
            send_local_ned_velocity(0.7,0,0)
            time.sleep(0.2)
        elif i > 85 and i <= 90:
            print(i)
            send_local_ned_velocity(1,0,0)
            time.sleep(0.2)
        elif i > 90 and i <= 95:
            print(i)
            send_local_ned_velocity(0,-0.1,0)
            time.sleep(0.2)
        elif i <=100:
            print(i)
            send_local_ned_velocity(0,0,0)
            time.sleep(0.2)

        i += 1
        #time.sleep(1)
        print("Done!") 