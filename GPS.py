#!/usr/bin/env python3

# MAVSDK
import asyncio
from mavsdk import System

# ROS python API
import rospy

# import a message from Raspberry pi GPS
# Message name: /Target_GPS_msg
from GPS_topic.msg import Target_GPS

# Instantiate a Target_GPS message
target_gps = Target_GPS()

class Controller:
    # initialization method
    def __init__(self):
        pass

    def GPS_callback(self, msg):
        global target_gps
        target_gps.latitude = msg.latitude
        target_gps.longitude = msg.longitude

# Execution: python3
async def run():
    drone = System()
    await drone.connect(system_address="serial:///dev/ttyUSB0")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"Drone discovered with UUID: {state.uuid}")
            break

    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok:
            print("Global position estimate ok")
            break

    print("Fetching amsl altitude at home location....")
    async for terrain_info in drone.telemetry.home():
        absolute_altitude = terrain_info.absolute_altitude_m
        break

    print("-- Arming")
    await drone.action.arm()
    
    print("-- Taking off")
    await drone.action.takeoff()

    await asyncio.sleep(1)
    flying_alt = absolute_altitude + 2.5 #To fly drone 3m above the ground plane
    
    while True:
        #goto_location() takes Absolute MSL altitude 
        await drone.action.goto_location(target_gps.latitude, target_gps.longitude, flying_alt, 0)


def main():
    rospy.init_node('SNU_drone', anonymous=True)
    cnt = Controller()
    
    rospy.Subscriber('/Target_GPS_msg', Target_GPS, cnt.GPS_callback)
    rospy.sleep(1)

    loop = asyncio.get_event_loop()
    loop.run_until_complete(run())


if __name__ == "__main__":
    main()



