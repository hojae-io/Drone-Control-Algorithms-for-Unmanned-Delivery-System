#!/usr/bin/env python3

# MAVSDK
import asyncio
from mavsdk import System
from mavsdk.mission import (MissionItem, MissionPlan)

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

    # Callbacks
    def GPS_callback(self, msg):
        global target_gps
        target_gps.latitude = msg.latitude
        target_gps.longitude = msg.longitude

        print('GPS_callback called')

async def run():
    drone = System()
    await drone.connect(system_address="serial:///dev/ttyUSB0")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"Drone discovered with UUID: {state.uuid}")
            break
    
    ##########################
    ### Mission Mode Start ###
    ##########################

    print_mission_progress_task = asyncio.ensure_future(print_mission_progress(drone))

    running_tasks = [print_mission_progress_task]
    termination_task = asyncio.ensure_future(observe_is_in_air(drone, running_tasks))

    mission_items = []
    #parameters = (latitude_deg, longitude_deg, relative_altitude_m, speed_m_s, is_flying_through
    #               gimbal_pitch, gimbal_yaw, camera_action, loiter_time_s, camera_photo_interval_s)
    mission_items.append(MissionItem(37.4539753,
                                     126.9518641,
                                     1,
                                     2,
                                     True,
                                     float('nan'),
                                     float('nan'),
                                     MissionItem.CameraAction.NONE,
                                     float('nan'),
                                     float('nan')))
    mission_items.append(MissionItem(37.4540048,
                                     126.9518121,
                                     1,
                                     2,
                                     True,
                                     float('nan'),
                                     float('nan'),
                                     MissionItem.CameraAction.NONE,
                                     float('nan'),
                                     float('nan')))
    mission_items.append(MissionItem(target_gps.latitude,
                                     target_gps.longitude,
                                     1,
                                     2,
                                     True,
                                     float('nan'),
                                     float('nan'),
                                     MissionItem.CameraAction.NONE,
                                     float('nan'),
                                     float('nan')))

    mission_plan = MissionPlan(mission_items)

    await drone.mission.set_return_to_launch_after_mission(True)

    print("-- Uploading mission")
    await drone.mission.upload_mission(mission_plan)
    await asyncio.sleep(1)

    print("-- Arming")
    await drone.action.arm()

    print("-- Starting mission")
    await drone.mission.start_mission()

    await termination_task

async def print_mission_progress(drone):
    async for mission_progress in drone.mission.mission_progress():
        print(f"Mission progress: "
              f"{mission_progress.current}/"
              f"{mission_progress.total}")


async def observe_is_in_air(drone, running_tasks):
    """ Monitors whether the drone is flying or not and
    returns after landing """

    was_in_air = False

    async for is_in_air in drone.telemetry.in_air():
        if is_in_air:
            was_in_air = is_in_air

        if was_in_air and not is_in_air:
            for task in running_tasks:
                task.cancel()
                try:
                    await task
                except asyncio.CancelledError:
                    pass
            await asyncio.get_event_loop().shutdown_asyncgens()

            return

def main():
    rospy.init_node('SNU_drone', anonymous=True)

    loop = asyncio.get_event_loop()
    loop.run_until_complete(run())

    cnt = Controller()

    rospy.Subscriber('/Target_GPS_msg', Target_GPS, cnt.GPS_callback)
    rospy.sleep(0.2)

if __name__ == "__main__":
    main()

