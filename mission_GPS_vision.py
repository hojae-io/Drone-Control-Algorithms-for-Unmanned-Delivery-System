#!/usr/bin/env python3

# MAVSDK
import asyncio
from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw)

# ROS python API
import rospy

# import a message from Raspberry pi CAMERA
# Message name: /Target_coord_msg
from targetDetection_topic.msg import TargetPosition
# import a message from Raspberry pi GPS
# Message name: /Target_GPS_msg
from GPS_topic.msg import Target_GPS

# Instantiate a TargetPosition & Target_GPS message
target_coord = TargetPosition()
target_gps = Target_GPS()

class Controller:
    # initialization method
    def __init__(self):
        pass

    # Callbacks
    def Coord_callback(self, msg):
        global target_coord
        target_coord.x_coord = msg.x_coord
        target_coord.y_coord = msg.y_coord
        target_coord.conf = msg.conf        # confidence whether object is detected in the camera

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
    
    print_mission_progress_task = asyncio.ensure_future(print_mission_progress(drone))

    running_tasks = [print_mission_progress_task]
    termination_task = asyncio.ensure_future(observe_is_in_air(drone, running_tasks))

    mission_items = []
    #parameters = (latitude_deg, longitude_deg, relative_altitude_m, speed_m_s, is_flying_through
    #               gimbal_pitch, gimbal_yaw, camera_action, loiter_time_s, camera_photo_interval_s)
    # Waypoint1
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
    # Waypoint2
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
    # Waypoint3
    mission_items.append(MissionItem(37.454065,
                                     126.9518092,
                                     1,
                                     2,
                                     True,
                                     float('nan'),
                                     float('nan'),
                                     MissionItem.CameraAction.NONE,
                                     float('nan'),
                                     float('nan')))

    mission_plan = MissionPlan(mission_items)

    await drone.mission.set_return_to_launch_after_mission(False)

    print("-- Uploading mission")
    await drone.mission.upload_mission(mission_plan)
    await asyncio.sleep(1)

    print("-- Arming")
    await drone.action.arm()

    ##########################################
    ### Departing a Truck & Mission Starts ###
    ##########################################

    print("-- Starting mission")
    await drone.mission.start_mission()

    await termination_task

    ##########################################
    ### Mission Ends  & Go back to a Truck ###
    ##########################################
    while True:
        # if the target is not within the camera yet
        if target_coord.conf < 0.9:
            await drone.action.goto_location(target_gps.latitude, target_gps.longitude, flying_alt, 0)
        
        # if the target is detected within the camera, chanage to offboard mode to finetune the drone's position
        else:
            if not drone.offboard.is_active:
                print("-- Setting initial setpoint")
                await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))

                print("-- Starting offboard")
                try:
                    await drone.offboard.start()
                except OffboardError as error:
                    print(f"Starting offboard mode failed with error code: {error._result.result}")
                    print("-- Disarming")
                    await drone.action.disarm()
                    return
            else:
                print(f"-- Go {target_coord.x_coord}m North, {target_coord.y_coord}m East, within local coordinate system")
                await drone.offboard.set_position_ned(PositionNedYaw(target_coord.x_coord, target_coord.y_coord, 0.0, 0.0))
    

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

    rospy.Subscriber('/Target_coord_msg', TargetPosition, cnt.Coord_callback)
    rospy.sleep(0.2)
    
    rospy.Subscriber('/Target_GPS_msg', Target_GPS, cnt.GPS_callback)
    rospy.sleep(0.2)

if __name__ == "__main__":
    main()


