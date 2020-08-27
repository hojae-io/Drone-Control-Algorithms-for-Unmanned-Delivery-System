// Publisher Header
#include "ros/ros.h"
#include "GPS_topic/Target_GPS.h"

// GPS code Header
#include <gps.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>

using namespace std;

int main(int argc, char **argv) {
	// ros Publisher 생성
	ros::init(argc, argv, "GPS_publisher");
	ros::NodeHandle nh;

	// Publisher 이름: GPS_pub, 메시지 이름: Target_GPS_msg
	ros::Publisher GPS_pub = nh.advertise<GPS_topic::Target_GPS>("Target_GPS_msg", 100); 

	ros::Rate loop_rate(10);

	GPS_topic::Target_GPS gpsData;  //gpsData라는 메세지를 선언. 이 안에 float32 latitude, float32 longitude 2개가 있다.
	
	// GPS code
	int rc;
	struct timeval tv;

	struct gps_data_t gps_data;
	if ((rc = gps_open("localhost", "2947", &gps_data)) == -1) {
	    cout << "code: " << rc << " reason: " <<  gps_errstr(rc) << endl;
	    return EXIT_FAILURE;
	}
	gps_stream(&gps_data, WATCH_ENABLE | WATCH_JSON, NULL);

	while (ros::ok()) {
	    /* wait for 2 seconds to receive data */
	    if (gps_waiting (&gps_data, 2000000)) {
		/* read data */
		if ((rc = gps_read(&gps_data)) == -1) {
		    // printf("error occured reading gps data. code: %d, reason: %s\n", rc, gps_errstr(rc));
		} else {
		    /* Display data from the GPS receiver. */
		    if ((gps_data.status == STATUS_FIX) &&
		        (gps_data.fix.mode == MODE_2D || gps_data.fix.mode == MODE_3D) &&
		        !isnan(gps_data.fix.latitude) &&
		        !isnan(gps_data.fix.longitude)) {
		            //gettimeofday(&tv, NULL); EDIT: tv.tv_sec isn't actually the timestamp!
		            //printf("latitude: %f, longitude: %f, speed: %f, timestamp: %lf\n", gps_data.fix.latitude, gps_data.fix.longitude, 
					//        gps_data.fix.speed, gps_data.fix.time); //EDIT: Replaced tv.tv_sec with gps_data.fix.time
					gpsData.latitude = gps_data.fix.latitude;
					gpsData.longitude = gps_data.fix.longitude;

					ROS_INFO("Targer latitude = %f", gpsData.latitude);
					ROS_INFO("Targer longitude = %f", gpsData.longitude);
					GPS_pub.publish(gpsData);

					loop_rate.sleep();

		    } else {
		        cout << "no GPS data available" << endl;
		    }
			}	
	    }

	    sleep(3);
	}

	/* When you are done... */
	gps_stream(&gps_data, WATCH_DISABLE, NULL);
	gps_close (&gps_data);

	return EXIT_SUCCESS;
}


