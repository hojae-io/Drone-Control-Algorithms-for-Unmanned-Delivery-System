#include <gps.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>

int main() {
int rc;
struct timeval tv;

struct gps_data_t gps_data;
if ((rc = gps_open("localhost", "2947", &gps_data)) == -1) {
    printf("code: %d, reason: %s\n", rc, gps_errstr(rc));
    return EXIT_FAILURE;
}
gps_stream(&gps_data, WATCH_ENABLE | WATCH_JSON, NULL);

while (1) {
    /* wait for 2 seconds to receive data */
    if (gps_waiting (&gps_data, 2000000)) {
        /* read data */
        if ((rc = gps_read(&gps_data)) == -1) {
            printf("error occured reading gps data. code: %d, reason: %s\n", rc, gps_errstr(rc));
        } else {
            /* Display data from the GPS receiver. */
            if ((gps_data.status == STATUS_FIX) && 
                (gps_data.fix.mode == MODE_2D || gps_data.fix.mode == MODE_3D) &&
                !isnan(gps_data.fix.latitude) && 
                !isnan(gps_data.fix.longitude)) {
                    //gettimeofday(&tv, NULL); EDIT: tv.tv_sec isn't actually the timestamp!
                    printf("latitude: %f, longitude: %f, speed: %f, timestamp: %lf\n", gps_data.fix.latitude, gps_data.fix.longitude, gps_data.fix.speed, gps_data.fix.time); //EDIT: Replaced tv.tv_sec with gps_data.fix.time
            } else {
                printf("no GPS data available\n");
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
