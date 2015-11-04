/*
 * Test application for Multiwii Serial Protocol library
 * Copyright (c) 2015 Michael Carter
 *
 * zerosignal1982@gmail.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include "mwsp/mwsp.h"
#include "mwsp/mwsp_util.h"

void usage(char *argv[]) {
    fprintf(stderr, "Usage: %s -s <serial_port>\n", argv[0]);
}

/* main */
int main(int argc, char *argv[])
{
    int opt, fd;
    char *serial_port;

    if(argc != 3){
        usage(argv);
        exit(EXIT_FAILURE);
    }

    while ((opt = getopt(argc, argv, "s:")) != -1) {
        switch(opt) {
            case 's':
                serial_port = malloc(sizeof(char) * (strlen(optarg) + 1));
                strncpy(serial_port, optarg, strlen(optarg));

                break;
            default:
                usage(argv);
                exit(EXIT_FAILURE);
        }
    }

    printf("Connecting to flight controller\n");

    /* connect the flight controller */
    fd = mwsp_connect(serial_port);
    if(fd < 0){
        perror("Failed to connect to serial port"); 
        exit(-1);
    }

    /* do some stuff with the flight controller */
    int ret;

    /*------------------------------------------------------*/

    /* MWSP_IDENT request */
    mwsp_ident ident;
    ret = mwsp_get_ident(fd, &ident);

    if(ret < 0) {
        printf("Error requesting ident: %i\n", ret);
    }

    /*------------------------------------------------------*/

    /* MWSP_STATUS request */
    mwsp_status status;
    ret = mwsp_get_status(fd, &status);

    if(ret < 0) {
        printf("Error requesting status: %i\n", ret);
    }

    /*------------------------------------------------------*/

    /* MWSP_RAW_IMU request */
    mwsp_raw_imu raw_imu;
    ret = mwsp_get_raw_imu(fd, &raw_imu);

    if(ret < 0) {
        printf("Error requesting raw_imu: %i\n", ret);
    }

    /*------------------------------------------------------*/

    /* MWSP_SERVO request */
    mwsp_servo servo;
    ret = mwsp_get_servo(fd, &servo);

    if(ret < 0) {
        printf("Error requesting servo: %i\n", ret);
    }

    /*------------------------------------------------------*/

    /* MWSP_MOTOR request */
    mwsp_motor motor;
    ret = mwsp_get_motor(fd, &motor);

    if(ret < 0) {
        printf("Error requesting motor: %i\n", ret);
    }

    /*------------------------------------------------------*/

    /* MWSP_RC request */
    mwsp_rc rc;
    ret = mwsp_get_rc(fd, &rc);

    if(ret < 0) {
        printf("Error requesting rc: %i\n", ret);
    }

    printf("Timing RC refresh rate (Hz): ");

    int cnt = 0;
    uint64_t now = mwsp_get_time();
    uint64_t end = now + NS_PER_SEC;

    /* MWSP_SET_RAW_RC set */
    rc.data.members.roll  = 1000;
    rc.data.members.pitch = 1000;
    rc.data.members.yaw   = 1000;
    rc.data.members.thro  = 1000;
    rc.data.members.aux1  = 1000;
    rc.data.members.aux2  = 1000;
    rc.data.members.aux3  = 1000;
    rc.data.members.aux4  = 1000;

    /*
    while(now < end){
        ret = mwsp_set_raw_rc(fd, &rc);

        now = mwsp_get_time();
        cnt++;
    }
    */

    printf("%i\n", cnt);

    /*------------------------------------------------------*/

    /* MWSP_RAW_GPS request */
    mwsp_raw_gps raw_gps;
    ret = mwsp_get_raw_gps(fd, &raw_gps);

    if(ret < 0) {
        printf("Error requesting raw_gps: %i\n", ret);
    }

    /* MWSP_RAW_GPS set */
    ret = mwsp_set_raw_gps(fd, &raw_gps);

    if(ret < 0) {
        printf("Error setting raw_gps: %i\n", ret);
    }

    /*------------------------------------------------------*/

    /* MWSP_ATTITUDE request */
    mwsp_attitude attitude;
    ret = mwsp_get_attitude(fd, &attitude);

    if(ret < 0) {
        printf("Error requesting attitude: %i\n", ret);
    }

    /*------------------------------------------------------*/

    /* MWSP_ALTITUDE request */
    mwsp_altitude altitude;
    ret = mwsp_get_altitude(fd, &altitude);

    if(ret < 0) {
        printf("Error requesting altitude: %i\n", ret);
    }

    /*------------------------------------------------------*/

    /* MWSP_ANALOG request */
    mwsp_analog analog;
    ret = mwsp_get_analog(fd, &analog);

    if(ret < 0) {
        printf("Error requesting analog: %i\n", ret);
    }

    /*------------------------------------------------------*/

    /* MWSP_RC_TUNING request */
    mwsp_rc_tuning rc_tuning;
    ret = mwsp_get_rc_tuning(fd, &rc_tuning);

    if(ret < 0) {
        printf("Error requesting RC tuning: %i\n", ret);
    }

    /*------------------------------------------------------*/

    /* MWSP_PID request */
    mwsp_pid pid;
    ret = mwsp_get_pid(fd, &pid);

    if(ret < 0) {
        printf("Error requesting PID: %i\n", ret);
    }

    /*------------------------------------------------------*/

    /* MWSP_MISC request */
    mwsp_misc misc;
    ret = mwsp_get_misc(fd, &misc);

    if(ret < 0) {
        printf("Error requesting misc: %i\n", ret);
    }

    /*------------------------------------------------------*/

    /* MWSP_MOTOR_PINS request */
    mwsp_motor_pins pins;
    ret = mwsp_get_motor_pins(fd, &pins);

    if(ret < 0) {
        printf("Error requesting motor pins: %i\n", ret);
    }

    /*------------------------------------------------------*/

    /* MWSP_WP request */
    mwsp_wp wp;
    ret = mwsp_get_wp(fd, &wp);

    if(ret < 0) {
        printf("Error requesting waypoints: %i\n", ret);
    }

    /*------------------------------------------------------*/

    /* MWSP_SELECT_SETTING request */
    mwsp_setting setting;
    setting.data.members.setting = 0;

    ret = mwsp_set_setting(fd, &setting);

    if(ret < 0) {
        printf("Error selecting setting: %i\n", ret);
    }

    /*------------------------------------------------------*/

    /* MWSP_SET_HEAD request */
    mwsp_heading heading;
    heading.data.members.heading = 120;

    ret = mwsp_set_head(fd, &heading);

    if(ret < 0) {
        printf("Error setting heading: %i\n", ret);
    }

    /*------------------------------------------------------*/

    /* MWSP_SET_MOTOR request */
    mwsp_motor motor_s;
    memset(&motor_s, 0, sizeof(motor_s));

    motor_s.data.members.motor1 = 1000;
    motor_s.data.members.motor2 = 2000;
    motor_s.data.members.motor3 = 1000;
    motor_s.data.members.motor4 = 2000;
    motor_s.data.members.motor5 = 1000;
    motor_s.data.members.motor6 = 2000;
    motor_s.data.members.motor7 = 1000;
    motor_s.data.members.motor8 = 2000;

    ret = mwsp_set_motor(fd, &motor_s);

    if(ret < 0) {
        printf("Error setting motors: %i\n", ret);
    }

    /*------------------------------------------------------*/

    printf("Calibrating accelerometer. Wait 5s.\n");

    /* MWSP_ACC_CALIBRATION command */
    ret = mwsp_acc_calibration(fd);

    if(ret < 0) {
        printf("Error calibrating acc: %i\n", ret);
    }

    /*------------------------------------------------------*/

    sleep(5);

    /*------------------------------------------------------*/

    /* MWSP_MAG_CALIBRATION command */
    /*
    ret = mwsp_mag_calibration(fd);

    if(ret < 0) {
        printf("Error calibrating mag: %i\n", ret);
    }

    sleep(30);
    */

    /*------------------------------------------------------*/

    printf("WARNING: Arming multicopter in 5s. Ctrl-C to abort!\n");

    sleep(5);

    ret = mwsp_arm(fd);
    if(ret < 0) {
        printf("Error arming multicopter: %i\n", ret);
    }

    /*------------------------------------------------------*/

    printf("WARNING: Disarming multicopter in 5s. Ctrl-C to abort!\n");

    sleep(5);

    ret = mwsp_disarm(fd);
    if(ret < 0) {
        printf("Error arming multicopter: %i\n", ret);
    }

    /*------------------------------------------------------*/

    printf("Disconnecting from flight controller\n");

    /* disconnect the flight controller */
    if(mwsp_disconnect(fd) < 0){
        printf("Something went wrong disconnecting!\n");
        exit(-2);
    }
}
