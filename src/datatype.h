#ifndef __DATATYPE_H_
#define __DATATYPE_H_

#include <iostream>
#include <stdint.h>

typedef struct {
        char head;//1
        uint32_t length; // 1
        uint32_t time_stamp; //4
        float gx;//4
        float gy;//4
        float gz;//4
        float ax;//4
        float ay;//4
        float az;//4
        float vl;
        float vr;
        char tail; //1
}IMU;

typedef struct{
        float x;
        float y;
        float theta;
        float v;
        float w;
}STATE;

typedef enum{
	forward,
	backward,
	turnleft,
	turnright,
	stop
}Command;

typedef struct{
	float pose_x;
	float pose_y;
	float pose_theta;
}position;

typedef struct{
        uint32_t time_stamp;
        uint32_t frame_id;

        float angle_min;
        float angle_max;
        float angle_increment;
        float scan_time;
        float time_increment;
        float range_min;
        float range_max;
        float ranges[360];
        float intensities[360];
}LidarData;


void printData( IMU imu )
{
        std::cout << "imu.head = " << imu.head << std::endl;
        std::cout << "imu.length = " << imu.length << std::endl;
        std::cout << "imu.time_stamp = " << imu.time_stamp << std::endl;
        std::cout << "imu.gx = " << imu.gx << std::endl;
        std::cout << "imu.gy = " << imu.gy << std::endl;
        std::cout << "imu.gz = " << imu.gz << std::endl;
        std::cout << "imu.ax = " << imu.ax << std::endl;
        std::cout << "imu.ay = " << imu.ay << std::endl;
        std::cout << "imu.az = " << imu.az << std::endl;
        std::cout << "imu.vl = " << imu.vl << std::endl;
        std::cout << "imu.vr = " << imu.vr << std::endl;
        std::cout << "imu.tail = " << imu.tail << std::endl;
	std::cout << "--------------------"<<std::endl;
}


#endif
