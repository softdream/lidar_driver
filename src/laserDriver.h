#ifndef __LIDARDRIVER_H_
#define _LIDARDRIVER_H_

#include <limits>
#include "rplidar.h"
#include "datatype.h"
#include <iostream>
#include <string.h>

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#define M_PI       3.14159265358979323846
#define DEG2RAD(x) ((x)*M_PI/180.)


using namespace rp::standalone::rplidar;


float getAngle(const rplidar_response_measurement_node_hq_t& node)
{
        return node.angle_z_q14 * 90.f / 16384.f;
}

bool checkRPLIDARHealth(RPlidarDriver * drv)
{
    u_result     op_result;
    rplidar_response_device_health_t healthinfo;


    op_result = drv->getHealth(healthinfo);
    if (IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
        printf("RPLidar health status : %d\n", healthinfo.status);
        if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
            fprintf(stderr, "Error, rplidar internal error detected. Please reboot the device to retry.\n");
            // enable the following code if you want rplidar to be reboot by software
            // drv->reset();
            return false;
        } else {
            return true;
        }

    } else {
        fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
        return false;
    }
}


void publish_scan( rplidar_response_measurement_node_hq_t *nodes, \
		   size_t node_count, \
		   uint32_t time_start,\
		   double scan_time, bool inverted,\
		   float angle_min, float angle_max,\
		   float max_distance,\
		   uint32_t frame_id )
{
	int scan_count = 0;
	LidarData scan_msg;
	scan_msg.time_stamp = time_start;
	scan_msg.frame_id = frame_id;
	scan_count ++;
	
	bool reversed = ( angle_max > angle_min );
	if( reversed ){
		scan_msg.angle_min = M_PI - angle_max;
		scan_msg.angle_max = M_PI - angle_min;
	}	
	else{
		scan_msg.angle_min = M_PI - angle_min;
		scan_msg.angle_max = M_PI - angle_max;
	}
	
	scan_msg.angle_increment = ( scan_msg.angle_max - scan_msg.angle_min ) / (double)( node_count - 1 );
	
	scan_msg.scan_time = scan_time;
	scan_msg.time_increment = scan_time / ( double )( node_count - 1 );
	
	scan_msg.frame_id = frame_id;	

	scan_msg.range_min = 0.15;
	scan_msg.range_max = max_distance ;//8.0

	bool reverse_data = ( !inverted && reversed ) || ( inverted && !reversed );
	if( !reverse_data ){
		for( size_t i = 0; i < node_count; i ++ ){
			float read_value = ( float )nodes[i].dist_mm_q2 / 4.0f / 1000;
			if( read_value == 0 ){
				scan_msg.ranges[i] = std::numeric_limits<float>::infinity();
			}
			else{
				scan_msg.ranges[i] = read_value;
			}
			scan_msg.intensities[i] = ( float )( nodes[i].quality >> 2 );
		}
	}
	else{
		for( size_t i = 0; i < node_count; i ++ ){
			float read_value = ( float )nodes[i].dist_mm_q2 / 4.0f / 1000;
			if( read_value == 0 ){
				scan_msg.ranges[ node_count - 1 - i ] = std::numeric_limits<float>::infinity(); 
			}
			else{
				scan_msg.ranges[ node_count - 1 - i ] = read_value;
			}
			scan_msg.intensities[ node_count - 1 - i ] =  ( float )( nodes[i].quality >> 2 );
		}
	}

	/* 打印测试结果 */
	std::cout << "frame_id: " << scan_msg.frame_id<<std::endl;
	std::cout << "time_stamp: "<< scan_msg.time_stamp<<std::endl;
	std::cout << " angle_max: " << scan_msg.angle_max << " angle_min: " << scan_msg.angle_min <<" angle_increment: "<<scan_msg.angle_increment<< std::endl;
	std::cout << " scan_time：" << scan_msg.scan_time << " time_increment: " << scan_msg.time_increment << std::endl;
	/*for (size_t i = 0; i < node_count; i++){
		std::cout << "ranges[" << i << "]: " << scan_msg.ranges[i] << std::endl;
	}*/
}


#endif


















