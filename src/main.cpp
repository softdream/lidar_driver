#include <iostream>
#include <unistd.h>
#include <stdlib.h>
#include "rplidar.h"
#include <signal.h>
#include <stdio.h>
#include <string.h>

#include "dataType.h"
#include "laserDriver.h"
#include "timestamp.h"
#include <fstream>


using namespace rp::standalone::rplidar;


bool ctrl_c_pressed;
void ctrlc(int)
{
    ctrl_c_pressed = true;
}


int main()
{
	std::cout<<"progress begins ..."<<std::endl;

	slam::sensor::LaserScan scan;

	long previous_time = 0;
	long current_time = 0;
	long delta_time = 0;

	const char * opt_com_path = "/dev/ttyUSB1";
	_u32 opt_com_baudrate = 115200;
	u_result op_result;
	
	bool angle_compensate = true;
	int angle_compensate_multiple = 1;
	bool inverted = false;	

	// create the driver instance
        RPlidarDriver * drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);

	if (!drv) {
        	fprintf(stderr, "insufficent memory, exit\n");
        	return false;
    	}
	
	rplidar_response_device_info_t devinfo;
	if (IS_OK(drv->connect(opt_com_path, opt_com_baudrate)))
        {
            op_result = drv->getDeviceInfo(devinfo);

            if (IS_OK(op_result))
            {   
              	std::cout<<"connected successfully ..."<<std::endl;
            }
            else
            {
		std::cout<<"get device information failed"<<std::endl;
                delete drv;
                drv = NULL;
            }
        }
	else{
		std::cout<<"connected failed ..."<<std::endl;
	}

	drv->startMotor();
	drv->startScan( 0, 1 );
	
	uint32_t scan_num  = 0;
	while(1){
		rplidar_response_measurement_node_hq_t nodes[360 * 8];
		size_t   count = _countof(nodes);
		op_result = drv->grabScanDataHq( nodes, count );
		
		if( op_result == RESULT_OK ){
			scan_num ++;
			if( scan_num == 1 ){
				current_time = previous_time = getCurrentTime_ms();
				continue;
			}
			current_time = getCurrentTime_ms();
			delta_time = current_time - previous_time;
			std::cout<<"delta_time = "<<delta_time<<std::endl;
			op_result = drv->ascendScanData( nodes, count );
			float angle_min = DEG2RAD( 0.0f );
			float angle_max = DEG2RAD( 359.0f );
			if( op_result == RESULT_OK ){
				if( angle_compensate ){
					std::cout<<"in here ..."<<std::endl;
					const int angle_compensate_nodes_count = 360 * angle_compensate_multiple;
					int angle_compensate_offset = 0;
					rplidar_response_measurement_node_hq_t angle_compensate_nodes[ angle_compensate_nodes_count ];
					memset( angle_compensate_nodes, 0, angle_compensate_nodes_count * sizeof( rplidar_response_measurement_node_hq_t ) );
					
					for( int i = 0; i < count; i ++ ){
						if( nodes[i].dist_mm_q2 != 0 ){
							float angle = getAngle( nodes[i] );
							float angle_value = (int)( angle * angle_compensate_multiple );
							if( ( angle_value - angle_compensate_offset ) < 0 ) angle_compensate_offset = angle_value;
							for( int j = 0; j < angle_compensate_multiple; j ++ ){
								int angle_compensate_nodes_index = angle_value - angle_compensate_offset + j ;
								if( angle_compensate_nodes_index >= angle_compensate_nodes_count )
									angle_compensate_nodes_index = angle_compensate_nodes_count - 1 ;
								angle_compensate_nodes[angle_compensate_nodes_index] = nodes[i];
							}
						}
					}
					
					publish_scan( angle_compensate_nodes, scan, angle_compensate_nodes_count, \
						        current_time, ( double )( delta_time * 0.001 ), inverted, angle_min, angle_max, 8.0, scan_num );
				}
				else{
					int start_node = 0, end_node = 0;
					int i = 0;	
					while( nodes[i++].dist_mm_q2 == 0 );
					start_node = i - 1;
					i = count - 1;
					while( nodes[i--].dist_mm_q2 == 0 );
					end_node = i + 1;

					angle_min = DEG2RAD( getAngle( nodes[start_node] ) );
					angle_max = DEG2RAD( getAngle( nodes[end_node] ) );

					publish_scan( nodes, scan, count, current_time, ( double )( delta_time * 0.001 ), inverted, angle_min, angle_max, 8.0, scan_num );
				}
			}
			else if( op_result == RESULT_OPERATION_FAIL ){
				float angle_min = DEG2RAD( 0.0f );
				float angle_max = DEG2RAD( 359.0f );
				//publish_scan(  );
			}

			previous_time = current_time;
		}
		
		if( ctrl_c_pressed ){
			break;
		}
        }
	
	drv->stop();
	drv->stopMotor();
	RPlidarDriver::DisposeDriver(drv);
    	drv = NULL;

	
	return 0;
}
