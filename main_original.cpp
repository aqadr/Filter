#include <vector>
#include <iostream>
#include <stdio.h>
#include <cstdlib>
#include <cstdio>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include "Socket.h" 
#define MYPORT "1600"
#include "Socket.h"
#include <sys/ioctl.h>
#include <time.h>
#include "custom_timer.h"
#include "serial_setup.h"
// pthread header
#include <pthread.h>
#include "vehicle_attitude.h"

/** kalman filtering for attitude*/

#include "codegen/attitudeKalmanfilter_initialize.h"
#include "codegen/attitudeKalmanfilter.h"


#include "ins.h"
#include "LinePolar.h"
#include "measurement.h"
#define NUM_THREADS	5


typedef uintptr_t	param_t;

using namespace std;

/**
 * test pthread function
 */


/*struct lidar_data{
  
  uint64_t time;
  char *ptr;
};
*/
struct lidar_data lidar_data_array;



/**
 * timer
 */
struct timespec requestStart, requestEnd;


/** 
 * lidar output stored in the file
 */
FILE *outfile;

FILE *acc_outfile;

/**
 * socket data receiving
 */
Socket test_socket;
custom_timer test;

/** print data modified
 */
static void test_print_data(char data[], int data_n) {
  
  fprintf(outfile,"Laser  %d\n ",data_n);
  for (int i = 0; i < data_n; ++i) {
      char l = data[i];
      fprintf(outfile,"%c", l);
    }
}


/**
 * variables for lidar data 
 */
char buf[100];
int num_bytes;
int bytes_available;
char buffer[15000];
char *ptr;
bool lidar_info=true;

/**
 * function for lidar data processing
 */
void *lidar_data_update(void *threadarg) {
   
    ptr=buffer;
    int n;
    outfile = fopen("/home/ashraf/Desktop/client_code/output.txt", "w");  // using absolute path name of file
    if (outfile == NULL) { 
	fprintf(stderr, "Unable to open file.\n");
    }
  
   
    test_socket.create_client();
    char *ptr=NULL;
    ptr =(char *) malloc (5500);
    
    int index=0;
    while(lidar_info=true) {
      
	test_socket.recv_data(ptr);
	uint64_t current_time=test.get_current_time();
	int data_size = strlen(ptr);
	lidar_data_array.ptr=ptr;
	lidar_data_array.time = current_time;
	test_print_data( lidar_data_array.ptr, data_size);  
//	 cout<<"current_time:"<<current_time<<endl;
	fprintf(outfile,"\n");
      
    }
 
    fclose(outfile);
    fclose(acc_outfile);
    
    pthread_exit(NULL);
}


double velocity[3]={0.0f, 0.0f, 0.0f};
double position[3]={0.0f, 0.0f, 0.0f};



void vel_estimation(double vel[3],float acc[3], uint64_t dt) {
  
    vel[0] =vel[0]+((double)dt/1000000.0)*acc[0];
    vel[1] =vel[1]+((double)dt/1000000.0)*acc[1];
    vel[2] =vel[2]+((double)dt/1000000.0)*acc[2];
    
}

void pos_estimation(double pos[3],double vel[3], float acc[3], uint64_t dt) {
  
    pos[0] =pos[0]+vel[0]*((double)dt/1000000.0)-acc[0]*((double)dt/1000000.0)*((double)dt/1000000.0)/2.0;
    pos[1] =pos[1]+vel[1]*((double)dt/1000000.0)-acc[1]*((double)dt/1000000.0)*((double)dt/1000000.0)/2.0;
    pos[2] =pos[2]+vel[2]*((double)dt/1000000.0)-acc[2]*((double)dt/1000000.0)*((double)dt/1000000.0)/2.0;
}
  










#define CONSTANTS_ONE_G -9.81

#define HRT_COUNTER_SCALE(_c)	(_c)

// Settings
int sysid = 42;             ///< The unique system id of this MAV, 0-127. Has to be consistent across the system
int compid = 110;
int serial_compid = 0;
bool silent = false;              ///< Wether console output should be enabled
bool verbose = false;             ///< Enable verbose output
bool debug = false;               ///< Enable debug functions and output
int fd;
int acc_cnt=0;


double accel_xyz[3];
double accel_x[2], accel_y[2], accel_z[2];
double vel_x[2], vel_y[2],vel_z[2];
double pos_x[2], pos_y[2],pos_z[2];

double att_R[3][3];
float accelerometer_m_s2[3]={0.0f, 0.0f, 0.0f};

float bias[3]={0.0f, 0.0f, 0.0f};

mavlink_highres_imu_t imu;

/* acceleration in NED frame */
float accel_NED[3] = { 0.0f, 0.0f, -9.81f };
float accel_correct[3] = {0.0f,0.0f,0.0f};

float acc_data[100][3];

float acc_mean[3]={0.0f,0.0f,0.0f};
float st_dev[3]={0.0f,0.0f,0.0f};


uint64_t time_current;
uint64_t dt;



/**
 * @brief Serial function
 *
 * This function blocks waiting for serial data in it's own thread
 * and forwards the data once received.
 */
int serial_wait(int serial_fd) {
	int fd = serial_fd;
	
	mavlink_status_t lastStatus;
	lastStatus.packet_rx_drop_count = 0;
	static unsigned int att_receive_counter =0;
	static unsigned int scaled_imu_receive_counter = 0;
	int cnt=0;
	// Blocking wait for new data
//	accel_NED[0]-bias[0], accel_NED[1]-bias[1], accel_NED[2]-bias[2],imu.pressure_alt,imu.xgyro, imu.ygyro, imu.zgyro,position[0],position[1],position[2]
	fprintf(acc_outfile,"acc_x,acc_y,acc_z,imu.pressure_alt,imu.xgyro, imu.ygyro, imu.zgyro, vel_x, vel_y, vel_z, position_x,position_y, position_z, dt\n ");
	while (lidar_info=true)
	{
		//if (debug) printf("Checking for new data on serial port\n");
		// Block until data is available, read only one byte to be able to continue immediately
		//char buf[MAVLINK_MAX_PACKET_LEN];
		uint8_t cp;
		mavlink_message_t message;
		mavlink_status_t status;
		uint8_t msgReceived = false;

		if (read(fd, &cp, 1) > 0)
		{
			// Check if a message could be decoded, return the message in case yes
			msgReceived = mavlink_parse_char(MAVLINK_COMM_1, cp, &message, &status);
			if (lastStatus.packet_rx_drop_count != status.packet_rx_drop_count)
			{
				if (verbose || debug) printf("ERROR: DROPPED %d PACKETS\n", status.packet_rx_drop_count);
				if (debug)
				{
					unsigned char v=cp;
					fprintf(stderr,"%02x ", v);
				}
			}
			lastStatus = status;
		}
		else
		{
			if (!silent) fprintf(stderr, "ERROR: Could not read from fd %d\n", fd);
		}
		
		// If a message could be decoded, handle it
		
//		cout<<"try the time here:"<<test.get_current_time()<<endl;
		if(msgReceived)
		{
			//if (verbose || debug) std::cout << std::dec << "Received and forwarded serial port message with id " << static_cast<unsigned int>(message.msgid) << " from system " << static_cast<int>(message.sysid) << std::endl;
			
			// Do not send images over serial port
			
			// DEBUG output
			if (debug)
			{
				fprintf(stderr,"Received serial data: ");
				unsigned int i;
				uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
				unsigned int messageLength = mavlink_msg_to_send_buffer(buffer, &message);
				if (messageLength > MAVLINK_MAX_PACKET_LEN)
				{
					fprintf(stderr, "\nFATAL ERROR: MESSAGE LENGTH IS LARGER THAN BUFFER SIZE\n");
				}
				else
				{
					for (i=0; i<messageLength; i++)
					{
						unsigned char v=buffer[i];
						fprintf(stderr,"%02x ", v);
					}
					fprintf(stderr,"\n");
				}
			}
			
			if (verbose || debug)
				printf("Received message from serial with ID #%d (sys:%d|comp:%d):\n", message.msgid, message.sysid, message.compid);
			
			static unsigned int local_pos_ned_receive_counter =0;					// for the local_position
			
 			switch (message.msgid)
			{
				case MAVLINK_MSG_ID_HIGHRES_IMU:
				{
					
						
					mavlink_msg_highres_imu_decode(&message, &imu);
					accelerometer_m_s2[0]=imu.xacc;
					accelerometer_m_s2[1]=imu.yacc;
					accelerometer_m_s2[2]=imu.zacc;
					
/*					printf("Got message HIGHRES_IMU (spec: https://pixhawk.ethz.ch/mavlink/#HIGHRES_IMU)\n");
					printf("\t time: %llu\n", imu.time_usec);
					printf("\t acc  (NED):\t% f\t% f\t% f (m/s^2)\n", imu.xacc, imu.yacc, imu.zacc);
					printf("\t gyro (NED):\t% f\t% f\t% f (rad/s)\n", imu.xgyro, imu.ygyro, imu.zgyro);
					printf("\t mag  (NED):\t% f\t% f\t% f (Ga)\n", imu.xmag, imu.ymag, imu.zmag);
					printf("\t baro: \t %f (mBar)\n", imu.abs_pressure);
					printf("\t altitude: \t %f (m)\n", imu.pressure_alt);
					printf("\t temperature: \t %f C\n", imu.temperature);
					printf("\n");
*/						
					

					/* transform acceleration vector from body frame to NED frame */
					for (int i = 0; i < 3; i++) {
					      accel_NED[i] = 0.0f;
					      for (int j = 0; j < 3; j++) {
						    accel_NED[i] += att_R[i][j] * accelerometer_m_s2[j];
					      }
					}
			
					if (scaled_imu_receive_counter <=200) {
					  
					    bias[0] +=accel_NED[0];
					    bias[1] +=accel_NED[1];
					    bias[2] +=accel_NED[2];
					    
					  
					}
					
					if(scaled_imu_receive_counter==200) {
					  
					  bias[0] /=201;
					  bias[1] /=201;
					  bias[2] /=201;
					}
					
					if (scaled_imu_receive_counter>200 & scaled_imu_receive_counter<=300) {
					  
					    acc_data[cnt][0]=accel_NED[0]-bias[0];
					    acc_data[cnt][1]=accel_NED[1]-bias[1];
					    acc_data[cnt][2]=accel_NED[2]-bias[2];
					  
					    cnt++;
					}
					
					if (scaled_imu_receive_counter==300) {
					  
					    for (int i =0;i<100;i++) {
						
						acc_mean[0] +=acc_data[i][0];
						acc_mean[1] +=acc_data[i][1];
						acc_mean[2] +=acc_data[i][2];
					    }
					    
					    acc_mean[0] /=100;
					    acc_mean[1] /=100;
					    acc_mean[2] /=100;
					    
					     for (int i =0;i<100;i++) {
						
						st_dev[0] +=(acc_data[i][0]-acc_mean[0])*(acc_data[i][0]-acc_mean[0]);
						st_dev[1] +=(acc_data[i][1]-acc_mean[1])*(acc_data[i][1]-acc_mean[1]);
						st_dev[2] +=(acc_data[i][1]-acc_mean[1])*(acc_data[i][1]-acc_mean[2]);
						
					    }
					    
					    st_dev[0]=sqrt(st_dev[0]/100);
					    st_dev[1]=sqrt(st_dev[1]/100);
					    st_dev[2]=sqrt(st_dev[2]/100);
					    
					}
					
					
					if (scaled_imu_receive_counter>300) {
					  
					  
					    for (int i =0; i<3; i++) {
					      
						 accel_correct[i]=  accel_NED[i]-bias[i];
					    }
					    
					    printf("acceleration_correction: %f\n",accel_correct[2]);
					    dt=imu.time_usec-time_current;
					    
					    for (int i =0; i<3; i++) {
					      
						if (fabs(accel_correct[i])<0.05){
						  
						    accel_correct[i]=0.0;
						    acc_cnt++;
						} else {
						  
						  acc_cnt=0;
						  accel_correct[2]=accel_correct[2]-acc_mean[2];
						}
					    }
					    
					    if (acc_cnt>4) {
					      
					      for (int i=0; i<3;i++) {
						
						velocity[i]=0.0;
					      }
					    } else {
					      
					      vel_estimation(velocity, accel_correct, dt);
					    }
					    pos_estimation(position,velocity, accel_correct, dt);
					    
					    
					    
					    
					    
					    
					      
						
					  
					  
					}
					  
					time_current=imu.time_usec;
					scaled_imu_receive_counter++;
					
					
					
					
					
				}
				break;
				
/*				case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
				{
					if (local_pos_ned_receive_counter % 10 == 0)
					{
						mavlink_local_position_ned_t local_pos;
						mavlink_msg_local_position_ned_decode(&message, &local_pos);
						printf("Got message LOCAL_POS (spec: https://pixhawk.ethz.ch/mavlink/#LOCAL_POS)\n");
						printf("\t time: %llu\n", local_pos.time_boot_ms);
						printf("\t pos  (NED):\t% f\t% f\t% f (m)\n", local_pos.x, local_pos.y, local_pos.z);
						printf("\t vel  (NED):\t% f\t% f\t% f (m/s)\n", local_pos.vx, local_pos.vy, local_pos.vz);
						printf("\n");
					}
					local_pos_ned_receive_counter++;
				}
				break;
*/				
				case MAVLINK_MSG_ID_ATTITUDE:
				{
					//if (att_receive_counter % 10 == 0)
					{
					  
					       
					  
						mavlink_attitude_t att;
						mavlink_msg_attitude_decode(&message, &att);
						
						att_R[0][0]=att.R_1; att_R[0][1]=att.R_2; att_R[0][2]=att.R_3; att_R[1][0]=att.R_4; att_R[1][1]=att.R_5; att_R[1][2]=att.R_6;
						att_R[2][0]=att.R_7; att_R[2][1]=att.R_8; att_R[2][2]=att.R_9;
						
/*						printf("Got message ATT (spec: https://pixhawk.ethz.ch/mavlink/#LOCAL_POS)\n");
						printf("size of att.R:  %i\n",sizeof(att));
						printf("message id: %i  \n", message.msgid);
						printf("\t time: %llu\n", att.time_boot_ms);
						printf("\t att (NED):\t% f\t% f\t% f (rad)\n", att.roll, att.pitch, att.yaw);
						printf("\t speed  (NED):\t% f\t% f\t% f (rad/s)\n", att.rollspeed, att.pitchspeed, att.yawspeed);
						printf("\t R (NED):\t%f	\t%f \t%f \t%f \t%f \t%f \t%f \t%f \t%f\n",att.R_1, att.R_2, att.R_3, att.R_4, att.R_5, att.R_6, att.R_7, att.R_8, att.R_9);

						printf("\n");
*/					}
					
					/* transform acceleration vector from body frame to NED frame */
/*					for (int i = 0; i < 3; i++) {
					      accel_NED[i] = 0.0f;
					      for (int j = 0; j < 3; j++) {
						    accel_NED[i] += att_R[i][j] * accelerometer_m_s2[j];
					      }
					}
			
					if (att_receive_counter <=200) {
					  
					    bias[0] +=accel_NED[0];
					    bias[1] +=accel_NED[1];
					    bias[2] +=accel_NED[2];
					    
					  
					}
					
					if(att_receive_counter==200) {
					  
					  bias[0] /=201;
					  bias[1] /=201;
					  bias[2] /=201;
					}
					
					if (att_receive_counter>200 & att_receive_counter<=300) {
					  
					    acc_data[cnt][0]=accel_NED[0]-bias[0];
					    acc_data[cnt][1]=accel_NED[1]-bias[1];
					    acc_data[cnt][2]=accel_NED[2]-bias[2];
					  
					    cnt++;
					}
					
					if (att_receive_counter==300) {
					  
					    for (int i =0;i<100;i++) {
						
						acc_mean[0] +=acc_data[i][0];
						acc_mean[1] +=acc_data[i][1];
						acc_mean[2] +=acc_data[i][2];
					    }
					    
					    acc_mean[0] /=100;
					    acc_mean[1] /=100;
					    acc_mean[2] /=100;
					    
					     for (int i =0;i<100;i++) {
						
						st_dev[0] +=(acc_data[i][0]-acc_mean[0])*(acc_data[i][0]-acc_mean[0]);
						st_dev[1] +=(acc_data[i][1]-acc_mean[1])*(acc_data[i][1]-acc_mean[1]);
						st_dev[2] +=(acc_data[i][1]-acc_mean[1])*(acc_data[i][1]-acc_mean[2]);
						
					    }
					    
					    st_dev[0]=sqrt(st_dev[0]/100);
					    st_dev[1]=sqrt(st_dev[1]/100);
					    st_dev[2]=sqrt(st_dev[2]/100);
					    
					}
					
					
					if (att_receive_counter>300) {
					  
					  
					    for (int i =0; i<3; i++) {
					      
						 accel_correct[i]=  accel_NED[i]-bias[i];
					    }
					    
					    printf("acceleration_correction: %f\n",accel_correct[2]);
					    dt=imu.time_usec-time_current;
					    
					    for (int i =0; i<3; i++) {
					      
						if (fabs(accel_correct[i])<0.2){
						  
						    accel_correct[i]=0.0;
						    acc_cnt++;
						} else {
						  
						  acc_cnt=0;
						  accel_correct[i]=accel_correct[i];
						}
					    }
					    
					    if (acc_cnt>4) {
					      
					      for (int i=0; i<3;i++) {
						
						velocity[i]=0.0;
					      }
					    } else {
					      
					      vel_estimation(velocity, accel_correct, dt);
					    }
					    pos_estimation(position,velocity, accel_correct, dt);
					    
					    
					    
					    
					    
					    
					      
						
					  
					  
					}
					
					  
					time_current=imu.time_usec;
*/
					att_receive_counter++;
					
					
					
					
				}
				break;
				
				
			}
			
			
			
			
			/* transform acceleration vector from body frame to NED frame */
			if (scaled_imu_receive_counter>301) {
			      printf("acc_mean and st_dev: \t%f \t%f \t%f \t%f \t%f \t%f\n",acc_mean[0],acc_mean[1],acc_mean[2], st_dev[0],st_dev[1],st_dev[2]);
			      printf("counter= %d\n",scaled_imu_receive_counter);
			      printf("\t acc_final  (NED):\t% f\t% f\t% f (m/s^2)\n", accel_NED[0], accel_NED[1], accel_NED[2]);
			      fprintf(acc_outfile,"%f,%f,%f,%f,%f,%f,%f,%lf,%lf,%lf,%lf,%lf,%lf,%i,%i\n",accel_NED[0]-bias[0]-acc_mean[0], accel_NED[1]-bias[1]-acc_mean[1], accel_NED[2]-bias[2]-acc_mean[2],imu.pressure_alt,imu.xgyro,
				      imu.ygyro, imu.zgyro,velocity[0], velocity[1], velocity[2], position[0],position[1],position[2],dt,scaled_imu_receive_counter);
			}
			
		}
	}
	return 0;
}







































int main(int argc, char *argv[]) {
   
    
    int rc;
    pthread_t threads;//[NUM_THREADS];
    long t;
    printf("In main: creating thread %ld\n", t);
   
     
    rc = pthread_create(&threads, NULL, lidar_data_update, (void *)t);
    if (rc){
         printf("ERROR; return code from pthread_create() is %d\n", rc);
         exit(-1);
    }
      
    /* default values for arguments */
    char *uart_name = (char*)"/dev/ttyUSB0";
    int baudrate = 115200;
    const char *commandline_usage = "\tusage: %s -d <devicename> -b <baudrate> [-v/--verbose] [--debug]\n\t\tdefault: -d %s -b %i\n";

    /* read program arguments */
    int i;

    for (i = 1; i < argc; i++) { /* argv[0] is "mavlink" */
	if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
	    printf(commandline_usage, argv[0], uart_name, baudrate);
	    return 0;
	}

	/* UART device ID */
	if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--device") == 0) {
	    if (argc > i + 1) {
		uart_name = argv[i + 1];
	    } else {
		printf(commandline_usage, argv[0], uart_name, baudrate);
		return 0;
	    }
	}

	/* baud rate */
	if (strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--baud") == 0) {
	    if (argc > i + 1) {
		baudrate = atoi(argv[i + 1]);
	    } else {
		printf(commandline_usage, argv[0], uart_name, baudrate);
		return 0;
	    }
	}

	/* terminating MAVLink is allowed - yes/no */
	if (strcmp(argv[i], "-v") == 0 || strcmp(argv[i], "--verbose") == 0) {
	    verbose = true;
	}

	if (strcmp(argv[i], "--debug") == 0) {
	    debug = true;
	}
    }      
      
    
    acc_outfile = fopen("/home/ashraf/Desktop/thread_code/acc_out.csv", "w");  // using absolute path name of file
    if (acc_outfile == NULL) { 
	fprintf(stderr, "Unable to open file.\n");
    }
    // SETUP SERIAL PORT

    // Exit if opening port failed
    // Open the serial port.
    if (!silent) printf("Trying to connect to %s.. ", uart_name);
    fflush(stdout);

    fd = open_port(uart_name);
    if (fd == -1) {
	if (!silent) printf("failure, could not open port.\n");
	exit(EXIT_FAILURE);
    } else {
	if (!silent) printf("success.\n");
    }
    if (!silent) printf("Trying to configure %s.. ", uart_name);
	bool setup = setup_port(fd, baudrate, 8, 1, false, false);
    if (!setup) {
	if (!silent) printf("failure, could not configure port.\n");
	exit(EXIT_FAILURE);
    } else {
	if (!silent) printf("success.\n");
    }

    int noErrors = 0;
    if (fd == -1 || fd == 0) {
	if (!silent) fprintf(stderr, "Connection attempt to port %s with %d baud, 8N1 failed, exiting.\n", uart_name, baudrate);
	exit(EXIT_FAILURE);
    } else {
	if (!silent) fprintf(stderr, "\nConnected to %s with %d baud, 8 data bits, no parity, 1 stop bit (8N1)\n", uart_name, baudrate);
    }
	
    if(fd < 0) {
	exit(noErrors);
    }

    // Run indefinitely while the serial loop handles data
    if (!silent) printf("\nREADY, waiting for serial data.\n");

    // while(true) wait loop
    serial_wait(fd);
	
    close_port(fd);  
      
    
      
      
      
      
      
      
      
//      cout<<"testing if it comes here"<<endl;
//      clock_gettime(CLOCK_REALTIME, &requestEnd);
//       double accum = requestEnd.tv_nsec-requestStart.tv_nsec;
      
       uint64_t current_time=test.get_current_time();
  //     cout<<"current_time:"<<current_time<<endl;
   

    lidar_info=  false;
    pthread_exit(NULL);
    return 0;
}
















