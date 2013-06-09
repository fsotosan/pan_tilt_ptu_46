#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

extern "C" {
	#include "ptu.h"
	#include "linuxser.h"
	#include "opcodes.h"
}


#define PI 3.1415927
#define PAN_RESOLUTION 185.1428
#define TILT_RESOLUTION 185.1428

using namespace std;

void posCallback(const sensor_msgs::JointState& inJointState);

ros::NodeHandle* myNodeHandle = 0;

char tmpChar;
char status;
char* mySerialDevice = "/dev/ttyUSB0";

char return_error () {
	printf("(Enter 'c' to continue): ");
	tmpChar = 'f';
	while (tmpChar != 'c')
		tmpChar = tolower(getchar());
	return(-1);
}

char return_error_status (char *failed_binary_op, char return_status)
{
	printf("! %s failed with status %d\n",failed_binary_op, return_status);
	return ( return_error() );
}


char test_position_commands(void)
{


	int val;

	val = 2500;

	status = set_desired(PAN, POSITION, (PTU_PARM_PTR *) &val, ABSOLUTE);

	if (status == TRUE) {
		return (return_error_status( "PAN_SET_ABS_POSITION", status));
	} else {
		printf("\nPAN_SET_ABS_POSITION executed\n");
	}

	 /*

*/
	 return(PTU_OK);
}




int main(int argc, char** argv) {

	ros::init(argc, argv, "pan_tilt_ptu_46");
	ros::NodeHandle theNodeHandle;
	myNodeHandle = &theNodeHandle;
	ros::Subscriber theSubscriber = theNodeHandle.subscribe("/ptu_46_in", 10, posCallback);
	ros::Publisher thePublisher = theNodeHandle.advertise<sensor_msgs::JointState>("/ptu_46_out",10);

	ROS_INFO("Nodo de control PTU-46 iniciado");

	// Init PTU-46

	portstream_fd fd = open_host_port(mySerialDevice);

	sleep(5);

	if ( fd == PORT_NOT_OPENED ) {
		printf("\nSerial Port setup error.\n");
		return -1;
	}

	set_mode(ASCII_ECHO_MODE, ON_MODE);

	test_position_commands();

	close_host_port(fd);

	ros::Rate r(10); // 10 hz
	while (myNodeHandle->ok()) {



		//thePublisher.publish(theMsg);
		ros::spinOnce();
	}

	close_host_port(fd);

	cout << "Programa terminado" << endl;
	return 0;

}



void posCallback(const sensor_msgs::JointState& inJointState) {

	float thePosDeg = inJointState.position * 180 / PI;
	int thePosVal;
	int theParam;

	switch(inJointState.name) {

		case "PAN":

			thePosVal = (int) (thePosDeg * 3600 / PAN_RESOLUTION);
			theParam = PAN;
			break;

		case "TILT":

			thePosVal = (int) (thePosDeg * 3600 / TILT_RESOLUTION);
			theParam = TILT;
			break;

		default:

			// inesperado
			return;

	}

	status = set_desired(theParam, POSITION, (PTU_PARM_PTR *) &thePosVal, ABSOLUTE);

}
