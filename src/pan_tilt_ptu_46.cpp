#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

extern "C" {
	#include "ptu.h"
	#include "linuxser.h"
	#include "opcodes.h"
}


#define TEST_ERROR	-1

using namespace std;

void posCallback(const sensor_msgs::JointState& inJointState);

ros::NodeHandle* myNodeHandle = 0;

char tmpChar;
char status;
char* mySerialDevice = "/dev/ttyUSB0";

char return_error ()
{ printf("(Enter 'c' to continue): ");
  tmpChar = 'f';
  while (tmpChar != 'c')
	    tmpChar = tolower(getchar());
  return(TEST_ERROR);
}

char return_error_status (char *failed_binary_op, char return_status)
{ printf("! %s failed with status %d\n",failed_binary_op, return_status);
  return ( return_error() );
}


char test_position_commands(void)
{


	int val;
	long lval_in;

	/* pan position command */
	 val = -2500;

	 if ((status = set_desired(PAN, POSITION, (PTU_PARM_PTR *) &val, ABSOLUTE)) == TRUE)
		 { return (  return_error_status( "PAN_SET_ABS_POSITION", status) );
				}
	 else printf("\nPAN_SET_ABS_POSITION executed\n");

	 /*

*/
	 return(PTU_OK);
}

int main(int argc, char** argv) {

	ros::init(argc, argv, "youbot_arm_move");
	ros::NodeHandle theNodeHandle;
	myNodeHandle = &theNodeHandle;
	ros::Subscriber theSubscriber = theNodeHandle.subscribe("/ptu_46_in", 10, posCallback);
	ros::Publisher thePublisher = theNodeHandle.advertise<sensor_msgs::JointState>("/ptu_46_out",10);

	ROS_INFO("Nodo de control PTU-46 iniciado");

	// Init PTU-46

	portstream_fd fd = openserial(mySerialDevice);


	closeserial(fd);

	ros::Rate r(10); // 10 hz
	while (myNodeHandle->ok()) {



		//thePublisher.publish(theMsg);
		ros::spinOnce();
	}

	//closeserial(fd);

	cout << "Programa terminado" << endl;
	return 0;

}


void posCallback(const sensor_msgs::JointState& inJointState) {




}
