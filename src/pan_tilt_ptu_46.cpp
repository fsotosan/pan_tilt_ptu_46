#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Joy.h"
#include "Serial_Q.h"

#define PI 				3.1415927

// Resolución por defecto del PTU en segundos de grado

#define PAN_RESOLUTION 	185.1428
#define TILT_RESOLUTION 185.1428

#define JOYSTEP_PAN_DEG	10.0
#define JOYSTEP_TILT_DEG	10.0

#define SERIALDEVICE	"/dev/ttyUSB0"

#define PTU_OK			0
#define PTU_ERROR		1
#define PTU_UNEXPECTED	2

#define PAN		0
#define TILT 	1

#define ABSOLUTE 0
#define RELATIVE 1

using namespace std;

void posCallback(const sensor_msgs::JointState& inJointState);
void joyCallback(const sensor_msgs::Joy& inJoyCommand);
bool getPosCommand(float inDeg, int inTargetJoint, int inMode, char* outCommand);
void mydelay_ms(int milliseconds);
void processPtuComm();

ros::NodeHandle* myNodeHandle = 0;

Serial::Serial_Q *Ptu;

char tmpChar;
char status;
char mySerialDevice[] = "/dev/ttyUSB0";

sensor_msgs::JointState thePtuJointState;

enum estado {
	IDLE = 0,
	WAIT_COMMAND_CONF = 1,
	WAIT_POS_PAN = 2,
	WAIT_POS_TILT = 3
};

estado STATUS = IDLE;


void envia(int inFd, const char* inStr) {

	Ptu->send("I ");
	processPtuComm();
	Ptu->send("inStr ");
	processPtuComm();
	Ptu->send("A ");
	processPtuComm();

}

int main(int argc, char** argv) {

	thePtuJointState.name.resize(2); 	// array de tamaño 2: PAN y TILT
	thePtuJointState.position.resize(2); // array de tamaño 2: PAN y TILT

	thePtuJointState.name[0] = "PAN";
	thePtuJointState.name[1] = "TILT";

	ros::init(argc, argv, "pan_tilt_ptu_46");
	ros::NodeHandle theNodeHandle;
	myNodeHandle = &theNodeHandle;
	//ros::Subscriber thePosSubscriber = theNodeHandle.subscribe("/ptu_46_pos_in", 10, posCallback);
	ros::Subscriber theJoySubscriber = theNodeHandle.subscribe("/joy", 10, joyCallback);
	ros::Publisher thePublisher = theNodeHandle.advertise<sensor_msgs::JointState>("/ptu_46_pos_out",10);

	ROS_INFO("Nodo de control PTU-46 iniciado");

	// Init PTU-46

	Ptu = new Serial::Serial_Q(mySerialDevice, B9600);

	if (Ptu->LastError != NULL) {
		ROS_ERROR("Error conectando con unidad PTU-46: %s",Ptu->LastError);
		return -1;
	}

	ROS_INFO("Puerto serie abierto %d",B9600);

	// Enviamos comando de modo de ejecución inmediato

	Ptu->send("I ");	// Modo inmediato
	Ptu->send("FT ");	// Respuestas escuetas

	ros::Rate r(1); // 1 hz
	while (myNodeHandle->ok()) {

/*
		// Enviamos al PTU petición de posición PAN
		Ptu->send("PP\n");
		STATUS = WAIT_POS_PAN;
		processPtuComm();

		// Enviamos al PTU petición de posición TILT
		Ptu->send("TP\n");
		STATUS = WAIT_POS_TILT;
		processPtuComm();
*/
		// Publicamos la posición obtenida
		thePublisher.publish(thePtuJointState);
		ros::spinOnce();
	}

	cout << "Programa terminado" << endl;
	return 0;

}


bool getPosCommand(float inDeg, int inTargetJoint, int inMode, char* outCommand) {

	int thePosVal;
	char theParam = 'P';
	char theMode = ABSOLUTE;
	bool ok = true;

	switch(inTargetJoint) {
		case PAN:
			thePosVal = (int) (inDeg * 3600 / PAN_RESOLUTION);
			theParam = 'P';
			break;
		case TILT:
			thePosVal = (int) (inDeg * 3600 / TILT_RESOLUTION);
			theParam = 'T';
			break;
		default: 	// inesperado
			ok = false;
	}

	switch (inMode) {
		case ABSOLUTE:	// PP o TP
			theMode = 'P';
			break;
		case RELATIVE:	// PO o TO
			theMode = 'O';
			break;
		default: 	// inesperado
			ok = false;
	}

	if (ok) {
		sprintf(outCommand,"%c%c%d ",theParam,theMode,thePosVal);
	}

	return ok;

}


void posCallback(const sensor_msgs::JointState& inJointState) {

	float thePanDeg = (float)inJointState.position[0] * 180 / PI;
	float theTiltDeg = (float)inJointState.position[1] * 180 / PI;
	char thePanCommand[16];
	char theTiltCommand[16];
	bool theBuildCommandOk = false;

	theBuildCommandOk = getPosCommand(thePanDeg,PAN,ABSOLUTE,thePanCommand);
	theBuildCommandOk = theBuildCommandOk && getPosCommand(theTiltDeg,TILT,ABSOLUTE,theTiltCommand);

	if (theBuildCommandOk) {
		if (Ptu->send(thePanCommand) > 0) {
			STATUS = WAIT_COMMAND_CONF;
			processPtuComm();
		}
		if (Ptu->send(theTiltCommand) > 0) {
			STATUS = WAIT_COMMAND_CONF;
			processPtuComm();
		}
	} else {
		// Error construyendo mensaje. no enviar nada
		// ...
	}

}


void joyCallback(const sensor_msgs::Joy& inJoyCommand) {

	char thePanCommand[16];
	char theTiltCommand[16];
	float theAxesH = (float)inJoyCommand.axes[0];
	float theAxesV = (float)inJoyCommand.axes[1];

	printf("Recibido comando joystick (%g,%g)\n",theAxesV,theAxesH);

	// Interpretar los comandos de Joystick como instrucciones
	// para mover JOYSTEP_PAN_DEG (JOYSTEP_PAN_TILT) grados el eje correspondiente (PAN o TILT) en el sentido indicado

	if (theAxesH != 0.0) {
		getPosCommand(theAxesH*JOYSTEP_PAN_DEG,PAN,RELATIVE,thePanCommand);
		if (Ptu->send(thePanCommand) > 0) {
			STATUS = WAIT_COMMAND_CONF;
			processPtuComm();
		}
	}

	if (theAxesV != 0.0) {
		getPosCommand(theAxesV*JOYSTEP_TILT_DEG,TILT,RELATIVE,theTiltCommand);
		if (Ptu->send(theTiltCommand) > 0) {
			STATUS = WAIT_COMMAND_CONF;
			processPtuComm();
		}
	}

}

/*
 * Función de retardo con espera activa
 * No usamos sleep porque por el modo de gestionar la comunicación serie
 * las señales recibidas al llegar datos serie no dejan terminar el período establecido en sleep
 * (sleep pausa el proceso hasta que se cumpla el intervalo establecido o hasta que se reciba una señal)
 */
void mydelay_ms(int milliseconds) {

	time_t t = time(NULL) + milliseconds;
	while(time(NULL) < t);

}

bool extractInt(const string inStr, int* outNum) {

	stringstream ss(inStr);
	string tmp;
	ss >> tmp >> *outNum;
	return true;

}

void processPtuComm() {

	string theResp;
	std::size_t theFound;

	if (Ptu->checkDataAndEnqueue()) {

		theResp.assign((char *)Ptu->getFullQueueContent(true));

		// Un signo de exclamación indica que se ha producido un error

		theFound = theResp.find("!");
		if (theFound != string::npos) {
			STATUS = IDLE;
			ROS_ERROR("PTU-46 ha devuelto un error: %s",theResp.c_str());
			return;
		}

		switch (STATUS) {

			case IDLE:

				// Recepción inesperada
				// Mostramos datos recibidos

				ROS_ERROR("PTU-46 ha enviado un dato inesperado: %s",theResp.c_str());

				break;

			case WAIT_COMMAND_CONF:

				theFound = theResp.find("*");
				if (theFound != string::npos) {
					ROS_INFO("PTU-46 ha confirmado la última orden: %s",theResp.c_str());
					STATUS = IDLE;
				}

				break;

			case WAIT_POS_PAN:

				break;

			case WAIT_POS_TILT:

				// Comprobamos que el comando se ha admitido
				// Y eliminamos el asterisco de confirmación

				int thePos;

				theFound = theResp.find("*");
				if (theFound != string::npos) {
					theResp.erase(0,theFound+1);
					STATUS = IDLE;
				}

				// Leemos el dato devuelto

				if (extractInt(theResp,&thePos)) {

					float theDeg;

					if (STATUS == WAIT_POS_PAN) {
						theDeg = (float)thePos * PAN_RESOLUTION / 3600;
						thePtuJointState.position[0] = theDeg;
					} else if (STATUS == WAIT_POS_TILT) {
						theDeg = (float)thePos * TILT_RESOLUTION / 3600;
						thePtuJointState.position[1] = theDeg;
					}

				}

				STATUS = IDLE;
				break;

			default:

				STATUS = IDLE;
				break;

		}

	}

	return;

}
