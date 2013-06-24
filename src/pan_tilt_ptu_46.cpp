#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
extern "C" {
	#include "serial.h"
	#include "time.h"
}


#define PI 				3.1415927
#define PAN_RESOLUTION 	185.1428
#define TILT_RESOLUTION 185.1428

#define PTU_OK			0
#define PTU_ERROR		1
#define PTU_UNEXPECTED	2

#define PAN		0
#define TILT 	1

#define ABSOLUTE 0
#define RELATIVE 1

using namespace std;

void posCallback(const sensor_msgs::JointState& inJointState);
bool getPosCommand(float inDeg, int inMode, int inTargetJoint, char* outCommand);
void delay(int milliseconds);
void processPtuComm();

ros::NodeHandle* myNodeHandle = 0;

char tmpChar;
char status;
char* mySerialDevice = "/dev/ttyUSB0";
int fd;
char theLastCommand[16];
char theLastResponseStatus;
queue RX_Q = {0, 0};

sensor_msgs::JointState thePtuJointState;

enum estado {
	IDLE = 0,
	WAIT_COMMAND_CONF = 1,
	WAIT_POS_PAN = 2,
	WAIT_POS_TILT = 3
};

estado STATUS = IDLE;

int main(int argc, char** argv) {

	thePtuJointState.name.resize(2); 	// array de tamaño 2: PAN y TILT
	thePtuJointState.position.resize(2); // array de tamaño 2: PAN y TILT

	thePtuJointState.name[0] = "PAN";
	thePtuJointState.name[1] = "TILT";

	ros::init(argc, argv, "pan_tilt_ptu_46");
	ros::NodeHandle theNodeHandle;
	myNodeHandle = &theNodeHandle;
	ros::Subscriber theSubscriber = theNodeHandle.subscribe("/ptu_46_in", 10, posCallback);
	ros::Publisher thePublisher = theNodeHandle.advertise<sensor_msgs::JointState>("/ptu_46_out",10);

	ROS_INFO("Nodo de control PTU-46 iniciado");

	// Init PTU-46

	fd = initSerial("/dev/ttyUSB0",9600,'N',8,1);
	if (fd < 0) {
		printf("No se puede abrir dispositivo serie\n");
		return fd;
	}

	ros::Rate r(2); // 2 hz
	while (myNodeHandle->ok()) {

		// Enviamos al PTU petición de posición PAN
		write(fd,"PP\n",3);
		STATUS = WAIT_POS_PAN;
		processPtuComm();

		// Enviamos al PTU petición de posición TILT
		write(fd,"TP\n",3);
		STATUS = WAIT_POS_TILT;
		processPtuComm();

		// Publicamos la posición obtenida
		thePublisher.publish(thePtuJointState);
		ros::spinOnce();
	}

	cout << "Programa terminado" << endl;
	return 0;

}


bool getPosCommand(float inDeg, int inMode, int inTargetJoint, char* outCommand) {

	int thePosVal;
	char theParam;
	char theMode;
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
		sprintf(outCommand,"%c%c%d\n",theParam,theMode,thePosVal);
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
		if (write(fd,thePanCommand,strlen(thePanCommand)) > 0) {
			STATUS = WAIT_COMMAND_CONF;
			processPtuComm();
		}
		if (write(fd,theTiltCommand,strlen(theTiltCommand)) > 0) {
			STATUS = WAIT_COMMAND_CONF;
			processPtuComm();
		}
	} else {
		// Error construyendo mensaje. no enviar nada
		// ...
	}

}

/*
 * Función de retardo con espera activa
 * No usamos sleep porque por el modo de gestionar la comunicación serie
 * las señales recibidas al llegar datos serie no dejan terminar el período establecido en sleep
 * (sleep pausa el proceso hasta que se cumpla el intervalo establecido o hasta que se reciba una señal)
 */
void delay(int milliseconds) {

	time_t t = time(NULL) + milliseconds;
	while(time(NULL) < t);

}


void processPtuComm() {

	int bytesReceived;

	/*
	 * Comprobar datos serie. Esta conexión serie es no bloqueante
	 * De modo que retornará inmediatamente
	 * devolviendo un código de error concreto cuando no haya datos disponibles para lectura
	 * Si hay datos disponibles se introducen en una cola para su posterior procesamiento.
	 */

	if(serialDataReceived()) {
		char buf[255];
		do {
			delay(100);
			bytesReceived = read(fd,buf,255);
			if (bytesReceived > 0) {
				int i;
				for (i=0;i<bytesReceived;i++) {
					RX_enqueue(&RX_Q,buf[i]);
				}
			}
		} while(bytesReceived > 0);

	}

	/*
	 * Comprobar los datos recibidos mediante puerto serie
	 */

	if(RX_Q.numElem > 0) {

		switch (STATUS) {

			case IDLE:

				// Recepción inesperada
				// Mostramos datos recibidos

				printf("Datos inesperados: %s",RX_toStr(&RX_Q));

				break;

			case WAIT_COMMAND_CONF:

				char c;

				do {
					c = RX_dequeue(&RX_Q);
				} while ((c != '!') && (c != '*') && (RX_Q.numElem > 0));

				switch(c) {
					case '!':
						// Error en el comando
						theLastResponseStatus = PTU_ERROR;
						if(RX_Q.numElem > 0) printf("Error en el último comando: %s",RX_toStr(&RX_Q));
						break;
					case '*':
						// Comando confirmado
						theLastResponseStatus = PTU_OK;
						break;
					default:
						// Inesperado
						theLastResponseStatus = PTU_UNEXPECTED;
						break;
				}

				STATUS = IDLE;
				break;

			case WAIT_POS_PAN:

				float panDeg = 0.0;

				// Extraer la información del mensaje de respuesta

				thePtuJointState.position[0] = panDeg;

				STATUS = IDLE;
				break;

			case WAIT_POS_TILT:

				float tiltDeg = 0.0;

				// Extraer la información del mensaje de respuesta

				thePtuJointState.position[1] = tiltDeg;

				STATUS = IDLE;
				break;


		}

	}



}
