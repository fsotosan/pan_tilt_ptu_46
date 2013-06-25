#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Joy.h"
extern "C" {
	#include "serial.h"
	#include "time.h"
	#include "error.h"
}


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

char tmpChar;
char status;
char* mySerialDevice = "/dev/ttyUSB0";
int fd;
queue RX_Q = {0, 0};

sensor_msgs::JointState thePtuJointState;

enum estado {
	IDLE = 0,
	WAIT_COMMAND_CONF = 1,
	WAIT_POS_PAN = 2,
	WAIT_POS_TILT = 3
};

estado STATUS = IDLE;


int envia(int inFd, const char* inStr, int numChars) {

	int i;
	//printf("Enviando: '%s'\n","I ");
	write(inFd,"I ",2);
	processPtuComm();
	//printf("Enviando: '%s'\n",inStr);
	write(inFd,inStr,numChars);
	processPtuComm();
	//printf("Enviando: '%s'\n","A ");
	write(inFd,"A ",2);
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

	fd = initSerial(SERIALDEVICE,9600,'N',8,1);
	if (fd < 0) {
		printf("No se puede abrir dispositivo serie '%s'. Error: '%s'\n",SERIALDEVICE,strerror(errno)) ;
		return fd;
	}

	ROS_INFO("Puerto serie abierto");

	// Enviamos comando de modo de ejecución inmediato

	// envia(fd,"I ",2);	// Modo inmediato
	// envia(fd,"FT ",3);	// Respuestas escuetas

	ros::Rate r(1); // 1 hz
	while (myNodeHandle->ok()) {

/*
		// Enviamos al PTU petición de posición PAN
		envia(fd,"PP\n",3);
		STATUS = WAIT_POS_PAN;
		processPtuComm();

		// Enviamos al PTU petición de posición TILT
		envia(fd,"TP\n",3);
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
		if (envia(fd,thePanCommand,strlen(thePanCommand)) > 0) {
			STATUS = WAIT_COMMAND_CONF;
			processPtuComm();
		}
		if (envia(fd,theTiltCommand,strlen(theTiltCommand)) > 0) {
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
		if (envia(fd,thePanCommand,strlen(thePanCommand)) > 0) {
			STATUS = WAIT_COMMAND_CONF;
			processPtuComm();
		}
	}

	if (theAxesV != 0.0) {
		getPosCommand(theAxesV*JOYSTEP_TILT_DEG,TILT,RELATIVE,theTiltCommand);
		if (envia(fd,theTiltCommand,strlen(theTiltCommand)) > 0) {
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


void processPtuComm() {

	int bytesReceived;
	int i;
	char c;
	float deg;

	/*
	 * Comprobar datos serie. Esta conexión serie es no bloqueante
	 * De modo que retornará inmediatamente
	 * devolviendo un código de error concreto cuando no haya datos disponibles para lectura
	 * Si hay datos disponibles se introducen en una cola para su posterior procesamiento.
	 */


	char buf[255];
	bytesReceived = read(fd,buf,255);
	if (bytesReceived < 0) {
		//printf("[Fer debug] Ignorando: %s\n", strerror(errno));
		errno = 0;
	}
	while(bytesReceived > 0) {
		printf("[Fer debug] Recibidos: %d bytes\n", bytesReceived);
		for (i=0;i<bytesReceived;i++) {
			RX_enqueue(&RX_Q,buf[i]);
		}
		bytesReceived = read(fd,buf,255);
		if (bytesReceived < 0) {
			//printf("[Fer debug] Ignorando: %s\n", strerror(errno));
			errno = 0;
		}
	}


	/*
	 * Comprobar los datos recibidos mediante puerto serie
	 */

	printf("[Fer debug] Estado %d. Elementos en cola: %d: %s\n", STATUS, RX_Q.numElem, RX_showContents(&RX_Q));

	if(RX_Q.numElem > 0) {

		switch (STATUS) {

			case IDLE:

				// Recepción inesperada
				// Mostramos datos recibidos

				printf("Datos inesperados: %s",RX_toStr(&RX_Q));

				break;

			case WAIT_COMMAND_CONF:

				do {
					c = RX_dequeue(&RX_Q);
				} while ((c != '!') && (c != '*') && (RX_Q.numElem > 0));

				switch(c) {
					case '!':
						// Error en el comando
						if(RX_Q.numElem > 0) printf("Error en el último comando: %s",RX_toStr(&RX_Q));
						break;
					case '*':
						// Comando confirmado
						printf("Ok\n");
						break;
					default:
						// Inesperado
						break;
				}

				printf("%s\n",RX_toStr(&RX_Q));

				STATUS = IDLE;
				break;

			case WAIT_POS_PAN:
			case WAIT_POS_TILT:

				// Extraer la información del mensaje de respuesta
				// Ejemplo de mensaje esperado:
				// * Current Pan position is -2500

				int pos;

				do {
					c = RX_dequeue(&RX_Q);
				} while ((c != '!') && (c != '*') && (RX_Q.numElem > 0));

				switch(c) {
					case '!':
						// Error en el comando
						if(RX_Q.numElem > 0) printf("Error en el último comando: %s",RX_toStr(&RX_Q));
						break;
					case '*':
						// Comando confirmado
						if (RX_Q.numElem == 0) {
							STATUS = IDLE;
							return;
						}
						//  Avanzamos en la cola hasta encontrar dígitos o signo '-'
						do {
							c = RX_dequeue(&RX_Q);
						} while ((RX_Q.numElem > 0)&&(!(c == '-')||((c >= '0')&&(c <= '9'))));
						if (RX_Q.numElem == 0) {
							STATUS = IDLE;
							return;
						}
						char posNumber[16];
						i = 1;
						posNumber[0] = c;
						while ((RX_Q.numElem > 0)&&(i<16)) {
							c = RX_dequeue(&RX_Q);
							if ((c >= '0')&&(c <= '9')) {
								posNumber[i++] = c;
							} else {
								break;
							}
						}
						posNumber[i] = '\0';
						pos = atoi(posNumber);
						if (STATUS == WAIT_POS_PAN) {
							deg = (float)pos * PAN_RESOLUTION / 3600;
							thePtuJointState.position[0] = deg;
						} else if (STATUS == WAIT_POS_TILT) {
							deg = (float)pos * TILT_RESOLUTION / 3600;
							thePtuJointState.position[1] = deg;
						}
						break;
					default:
						// Inesperado
						break;
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
