int readSamples(int fd, char* recv);
int initSerial(const char *serialDevice,int baudRate, char parity, int dataBits,int stopBits);
void signal_handler_IO (int status);
int serialDataReceived();
void int2littleEndianStr(unsigned int operand, unsigned char *str);
unsigned int littleEndianStr2int(unsigned char *str);

#define QUEUESIZE	2048
typedef struct {
	int head;
	unsigned int numElem;
	unsigned char buff[QUEUESIZE];
} queue;

void RX_enqueue(queue* q,unsigned char c);
unsigned char RX_dequeue(queue* q);
char* RX_toStr(queue* q);
