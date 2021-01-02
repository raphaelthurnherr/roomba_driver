#define ERR_UART_COM_OPEN       0
#define ERR_UART_GET_COM_STATE  1
#define ERR_UART_SET_COM_STATE  2
#define ERR_UART_SET_TIMEOUT    3



int serial_init();
int serial_write_string(char * ptrDataToWrite);
int serial_write(char * ptrDataToWrite, int count);
int serial_read(char * ptrDataReceived);

void Exit1();