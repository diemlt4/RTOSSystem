/*****************************************************************************/
/* Includes                                                                  */
/*****************************************************************************/
#include <amw_uart.h>
#include "main.h"


char transmitBuff[100]= {0};
uint8_t transferCplt = 0;
uint8_t rxIdx = 0;
uint8_t i;
extern UART_HandleTypeDef huart1;
static uint8_t uart_command[128];
UART_HandleTypeDef *amw_uart = &huart1;
void TimerInit(Timer* timer)
{
	timer->xTicksToWait = 0;
	memset(&timer->xTimeOut, '\0', sizeof(timer->xTimeOut));
}

char TimerIsExpired(Timer* timer)
{
	return xTaskCheckForTimeOut(&timer->xTimeOut, &timer->xTicksToWait) == pdTRUE;
}

void TimerCountdownMS(Timer* timer, unsigned int timeout)
{
	timer->xTicksToWait = timeout / portTICK_PERIOD_MS; /* convert milliseconds to ticks */
	vTaskSetTimeOutState(&timer->xTimeOut); /* Record the time at which this function was entered. */
}

void TimerCountdown(Timer* timer, unsigned int timeout)
{
	TimerCountdownMS(timer, timeout * 1000);
}

int TimerLeftMS(Timer* timer)
{
	xTaskCheckForTimeOut(&timer->xTimeOut, &timer->xTicksToWait); /* updates xTicksToWait to the number left */
	return (timer->xTicksToWait < 0) ? 0 : (timer->xTicksToWait * portTICK_PERIOD_MS);
}


uint8_t amw_write(unsigned char *str){
	strcpy(transmitBuff, str);
	HAL_UART_Transmit(amw_uart, transmitBuff, strlen(transmitBuff), 100);
	return 0;
}

int getCmdResponse(unsigned char* uart_reponse)
{
    uint8_t header[16] = { 0 };
    uint8_t error_code[1];
    int rc = 0;

    HAL_UART_Receive(amw_uart, header, 9, 0xFFFF);

    if(header[0] == 'R')
    {
    	if(header[1] == '0')
		{
			int len = atoi((char *)&header[2]);
			// every response always ends in /r/n (i.e., always > 2 bytes)
			if(len > 2)
			{
				unsigned char temp[2];

				// read the data (without the trailing /r/n)
				HAL_UART_Receive(amw_uart, uart_reponse, len - 2, 0xFFFF);
				// cleanup the trailing /r/n
				HAL_UART_Receive(amw_uart, temp, 2, 0xFFFF);
				// return actual data length
				return len - 2;
			}
			else
			{
				return 0;
			}

		}else{
			int len = atoi((char *)&header[2]);
			HAL_UART_Receive(amw_uart, uart_reponse, len, 0xFFFF);
		}

    }
    return -1;
}

int OsWrapper_read(Network* n, unsigned char* buffer, int len, int timeout_ms)
{
    int bytes = 0;
    TickType_t xTicksToWait = timeout_ms / portTICK_PERIOD_MS; /* convert milliseconds to ticks */
    TimeOut_t xTimeOut;
    size_t command_len;
    int i;
    vTaskSetTimeOutState(&xTimeOut); /* Record the time at which this function was entered. */
#ifdef BUS_COMMAND_MODE
    while(bytes < len)
    {
        sprintf((char*)uart_command, "read %u %u\r\n", n->my_socket, len - bytes);
        command_len = strlen((char*)uart_command);
        HAL_UART_Transmit(amw_uart, uart_command, command_len, 0xFFFF);

        bytes += getCmdResponse(buffer+bytes);

        if(xTaskCheckForTimeOut(&xTimeOut, &xTicksToWait) == pdTRUE)   // Timeout
        {
            break;
        }

        // slow down, give chance to other processes
        osDelay(5);
    }
#else
    HAL_UART_Receive(amw_uart, buffer, len, 0xFFFF);
    bytes = len;

    // ToDo: should check for timeout here as well
#endif

    return bytes;
}

int OsWrapper_write(Network* n, unsigned char* buffer, int len, int timeout_ms)
{
#ifdef BUS_COMMAND_MODE
    size_t command_len;
    int i;
    unsigned char read_buf[128] = { 0 };

    sprintf((char*)uart_command, "write %u %u\r\n", n->my_socket, len);
    command_len = strlen((char*)uart_command);
    HAL_UART_Transmit(amw_uart, uart_command, command_len, 100);
#endif

    HAL_UART_Transmit(amw_uart, buffer, len, 0xFFFF);

#ifdef BUS_COMMAND_MODE
    // cleanup ZentriOS response
    getCmdResponse(read_buf);
#endif

    // ToDo: should return only the succeeded transmitted length, not the input len
    return len;
}

int setJoinTimeout(int timeout){
	char time[10] = {0};
	char resp_buff[128];
	size_t command_len;
	sprintf((char*)uart_command, "set wl j t %s\r\n", itoa(timeout, time, 10));
	command_len = strlen((char*)uart_command);
	HAL_UART_Transmit(amw_uart, uart_command, command_len, 0xFFFF);
	if(getCmdResponse(resp_buff)<0)
		return -1;
	else
		return 0;
}

int setCmdMode(const char *mode){
	size_t command_len;
	uint8_t resp_buff[128] = { 0 };
	sprintf((unsigned char*)uart_command, "set sy c f %s\r\n", mode);
	command_len = strlen((char*)uart_command);
	HAL_UART_Transmit(amw_uart, uart_command, command_len, 0xFFFF);
	if(getCmdResponse(resp_buff)<0)
		return -1;
	else
		return 0;
}

int setupNetwork(char *ssid, char *password){
#ifdef BUS_COMMAND_MODE
    uint8_t resp_buff[128] = { 0 };
    size_t command_len;
    sprintf((unsigned char*)uart_command, "set wl s \"%s\"\r\n", ssid);
    command_len = strlen((char*)uart_command);
    HAL_UART_Transmit(amw_uart, uart_command, command_len, 0xFFFF);

    if(getCmdResponse(resp_buff)<0)
    	return -1;

    sprintf((unsigned char*)uart_command, "set wl p %s\r\n", password);
    command_len = strlen((char*)uart_command);
    HAL_UART_Transmit(amw_uart, uart_command, command_len, 0xFFFF);
    if(getCmdResponse(resp_buff)<0)
        return -1;
    else
    	return 0;
#else
    return 0;
#endif
}

int checkNetworkStatus(){
#ifdef BUS_COMMAND_MODE
	size_t command_len;
    uint8_t resp_buff[128] = { 0 };
    char *ptr = NULL;
    char *sub = NULL;
    command_len = strlen("get wlan.info\r\n");
    HAL_UART_Transmit(amw_uart, "get wlan.info\r\n", command_len, 0xFFFF);

    if(getCmdResponse(resp_buff)>=0)
    {
    	ptr = strstr(resp_buff, " ");
    	sub = strstr(resp_buff, "\r\n");
    	*sub = '\0';
    	if(!strcmp(&ptr[1], "up")){
    		return NET_UP;
    	}else
    		return NET_DOWN;
    }
    else
    {
        return BUS_ERROR;
    }

#else
    return 0;
#endif
}

int NetworkInit(Network* n)
{
	uint8_t resp_buff[128] = { 0 };
    n->my_socket = 0;
    n->mqttread = OsWrapper_read;
    n->mqttwrite = OsWrapper_write;
    n->disconnect = NetworkDisconnect;
#ifdef BUS_COMMAND_MODE
    size_t command_len;
    command_len = strlen("nup\r\n");
    HAL_UART_Transmit(amw_uart, "nup\r\n", command_len, 0xFFFF);
    if(getCmdResponse(resp_buff)<0)
    	return -1;
    else{
    	Timer timer;
		TimerInit(&timer);
		TimerCountdownMS(&timer, 7000);
		osDelay(50);
    	while(checkNetworkStatus()!=NET_UP){
    		osDelay(50);
    	}
    	if(checkNetworkStatus()!=NET_UP){
    		return -1;
    	}else
    		return 0;
    }

#else
    return 0;
#endif
}

int NetworkConnect(Network* n, char* addr, int port)
{
#ifdef BUS_COMMAND_MODE
    size_t command_len;
    uint8_t resp_buff[128] = { 0 };

    if(port == MQTT_SECURE_PORT)
    {
        // secure - use TLS
        sprintf((char*)uart_command, "tls_client %s %u\r\n", addr, port);
    }
    else
    {
        // clear - use TCP
        sprintf((char*)uart_command, "tcpc %s %u\r\n", addr, port);
    }
    command_len = strlen((char*)uart_command);
    HAL_UART_Transmit(amw_uart, uart_command, command_len, 0xFFFF);

    if(getCmdResponse(resp_buff)>=0)
    {
        n->my_socket = atoi((char *)resp_buff);
    }
    else
    {
    	printf("socket failed\r\n");
        n->my_socket = -1;
    }
    osDelay(100);
    return n->my_socket;
#else
    return 0;
#endif
}

void NetworkDisconnect(Network* n)
{
#ifdef BUS_COMMAND_MODE
    size_t command_len;
    unsigned char read_buf[128] = { 0 };

    sprintf((char*)uart_command, "close %u\r\n", n->my_socket);
    command_len = strlen((char*)uart_command);

    HAL_UART_Transmit(amw_uart, uart_command, command_len, 100);
    // cleanup ZentriOS response
    getCmdResponse(read_buf);
#else
    return;
#endif
}

void closeTCPPort(int file)
{
#ifdef BUS_COMMAND_MODE
    size_t command_len;
    unsigned char read_buf[128] = { 0 };

    sprintf((char*)uart_command, "close %u\r\n", file);
    command_len = strlen((char*)uart_command);

    HAL_UART_Transmit(amw_uart, uart_command, command_len, 100);
    // cleanup ZentriOS response
    getCmdResponse(read_buf);
#else
    return;
#endif
}

int isTCPPortOpen(){
#ifdef BUS_COMMAND_MODE
    size_t command_len;
    unsigned char read_buf[128] = { 0 };
    unsigned char *ptr = NULL;
    command_len = strlen("stream_list\r\n");

    HAL_UART_Transmit(amw_uart, "stream_list\r\n", command_len, 100);
    // cleanup ZentriOS response
    if(getCmdResponse(read_buf)>=0){
    	ptr = strstr(read_buf, "TCPC");
    	if(ptr){
    		return (read_buf[ptr-read_buf-2] - 48); // return int type of character
    	}else
    		return -1;
    }else
    	return BUS_ERROR;
#else
    return 0;
#endif
}
