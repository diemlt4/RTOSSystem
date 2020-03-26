/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "string.h"
#include "stdio.h"
#include "math.h"

#include "MQTTClient.h"
#include "MQTTConnect.h"
#include "amw_uart.h"
#include "fan.h"
#include "stepmotor.h"
#include "svm30.h"
#include "sensirion_i2c.h"
#include "sm_uart-04l.h"
#include "tiny-json.h"
#include "ringbuf.h"
#include "IRremote.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define SSID_WIFI "Samsung1"
#define PASS_WIFI "123456789"

#define MQTT_MESSAGE                "[{\"name\": \"ERV\", \"id\": \"hethong123\"}]"

#define MQTT_PUB_TOPIC              "leetest"
#define MQTT_SUB_TOPIC              "diem"
#define MQTT_CLIENT_ID              "Zentri"
#define MQTT_USER                   "myuncjoa"
#define MQTT_PASSWORD               "SvVx1BPwTVqk"
#define MQTT_HOST                   "m24.cloudmqtt.com"
#define MQTT_PORT                   14022

// For new app
#define MQTT_USER_N                 "erdfotqc"
#define MQTT_PASSWORD_N             "e8HXfhsM8aD6"
//#define MQTT_HOST_N                   "m16.cloudmqtt.com"
//#define MQTT_PORT_N                   15044
#define MQTT_HOST_N                   "broker.hivemq.com"
#define MQTT_PORT_N                   1883

//** Define MQTT TOPIC cần subcribse từ Broker để giao tiếp vs app **//
#define TOPIC_DEVICE_ID "esp/request/collect/device-info"
#define TOPIC_AIR_QUALITY "esp/request/collect/air-quality"
#define TOPIC_CONTROL_STATE "esp/request/collect/control-state"
#define TOPIC_POWER_ON "esp/request/control/power-on"
#define TOPIC_POWER_OFF "esp/request/control/power-off"
#define TOPIC_NIGHT_MODE "esp/request/control/night_mode"
#define TOPIC_ION "esp/request/control/ion"
#define TOPIC_UV "esp/request/control/uv"
#define TOPIC_FILTER "esp/request/control/filter_mode"
#define TOPIC_CONTROL "esp/request/control/control_mode"
#define TOPIC_SPEED_LOW "esp/request/control/low"
#define TOPIC_SPEED_MED "esp/request/control/med"
#define TOPIC_SPEED_HIGH "esp/request/control/high"

#define RESPONSE_DEVICE_INFO_TOPIC	"esp/response/collect/device-info"
#define TOPIC_PUB_AIR_QUALITY 	"esp/response/collect/air-quality"
#define TOPIC_PUB_CONTROL_STATE  "esp/response/collect/control-state"
#define TOPIC_PUB_INFO "esp/response/collect/device-info"

// For new mobile app
#define TOPIC_LOGIN "hungdv39/ap01/server-device/account/login"
#define TOPIC_CHANGE_PASSWORD "hungdv39/ap01/request/login/change_password"
// Lấy các chỉ số chất lượng không khí và đi�?u khiển
#define TOPIC_GET_AIR_QUALITY  "session_01/ap01/request/home/get_air_quality"
#define TOPIC_GET_CONTROL_STATE "session_01/ap01/request/control/get_state"
#define TOPIC_REQUEST_CONTROL_MODE "session_01/ap01/request/control/control_state"
#define TOPIC_REQUEST_ALERT "session_01/ap01/request/alert/alert"
/** Topic Publish **/
#define REPONSE_LOGIN "hungdv39/ap01/response/login/login"
#define REPONSE_CHANGE_PASSWORD "hungdv39/ap01/response/login/change_password"
#define REPONSE_GET_AIR_QUALITY "session_01/ap01/response/home/get_air_quality"
#define REPONSE_GET_STATE "session_01/ap01/response/control/get_state"
#define REPONSE_CONTROL_STATE "session_01/ap01/response/control/control_state"
#define REPONSE_ALERT "session_01/ap01/response/alert/alert"

#define MESSAGE_LOGIN_SUCCESS  "[{ \"success\" : \"true\", \"session_id\" : \"session_01\", \"info\" : {\"device_id\" : \"air123\", \"name\" : \"May loc 1\", \"position\" : \"Phong khach\"} }]"
#define MESSAGE_LOGIN_FAIL "{ \"success\" : \"false\", \"error\" : \"Wrong user name or password\"}"
#define CHANGE_PASSWORD_SUCCESS "{ \"success\" : \"true\" }"
#define CHANGE_PASSWORD_FAIL "{ \"success\" : \"false\" }"
char username_Login[] = "hungdv39";
char password_Login[] = "hungdv39";
char sessionID[] = "session_01";

#define TIMEOUT_SVM		5000
#define BUFF_SIZE 		2048

#define CO2_LEVEL_1		500
#define CO2_LEVEL_2		700
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#ifdef __GNUC__
  /* With GCC, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

osThreadId updateTaskHandle;
osThreadId IRListeningTaskHandle;
osThreadId displayTaskHandle;
osThreadId myLogTaskHandle;
/* USER CODE BEGIN PV */
MQTTClient client = DefaultClient;
Network network;
MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
svm30_t sensorSVM30data;

uint8_t Rx_data[2];

unsigned char send_buf[512];
unsigned char recv_buf[2048];

char controlState[40];
char airAQI[50];
char AQI_newver[100];
char state_newver[100];

//Iar quality variables
uint16_t tvoc_ppb = 0, co2_ppm = 0;
int32_t temperature = 0, humidity = 0;
uint16_t pm2_5, pm1_0, pm10;
// IR control variables
char trans_str[96] = {0,};
static char *decode_str[] = {"UNUSED", "UNKNOWN", "RC5", "RC6", "NEC", "SONY", "PANASONIC", "JVC", "SAMSUNG", "WHYNTER", "AIWA_RC_T501", "LG", "SANYO", "MITSUBISHI", "DISH", "SHARP", "DENON", "PRONTO"};
int flagSW, flagSP, flagMode, flagIon;
// Button Remote
int state1 = 1, state2 = 1;
int flagPBT, flagSBT, flagMBT ;
uint32_t nowb;
uint32_t future;
int long_press = 0;

//control mode variables
uint8_t power = 0, fan_speed = 0, night_mode = 0, filter_mode = 0, control_mode = 0, uv_state = 0, ion_state = 0;
uint8_t sys_mode = 0; // For new app
uint8_t vanst = 0, preVanst = 0;

uint8_t cap_speed_available = 0;
uint32_t cap_speed;

uint8_t fan_flag = 0;

uint32_t i = 0, value1 = 1;
uint8_t ring2_buff[BUFF_SIZE];
RINGBUF RxUart2RingBuff;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM5_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM3_Init(void);
void StartUpdateTask(void const * argument);
void StartIR_Task(void const * argument);
void StartDisplayTask(void const * argument);
void StartTaskLogging(void const * argument);

/* USER CODE BEGIN PFP */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
  //HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
	RINGBUF_Put(&RxUart2RingBuff, ch);
  return ch;
}

void init_MQTT(void);
void setUVState(uint8_t state);
void setIONState(uint8_t state);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void fan_run(freq)
{
	uint16_t cycle = 0, duty = 0;
	uint16_t freq_v;
	freq_v = freq;
	if(freq_v < 50) freq_v = 50;
	if(freq_v >275) freq_v = 275;
	cycle = round(1000000/freq_v);
	duty = cycle/2;
	//level_t = 100 - level;
//	TIM1->ARR = (3636 + level_t*163);
//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 1818 + level_t*81);
	TIM1->ARR = cycle;
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4,duty);
/*	if(level==6)
	{
		TIM1->ARR = 1999;
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 1000);
	}
	else if(level==5)
	{
		TIM1->ARR = 2999;
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 1500);
	}
	else if(level==4)
	{
		TIM1->ARR = 3999;
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 2000);
	}
	else if(level==3)
	{
		TIM1->ARR = 4999;
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 2500);
	}
	else if(level==2)
	{
		TIM1->ARR = 5999;
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 3000);
	}
	else if(level==1)
	{
		TIM1->ARR = 19999;
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 10000);
	}	*/
}

uint32_t fan_read()
{
	uint32_t speed_t;
	HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_1);
	HAL_Delay(1);
	while(value1==-1)
	{
	}
	speed_t = 4000000/value1;
	HAL_TIM_IC_Stop_IT(&htim5, TIM_CHANNEL_1);
	if((speed_t >0)&&(speed_t < 2001))
			return speed_t;
		else return 0;
}


uint32_t xiaomi_fan_read(void){
	HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_1);
	while(!cap_speed_available){
		printf("not yet\r\n");
		HAL_Delay(50);
	}
	cap_speed_available = 0;
	printf("capture speed %u\r\n", cap_speed);
	return cap_speed;
}

void Get_Json_login(void* message){

	enum { MAX_FIELDS = 4 };
	json_t pool[ MAX_FIELDS ];

	 json_t const* parent = json_create( message, pool, MAX_FIELDS );
	 if ( parent == NULL )
		 printf("Nothing in payload\r\n");
	 json_t const* namefield = json_getProperty( parent, "user_name" );
	 if ( namefield == NULL )
		 printf("user_name is not exist in payload\r\n");
	 if ( json_getType( namefield ) != JSON_TEXT )
		 printf("Value is wrong\r\n");
	 json_t const* passfield = json_getProperty( parent, "password" );
	 if ( passfield == NULL )
		 printf("password is not exist in payload\r\n");
	 if ( json_getType( passfield ) != JSON_TEXT )
		 printf("Value is wrong\r\n");
	 json_t const* sessionField = json_getProperty( parent, "session_id" );
	 if ( sessionField == NULL )
		 printf("password is not exist in payload\r\n");
	 if (json_getType( sessionField ) != JSON_TEXT )
		 printf("Value is wrong\r\n");
	 char const* namevalue = json_getValue( namefield );
	 char const* passvalue = json_getValue( passfield );
	 char const* sessionID = json_getValue(sessionField);
	 printf( "%s%s%s", passvalue, namevalue, "'.\n" );
	 /*if(!strncmp(namevalue,username_Login,strlen(username_Login) ) && strncmp(passvalue,password_Login,strlen(password_Login))){
		 publish(MESSAGE_LOGIN_SUCCESS, REPONSE_LOGIN);
	 } else {
		 publish(MESSAGE_LOGIN_FAIL,REPONSE_LOGIN);
	 }*/
	 /* Subcribe topic */
}

void Get_Json_changepass(void* message){

	enum { MAX_FIELDS = 4 };
	json_t pool[ MAX_FIELDS ];

	json_t const* parent = json_create( message, pool, MAX_FIELDS );
	if ( parent == NULL )
		printf("Nothing in payload\r\n");
	json_t const* namefield = json_getProperty( parent, "user_name" );
	if ( namefield == NULL )
		printf("Nothing in payload\r\n");
	if ( json_getType( namefield ) != JSON_TEXT )
		printf("Nothing in payload\r\n");
	json_t const* old_passfield = json_getProperty( parent, "old_password" );
	if ( old_passfield == NULL )
		printf("Nothing in payload\r\n");
	if ( json_getType( old_passfield ) != JSON_TEXT )
		printf("Nothing in payload\r\n");
	json_t const* new_passfield = json_getProperty( parent, "new_password" );
	if ( new_passfield == NULL )
		printf("Nothing in payload\r\n");
	if ( json_getType( new_passfield ) != JSON_TEXT )
		printf("Nothing in payload\r\n");
	char const* namevalue = json_getValue( namefield );
	char const* old_passvalue = json_getValue( old_passfield );
	char const* new_passvalue = json_getValue( new_passfield);
	printf( "%s%s%s%s", old_passvalue, new_passvalue, namevalue, "'.\n" );
	if(!strncmp(namevalue,username_Login,strlen(username_Login) ) && strncmp(old_passvalue,password_Login,strlen(password_Login))){
		publish(CHANGE_PASSWORD_SUCCESS ,REPONSE_CHANGE_PASSWORD);
	} else {
		publish(CHANGE_PASSWORD_FAIL ,REPONSE_CHANGE_PASSWORD);
	}
}

void Get_Json_collect_air_quality(void* message){
	//
}

void Get_Json_control(void* message){

	enum { MAX_FIELDS = 4 };
	json_t pool[ MAX_FIELDS ];

	json_t const* parent = json_create( message, pool, MAX_FIELDS );
	json_t const* modefield = json_getProperty( parent, "mode" );
	if(modefield != NULL){
		char const* modevalue = json_getValue( modefield );
		printf("mode value %s\r\n", modevalue);
		if(!strncmp(modevalue,"auto",strlen("auto"))){
			sys_mode = AUTO_MODE;
			publish(state_newver,REPONSE_CONTROL_STATE);
		}
		else if(!strncmp(modevalue,"fresh_air",strlen("fresh_air"))){
			vanst = DAMPER_OUT;

			sys_mode = FRESH_MODE;
			publish(state_newver,REPONSE_CONTROL_STATE);
		}
		else if(!strncmp(modevalue,"indoor",strlen("indoor"))){
			vanst = DAMPER_IN;

			sys_mode = INDOOR_MODE;
			publish(state_newver, REPONSE_CONTROL_STATE);

		}
		else if(!strncmp(modevalue,"night_mode",strlen("night_mode"))){
			fan_speed = LOW_SPEED;
			//xiaomi_fan_run(LOW_SPEED);

			sys_mode = NIGHT_MODE;
			publish(state_newver, REPONSE_CONTROL_STATE);

		}
	}
	json_t const* ionfield = json_getProperty( parent, "ion" );
	if(ionfield != NULL){
		char const* ionvalue = json_getValue(ionfield);
		if(!strncmp(ionvalue,"on",strlen("on"))){
			ion_state = ST_ON;
			setIONState(ST_ON);
			publish(state_newver, REPONSE_CONTROL_STATE);

		}
		else if(!strncmp(ionvalue,"off",strlen("off"))){
			ion_state = ST_OFF;
			setIONState(ST_OFF);

			publish(state_newver, REPONSE_CONTROL_STATE);
		}

	}
	json_t const* speedfield = json_getProperty( parent, "speed" );
	if(speedfield != NULL){
		char const* speedvalue = json_getValue(speedfield);
		if(!strncmp(speedvalue,"0",strlen("0"))){
			//xiaomi_fan_stop();

			fan_speed = STOP_SPEED;
			vanst = DAMPER_CLOSE;
			publish(state_newver, REPONSE_CONTROL_STATE);

		}
		else if(!strncmp(speedvalue,"1",strlen("1"))){
			fan_speed = LOW_SPEED;
			//xiaomi_fan_run(LOW_SPEED);

			publish(state_newver, REPONSE_CONTROL_STATE);

		}
		else if(!strncmp(speedvalue,"2",strlen("2"))){
			fan_speed = MED_SPEED;
			//xiaomi_fan_run(MED_SPEED);

			publish(state_newver, REPONSE_CONTROL_STATE);

		}
		else if(!strncmp(speedvalue,"3",strlen("3"))){
			fan_speed = HIGH_SPEED;
			//xiaomi_fan_run(HIGH_SPEED);

			publish(state_newver, REPONSE_CONTROL_STATE);
		}

	}
	json_t const* speedTfield = json_getProperty( parent, "speedTest" );
		if(speedTfield != NULL){
			char const* speedTValue = json_getValue(speedTfield);
			printf("speed Test value %s\r\n", speedTValue);
			uint16_t speed_test = atoi(speedTValue);
			if(speed_test){
				fan_run(speed_test);
			}
		}

}

void receive_handler_collect(MessageData* rx_msg){

	char* topic = rx_msg->topicName->lenstring.data;
	printf("receive mqtt topic %s\r\n", topic);
	char *message = rx_msg->message->payload;
	message[rx_msg->message->payloadlen] = '\0';

	printf("receive mqtt %s %d\r\n", rx_msg->message->payload, rx_msg->message->payloadlen);
	 if (!strncmp(topic,TOPIC_GET_AIR_QUALITY, strlen(TOPIC_GET_AIR_QUALITY))){
			publish(AQI_newver, REPONSE_GET_AIR_QUALITY);
	 }
	 else if (!strncmp(topic,TOPIC_GET_CONTROL_STATE, strlen(TOPIC_GET_CONTROL_STATE))){
			publish(state_newver,REPONSE_GET_STATE);
	}


}

void receive_handler_login(MessageData* rx_msg){

	char* topic = rx_msg->topicName->lenstring.data;
	printf("receive mqtt topic %s\r\n", topic);
	char *message = rx_msg->message->payload;
	message[rx_msg->message->payloadlen] = '\0';
	printf("receive mqtt %s %d\r\n", rx_msg->message->payload, rx_msg->message->payloadlen);

	if (!strncmp(topic,TOPIC_LOGIN, strlen(TOPIC_LOGIN))){

		Get_Json_login(rx_msg->message->payload);
		// su dung parse json de lay user_name and password
		//publsih thong tin tai khoan
	}
	if (!strncmp(topic,TOPIC_CHANGE_PASSWORD, strlen(TOPIC_CHANGE_PASSWORD))){
		Get_Json_changepass(rx_msg->message->payload);
		// su dung parse json de lay truong old_password, new_password
		// publish lai success hoac fail
	}
}

void receive_handler_control(MessageData* rx_msg){

	char* topic = rx_msg->topicName->lenstring.data;
	printf("receive mqtt topic %s\r\n", topic);
	char *message = rx_msg->message->payload;
	message[rx_msg->message->payloadlen] = '\0';
	printf("receive mqtt %s %d\r\n", rx_msg->message->payload, rx_msg->message->payloadlen);

	if (!strncmp(topic,TOPIC_REQUEST_CONTROL_MODE, strlen(TOPIC_REQUEST_CONTROL_MODE))){

		// sử dụng parse json de lay gia trị các trư�?ng nếu buffer khác 0 thì xét để đi�?u khiển trạng thái
		Get_Json_control(rx_msg->message->payload);
	}

	if (!strncmp(topic,TOPIC_REQUEST_ALERT, strlen(TOPIC_REQUEST_ALERT))){

		//publish phản hồi v�? Alert
	}

}

// Subcribse Mqtt ver mới

void Subcribes_MQTT_New(void){

	MQTTSubscribe(&client, TOPIC_LOGIN , QOS0, receive_handler_login);
	HAL_Delay(100);
	MQTTSubscribe(&client, TOPIC_CHANGE_PASSWORD  , QOS0, receive_handler_login);
	HAL_Delay(100);
	MQTTSubscribe(&client, TOPIC_GET_AIR_QUALITY  , QOS0, receive_handler_collect);
	HAL_Delay(100);
	MQTTSubscribe(&client, TOPIC_GET_CONTROL_STATE , QOS0, receive_handler_collect);
	HAL_Delay(100);
	MQTTSubscribe(&client, TOPIC_REQUEST_CONTROL_MODE , QOS0, receive_handler_control);
	HAL_Delay(100);
	MQTTSubscribe(&client, TOPIC_REQUEST_ALERT , QOS0, receive_handler_control);
}


void setUVState(uint8_t state){
	HAL_GPIO_WritePin(UV_GPIO_Port, UV_Pin, state);
	uv_state = state;
}

void setIONState(uint8_t state){
	HAL_GPIO_WritePin(ION_GPIO_Port, ION_Pin, state);
	ion_state = state;
}

static void updateAirAQI(void){
	int tmpIntte1,tmpInthu1,tmpIntte2,tmpInthu2;
	float temp, hum;
	/*tmpIntPM1 = (int)pm2_5;
	float tmpFPM = pm2_5 - tmpIntPM1;
	tmpIntPM2 = trunc(tmpFPM * 100);*/
	//temperature = 9977;
	temp = (float)(temperature/1000.0f);
	tmpIntte1 = (int)temp;
	float tmpT = temp - tmpIntte1;
	tmpIntte2 = trunc(tmpT * 100);
	//humidity = 9876;
	hum = (float)(humidity/1000.0f);
	tmpInthu1 = (int) hum;
	float tmpH = hum - tmpInthu1;
	tmpInthu2 = trunc(tmpH * 100);

	sprintf(airAQI,"%d.%d-%d.%d-%d-%d-%d",tmpIntte1,tmpIntte2,tmpInthu1,tmpInthu2,pm2_5, co2_ppm, tvoc_ppb);
	sprintf(AQI_newver,"{\"temp\":\"%d.%d\",\"rh\":\"%d.%d\",\"pm25\":\"%d\",\"co2\":\"%d\",\"tvoc\":\"%d\",\"pm1\":\"%d\",\"pm10\":\"%d\",\"aq\":\"good\"}", tmpIntte1, tmpIntte2,tmpInthu1,tmpInthu2, pm2_5, co2_ppm, tvoc_ppb, pm1_0, pm10);
}

void sysMode2Str(char *mode_str){
	if(sys_mode == AUTO_MODE){
		sprintf(mode_str, "auto");
	}else if(sys_mode == FRESH_MODE){
		sprintf(mode_str, "fresh_air");
	}else if(sys_mode == INDOOR_MODE){
		sprintf(mode_str, "indoor");
	}else if(sys_mode == NIGHT_MODE){
		sprintf(mode_str, "night_mode");
	}
}

void ion2Str(char *ion_str){
	if (ion_state == ST_OFF){
		sprintf(ion_str,"off");
	}
	else {
		sprintf(ion_str,"on");
	}
}

void speed2Str(char *speed_str){
	if (fan_speed == STOP_SPEED){
		sprintf(speed_str,"0");
		}
	else if (fan_speed == LOW_SPEED) {
		sprintf(speed_str,"1");
	}
	else if (fan_speed == MED_SPEED) {
		sprintf(speed_str,"2");
	}
	else {
		sprintf(speed_str,"3");
	}
}

void updateControlState(void){
	char powerc[4];
	char speedc[2];
	char night[4];
	char filter[7];
	char control[7];
	char uv[4];
	char ion[4];
	char mode_str[10] = {0};
	if (power == ST_OFF){
		sprintf(powerc,"OFF");
	}
	else {
		sprintf(powerc,"ON");
	}
	if (fan_speed == 0){
		sprintf(speedc,"0");
		}
	else if (fan_speed == LOW_SPEED) {
		sprintf(speedc,"1");
	}
	else if (fan_speed == MED_SPEED) {
		sprintf(speedc,"2");
	}
	else {
		sprintf(speedc,"3");
	}
	if (night_mode == ST_OFF){
		sprintf(night,"off");
	}
	else {
		sprintf(night,"on");
	}

	if (filter_mode == FRESH){
		sprintf(filter,"fresh");
	}
	else {
		sprintf(filter,"indoor");
	}

	if (control_mode == MANUAL){
		sprintf(control,"manual");
	}
	else {
		sprintf(control,"auto");
	}

	if (uv_state == ST_OFF){
		sprintf(uv,"off");
	}
	else {
		sprintf(uv,"on");
	}
	if (ion_state == ST_OFF){
		sprintf(ion,"off");
	}
	else {
		sprintf(ion,"on");
	}

	if(sys_mode == AUTO_MODE){
		sprintf(mode_str, "auto");
	}else if(sys_mode == FRESH_MODE){
		sprintf(mode_str, "fresh_air");
	}else if(sys_mode == INDOOR_MODE){
		sprintf(mode_str, "indoor");
	}else if(sys_mode == NIGHT_MODE){
		sprintf(mode_str, "night_mode");
	}
	sprintf(controlState,"%s-%s-%s-%s-%s-%s-%s",powerc,speedc,night,filter,control,uv,ion);
	sprintf(state_newver,"{\"speed\":\"%s\",\"mode\":\"%s\",\"ion\":\"%s\"}",speedc, mode_str , ion );
}

void init_MQTT(void){
	int ret;
	if(setCmdMode("machine") < 0){
		printf("set cmd failed\r\n");
	}

	data.MQTTVersion = 4;
	data.clientID.cstring = MQTT_CLIENT_ID;
	data.username.cstring = MQTT_USER;
	data.password.cstring = MQTT_PASSWORD;
	data.keepAliveInterval = 20;
	HAL_Delay(100);
	if(setupNetwork(SSID_WIFI, PASS_WIFI)){
	  printf("setup network failed\r\n");
	}
	HAL_Delay(100);
	printf("init network\r\n");
	if(NetworkInit(&network))
	  printf("init failed\r\n");
	else
	  printf("network up\r\n");
	HAL_Delay(100);
	ret = isTCPPortOpen();
	if(ret >= 0){
		closeTCPPort(ret);
	}
	HAL_Delay(100);
	ret = NetworkConnect(&network, MQTT_HOST, MQTT_PORT);
	if(ret == FAILURE)
	{
	  // error, return
	  printf("network connect tcp failed\r\n");
	  return ret;
	}

	printf("network connect tcp success\r\n");
	HAL_Delay(100);

	MQTTClientInit(&client, &network, 10000, send_buf, sizeof(send_buf), recv_buf, sizeof(recv_buf));

	ret = MQTTConnect(&client, &data);
	if(ret != SUCCESS)
	{
	  // error, return
	  printf("failed to connect mqtt\r\n");
	  NetworkDisconnect(&network);
	  return ret;
	}
	printf("success to connect mqtt\r\n");

}

void init_MQTT_New(void){
	int ret;
	uint8_t rxbuf[128];
	HAL_UART_Receive(&huart1, rxbuf, sizeof(rxbuf), 1000);
	if(setCmdMode("machine") < 0){
		printf("set cmd failed\r\n");
	}

	data.MQTTVersion = 4;
	data.clientID.cstring = MQTT_CLIENT_ID;
	//data.username.cstring = MQTT_USER_N;
	//data.password.cstring = MQTT_PASSWORD_N;
	data.keepAliveInterval = 10;
	HAL_Delay(100);
	if(setupNetwork(SSID_WIFI, PASS_WIFI)){
	  printf("setup network failed\r\n");
	}
	HAL_Delay(100);
	printf("init network\r\n");
	if(NetworkInit(&network))
	  printf("init failed\r\n");
	else
	  printf("network up\r\n");
	HAL_Delay(100);
	ret = isTCPPortOpen();
	if(ret >= 0){
		closeTCPPort(ret);
	}
	HAL_Delay(100);
	ret = NetworkConnect(&network, MQTT_HOST_N, MQTT_PORT_N);
	if(ret == FAILURE)
	{
	  // error, return
	  printf("network connect tcp failed\r\n");
	  return ret;
	}

	printf("network connect tcp success\r\n");
	HAL_Delay(100);

	MQTTClientInit(&client, &network, 10000, send_buf, sizeof(send_buf), recv_buf, sizeof(recv_buf));

	ret = MQTTConnect(&client, &data);
	if(ret != SUCCESS)
	{
	  // error, return
	  printf("failed to connect mqtt\r\n");
	  NetworkDisconnect(&network);
	  return ret;
	}
	printf("success to connect mqtt\r\n");

}


int publish(void* payload, void* topic)
{
    MQTTMessage message;
    int rc;
    message.qos = QOS1;
    message.retained = 1;
    message.dup = 0;
    message.payload = payload;
    message.payloadlen = strlen((char*)payload);

    rc = MQTTPublish(&client, topic, &message);
    return rc;
}

static void receive_handler(MessageData* rx_msg)
{

    char* topic = rx_msg->topicName->lenstring.data;
    //topic[rx_msg->topicName->lenstring.len] = '\0';
    printf("receive mqtt topic %s\r\n", topic);
    char *message = rx_msg->message->payload;
	message[rx_msg->message->payloadlen] = '\0';
	printf("receive mqtt %s %d\r\n", rx_msg->message->payload, rx_msg->message->payloadlen);
    if (!strncmp(topic,TOPIC_DEVICE_ID, strlen(TOPIC_DEVICE_ID))){
    	publish(MQTT_MESSAGE, RESPONSE_DEVICE_INFO_TOPIC);
    }
    else if (!strncmp(topic,TOPIC_AIR_QUALITY, strlen(TOPIC_AIR_QUALITY))){
    	if (!strncmp(rx_msg->message->payload, "get-hethong123", rx_msg->message->payloadlen)){
    		publish(airAQI, TOPIC_PUB_AIR_QUALITY);
    	}
    }
    else if (!strncmp(topic,TOPIC_CONTROL_STATE, strlen(TOPIC_CONTROL_STATE))){
		if (!strncmp(rx_msg->message->payload, "get-hethong123", rx_msg->message->payloadlen)){
			publish(controlState,TOPIC_PUB_CONTROL_STATE);
		}
    }
    else if (!strncmp(topic,TOPIC_POWER_ON, strlen(TOPIC_POWER_ON))){
		if (!strncmp(rx_msg->message->payload, "control-hethong123", rx_msg->message->payloadlen)){
			power = ST_ON;
			fan_speed = MED_SPEED;
			//fan_12_run(medspeed);
			xiaomi_fan_run(MED_SPEED);
			//SwitchIR = 1;
		}
	}
    else if (!strncmp(topic,TOPIC_POWER_OFF, strlen(TOPIC_POWER_OFF))){
		if (!strncmp(rx_msg->message->payload, "control-hethong123", rx_msg->message->payloadlen)){
			power = ST_OFF;
			fan_speed = ST_OFF;
			vanst = 3;
			xiaomi_fan_stop();
			//SwitchIR = 2;
		}
	}
    else if (!strncmp(topic,TOPIC_SPEED_LOW, strlen(TOPIC_SPEED_LOW))){
		if (!strncmp(rx_msg->message->payload, "control-hethong123", rx_msg->message->payloadlen)){
			fan_speed = LOW_SPEED;
			xiaomi_fan_run(LOW_SPEED);
			//SpeedIR = 1;
		}
	}
    else if (!strncmp(topic, TOPIC_SPEED_MED, strlen(TOPIC_SPEED_MED))){
   		if (!strncmp(rx_msg->message->payload, "control-hethong123", rx_msg->message->payloadlen)){
   			fan_speed = MED_SPEED;
   			xiaomi_fan_run(MED_SPEED);
   			//SpeedIR=2;
   		}
   	}
    else if (!strncmp(topic, TOPIC_SPEED_HIGH, strlen(TOPIC_SPEED_HIGH))){
		if (!strncmp(rx_msg->message->payload, "control-hethong123", rx_msg->message->payloadlen)){
			fan_speed = HIGH_SPEED;
			xiaomi_fan_run(HIGH_SPEED);
			//SpeedIR=3;
		}
	}
    else if (!strncmp(topic,TOPIC_NIGHT_MODE, strlen(TOPIC_NIGHT_MODE))){
		if (!strncmp(rx_msg->message->payload, "control-hethong123-on", rx_msg->message->payloadlen)){
			fan_speed = LOW_SPEED;
			xiaomi_fan_run(LOW_SPEED);
			night_mode = ST_ON;
			//ModeIR = 3;
		}
		else if (!strncmp(rx_msg->message->payload, "control-hethong123-off", rx_msg->message->payloadlen)){
			fan_speed = MED_SPEED;
			xiaomi_fan_run(MED_SPEED);
			night_mode = ST_OFF;
		}
	}
    else if (!strncmp(topic,TOPIC_FILTER, strlen(TOPIC_FILTER))){
		if (!strncmp(rx_msg->message->payload, "control-hethong123-fresh", rx_msg->message->payloadlen)){
			filter_mode = FRESH;
			vanst = DAMPER_OUT;
			//ModeIR = 1;
		}
		else if (!strncmp(rx_msg->message->payload, "control-hethong123-indoor", rx_msg->message->payloadlen)){
			filter_mode = INDOOR;
			vanst = DAMPER_IN;
			//ModeIR = 2;
		}
	}
    else if (!strncmp(topic, TOPIC_CONTROL, strlen(TOPIC_CONTROL))){
  		if (!strncmp(rx_msg->message->payload, "control-hethong123-auto", rx_msg->message->payloadlen)){
  			control_mode = AUTOMATIC ;
  		}
  		else if (!strncmp(rx_msg->message->payload, "control-hethong123-manual", rx_msg->message->payloadlen)){
  			control_mode = MANUAL;
  		}
  	}
    else if (!strncmp(topic, TOPIC_UV, strlen(TOPIC_UV))){
   		if (!strncmp(rx_msg->message->payload, "control-hethong123-on", rx_msg->message->payloadlen)){
   			uv_state = ST_ON;
   			setUVState(ST_ON);
   		}
   		else if (!strncmp(rx_msg->message->payload, "control-hethong123-off", rx_msg->message->payloadlen)){
   			uv_state = ST_OFF;
   			setUVState(ST_OFF);
   		}
   	}
    else if (!strncmp(topic, TOPIC_ION, strlen(TOPIC_ION))){
		if (!strncmp(rx_msg->message->payload, "control-hethong123-on", rx_msg->message->payloadlen)){
			ion_state = ST_ON;
			setIONState(ST_ON);
			//IonIR = 1;

		}
		else if (!strncmp(rx_msg->message->payload, "control-hethong123-off", rx_msg->message->payloadlen)){
			ion_state = ST_OFF;
			setIONState(ST_OFF);
			//IonIR = 2;
		}
	}
}

void subcribesMQTT(void){
	MQTTSubscribe(&client, MQTT_SUB_TOPIC, QOS0, receive_handler);
	HAL_Delay(100);
	MQTTSubscribe(&client, TOPIC_DEVICE_ID, QOS0, receive_handler);
	HAL_Delay(100);
	MQTTSubscribe(&client, TOPIC_AIR_QUALITY, QOS0, receive_handler);
	HAL_Delay(100);
	MQTTSubscribe(&client, TOPIC_CONTROL_STATE, QOS0, receive_handler);
	HAL_Delay(100);
	MQTTSubscribe(&client, TOPIC_POWER_ON, QOS0, receive_handler);
	HAL_Delay(100);
	MQTTSubscribe(&client, TOPIC_POWER_OFF, QOS0, receive_handler);
	HAL_Delay(100);
	MQTTSubscribe(&client, TOPIC_NIGHT_MODE, QOS0, receive_handler);
	HAL_Delay(100);
	MQTTSubscribe(&client, TOPIC_ION, QOS0, receive_handler);
	HAL_Delay(100);
	MQTTSubscribe(&client, TOPIC_UV, QOS0, receive_handler);
	HAL_Delay(100);
	MQTTSubscribe(&client, TOPIC_FILTER, QOS0, receive_handler);
	HAL_Delay(100);
	MQTTSubscribe(&client, TOPIC_CONTROL, QOS0, receive_handler);
	HAL_Delay(100);
	MQTTSubscribe(&client, TOPIC_SPEED_LOW, QOS0, receive_handler);
	HAL_Delay(100);
	MQTTSubscribe(&client, TOPIC_SPEED_MED, QOS0, receive_handler);
	HAL_Delay(100);
	MQTTSubscribe(&client, TOPIC_SPEED_HIGH, QOS0, receive_handler);
	printf("finish subcribe\r\n");
}

void Control_IR(void){

	 if (power == ST_ON && flagSW == 1){

		  fan_speed = MED_SPEED;
		  xiaomi_fan_run(MED_SPEED);
		  flagSW = 0;
	  }
	  else if (power == ST_OFF && flagSW == 1) {

		  fan_speed = 0;
		  xiaomi_fan_stop();
		  flagSW = 0;
	  }
	  if (fan_speed == LOW_SPEED && flagSP == 1 && power == ST_ON){

		  xiaomi_fan_run(LOW_SPEED);
		  flagSP = 0;
	  }
	  else if (fan_speed == MED_SPEED && flagSP == 1 && power == ST_ON){

		  xiaomi_fan_run(MED_SPEED);
		  flagSP = 0;
	  }
	  else if (fan_speed == HIGH_SPEED && flagSP == 1 && power == ST_ON){

		  xiaomi_fan_run(HIGH_SPEED);
		  flagSP = 0;
	  }
	  if (sys_mode == FRESH_MODE && flagMode == 1 && power == ST_ON){
		  vanst = DAMPER_OUT;
		  flagMode = 0;
	  }
	  else if (sys_mode == INDOOR_MODE && flagMode == 1 && power == ST_ON){
		  vanst = DAMPER_IN;
		  flagMode = 0;
	  }
	  else if (sys_mode == NIGHT_MODE && flagMode == 1 && power == ST_ON){
		   fan_speed = LOW_SPEED;
		    xiaomi_fan_run(LOW_SPEED);
			flagMode = 0;

	  }
	  if (ion_state == ST_ON && flagIon == 1 && power == ST_ON){
		  setIONState(ST_ON);
		  flagIon = 0;
	  }
	  else if (ion_state == ST_OFF && flagIon == 1 && power == ST_ON) {
		  setIONState(ST_OFF);
		  flagIon = 0;
	  }
}

void Receive_IR(void){


		if(my_decode(&results)) {
			snprintf(trans_str, 96, "Cod: %p | Type: %s | Bits: %d\n", (void*)results.value, decode_str[results.decode_type + 1], results.bits);
			HAL_UART_Transmit(&huart2, (uint8_t*)&trans_str, sizeof(trans_str), 100);

			if ((void*)results.value == 0xffb04f ){
				power++;
				flagSW = 1;
				if (power == 2){
					power = 0;
				}
			}
			if ((void*)results.value == 0xff22dd ){
				fan_speed++;
				flagSP = 1;
				if (fan_speed == 4){
					fan_speed = 1;
				}
			}
			if ((void*)results.value == 0xff7887 ){
				sys_mode++;
				flagMode = 1;
				if (sys_mode == 5){
					sys_mode = 1;
				}
			}
			if ((void*)results.value == 0xff8877  ){
				ion_state++;
				flagIon = 1;
				if (ion_state == 2){
					ion_state = 0;
				}
			}
			Control_IR();

			my_resume();
		}

}

void Check_button(void){
	if (power == ST_OFF && flagPBT == 1){
		flagPBT = 0;
		fan_speed = MED_SPEED;
		xiaomi_fan_run(MED_SPEED);

	}
	else if (power == ST_ON && flagPBT == 1){
		flagPBT = 0;
		fan_speed = 0;
		xiaomi_fan_stop();

	}
	 if (fan_speed == LOW_SPEED && flagSBT == 1 && power == ST_ON){

		  xiaomi_fan_run(LOW_SPEED);
		  flagSBT = 0;
	  }
	  else if (fan_speed == MED_SPEED && flagSBT == 1 && power == ST_ON){

		  xiaomi_fan_run(MED_SPEED);
		  flagSBT = 0;
	  }
	  else if (fan_speed == HIGH_SPEED && flagSBT == 1 && power == ST_ON){

		  xiaomi_fan_run(HIGH_SPEED);
		  flagSBT = 0;
	  }
	  if (sys_mode == FRESH_MODE && flagMBT == 1 && power == ST_ON){
		  vanst = DAMPER_OUT;
		  flagMBT = 0;
	  }
	  else if (sys_mode == INDOOR_MODE && flagMode == 1 && power == ST_ON){
		  vanst = DAMPER_IN;
		  flagMBT = 0;
	  }
	  else if (sys_mode == NIGHT_MODE && flagMode == 1 && power == ST_ON){
		   fan_speed = LOW_SPEED;
		    xiaomi_fan_run(LOW_SPEED);
			flagMBT = 0;

	  }

}

void Check_long_button(void){

	if (!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_7)){
		future = HAL_GetTick();

	}
	else {

		nowb = HAL_GetTick();
		if (nowb > (future + 4000) && power == ST_ON){
			power = ST_OFF;
			flagPBT = 1;
			printf("turn off power \r\n");
			future = HAL_GetTick();
		}
		else if (nowb > (future + 4000) && power == ST_OFF){
			power = ST_ON;
			flagPBT = 1;
			printf("turn on power \r\n");
			future = HAL_GetTick();
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	uint8_t buff[10];
	uint16_t cs, crc;
	int i;
  if(huart->Instance == USART2){
	  HAL_UART_Transmit(&huart2, (uint8_t *)&Rx_data[0], 1, 0xFFFF);
	  if(Rx_data[0] == 'h'){
		  xiaomi_fan_run(HIGH_SPEED);
	  }else if(Rx_data[0] == 'm'){
		  xiaomi_fan_run(MED_SPEED);
	  }else if(Rx_data[0] == 'l'){
		  xiaomi_fan_run(LOW_SPEED);
	  }else if(Rx_data[0] == 'r'){
		  //cap_speed = fan_read();
		  sprintf(buff, "%d\r\n", cap_speed);
		  HAL_UART_Transmit(&huart2, (uint8_t *)&buff[0], 10, 0xFFFF);
	  }
	  HAL_UART_Receive_IT(&huart2, Rx_data, 1);
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  int err;
  uint64_t now, end;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C2_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  //Slow down after init
  RINGBUF_Init(&RxUart2RingBuff, ring2_buff, BUFF_SIZE);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIM_IC_Stop_IT(&htim5, TIM_CHANNEL_1);
  HAL_Delay(100);
  printf("start\r\n");
  // Init IR
  my_enableIRIn();
  night_mode = ST_OFF;
  fan_speed = LOW_SPEED;
  power = ST_ON;
  control_mode = AUTOMATIC;
  filter_mode = FRESH;
  uv_state = ST_ON;
  ion_state = ST_OFF;
  // For new app
  sys_mode = AUTO_MODE;
  i2c_init_sensirion(&hi2c2);

  fan_run(70);
  /* Init MQTT */
  init_MQTT_New();
  Subcribes_MQTT_New();
  //init_MQTT();
  //subcribesMQTT();
  /* Init SVM */
  now = HAL_GetTick();
  end = now + TIMEOUT_SVM;
  while (svm_probe() != STATUS_OK) {
      printf("SVM30 module probing failed\r\n");
      now = HAL_GetTick();
      if(end < now)   // Timeout
      {
    	  break;
      }
  }
  if(now < end){
	  printf("SVM30 module probing successful\r\n");
  }else
	  printf("timeout to probe svm30\r\n");

  err = sgp30_iaq_init();

  initSMUARTPort(&huart6);
  setSMOutputMode(ASK_ANSWER); // Set SM to ask-answer mode
  HAL_UART_Receive_IT(&huart2, Rx_data, 1);

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of updateTask */
  osThreadDef(updateTask, StartUpdateTask, osPriorityNormal, 0, 512);
  updateTaskHandle = osThreadCreate(osThread(updateTask), NULL);

  /* definition and creation of IRListeningTask */
  osThreadDef(IRListeningTask, StartIR_Task, osPriorityNormal, 0, 128);
  IRListeningTaskHandle = osThreadCreate(osThread(IRListeningTask), NULL);

  /* definition and creation of displayTask */
  osThreadDef(displayTask, StartDisplayTask, osPriorityNormal, 0, 128);
  displayTaskHandle = osThreadCreate(osThread(displayTask), NULL);

  /* definition and creation of myLogTask */
  osThreadDef(myLogTask, StartTaskLogging, osPriorityIdle, 0, 128);
  myLogTaskHandle = osThreadCreate(osThread(myLogTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();
 
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV8;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV8;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 90000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 20;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 4999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 21000;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 50;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 20;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 50000;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, UV_Pin|ION_Pin|DAMPER_IN_2_Pin|DAMPER_IN_3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DAMPER_OUT_0_Pin|DAMPER_OUT_1_Pin|DAMPER_OUT_2_Pin|DAMPER_OUT_3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DAMPER_IN_0_Pin|DAMPER_IN_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : UV_Pin ION_Pin DAMPER_IN_2_Pin DAMPER_IN_3_Pin */
  GPIO_InitStruct.Pin = UV_Pin|ION_Pin|DAMPER_IN_2_Pin|DAMPER_IN_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : DAMPER_OUT_0_Pin DAMPER_OUT_1_Pin DAMPER_OUT_2_Pin DAMPER_OUT_3_Pin */
  GPIO_InitStruct.Pin = DAMPER_OUT_0_Pin|DAMPER_OUT_1_Pin|DAMPER_OUT_2_Pin|DAMPER_OUT_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DAMPER_IN_0_Pin DAMPER_IN_1_Pin */
  GPIO_InitStruct.Pin = DAMPER_IN_0_Pin|DAMPER_IN_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : IR_RECEIVE_PIN_Pin */
  GPIO_InitStruct.Pin = IR_RECEIVE_PIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IR_RECEIVE_PIN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPEED_BUTTON_Pin MODE_BUTTON_Pin */
  GPIO_InitStruct.Pin = SPEED_BUTTON_Pin|MODE_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	uint32_t value;
	if (htim->Instance==TIM5)
	{
		fan_flag++;
		if(fan_flag==1)
		{
//			value1 = __HAL_TIM_GetCounter(&htim2);    //read TIM2 channel 1 capture value
			__HAL_TIM_SetCounter(&htim5, 0);    //reset counter after input capture interrupt occurs
		}
		if(fan_flag==2)
		{
			fan_flag = 0;
			value = __HAL_TIM_GetCounter(&htim5);    //read TIM2 channel 1 capture value
			__HAL_TIM_SetCounter(&htim5, 0);    //reset counter after input capture interrupt occurs
			cap_speed = (1000000/value)*60/15;
			cap_speed_available = 1;
			HAL_TIM_IC_Stop_IT(&htim5, TIM_CHANNEL_1);
		}
	}
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartUpdateTask */
/**
  * @brief  Function implementing the updateTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartUpdateTask */
void StartUpdateTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
	int err;
	amphenol_t SMAmp;
	uint8_t SMAlarm;
  /* Infinite loop */
  for(;;)
  {
	  printf("Update task\r\n");
	  err = getSVM30(&sensorSVM30data);
	  if(err){
		  printf("Failed to get SVM30 data\r\n");
	  }else {
		  tvoc_ppb = sensorSVM30data.tvoc_ppb;
		  co2_ppm  = sensorSVM30data.co2_eq_ppm;
		  temperature = sensorSVM30data.temperature;
		  humidity = sensorSVM30data.humidity;
	  }
	  err = getSMData(&SMAmp, &SMAlarm);
	  if(err){
		  printf("Failed to get SM data\r\n");
	  }
	  pm2_5 = SMAmp.PM2p5_Environment;
	  pm1_0 = SMAmp.PM1_Environment;
	  pm10 = SMAmp.PM10_Environment;
	  updateAirAQI();
	  updateControlState();

	  //Behavior when run auto mode.
	  if(sys_mode == AUTO_MODE){
		  if(co2_ppm < CO2_LEVEL_1){
			  vanst = DAMPER_IN; // Turn on internal filter
			  //xiaomi_fan_run(MED_SPEED);
		  }else if(co2_ppm > CO2_LEVEL_1 && co2_ppm < CO2_LEVEL_2){
			  vanst = DAMPER_OUT; // Turn on fresh supply
			  //xiaomi_fan_run(LOW_SPEED);
		  }else {
			  vanst = DAMPER_OUT;
			  //xiaomi_fan_run(HIGH_SPEED);
		  }
	  }

	  if((vanst == DAMPER_OUT) && (vanst != preVanst)){
		  directionOfRotation(DAMPER_OUT, true, 220);
		  directionOfRotation(DAMPER_IN, false, 220);
		  preVanst = vanst;
		  printf("change vanst to damper out\r\n");
	  }else if((vanst == DAMPER_IN) && (vanst != preVanst)){
		  directionOfRotation(DAMPER_IN, true, 220);
		  directionOfRotation(DAMPER_OUT, false, 220);
		  preVanst = vanst;
		  printf("change vanst to damper in\r\n");
	  }else if(vanst != preVanst) {
		  directionOfRotation(DAMPER_IN, false, 220);
		  directionOfRotation(DAMPER_OUT, false, 220);
		  preVanst = vanst;
		  printf("close vanst\r\n");
	  }
	  if(TimerIsExpired(&client.last_conn_timer)){
		  printf("loss connect mqtt\r\n");
		  if(!isTCPPortOpen()){
			  printf("TCP port was closed by some reason\r\n");
		  }
		  if(checkNetworkStatus() != NET_UP){
			  printf("Loss wifi connection\r\n");
		  }
	  }else {
		  MQTTYield(&client, 300);
	  }

	  osDelay(100);
  }
  /* USER CODE END 5 */ 
}

/* USER CODE BEGIN Header_StartIR_Task */
/**
* @brief Function implementing the IRListeningTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartIR_Task */
void StartIR_Task(void const * argument)
{
  /* USER CODE BEGIN StartIR_Task */
	int i;
  /* Infinite loop */
  for(;;)
  {
	  i++;
	  Receive_IR();
	  Check_long_button();
	  Check_button();
	  //getSVM30(&sensorSVM30data);
	  osDelay(100);
  }
  /* USER CODE END StartIR_Task */
}

/* USER CODE BEGIN Header_StartDisplayTask */
/**
* @brief Function implementing the displayTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDisplayTask */
void StartDisplayTask(void const * argument)
{
  /* USER CODE BEGIN StartDisplayTask */
	int i;
  /* Infinite loop */
  for(;;)
  {
	  i++;
	  //getSVM30(&sensorSVM30data);
	  osDelay(100);
  }
  /* USER CODE END StartDisplayTask */
}

/* USER CODE BEGIN Header_StartTaskLogging */
/**
* @brief Function implementing the myLogTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskLogging */
void StartTaskLogging(void const * argument)
{
  /* USER CODE BEGIN StartTaskLogging */
	uint8_t rxchar;
  /* Infinite loop */
  for(;;)
  {
	  if(RINGBUF_GetFill(&RxUart2RingBuff)){
	  	RINGBUF_Get(&RxUart2RingBuff, &rxchar);
	  	HAL_UART_Transmit(&huart2, &rxchar, 1, 0xFFFF);
	  }
    //osDelay(1);
  }
  /* USER CODE END StartTaskLogging */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
    user_TIM_PeriodElapsedCallback(htim);

	if(htim->Instance == TIM3){
		if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_6) == GPIO_PIN_SET){
			//if (power == ST_ON ){
			fan_speed++;
			flagSBT = 1;
			printf("nhan button PB6 \r\n");
			if (fan_speed == 4){
				fan_speed = 1;
			}
			state1 = 1;
			HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);
			HAL_TIM_Base_Stop_IT(&htim3);

			//}
		}
		if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_7) == GPIO_PIN_SET){
		//if (power == ST_ON && button2 == 1){
			sys_mode++;
			printf("nhan button PB7 \r\n");
			flagMBT = 1;
			if (sys_mode == 5){
				sys_mode = 1;
			}
			state2 = 1;
			//button2 =0;
			HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);
			HAL_TIM_Base_Stop_IT(&htim3);
		//}
		}

	}

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
