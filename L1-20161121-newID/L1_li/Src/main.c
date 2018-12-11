/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

#include "Console.h"
#include "Tick.h"
#include "Thread.h"
#include "FPE.h"
#include "DelayCall.h"
#include "Button.h"
#include "CJSON.h"

#include "sim808.h"
#include "control.h"
#include "fpe.h"

#include "stdlib.h" //malloc
#include "math.h"  //fmin fmax

#include "md5.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

uint8_t g_main_consoleBuffer[0x100];
uint8_t g_main_dustBuffer[8];
uint8_t g_main_cmdBuffer[64];
float g_main_start_battery_voltage=0;      //2015-12-7, locate in line 161, before pm25,sim808 start.
float g_main_end_battery_voltage=0;        //2015-12-7 ,locate in 'do_sleep' function, after pm25,sim808 stop.
float g_main_battery_voltage=0;
int g_main_charge_flag = 0;

char defaultId[] = "L1";
char deviceId[(32+16)]={0};

uint8_t g_main_uid[8] = {0};
char g_main_uid_suffix[5] = {0};

#define NUMBER_APN_INFO_LENGTH_MAX 17
char g_main_apn[NUMBER_APN_INFO_LENGTH_MAX] = {0};
char g_main_user[NUMBER_APN_INFO_LENGTH_MAX] = {0};
char g_main_pass[NUMBER_APN_INFO_LENGTH_MAX] = {0};
uint8_t g_main_setApnMask = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

void battery_update(void);
void feedExdog(void);
void startAlarm(uint16_t second);
void enable_pm2d5(void);
void disable_pm2d5(void);
void update_gps(void);
void check_gprs(void);

void check_action_if_done(void);
void println(char* string);

static inline void ME2139_init(void);
//char *sim808_jsoninfo_creat(void);
static inline void pm2d5_receive_int(void);
void sim808_update(void);
void pm25_update(void);
void do_sleep(void);
void check_sleep(void);

void hts221_init(void);
void th_check_end(void);
void th_update(void);
static void hts221_enable(void);
static void hts221_disable(void);
void newUploadInterval_init(void);
void battery_charge_start_test(int times, uint32_t delay);

void optData2Usart2(void);
void updateSim808Status(void);

#define DEVICE_VERSION "L1" //20161123 changed
#define FPE_UID_ADDRESS 0x00110000

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

bool g_main_gps_updated = false;
bool g_main_pm25_stable = false;
bool g_main_startSleep = false;

//20160601
bool g_main_sim808_inited = false;
//20160601

bool g_main_gps_got = false;
bool g_main_http_action_got = false;
bool g_main_http_action_done = false;


//char* g_main_udpAddressDefault = "hwlantian.com";//"172.28.50.1";//
char* g_main_udpAddressDefault = "sensor.new.hwlantian.com";//"172.28.50.1";//  20161123 new server
char* g_main_udpAddress = NULL;
uint16_t g_main_udpPortDefault = 59830;
uint16_t g_main_udpPort = 0;

uint32_t g_main_uploadDataInterval = UPLOAD_DATA_INTERVAL_MS;


typedef struct
{	
	uint32_t batteryFlag;
	
	uint32_t uploadInterval;
	uint32_t uploadIntervalEnsure;
	
	double longitude;
	double latitude;
	double gpsTime;
	uint32_t gpsDataEnsure;
	uint32_t gpsNotUpdateCnt;
	
//	Sim900aStatus_t sim900aStatus;
}flashStored_Info_t;

flashStored_Info_t g_flashStored_Info = 
{
	.batteryFlag = RANGE_START_BATTERY_LOW,
	.uploadInterval = 1000,
	.uploadIntervalEnsure = 0,	//not ensured, ensure -> 0x0a0a
	.longitude = 0,
	.latitude = 0,
	.gpsTime = 0,
	.gpsDataEnsure = 0,
	.gpsNotUpdateCnt = 0,
	
};


volatile float g_main_temperature = 0;							
volatile float g_main_humidity = 0;
int32_t g_main_pm25 = 0;
int g_main_uploadTime = 0;




void loadFlashData(void)
{

	g_flashStored_Info.batteryFlag = fpe_read(ADDRESS_BATTERY_RANGE);

	g_flashStored_Info.uploadInterval = fpe_read(ADDRESS_UPLOAD_INTERVAL_VALUE);
	g_flashStored_Info.uploadIntervalEnsure = fpe_read(ADDRESS_UPLOAD_INTERVAL_ENSURE);

	g_flashStored_Info.longitude = fpe_read(ADDRESS_VALUE_GPS_LONGITUDE)/1000000.0;
	g_flashStored_Info.latitude = fpe_read(ADDRESS_VALUE_GPS_LATITUDE)/1000000.0;
	g_flashStored_Info.gpsTime = fpe_read(ADDRESS_VALUE_GPS_GPSTIME1)*1000000.0 + fpe_read(ADDRESS_VALUE_GPS_GPSTIME2);
	g_flashStored_Info.gpsDataEnsure = fpe_read(ADDRESS_GPS_UPDATED_ENSURE);
	g_flashStored_Info.gpsNotUpdateCnt = fpe_read(ADDRESS_CNT_GPS_NOUPDATE_TIME);
}


#define getSetting(name) (name?name:name##Default)
void mainThread(void)
{
	
	sim808_Info_t sim808_Info = sim808_GetInfo();

//		( (g_main_pm25_stable || tick_ms()>MAX_PM25_MS)	&& ( g_main_gps_updated || tick_ms()>MAX_GPS_MS) )
	if( !(//wait gps new data until timeout,when time out,we use data in flash if exist
		( (g_main_pm25_stable || tick_ms()>MAX_PM25_MS)	&& ( sim808_Info.gpsUpdated || tick_ms()>MAX_GPS_MS) )\
	||(tick_ms()>MAX_WAKE_MS -20000)
	) )
	{
		return;
	}
	
	//th_update();
	
	cJSON* json = cJSON_CreateObject();
	if (json)
	{
		deviceId[0] = '\0';
		sprintf(deviceId,"L1");  //2015-12-7
		if(strlen(g_main_uid_suffix))
		{
			sprintf(deviceId,"L1-");  //2015-12-7
			for (uint8_t i=0; i<8; i++)
			{
				sprintf(deviceId+strlen(deviceId),"%02x", g_main_uid[i]);  //2015-12-7
			}
			sprintf(deviceId+strlen(deviceId),"%.4s", g_main_uid_suffix);  //2015-12-7
		}
		
		cJSON_AddStringToObject(json,"devId",deviceId);
		cJSON_AddStringToObject(json,"ver",DEVICE_VERSION);
		
		cJSON_AddNumberToObject(json,"pm2d5",g_main_pm25/100.0);
		cJSON_AddNumberToObject(json,"temp",g_main_temperature);
		cJSON_AddNumberToObject(json,"hum",g_main_humidity);
		
		if (g_main_gps_updated)
		{
			cJSON_AddNumberToObject(json,"lon",(float)sim808_Info.longitude);
			cJSON_AddNumberToObject(json,"lat",(float)sim808_Info.latitude);
			cJSON_AddNumberToObject(json,"timestamp",sim808_Info.gpsTime);
		}
		cJSON_AddNumberToObject(json,"start_charge",g_main_start_battery_voltage); //2016-1-18
		cJSON_AddNumberToObject(json,"systemTick",tick_ms());
		
		char * str = cJSON_PrintUnformatted(json);
		cJSON_Delete(json);

		if (str)
		{
			uint16_t length = strlen(str);
			char* buffer = malloc(length+3);//20160808
			if (buffer)
			{
				sprintf(buffer,"%s\n%c",str,0x1a);//20160808
				char* address = getSetting(g_main_udpAddress);//?g_main_udpAddress:g_main_udpAddressDefault;
				uint16_t port = getSetting(g_main_udpPort);//g_main_udpPort!=0?g_main_udpPort:g_main_udpPortDefault;
				sim808_udp(buffer,address,port);
				free(buffer);
			}
			free(str);
		}
	}
	
}



static void updateStringSetting(char* value, char** pCurrent)
{
	if (*pCurrent)
	{
		free(*pCurrent);
	}
	if (strlen(value) == 0)
	{
		*pCurrent = NULL;
	}
	else
	{
		char* copy = malloc(strlen(value)+1);
		if (copy)
		{
			strcpy(copy,value);
			*pCurrent = copy;
		}
	}
}

void sim808_HandleResponse(void)
{
	char* receive = sim808_getReceiveString();
//	if (!receive)
//	{
//		if(tick_ms() - g_main_lastWifiTick < (UINT32_MAX>>1))
//		{
//			if (!esp8266_smarting())
//			{
//				esp8266_reset();
//				g_main_lastWifiTick = tick_ms() + RESET_WHILE_NO_DATEGET;
//			}
//		}
//	}
//	else
//	{
//		g_main_lastWifiTick = tick_ms() + RESET_WHILE_NO_DATEGET;
//	}
	if (receive)
	{
		cJSON* json = cJSON_Parse(receive);
		if(json)
		{
//			g_main_lastResponseTick = tick_ms();
			cJSON* child;
			child = cJSON_GetObjectItem(json,"udpAddress");
			if (child)
			{
				updateStringSetting(child->valuestring, &g_main_udpAddress);
			}
			child = cJSON_GetObjectItem(json,"udpPort");
			if (child)
			{
				g_main_udpPort = child->valueint;
			}
			
			child = cJSON_GetObjectItem(json,"setInterval");
			uint32_t interval = 0;
			if (child)
			{
				interval = child->valueint;//1 to 10s
			}	
			if (interval>1000)
			{
//				interval = interval*SERVER_INTERVAL_MULTIPLE; //server unit (s) to  local unit (ms)
				if (interval != g_main_uploadDataInterval)// || g_main_wifi_status != WIFI_CONNECT)
				{
					g_main_uploadDataInterval = interval;
					fpe_write(ADDRESS_UPLOAD_INTERVAL_VALUE, ( interval ) );
					fpe_write(ADDRESS_UPLOAD_INTERVAL_ENSURE, FLAG_FLASH_DATA_ENSURE);
				
				//	g_main_wifi_getDelay = 5000;
				//	g_main_wifi_triggerTick = tick_ms()+g_main_uploadDataInterval+g_main_wifi_getDelay;
				}
			}
			cJSON_Delete(json);
		}
		free(receive);
										
		g_main_startSleep = true;	//set system to sleep mode
	}
}

void sim808_UpdateGPS(void)
{
	
//	if(g_main_gps_updated==true || g_main_gps_got == true || tick_ms()>MAX_GPS_MS)
//	{
//		return;
//	}

uint32_t tick = tick_ms();
	uint32_t compare = MAX_GPS_MS;
	if(tick>MAX_GPS_MS)
	{
		return;
	}
	
	sim808_Info_t sim808_Info = sim808_GetInfo();
	
	if(sim808_Info.gpsUpdated) //no need to update,so we just return
	{
		g_main_gps_updated = true;
		return;
	}

	uint32_t gpsMaxIntervalCnt = GPS_INTERVAL_MS/g_main_uploadDataInterval;	//updated by main work interval, 1 time cnt++;

	uint32_t gpsNotUpdateCnt = g_flashStored_Info.gpsNotUpdateCnt;
	uint32_t gpsDataEnsure = g_flashStored_Info.gpsDataEnsure;
	
	if(gpsDataEnsure == FLAG_FLASH_DATA_ENSURE)
	{
//		sim808_Info_t sim808_Info;
		
		sim808_Info.gpsTime = g_flashStored_Info.gpsTime;
		sim808_Info.latitude = g_flashStored_Info.latitude;
		sim808_Info.longitude = g_flashStored_Info.longitude;
		if(gpsNotUpdateCnt < gpsMaxIntervalCnt) //data not need update
		{
			sim808_Info.gpsUpdated = true;
		}
		else	//data need update,not update time over GPS_INTERVAL_MS,but before data update,we still use old data
		{
			sim808_Info.gpsUpdated = false;
		}
		sim808_SetGpsInfo(sim808_Info);
		g_main_gps_updated = true; //use  data in flash,although maybe outdate,we may can't get new gps data
	}
	
//	sim808_Info = sim808_GetInfo();
	
	if(!sim808_Info.gpsUpdated)
	{
		sim808_UpdateGPSStatus();
		
		sim808_Info = sim808_GetInfo();
		
		if(sim808_Info.gpsUpdated)
		{
					/*handle, save gps value if it had been got, value is mutiplied by 1000000 to convert to uint32_t. */
			fpe_write(ADDRESS_VALUE_GPS_LATITUDE, sim808_Info.latitude*1000000);
			fpe_write(ADDRESS_VALUE_GPS_LONGITUDE, sim808_Info.longitude*1000000);
	//		fpe_write(0x011c, g_main_time);
			__IO uint32_t gpsTimePart1 = (sim808_Info.gpsTime)/1000000;
			fpe_write(ADDRESS_VALUE_GPS_GPSTIME1, gpsTimePart1);
			__IO uint32_t gpsTimePart2 = sim808_Info.gpsTime - 1000000*gpsTimePart1;
			fpe_write(ADDRESS_VALUE_GPS_GPSTIME2, gpsTimePart2 );
			
			fpe_write(ADDRESS_GPS_UPDATED_ENSURE, FLAG_FLASH_DATA_ENSURE); //ensure flash data is ok
			
			//
			gpsNotUpdateCnt = 0;
			fpe_write(ADDRESS_CNT_GPS_NOUPDATE_TIME, gpsNotUpdateCnt);
			
			loadFlashData();
			/*end save.*/
		}
	}
	
//	if(sim808_Info.gpsUpdated)
//	{
//		g_main_gps_updated = true;
//	}
	
}



uint8_t g_main_factoryMode = 1;
console_t* g_mainConsole = NULL;
void reset(consoleCommand_t* cc)
{
	//console_deleteCommand(cc);
	NVIC_SystemReset();
}

void consoleStuff(void)
{
//	if (tick_ms()<8000 || g_main_factoryMode)
	if (tick_ms()<8000)
	{
		console_tick();
		console_run();
		if(console_clearAllCommand(g_mainConsole))
		{
			//TransmitString("Command error\r\n");
		}
	
	}
	else
	{
		HAL_UART_DMAStop(&huart2);
		HAL_UART_Receive_DMA(&huart2,g_main_dustBuffer,sizeof(g_main_dustBuffer));
		enable_pm2d5();
		g_main_factoryMode = 0;
	}

}

static void TransmitChar(char ch)
{
	USART2->TDR = (ch & (uint8_t)0xFF);  
	while((USART2->ISR&UART_FLAG_TC)==0)
		;
}
void TransmitString(char * input)
{
	while(1)
	{
		char ch = *(input++);
		if (ch == '\0')
		{
			break;
		}
		TransmitChar(ch);
	}
}
void TransmitBytes(uint8_t *buffer, uint16_t size)
{
	for( uint16_t i = 0; i<size; i++)
	{
		uint8_t ch = *(buffer+i);
		TransmitChar(ch);
	}
}
void TransmitDec(uint32_t input)
{
	uint32_t quotient = 0;
	uint32_t divisor = 1000000000UL;
	uint8_t RightJustfy = 1;
	uint8_t showFirstZeros = 0;
	while(divisor>0)
	{
		quotient = (input)/divisor;
		if (quotient>0 || divisor == 1 || RightJustfy) 
		{
			char ch = quotient + '0';
			if ((!showFirstZeros && quotient == 0) && divisor != 1)
			{
				ch = 0x20;
			}
			else
			{
				showFirstZeros = 1;
			}
			TransmitChar(ch);
			input -= quotient*divisor;
			RightJustfy = 1;
		}
		divisor/=10;
	}
}
void TransmitHex(uint32_t input)
{
	for (uint16_t i = 0; i<8; i++)
	{
		uint8_t index;
		index = (input >> ((7-i)<<2))&0x0f;
		uint8_t ch;
		if(index >9)
		{
			ch = index - 10 + 'A';
		}
		else
		{
			ch = index + '0';
		}
		TransmitChar(ch);
	}
}


void factoryMode(consoleCommand_t* cc)
{
	stringList_t* sl = cc->first;
	if (sl &&sl->next)
	{
		uint32_t value = atol(sl->next->string);
		g_main_factoryMode = value;
		
	}
	TransmitString("factoryMode: ");
	TransmitHex(g_main_factoryMode);
	//TransmitHex(g_correctThreshold);
	TransmitString("\n");
}

void factorySetUid(consoleCommand_t* cc)
{
	stringList_t* sl = cc->first;
	if (sl &&sl->next)
	{
		
		if (sl->next->string[0] == '?')
		{
			for (uint8_t i=0 ; i<8 ; i++)
			{
				g_main_uid[i]=fpe_readOr(FPE_FACTORY_UID+i,0);
			}
			uint32_t id[2] = {0};
			for(uint8_t i=0; i<2; i++)
			{
				id[i] = (g_main_uid[i*4]<<24)+(g_main_uid[i*4+1]<<16)+(g_main_uid[i*4+2]<<8)+(g_main_uid[i*4+3]<<0);
			}
			TransmitHex(id[0]);
			TransmitHex(id[1]);
//			TransmitBytes(g_main_uid,8);
			TransmitString("\n");
		}
		else if (strlen(sl->next->string) == 16)
		{

			uint8_t idBuffer[16];
			MD5_CTX md5;
			MD5Init(&md5);         		
			MD5Update(&md5,(unsigned char *)(sl->next->string),strlen(sl->next->string));
//			MD5Update(&md5,(unsigned char *)(idString),strlen(idString));
			MD5Final(&md5,idBuffer);  
//			strcpy(g_main_uid_suffix,sl->next->string);
			for (uint8_t i=4 ; i<12 ; i++)
			{
				fpe_write(FPE_FACTORY_UID+i-4,idBuffer[i]);
				
			}
			for (uint8_t i =0;i<4;i++)
			{
				fpe_write(FPE_FACTORY_UID_STRING+i,*((sl->next->string)+i+12));
				g_main_uid_suffix[i] = *((sl->next->string)+i+12);
			}
			for (uint8_t i=0 ; i<8 ; i++)
			{
				g_main_uid[i] = idBuffer[i+4];
			}
			
			TransmitString("uidReceived:");
			TransmitString(sl->next->string);
			TransmitString("\n");
		//	fpe_write(FPE_FACTORY_UID+1,g_main_uid);
		}

		//TransmitHex(g_correctThreshold);

	}
	//console_deleteCommand(cc);
}

void factorySetAPN(consoleCommand_t* cc)
{
	stringList_t* sl = cc->first;
	uint8_t lenAPN = strlen(sl->next->string);
	uint8_t lenUser = strlen(sl->next->next->string);
	uint8_t lenPass = strlen(sl->next->next->next->string);
	if(lenAPN==1 && sl->next->string[0]=='0')
	{
		fpe_write(FPE_CUSTOM_APN,0);
	}
	if(lenUser==1 && sl->next->next->string[0]=='0')
	{
		fpe_write(FPE_CUSTOM_USER,0);
	}
	if(lenPass==1 && sl->next->next->next->string[0]=='0')
	{
		fpe_write(FPE_CUSTOM_PASS,0);
	}
	if (sl &&sl->next && lenAPN<NUMBER_APN_INFO_LENGTH_MAX && lenAPN>1)
	{
		fpe_writeString(FPE_CUSTOM_APN,sl->next->string);
		for(uint8_t i=0; i<lenAPN; i++)
		{
			g_main_apn[i] = sl->next->string[i];
		}
	}
	if(sl &&sl->next->next && lenUser<NUMBER_APN_INFO_LENGTH_MAX  && lenUser>1)
	{
		fpe_writeString(FPE_CUSTOM_USER ,sl->next->next->string);
		for(uint8_t i=0; i<lenUser; i++)
		{
			g_main_user[i] = sl->next->next->string[i];
		}
	}
	if(sl &&sl->next->next->next && lenPass<NUMBER_APN_INFO_LENGTH_MAX  && lenPass>1)
	{
		fpe_writeString(FPE_CUSTOM_PASS,sl->next->next->next->string);
		for(uint8_t i=0; i<lenPass; i++)
		{
			g_main_pass[i] = sl->next->next->next->string[i];
		}
	}
}

void LoadId(void)
{
	for (uint8_t i=0 ; i<8 ; i++)
	{
		g_main_uid[i]=fpe_readOr(FPE_FACTORY_UID+i,0);
//		g_main_uid[i]=fpe_read(FPE_FACTORY_UID+i);
	}
	
	for (uint8_t i=0 ; i<4 ; i++)
	{
		g_main_uid_suffix[i]=fpe_readOr(FPE_FACTORY_UID_STRING+i,'\0');
//		g_main_uid_suffix[i]=fpe_read(FPE_FACTORY_UID_STRING+i);
	}
//	g_main_uid_suffix[4] = '\0';
}
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init(); 

  /* Configure the system clock */
  SystemClock_Config();
	
	if (__HAL_PWR_GET_FLAG(PWR_FLAG_SB) != RESET)    //检查AD标志
  {
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);    //清除标志位
  }

     /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC_Init();
  MX_I2C2_Init();
  MX_RTC_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
	
	ME2139_init();  

	tick_init(&htim1);
	thread_init(0,tick_ms());  //开启毫秒级的线程？
//	g_main_console = console_init(&huart1,g_main_consoleBuffer,sizeof(g_main_consoleBuffer));
	fpe_init(62,63); //初始化FLASH
	thread_quickAdd(0,delayCall_run,1,0,0);  //线程加入 延时  1毫秒
	tick_add(delayCall_tick,10);  
	tick_start();
	
	
	LoadId();

	delayCall_call(feedExdog,1000,150,1);////////////*******************************annotate this in 2015-12-15.*************************///////////////
	delayCall_call(optData2Usart2,1000,350,1);////////////*******************************move this in 2015-12-15.*************************///////////////
	//看门狗程序   1秒喂狗


//检测电池电压.
	battery_charge_start_test(20,10); //2015-12-7. //2016-11-24 chagne(20,100)->(20,10)
//	printf("%f",g_main_start_battery_voltage);

//***********************2015-12-10 charge code start*****************************//
	if(g_main_start_battery_voltage < RANGE_START_BATTERY_LOW)
	{
		fpe_write(ADDRESS_BATTERY_RANGE, RANGE_START_BATTERY_LOW);
	}
	else if(g_main_start_battery_voltage > RANGE_START_BATTERY_NORMAL)
	{
		fpe_write(ADDRESS_BATTERY_RANGE, RANGE_START_BATTERY_NORMAL);
	}
	
	loadFlashData(); //20160809
	if(g_flashStored_Info.uploadIntervalEnsure == FLAG_FLASH_DATA_ENSURE)
	{
		g_main_uploadDataInterval = g_flashStored_Info.uploadInterval;
	}
	if( g_flashStored_Info.batteryFlag == RANGE_START_BATTERY_LOW )
	{
		do_sleep();
	}

	pm2d5_receive_int();
//	enable_pm2d5();

	thread_quickAdd(0,pm25_update,1000,0,0);////////////*******************************move this in 2015-12-15.*************************///////////////
//	HAL_Delay(1000);
	hts221_enable();//I2C enable.	
//	while(1);
	HAL_Delay(100);//wait to set PB12 port high to enable I2C, this delay is integrant !!!.等待PB12 拉高使能I2C
	hts221_init();//hts221.
	HAL_Delay(100);
	thread_quickAdd(0,th_update,1000,0,0);////////////*******************************move this in 2015-12-15.*************************///////////////
	
//	newUploadInterval_init();
	fpe_readString(FPE_CUSTOM_APN, g_main_apn, NUMBER_APN_INFO_LENGTH_MAX);
	fpe_readString(FPE_CUSTOM_USER, g_main_user, NUMBER_APN_INFO_LENGTH_MAX);
	fpe_readString(FPE_CUSTOM_PASS, g_main_pass, NUMBER_APN_INFO_LENGTH_MAX);
	
	sim808_init(&huart1);//wait to init successfully.   
	//setUid

	
	g_mainConsole = console_init( &huart2,g_main_cmdBuffer,sizeof(g_main_cmdBuffer));
	console_addListener(g_mainConsole,"reset",reset,0);
	console_addListener(g_mainConsole,"factoryMode",factoryMode,0);
	console_addListener(g_mainConsole,"uid",factorySetUid,0);
	console_addListener(g_mainConsole,"apn",factorySetAPN,0);
	console_printf(g_mainConsole,"systemStart\r\n");
	console_clearAllCommand(g_mainConsole);

	thread_quickAdd(0,sim808_HandleResponse,100,0,0);
	thread_quickAdd(0,sim808_UpdateGPS,1000,0,0);			//更新 gps 
	thread_quickAdd(0,sim808_UpdateStatus,1000,0,0);		//根据电池电压 检查是否 进入睡眠状态. 电压低于 3.4 V 进入睡眠状态.//是否上传成功，若成功 ，则进入休眠.
	thread_quickAdd(0,mainThread,5000,0,0);
	thread_quickAdd(0,sim808_run,10,0,0);
	
	
	thread_quickAdd(0,consoleStuff,10,0,0);
	
	thread_quickAdd(0,feedExdog,500,0,0);////////////*******************************annotate this in 2015-12-15.*************************///////////////
//看门狗	
//	thread_quickAdd(0,pm25_update,1000,0,0);
	thread_quickAdd(0,check_sleep,1000,0,0);		//根据电池电压 检查是否 进入睡眠状态. 电压低于 3.4 V 进入睡眠状态.//是否上传成功，若成功 ，则进入休眠.
//	thread_quickAdd(0,update_gps,1000,0,0);			//更新 gps 
//	thread_quickAdd(0,check_gprs,1000,0,0);			//检查gprs状态，是否开始上传数据.
	thread_quickAdd(0,battery_update,1000,0,0);		//更新 电池电压数值.
	//thread_quickAdd(0,check_action_if_done,10,0,0);		//检查数据是否上传成功.
//	thread_quickAdd(0,th_update,1000,0,0);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */		
		thread_run(0,tick_ms(),0,0);
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14
                              |RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_RTC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

}

/* USER CODE BEGIN 4 */



char *test_js;//just to test.

void battery_charge_start_test(int times, uint32_t delay)
{
	for(int i=0; i<times; i++)
	{
		battery_update();
		g_main_start_battery_voltage += g_main_battery_voltage;
		HAL_Delay(delay);
	}
	g_main_start_battery_voltage = g_main_start_battery_voltage/times ;
}


void optData2Usart2(void)
{
	
	if (g_main_factoryMode== 0)
	{
		sim808_Info_t sim808_Info = sim808_GetInfo();
		
	//		char str[255]={0};
	//		if(sim808_Info.idUpdated )
	//		{
	//			sprintf(str+strlen(str), "id:");
	//			sprintf(str+strlen(str), "%s", sim808_Info.IMEI_number);
	//			sprintf(str+strlen(str), ",");
	//		}
	//		sprintf(str+strlen(str), "pm25:");
	//		sprintf(str+strlen(str), "%ld", g_main_pm25);
	//		sprintf(str+strlen(str), ",");
	//		sprintf(str+strlen(str), "temp:");
	//		sprintf(str+strlen(str), "%f", g_main_temperature);
	//		sprintf(str+strlen(str), ",");
	//		sprintf(str+strlen(str), "wet:");
	//		sprintf(str+strlen(str), "%f", g_main_humidity);
	//		sprintf(str+strlen(str), ",");
	//		sprintf(str+strlen(str), "charge:");
	//		sprintf(str+strlen(str), "%f", g_main_battery_voltage);
	//		sprintf(str+strlen(str), ",");
	//		sprintf(str+strlen(str), "\r\n");
			
		cJSON* json = cJSON_CreateObject();
		if (json)
		{
			deviceId[0] = '\0';
			sprintf(deviceId,"L1");  //2015-12-7
			if(strlen(g_main_uid_suffix))
			{
				sprintf(deviceId,"L1-");  //2015-12-7
				for (uint8_t i=0; i<8; i++)
				{
					sprintf(deviceId+strlen(deviceId),"%02x", g_main_uid[i]);  //2015-12-7
				}
				sprintf(deviceId+strlen(deviceId),"%.4s", g_main_uid_suffix);  //2015-12-7
			}
			cJSON_AddStringToObject(json,"devId",deviceId);
			cJSON_AddNumberToObject(json,"lat",sim808_Info.latitude);
			cJSON_AddNumberToObject(json,"lon",sim808_Info.longitude);
			
//			if(sim808_Info.idUpdated )
//			{
//				cJSON_AddStringToObject(json,"devId",sim808_Info.IMEI_number);
//				cJSON_AddNumberToObject(json,"lat",sim808_Info.latitude);
//				cJSON_AddNumberToObject(json,"lon",sim808_Info.longitude);
//			}
			cJSON_AddNumberToObject(json,"pm2d5", g_main_pm25/100.0); 

			cJSON_AddNumberToObject(json,"co2",1);
			cJSON_AddNumberToObject(json,"ch2o",1);
			
			cJSON_AddNumberToObject(json,"temp",g_main_temperature);
			cJSON_AddNumberToObject(json,"hum",g_main_humidity);
			
			
	//		cJSON_AddNumberToObject(json,"systemTick",tick_ms());
			
			
			char * str = cJSON_PrintUnformatted(json);
			cJSON_Delete(json);
			
			if(str)
			{
				uint16_t length = strlen(str);
	//			char* buffer = malloc(length+3);
	//			if (buffer)
	//			{
	//				sprintf(buffer,"%s\r\n",str);
	//				HAL_UART_Transmit(&huart2, (uint8_t *)buffer, length, 1000);
	//				free(buffer);
	//			}
				HAL_UART_Transmit(&huart2, (uint8_t *)str, length, 1000);
				HAL_UART_Transmit(&huart2, (uint8_t *)"\r\n", 2, 1000);
				free(str);
			}
		}
	}
		//cJSON_Delete(js);
	//}
}


static inline uint32_t pm25_getValue(void)
{

	if (g_main_factoryMode== 0)
	{
		
		uint8_t data[sizeof(g_main_dustBuffer)];                           //uint8_t
		int temp=0;
		uint8_t sum=0;                                                     //same type with data[].
		uint32_t value=0;
		for (int i=0;i<sizeof(g_main_dustBuffer);i++)
		{	
			data[i]=g_main_dustBuffer[i];		
			
		}
		for (int i=0;i<sizeof(g_main_dustBuffer);i++)
		{
			if (data[i] == 0xaa)
			{
				if (data[(i+6)&0x07]==0xff)
				{
						for (int j=0;j<4;j++)
					{
						sum += data[(i+j+1)&0x07];
						value += data[(i+j+1)&0x07]<<(j<<3);                              //(8*j)==(j<<3)
					}
					
					if (sum==data[(i+5)&0x07])
					{	
						return value;
					}
				}
			
			}
		}
	
	}
	return 0;
	
//	uint8_t data[sizeof(g_main_dustBuffer)];                           //uint8_t
//	int temp=0;
//	uint8_t sum=0;                                                     //same type with data[].
//	uint32_t value=0;
//	for (int i=0;i<sizeof(g_main_dustBuffer);i++)
//	{	
//		data[i]=g_main_dustBuffer[i];		
//	}
//	
//	if(data[0]!=0xaa)
//	{
//		for(int i=0;i<sizeof(g_main_dustBuffer);i++)
//		{
//			if(data[i]==0xaa)
//				temp=i;
//		}
//		for(int i=0;i<sizeof(g_main_dustBuffer);i++)
//		{
//			if (temp+i<sizeof(g_main_dustBuffer))
//			{
//				data[i]=g_main_dustBuffer[temp+i];
//			}
//			else
//			{
//				data[i]=g_main_dustBuffer[temp+i-sizeof(g_main_dustBuffer)];
//			}
//		}
//	}

//	if (data[0]==0xaa&&data[6]==0xff)                                        //desert bad datas.
//	{
//		for (int j=0;j<4;j++)
//		{
//			sum += data[j+1];
//			value += data[j+1]<<(j<<3);                              //(8*j)==(j<<3)
//		}
//		
//		if (sum==data[5])
//		{	
//			return value;
//		}
//	}
//	return 0;
}


void pm25_update(void)
{
	
//				g_main_pm25_stable = true;
//	return;
	if(g_main_factoryMode)
		{
			return;
		}
	if(g_main_pm25_stable==true || tick_ms()>MAX_PM25_MS)
	{
		return;
	}
	
	if (!g_main_pm25_stable)
	{
		int32_t value = pm25_getValue();

#ifdef STRICT_PM25_STABLE_VALUE
		if (value !=0)
		{
			g_main_pm25 = g_main_pm25 + (value-g_main_pm25) * PM25_STABLE_ACCURRACY;
//			if (g_main_pm25>=value && (g_main_pm25-value) < g_main_pm25 * 0.05)
			if (g_main_pm25>=value && (g_main_pm25-value) < g_main_pm25 * 0.1)
			{
				g_main_pm25_stable = true;
				
				disable_pm2d5();
				hts221_disable(); //////////////2016-1-12.
				
			}
		}
#endif
#ifdef WITHOUT_PM25_VALUE
		{
			{
				g_main_pm25_stable = true;
				disable_pm2d5();
				
				hts221_disable(); //////////////2016-1-12.
				
			}
		}
#endif
		
		printf("pm2.5 = %f  and  value = %f.  ",g_main_pm25/100.0,value/100.0);
		printf("start_charge: %f.\t",g_main_start_battery_voltage);  //2015-12-10.
		printf("temp = %f  and  hum = %f. \t",g_main_temperature, g_main_humidity);   // Do use cautiously with printf("\r\n"), or you will get 'error'. 
		printf("%d,%ld\t",g_flashStored_Info.gpsNotUpdateCnt, g_main_uploadDataInterval);   // Do use cautiously with printf("\r\n"), or you will get 'error'. 
		HAL_Delay(10); //2015-12-2.
		
	}//end if(!g_main_pm25_stable)
	
}//end function_pm25_update.

void println(char* string)  //no use.
{
//	int length = sizeof(string);
	uint8_t transChar[256] = {0};
	sprintf((char*)transChar, "%s\r\n", string);
	HAL_UART_Transmit(&huart1,transChar,strlen((char*)transChar),1000);
}


static inline void ME2139_init(void)
{
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,GPIO_PIN_SET);
}

void do_sleep(void)
{	
//	if(g_main_battery_voltage >= 3300)  //3.3V 以上时stop模式，正常喂狗.
//	{
//		uint32_t resetTimes = fpe_read(0x005c);
//		uint32_t everyUpdateGpsTimes = fpe_read(0x004c);
//		if (g_main_gps_updated)
//		{
//			resetTimes = 0;
//		}
//		else
//		{
//			if (resetTimes!= UINT32_MAX)
//			{
//				resetTimes++;
//			}
//		}
//		
//		everyUpdateGpsTimes = everyUpdateGpsTimes + 1;
//		if(everyUpdateGpsTimes == GPS_INTERVAL_MS/g_main_uploadTime)  //every one day(24h), change everyUpdateGpsTimes from some values to zero(0).
//		{
//			everyUpdateGpsTimes = 0;
//		}
//		
//		fpe_write(0x005c,resetTimes);
//		fpe_write(0x004c,everyUpdateGpsTimes);
//		
		g_flashStored_Info.gpsNotUpdateCnt++;
		fpe_write(ADDRESS_CNT_GPS_NOUPDATE_TIME, g_flashStored_Info.gpsNotUpdateCnt); // gps notupdate cnt updated by main work interval, 1 time cnt++;
		
	//	while(1);
		disable_pm2d5();
		sim808_deinit();
	//	while(1);
		
		HAL_Delay(100);                                       //2015-12-7
//		g_main_end_battery_voltage = g_main_battery_voltage;  //2015-12-7
//		fpe_write(0x008c,g_main_end_battery_voltage);         //2015-12-7
		int resetCount = 0;
		if(g_main_uploadDataInterval>tick_ms())
		{
			resetCount = (g_main_uploadDataInterval - /*(int)*/tick_ms())/1000;        // 上传时间+工作时间=上传间隔
		}
		while (resetCount != 0)
		{
			startAlarm(1);
			HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
			//SystemClock_Config();
			//printf("g_main_resetCount = %d\r\n",g_main_resetCount);
			feedExdog();
			resetCount--;
		}
		HAL_NVIC_SystemReset();		
//	}
//	else if(g_main_battery_voltage < 3300) //3.3V 以下时进入 standby 模式，最低功耗，放置电池低电压保护.
//	{
//		startAlarm(0x20);
//		HAL_PWR_EnterSTANDBYMode();  //最低功耗.	
//		HAL_NVIC_SystemReset();
//	}
}


void check_sleep(void)
{
	if (g_main_startSleep  || tick_ms()>MAX_WAKE_MS || g_main_battery_voltage<RANGE_RUNTIME_BATTERY_LOW)//tickms超过该时间时会自动进入睡眠模式, 即使未完成整个流程.
	{
		if(g_main_battery_voltage < RANGE_RUNTIME_BATTERY_LOW)
		{
			fpe_write(ADDRESS_BATTERY_RANGE,RANGE_START_BATTERY_LOW);  //refer to 'charge code', locating beginning of 'main()'function, about line 169.
		}
		do_sleep();
	}
}


static inline void pm2d5_receive_int(void)
{
//	HAL_UART_Receive_DMA(&huart2,g_main_dustBuffer,sizeof(g_main_dustBuffer));
}

void enable_pm2d5(void)
{
	
	//there are 2 steps to enablepm2d5. Step1 is for saving electricity: when pm2d5 is not used,we also shut down U301.
	
	// step1: enable U301(ME2139-5.0) CE port     PC15 port as High.
		//HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,GPIO_PIN_SET);    //该管脚在单片机上电时就应开启.
	
	// step2: enable Q801(AO3401 P-MOS)         PA1 port as High.
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_SET);
	

}

void disable_pm2d5(void)
{
	//ALL port will be closed to save electricity.
	
	// step1: disable Q801(AO3401 P-MOS)         PA1 port as Low.
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET);
	
	// step2: disable U301(ME2139-5.0) CE port     PC15 port as Low.
		//HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,GPIO_PIN_RESET); //该管脚还负责单片机电压供给，不能关.
}

void startAlarm(uint16_t second)
{
	//__PWR_CLK_ENABLE();
	HAL_PWR_EnableBkUpAccess();
	
  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef sDate;
  RTC_AlarmTypeDef sAlarm;
	
  sTime.Hours = 0;
  sTime.Minutes = 0;
  sTime.Seconds = 0;
  sTime.SubSeconds = 0;
  sTime.TimeFormat = RTC_HOURFORMAT12_AM;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  HAL_RTC_SetTime(&hrtc, &sTime, FORMAT_BIN);

  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0;
  sDate.Year = 0;
  HAL_RTC_SetDate(&hrtc, &sDate, FORMAT_BIN);

    /**Enable the Alarm A 
    */
  sAlarm.AlarmTime.Hours = 0;
  sAlarm.AlarmTime.Minutes = 0;
  sAlarm.AlarmTime.Seconds = second;
  sAlarm.AlarmTime.SubSeconds = 0;
  sAlarm.AlarmTime.TimeFormat = RTC_HOURFORMAT12_AM;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_WEEKDAY;
  sAlarm.AlarmDateWeekDay = RTC_WEEKDAY_MONDAY;
  sAlarm.Alarm = RTC_ALARM_A;
  HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, FORMAT_BIN);
}

void feedExdog(void)
{
	HAL_GPIO_TogglePin(GPIOF,GPIO_PIN_1);
}

#define ADCDATASIZE 8

__IO bool g_main_adc_cplt = false;
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	g_main_adc_cplt = true;
}

void battery_update(void)
{
//	HAL_Delay(100);
	uint16_t adcData[ADCDATASIZE]={0};
	HAL_ADCEx_Calibration_Start(&hadc);
	g_main_adc_cplt = false;
	HAL_ADC_Start_DMA(&hadc,(uint32_t*)adcData,ADCDATASIZE);
	uint32_t stopTick = tick_ms() + 100;
	while (!g_main_adc_cplt)
	{
		if (tick_ms()-stopTick < (UINT32_MAX>>1))
		{
			break;
		}
	}
//	HAL_Delay(100);

//there 'i<1' meams we just collect once everytime 'ADCread()' is	called,------ 
//-------for minimal time to let thread 'thread_quickAdd(ADCread)' run regulaly.
	uint32_t sum=0;
	for (int i = 0; i<ADCDATASIZE; i++)
	{
		sum += adcData[i];
//			usart_printf(&huart1,"voltage is %d\r\n",aResultDMA);
	}

	g_main_battery_voltage = (float)(sum*6600.0)/4096.0/ADCDATASIZE;
//voltage'unit is 'mV',which reasonable range is '3000mV' to '4200mV'.

//this printf just wants to test wether thread 'thread_quickAdd(ADCread)' run regulaly or not.
//	printf("voltage of battery is %5.2f mV.\r\n",g_main_battery_voltage);
}


typedef struct
{
	uint8_t H0_rH;
	uint8_t H1_rH;
	int16_t H0_T0_out;
	int16_t H1_T0_out;

	uint16_t	T0_degC;
	uint16_t	T1_degC;
	int16_t T0_out;
	int16_t T1_out;

	int16_t T_out;
	int16_t H_out;
}hts221_t;

hts221_t hts221_var;
//#define TH_I2C_ADDRESS 0xe0
#define ST_I2C_ADDRESS 0xBE

static void hts221_enable(void)
{
//	while(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_12) == RESET)
//	{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
//	}
}

static void hts221_disable(void)
{
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);
}

void hts221_init(void)
{
	uint8_t setupBytes[] = {0x81};
	uint8_t sendBytes[] = {0xB0};
	uint8_t retries = 3;
	while(retries>0)
	{
		if	(HAL_I2C_Mem_Write(&hi2c2,ST_I2C_ADDRESS,0x20,I2C_MEMADD_SIZE_8BIT, setupBytes,sizeof(setupBytes),15)==HAL_OK)
		{
		//	HAL_Delay(10);
			if (HAL_I2C_Master_Transmit(&hi2c2,ST_I2C_ADDRESS,sendBytes,sizeof(sendBytes),30) == HAL_OK)
			{

				uint8_t receiveBytes[16]= {0};
				memset(receiveBytes,0xff,16);
			//	HAL_Delay(10);
				if (HAL_I2C_Master_Receive(&hi2c2,ST_I2C_ADDRESS,receiveBytes,sizeof(receiveBytes),15) == HAL_OK)
				{
					hts221_var.T0_degC = (receiveBytes[2] | ((uint16_t)receiveBytes[5] & 0x03 )<<8)>>3;
					hts221_var.T1_degC = (receiveBytes[3] | ((uint16_t)receiveBytes[5] & 0x0C )<<6)>>3;
					hts221_var.T0_out = receiveBytes[12] | ((uint16_t)receiveBytes[13]<<8 );
					hts221_var.T1_out = receiveBytes[14] | ((uint16_t)receiveBytes[15]<<8 );

					hts221_var.H0_rH = receiveBytes[0]>>1;
					hts221_var.H1_rH = receiveBytes[1]>>1;
					hts221_var.H0_T0_out = receiveBytes[6] | ((uint16_t)receiveBytes[7]<<8 );
					hts221_var.H1_T0_out = receiveBytes[10] | ((uint16_t)receiveBytes[11]<<8 );
					break;
				}
			}
		}
		retries--;
	}
}


void th_check_end(void)
{
	uint8_t receiveBytes[5]= {0};
	memset(receiveBytes,0xff,5);
	if (HAL_I2C_Master_Receive(&hi2c2,ST_I2C_ADDRESS,receiveBytes,sizeof(receiveBytes),15) == HAL_OK)
	{
		if (receiveBytes[0]==0x03)//check data ready register   receiveBytes[0]==0x03
		{
			hts221_var.T_out = ((int16_t)receiveBytes[4] << 8) + ((int16_t)receiveBytes[3] ); //actual temperature before calibration
			__IO double slope = ( hts221_var.T0_degC*1.0 - hts221_var.T1_degC)/(hts221_var.T0_out - hts221_var.T1_out); //slope of temp calibration line
			__IO double intercept = hts221_var.T0_degC - slope * hts221_var.T0_out ; 							//intercept of temp calibration line
			g_main_temperature = slope * hts221_var.T_out + intercept; 														//actual temperature
			hts221_var.H_out = ((int16_t)receiveBytes[2] << 8) + ((int16_t)receiveBytes[1] );//actual humidity before calibration
			slope = ( hts221_var.H0_rH*1.0 - hts221_var.H1_rH)/(hts221_var.H0_T0_out - hts221_var.H1_T0_out);//slope of humi calibration line
			intercept = hts221_var.H0_rH - slope * hts221_var.H0_T0_out;										//intercept of humi calibration line
			g_main_humidity = fmin(99,fmax(0,slope * hts221_var.H_out + intercept)*1.45);   //*1.35
		
			if (g_main_temperature>15) 
			{
				g_main_temperature = 15+ (g_main_temperature-15)*0.7;
			}
			
//			g_main_temperature = 666; //test.
//			g_main_humidity = 666; //test.
		}
	}

}

void th_update(void)
{
	uint8_t sendBytes2[] = {0xA7};//ST
	if (HAL_I2C_Master_Transmit(&hi2c2,ST_I2C_ADDRESS,sendBytes2,sizeof(sendBytes2),30) == HAL_OK)
	{
		delayCall_call(th_check_end,30,1,0);
	}
}


/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
