
#include "string.h"
#include "stdio.h"
#include "stdarg.h"
#include "stdlib.h"
#include "SIM808.h"
#include "stm32f0xx_hal.h"

#include "md5.h"
#include "control.h"

//hardware start here...
UART_HandleTypeDef* volatile g_sim808_huart = NULL;
volatile uint32_t* volatile g_sim808_dmaRemain = NULL;

extern char g_main_apn[17];
extern char g_main_user[17];
extern char g_main_pass[17];


typedef enum
{
	SIM808_GPRS_NOTOPEN,
	SIM808_GPRS_CONFIG,
	SIM808_GPRS_CONNECTTING,
	SIM808_GPRS_CONNECTTED,	
}sim808_GPRSStatus_t;


typedef enum
{
	SIM808_GSM_SIM_NOTINSERT,
	SIM808_GSM_NOTREADY,
	SIM808_GSM_READY,	
	SIM808_GPRS_READY,	
}sim808_GSMStatus_t;

typedef struct
{
	sim808_GSMStatus_t GSMStatus;
	sim808_GPRSStatus_t GPRSStatus;
}sim808_localInfo_t;

static sim808_localInfo_t g_sim808_localInfo = {  
		.GPRSStatus = SIM808_GPRS_NOTOPEN, 
		.GSMStatus = SIM808_GSM_SIM_NOTINSERT
};


static sim808_Info_t g_sim808_Info = {
	.gsmOk = false,
	.gsmSigStrenth = 0,
	.gprsConnected = false,
	.gpsUpdated = false,
	.longitude = 0,
	.latitude = 0,
	.idUpdated = false,
//	.sim808Status = SIM808_IDLE,
};


//overwrite fputc function, or you will unable to use printf(...) function.
int fputc(int ch,FILE *f)
{
	if (g_sim808_huart != NULL)
	{
		uint8_t c=ch;
		//PLATFORM uart transmit a single char.
		HAL_UART_Transmit(g_sim808_huart,&c,1,1000);
	}
	return ch;
}

// this function return a (uint32_t) tick which should increase by 1 every 1ms.
static inline uint32_t getTick(void)
{
	//PLATFORM interrupt and volatile must be used.
	return HAL_GetTick();
}

// delay 1 ms
static inline void hw_delay(uint32_t ms)
{
	//PLATFORM
	HAL_Delay(ms);
}

// size depends on the RAM size. must be 1<<n. if you need more buffer for http response. declare a bigger one.
#define SIM808_BUFFER_SIZE (1<<8)
#define SIM808_BUFFER_MASK (SIM808_BUFFER_SIZE-1)

static void softwareInit(void);
//PLATFORM must be volatile, as it can be modified by DMA and IT.
volatile uint16_t g_sim808_bufferIndex = 0;
volatile uint8_t g_sim808_buffer[SIM808_BUFFER_SIZE] = {0};

//pull down reset pin of the WIFI chip, then pull up.
static void hw_reset(void)
{
	//HARDWARE set the gpio below as your REST pin of WIFI chip.(would be gpio16 instead, see the sch of WIFI moudle)
	HAL_GPIO_WritePin(GPIOF,GPIO_PIN_0,GPIO_PIN_RESET);
	hw_delay(10);
	HAL_GPIO_WritePin(GPIOF,GPIO_PIN_0,GPIO_PIN_SET);
}

static void hw_init(void)
{
	//////*over write  IGNORED *//////
	//Step 1: init the buffer to UINT16_MAX, so that we would know the buffer has been wright.
	//memset((void*)g_sim808_buffer,UINT16_MAX,sizeof(g_sim808_buffer));
	g_sim808_dmaRemain = &(g_sim808_huart->hdmarx->Instance->CNDTR);
	//Step 2: Start DMA. In circle mode, dma would NEVER STOPPED.
	//start an automatic thread to collect the byte comming from RX pin.

	HAL_UART_Receive_DMA(g_sim808_huart, (uint8_t *)g_sim808_buffer, SIM808_BUFFER_SIZE);
	
	//SIM_EN port PB15 enable.
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET);
	
	//S_PWRKEY port PB14 enalbe.
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET);
	//low voltage >1S to turn on or turn off SIM808.
	
	HAL_Delay(1000);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET);
}

static void hw_deinit(void)
{
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);
}


//PLATFORM   init with a uart port to communicate with WIFI chip. 
//the uart must has a dma handle, which from Rx to memory, P-increase disable, M-increase enable, circle mode, byte to byte.
//the dma interrupt must disable(which can be modified in uart.c), or the dma would be stop after the first circle.
void sim808_init(UART_HandleTypeDef* huart)
{
	g_sim808_huart = huart;
	hw_init();
	//softwareInit();
}

void sim808_deinit(void)
{
	hw_deinit();
}

static inline uint16_t getWriteIndex(void)
{
	return SIM808_BUFFER_SIZE - *(g_sim808_dmaRemain);
}

/******************************************************************************/
/***********************************WARNING************************************/
/******************************************************************************/
/********software start... beginner would NOT change any code below.***********/
/********if you need call some function, please read SIM808.h before**********/
/******************************************************************************/
/******************************************************************************/
/******************************************************************************/

__IO uint8_t g_sim808_status = 5;
uint32_t g_sim808_smartTick  = 0;
uint32_t g_sim808_pairTick = 0;
uint8_t g_sim808_pairTodo = 0;
uint8_t g_sim808_isAtMode = 0;
uint32_t g_sim808_resetTick = 0;
uint8_t g_sim808_smartStarted = 0;
char* g_sim808_receiveString = NULL;
char* g_sim808_ssidString = NULL;
char* g_sim808_passwordString = NULL;
void(*g_sim808_smartFirstReceived)(void) = NULL; 
void(*g_sim808_smartSsidAndPasswordReceived)(char*, char*) = NULL; 

void sim808_setSsidAndPasswordCallback(void(*fcn)(char*, char*))
{
	g_sim808_smartSsidAndPasswordReceived = fcn;
}

void sim808_setSmartCallback(void(*fcn)(void))
{
	g_sim808_smartFirstReceived = fcn;
}
cJSON * g_sim808_responseJson = NULL;

uint8_t jsonResponse(char* str)
{
	if(*str == '{')
	{
		cJSON* json = cJSON_Parse(str);
		if (json)
		{
			if (g_sim808_responseJson)
			{
				cJSON_Delete(g_sim808_responseJson);
			}
			g_sim808_responseJson = json;
			return 1;
		}
	}
	return 0;
}

uint8_t cipClosed(char* str)
{
	if(strcasecmp(str,"CLOSED")==0)
	{
		//reset();
		g_sim808_status = 2;
		return 1;
	}
	
	if(strcasecmp(str,"link is not valid")==0)
	{
		//reset();
		g_sim808_status = 2;
		return 1;
	}
	
	return 0;
}

uint8_t smartInfo(char* str)
{
	if (g_sim808_smartStarted)
	{
		if(strstr(str,"smartconfig connected wifi"))
		{
			g_sim808_smartFirstReceived();
			sim808_smartStop();
			if (g_sim808_ssidString && g_sim808_passwordString)
			{
				if (g_sim808_smartSsidAndPasswordReceived)
				{
					g_sim808_smartSsidAndPasswordReceived(g_sim808_ssidString,g_sim808_passwordString);
				}
			}
			return 1;
		}
		if(strstr(str,"smartconfig type:"))
		{
			if (g_sim808_smartFirstReceived)
			{
				g_sim808_smartFirstReceived();
			}
			if (g_sim808_smartTick - (getTick() + 60000) > (UINT32_MAX>>1) )
			{
				g_sim808_smartTick = getTick() + 60000;
			}
			return 1;
		}
		if(strstr(str,"ssid:"))
		{
			char* ssid = str+5;
			if (strlen(ssid)>0)
			{
				if (g_sim808_ssidString)
				{
					free(g_sim808_ssidString);
				}
					g_sim808_ssidString = (char*) malloc(strlen(ssid)+1);
				if (g_sim808_ssidString)
				{
					strcpy(g_sim808_ssidString,ssid);
				}
			}
		}
		if(strstr(str,"password"))
		{
			char* password = str+9;
			if (strlen(password)>0)
			{
				if (g_sim808_passwordString)
				{
					free(g_sim808_passwordString);
				}
					g_sim808_passwordString = (char*) malloc(strlen(password)+1);
				if (g_sim808_passwordString)
				{
					strcpy(g_sim808_passwordString,password);
				}
			}
			
		}
	}
	return 0;
}
uint8_t statusChanged(char* str)
{
	uint8_t status = 0;
	if(strcasecmp(str,"+CPIN: READY")==0)
	{
		if(g_sim808_localInfo.GSMStatus == SIM808_GSM_SIM_NOTINSERT)
		{
			g_sim808_localInfo.GSMStatus = SIM808_GSM_NOTREADY;
		}
	}
	if(strcasecmp(str,"+CREG: 1,1")==0 || strcasecmp(str,"+CREG: 1,5")==0 || strcasecmp(str,"+CREG: 0,5")==0 || strcasecmp(str,"+CREG: 0,1")==0)
	{
		if(g_sim808_localInfo.GSMStatus == SIM808_GSM_NOTREADY)
		{
			g_sim808_localInfo.GSMStatus = SIM808_GSM_READY;
		}
	}
	if(strcasecmp(str,"+CGATT: 1")==0)
	{
//		if(g_sim808_localInfo.GSMStatus == SIM808_GSM_READY)
//		{
			g_sim808_localInfo.GSMStatus = SIM808_GPRS_READY;
//		}
	}
	
	
//	if(strstr(str,"STATUS:") && strlen(str) == 8)
//	{
//		status = str[7] - '0';
//	}
	
	if(strcasecmp(str,"CONNECT OK")==0)
	{
		status = 3;
	}
	if(strcasecmp(str,"CLOSED")==0)
	{
		status = 2;
	}
	
	if(strcasecmp(str,"link is not valid")==0)
	{
		status = 2;
	}	
//	if(strcasecmp(str,"ALREADY CONNECT")==0)
//	{
//		status = 3;
//	}
	if(strstr(str,"+CIPSTATUS:0,\"UDP\","))
	{
//		status = 3;
	}
	if (status != 0)
	{
		g_sim808_status = status;
		return 1;
	}
	return 0;
}

uint8_t receiveData(char* str)
{
	if(strstr(str,"+IPD,"))
	{
		char* douhao = strchr(str,',');
		char* maohao = strchr(str,':');
		if (douhao && maohao)
		{
			*douhao = 0;
			*maohao = 0;
			uint8_t len = atoi(douhao+1);
			if (g_sim808_receiveString)
			{
				free(g_sim808_receiveString);
			}
			g_sim808_receiveString = malloc(len+1);
			strncpy(g_sim808_receiveString,maohao+1,len);
			//printf("RECV:%d,%s",len,maohao+1);
			return 1;
		}
	}
	return 0;
}
typedef uint8_t(*focus_t)(char*);
focus_t g_sim808_focus[] = {smartInfo,statusChanged,receiveData};
static uint16_t checkNewLine(void)
{
	for (uint16_t i = 0; i < SIM808_BUFFER_SIZE; i++) //#define SIZE_FOR_UART_RX 16
	{
		uint16_t index = (g_sim808_bufferIndex+i)&SIM808_BUFFER_MASK;
		if (index == getWriteIndex())
		{
			break;
		}
		uint16_t c = g_sim808_buffer[index];
		if (c == 0x0a || c == 0x00)
		{
			return i+1;
		}
	}
	return 0;
}

static uint16_t readBuffer(char * data, uint16_t maxlen)
{
	uint16_t current_len = 0;
	for (uint16_t i = 0; i < maxlen; i++) //#define SIZE_FOR_UART_RX 16
	{
		if (g_sim808_bufferIndex == getWriteIndex())
		{
			break;
		}
		uint16_t c = g_sim808_buffer[g_sim808_bufferIndex];
		g_sim808_bufferIndex = (g_sim808_bufferIndex + 1)&SIM808_BUFFER_MASK;
		*(data+current_len) = c;
		current_len++;
	}
	return current_len;
}

static char* readNewLine(void)
{
	uint16_t lineLength = checkNewLine();
	if (lineLength == 0)
	{
		return NULL;
	}
	char* buffer = malloc(lineLength);
	if (buffer)
	{
		readBuffer(buffer,lineLength);
		*(buffer+lineLength-1) = '\0';
		if (lineLength>1)
		{
			char c = *(buffer+lineLength-2);
			if (c == 0x0d || c == 0x0a)
			{
				*(buffer+lineLength-2) = '\0';
			}
		}
	}
	return buffer;
}

static inline uint16_t lastIndex(void)
{
	return (getWriteIndex()-1)&SIM808_BUFFER_MASK;
}
static char bufferLastChar(void)
{
	if (g_sim808_bufferIndex == getWriteIndex())
	{
		return '\0';
	}
	else
	{
		return g_sim808_buffer[lastIndex()];
	}
}



void readFocus(void)
{
	char* line = NULL;
	while(1)
	{
		line = readNewLine();
		if (line)
		{
			for (uint8_t i = 0; i< sizeof(g_sim808_focus)/sizeof(focus_t); i++)
			{
				if (g_sim808_focus[i](line))
				{
					break;
				}
			}
			free(line);
		}
		else
		{
			break;
		}
	}
}

uint32_t waitUntilBlankLine(uint32_t ms)
{
	uint32_t timeOutTick = getTick() + ms;
	uint32_t currentTick = getTick();
	while(timeOutTick - currentTick< (UINT32_MAX>>1))
	{
		char* line = readNewLine();
		if (line)
		{
			if (strlen(line) == 0)
			{
				free(line);
				return timeOutTick - currentTick;
			}
			for (uint8_t i = 0; i< sizeof(g_sim808_focus)/sizeof(focus_t); i++)
			{
				if (g_sim808_focus[i](line))
				{
					break;
				}
			}
			free(line);
		}
		currentTick = getTick();
	}
	return 0;
}


uint32_t waitUntilRxStopped(uint32_t ms, uint16_t interval)
{
	uint32_t timeOutTick = getTick() + ms;
	uint32_t lastRxTick = getTick();
	uint32_t currentTick = getTick();
	uint16_t i = lastIndex(),j;
	while(timeOutTick - currentTick< (UINT32_MAX>>1))
	{
		j = i;
		i = lastIndex();
		if(i!=j)
		{
			lastRxTick = getTick();
		}
		if (currentTick - (lastRxTick + interval) < (UINT32_MAX>>1))
		{
			return timeOutTick - currentTick;
		}
		currentTick = getTick();
	}
	return 0;
}

uint8_t waitUntilOk(uint32_t ms)
{
	uint32_t timeOutTick = getTick() + ms;
	uint32_t currentTick = getTick();
	while(timeOutTick - currentTick< (UINT32_MAX>>1))
	{
		char* line = readNewLine();
		if (line)
		{
			if (strcmp(line,"OK") == 0 || strcmp(line,"SEND OK") == 0)
			{
				free(line);
				return timeOutTick - currentTick;
			}
			if (strcmp(line,"ERROR") == 0 || strcmp(line,"busy s...") == 0)
			{
				free(line);
				return 0;
			}
			for (uint8_t i = 0; i< sizeof(g_sim808_focus)/sizeof(focus_t); i++)
			{
				if (g_sim808_focus[i](line))
				{
					break;
				}
			}
			free(line);
		}
		currentTick = getTick();
	}
	return 0;
}
uint8_t waitUntilStrcmp(uint32_t ms, char* str)
{
	uint32_t timeOutTick = getTick() + ms;
	uint32_t currentTick = getTick();
	while(timeOutTick - currentTick< (UINT32_MAX>>1))
	{
		char* line = readNewLine();
		if (line)
		{
			if (strcmp(line,str) == 0)
			{
				free(line);
				return timeOutTick - currentTick;
			}
			for (uint8_t i = 0; i< sizeof(g_sim808_focus)/sizeof(focus_t); i++)
			{
				if (g_sim808_focus[i](line))
				{
					break;
				}
			}
			free(line);
		}
		currentTick = getTick();
	}
	return 0;
}
	
uint8_t waitUntilStrstr(uint32_t ms, char* str)
{
	uint32_t timeOutTick = getTick() + ms;
	uint32_t currentTick = getTick();
	while(timeOutTick - currentTick< (UINT32_MAX>>1))
	{
		char* line = readNewLine();
		if (line)
		{
			if (strstr(line,str) != 0)
			{
				free(line);
				return timeOutTick - currentTick;
			}
			for (uint8_t i = 0; i< sizeof(g_sim808_focus)/sizeof(focus_t); i++)
			{
				if (g_sim808_focus[i](line))
				{
					break;
				}
			}
			free(line);
		}
		currentTick = getTick();
	}
	return 0;
}
static void vSetCommand(char * commond,va_list argptr)
{
	char* connect = "=";
	char * para;
	if (commond!=NULL)
	{
		printf("AT+%s",commond);
		while ((para = va_arg(argptr,char*))!=NULL)
		{
			printf("%s%s",connect,para);
			connect = ",";
		}
	}
	else
	{
		printf("AT");		
	}
	printf("\r\n");
}
static void setCommand(char* commond, ...)
{
	va_list argptr;
	va_start(argptr, commond);
	vSetCommand(commond,argptr);
	va_end(argptr);
}

static void setCommandWait(uint32_t ms, char * commond,...)
{
	waitUntilRxStopped(10,2);
	readFocus();
	va_list argptr;
	va_start(argptr, commond);
	vSetCommand(commond,argptr);
	va_end(argptr);
	waitUntilRxStopped(waitUntilBlankLine(ms),25);
}
char* getCommand(uint32_t ms, char* commond)
{
	printf("AT+%s\r\n",commond);
	uint32_t timeOutTick = getTick() + ms;
	uint32_t currentTick = getTick();
	while(timeOutTick - currentTick< (UINT32_MAX>>1))
	{
		char* line = readNewLine();
		if (line)
		{
			if(line[0] == '+')
			{
				uint16_t i;
				for(i = 1; i<strlen(line);i++)
				{
					if(line[i] == ':' || line[i] == '=')
					{
						break;
					}
				}
				uint16_t size = strlen(line) - i;
				if (size)
				{
					char* rst = malloc(size);
					if (rst)
					{
						strcpy(rst,line+i+1);
					}
					free(line);
					return rst;
				}
			}
			for (uint8_t i = 0; i< sizeof(g_sim808_focus)/sizeof(focus_t); i++)
			{
				if (g_sim808_focus[i](line))
				{
					break;
				}
			}
			free(line);
		}
		currentTick = getTick();
	}
	return NULL;
}

static void setATMode(void)
{
	hw_delay(500);
	readFocus();
	printf("+++");
	hw_delay(500);
	printf("\n");
	setCommandWait(5000,NULL);
	setCommandWait(5000,NULL);
	g_sim808_isAtMode = 1;
}

static void reset(void)
{
	hw_reset();
	if (g_sim808_isAtMode)
	{
	//	setATMode();
	}
	if (g_sim808_smartStarted)
	{
		uint32_t smartRemain = g_sim808_smartTick - getTick();
		if (smartRemain<(UINT32_MAX>>1) && smartRemain>0)
		{
			sim808_smart(smartRemain);
		}
	}
	g_sim808_resetTick = getTick();
}

uint8_t getStatus(uint8_t force)
{
	if (!force)
	{
		if (g_sim808_status == 3)
		{
			return g_sim808_status;
		}
	}
	setCommand("CIPSTATUS",NULL);
	uint32_t timeOutTick = getTick() + 1000;
	uint32_t currentTick = getTick();
	while(timeOutTick - currentTick< (UINT32_MAX>>1))
	{
		char* line = readNewLine();
		if (line)
		{
			if(strstr(line,"STATUS:") && strlen(line) == 8)
			{
				g_sim808_status = line[7] - '0';
				free(line);
				return g_sim808_status;
			}
			for (uint8_t i = 0; i< sizeof(g_sim808_focus)/sizeof(focus_t); i++)
			{
				if (g_sim808_focus[i](line))
				{
					break;
				}
			}
			free(line);
		}
		currentTick = getTick();
	}
	return 5;
	
}
void sim808_run(void)
{
	readFocus();
	if(g_sim808_smartStarted)
	{
		if ((getTick() - g_sim808_smartTick)<(UINT32_MAX>>1))
		{
			sim808_smartStop();
			reset();
		}
	}
	//sim808_resetForCrash(60000);
}

void sim808_sendHTTPRequest(char* path, cJSON* json)
{
	if (g_sim808_isAtMode) 
	{
		return;
	}
	char * str = cJSON_PrintUnformatted(json);
	if (str)
	{
		uint16_t length = strlen(str);
		char* head = "POST %s HTTP/1.1\r\n"
		"Content-Length: %i\r\n"
		"Content-Type: text/plain\r\n"
		"Host: api.hanwanglantian.com\r\n"		
		"Connection: keep-alive\r\n"
		"User-Agent: konggan\r\n"
		"Accept: \r\n"		
		"\r\n";
		printf(head,path,length);
		printf(str);
//	printf("\r\n");
		free(str);
	}
}

cJSON* sim808_getHTTPResponse(void)
{
	if (g_sim808_responseJson)
	{
		cJSON* json = g_sim808_responseJson;
		g_sim808_responseJson = NULL;
		return json;
	}
	else
	{
		return NULL;
	}
}

static void softwareInit(void)
{
	//Step 3: reset
	reset();
	//Step 4: wait until the "ready" appeared.
	hw_delay(500);
	waitUntilRxStopped(5000,100);
	readFocus();
	//Step 5: check if in TM mode.
	printf("\n");
	hw_delay(1000);
	uint8_t test = lastIndex();
	char c = bufferLastChar();
	if (c != '\n' )
	{
		//Setp 5.1: if not in TM mode. Set the default properties.
		setATMode();
	}
	setCommandWait(5000,"CWMODE_DEF","1",NULL);  //sta mode.
	//*
//	char* ssid = "\"AIRJ_NB\"";
//	setCommandWait(20000,"CWJAP_DEF?",NULL);
//	setCommandWait(20000,"CWJAP_DEF",ssid,"\"airj2123\"",NULL); //default ssid.
	//*/
	setCommandWait(5000,"CWAUTOCONN","1",NULL);  //auto connect after reset.
//		setCommandWait(5000,"CIPSTART","\"TCP\"",g_sim808_tcpServerIP,g_sim808_tcpServerPort,NULL);  //set tcp target
	setCommandWait(5000,"CIPMODE","0",NULL);  //
	setCommandWait(5000,"SAVETRANSLINK","0",NULL);  //
	setCommandWait(5000,"CIPMUX","0",NULL);  //
	g_sim808_isAtMode = 0;
//		reset();  // reset to enter tm mode.
//		waitUntilRxStopped(waitUntilBlankLine(5000),25);
//		readFocus();
}
uint8_t smart(int8_t type)
{
	if (type == 0)
	{
		setCommandWait(0,"CWSTARTSMART",NULL);
	}
	else
	{
		char buffer[4];
		sprintf(buffer,"%d",type);
		setCommandWait(0,"CWSTARTSMART",buffer,NULL);
	}
	if (waitUntilOk(5000))
	{
		return 1;
	}
	else
	{
		type--;
		if(type == 4 || type == -1)
		{
			return 0;
		}
		else
		{
			return smart(type);
		}
	}
}
void sim808_smart(uint32_t ms)
{
	if(smart(3))
	{
		g_sim808_smartStarted = 1;
		g_sim808_smartTick = getTick()+ms;
		
	}
	readFocus();
}

void sim808_smartStop(void)
{
	setCommandWait(5000,"CWSTOPSMART",NULL);
	if (waitUntilOk(5000))
	{
		g_sim808_smartStarted = 0;
	}
}


uint8_t sim808_smarting(void)
{
	return g_sim808_smartStarted;
}
uint8_t sim808_getStatus(void)
{
	if (g_sim808_smartStarted)
	{
		return 1;
	}
	if (g_sim808_status == 2)
	{
		return 4;
	}
	return g_sim808_status;
}
/*
void sim808_connectTCP(char* address, uint16_t port)
{
	char buffer[0x10];
	sprintf(buffer,"%d",port);
	char* addressForAT = (char*)malloc(strlen(address)+3);
	sprintf(addressForAT,"\"%s\"",address);
	setCommandWait(5000,"CIPSTART","\"TCP\"",address,buffer,NULL);
	free(addressForAT);
}

uint8_t sim808_tcp(char * str)
{
//	char* status = getCommand(1000,"CIPSTATUS");
	if (!g_sim808_smartStarted)
	{
		uint8_t status = getStatus(0);
		if(status == 2 || status == 4 || status == 3)
		{
			sim808_connectTCP("192.168.2.89",82);
		}
		else if (status == 3)
		{
			char buffer[0x10];
			sprintf(buffer,"%d",strlen(str));
			setCommandWait(0,"CIPSENDEX",buffer,NULL);
			if(waitUntilOk(500))
			{
				printf(str);
				waitUntilOk(5000);
			}
		}
	}
	return sim808_getStatus();
}*/


void sim808_connectUDP(char* address, uint16_t port)
{
	char buffer[0x10];
	sprintf(buffer,"%d",port);
	char* addressForAT = (char*)malloc(strlen(address)+3);
	sprintf(addressForAT,"\"%s\"",address);
	setCommandWait(5000,"CIPSTART","\"UDP\"",addressForAT,buffer,NULL);
	free(addressForAT);
}
uint8_t sim808_udp(char * str, char* address, uint16_t port)
{
	static char* lastAddress = NULL;
	static uint16_t lastPort = 0;
	if (!g_sim808_smartStarted)
	{
		uint8_t status;
		status = g_sim808_status;
		if (port!=lastPort || strncmp(address,lastAddress,strlen(address)))
		{
//			sim808_disconncet(0);
//			status = g_sim808_status;
		}
		else
		{
//			status = getStatus(0);
		}
		if(status == 2 || status == 4)
		{
			sim808_connectUDP(address,port);
			lastAddress = address;
			lastPort = port;
		}
		else if (status == 3)
		{
//			char buffer[0x10];
//			sprintf(buffer,"%d",strlen(str));
//			setCommandWait(0,"CIPSEND",buffer,NULL);
			setCommandWait(0,"CIPSEND",NULL);
			hw_delay(100);
//			if(waitUntilStrstr(500,">"))
//			{
				printf(str);
//				char endFlag[1]={0x1a};
//				HAL_UART_Transmit(&huart2, (uint8_t *)abc, 1, 1000);
//				waitUntilOk(5000);
//			}
		}
	}
	return 0;
//	return sim808_getStatus();
}

uint8_t sim808_disconncet(uint8_t force)
{
	if (!g_sim808_smartStarted)
	{
		uint8_t status = getStatus(0);
		if(force || status == 3)
		{
			setCommandWait(0,"CIPCLOSE",NULL);
			if(waitUntilOk(500))
			{
				return 1;
			}
		}
	}
	return 0;
}

void sim808_reset(void)
{
	reset();
}
char* sim808_getSSID(void)
{
	if (g_sim808_ssidString == NULL)
	{
		if(!g_sim808_smartStarted)
		{
			char* get = getCommand(200,"CWJAP_DEF?");
			if(get)
			{
				char* start,*end;
				start = strstr(get,"\"");
				end = strstr(get,"\",\"");
				if(start && end && start+1<end)
				{
					char* ssid = malloc(end-start);
					*end = '\0';
					strcpy(ssid,start+1);
					g_sim808_ssidString = ssid;
				}
				free(get);
			}
		}
	}
	if (g_sim808_ssidString)
	{
		char* ssid = malloc(strlen(g_sim808_ssidString)+1);
		strcpy(ssid,g_sim808_ssidString);
		return ssid;
	}
	return NULL;
}
char* sim808_getReceiveString(void)
{
	char* str = g_sim808_receiveString;
	g_sim808_receiveString = NULL;
	return str;
}
void sim808_test(void)
{
	setCommandWait(1000,"GMR",NULL);
//	char* ssid = "\"AIRJ_NB\"";
//	setCommandWait(20000,"CWJAP_DEF",ssid,"\"airj2123\"",NULL); //default ssid.
//	setCommandWait(0,"CIUPDATE",NULL);
//	if(waitUntilStrcmp(5000,"+CIPUPDATE:4"))
//	{
//		waitUntilStrstr(60000,"jump to run user");
//	}
	
//	setCommandWait(0,"CWSAP","\"AIRJESPAIRJ\"","\"airj2123\"","11","3",NULL);
}

uint8_t sim808_setSsidAndPassword(char *ssid, char *password)
{
	char rssid[34], rpassword[34];
	sprintf(rssid,"\"%s\"",ssid);
	sprintf(rpassword,"\"%s\"",password);
	setCommandWait(1,"CWJAP_DEF",rssid,rpassword,NULL);
	return waitUntilOk(20000)>0;
}




void sim808_UpdateGSMStatus(void)
{
	if(g_sim808_localInfo.GSMStatus != SIM808_GSM_SIM_NOTINSERT)
	{
		setCommandWait(1,"CSQ",NULL);
	}
	
	if(g_sim808_localInfo.GSMStatus == SIM808_GPRS_READY)
	{
		return;
	}
	
	setCommandWait(1,"CGNSPWR","1",NULL); //gps poweron
	setCommandWait(1,"CPIN?",NULL);	//sim card insert statu
	if(g_sim808_localInfo.GSMStatus==SIM808_GSM_SIM_NOTINSERT) //should be  SIM808_GSM_NOTREADY
	{
		return;
	}
	
	setCommandWait(1,"CREG","1",NULL);
	setCommandWait(1,"CREG?",NULL);//gsm service is exist
	if(g_sim808_localInfo.GSMStatus == SIM808_GSM_NOTREADY) //should be SIM808_GSM_READY
	{
		return;
	}
	
	setCommandWait(1,"CGATT?",NULL);
	if(g_sim808_localInfo.GSMStatus == SIM808_GSM_READY) //should be SIM808_GPRS_READY
	{
		return;
	}
	
	
}
void sim808_UpdateGPRSStatus(void)
{
	if(g_sim808_localInfo.GPRSStatus == SIM808_GPRS_CONNECTTED)
	{
		return;
	}
	if(1)
	{
//	if(g_sim808_localInfo.GSMStatus == SIM808_GPRS_READY)
//	{
//		setCommandWait(1,"CGNSPWR","1",NULL);
		setCommandWait(0,"CIPHEAD","1",NULL); //should befor sapbr,or will not exe sometime
		setCommandWait(0,"SAPBR","3","1","\"Contype\"","\"GPRS\"",NULL);
		if(strlen(g_main_apn))
		{
			char apnBufer[17] = {0};
			sprintf(apnBufer,"\"%s\"", g_main_apn);
			setCommandWait(0,"SAPBR","3","1","\"APN\"",apnBufer,NULL);
		}
		else
		{
			setCommandWait(0,"SAPBR","3","1","\"APN\"","\"CMNET\"",NULL);
		}
		if(strlen(g_main_user))
		{
			char apnBufer[17] = {0};
			sprintf(apnBufer,"\"%s\"", g_main_user);
			setCommandWait(0,"SAPBR","3","1","\"USER\"",apnBufer,NULL);
		}
		if(strlen(g_main_pass))
		{
			char apnBufer[17] = {0};
			sprintf(apnBufer,"\"%s\"", g_main_pass);
			setCommandWait(0,"SAPBR","3","1","\"PWD\"",apnBufer,NULL);
		}
		setCommandWait(0,"SAPBR","1","1",NULL);
		if (waitUntilOk(5000))
		{
			g_sim808_localInfo.GPRSStatus = SIM808_GPRS_CONNECTTED;
			g_sim808_status = 2;
		}
		else
		{
			setCommandWait(0,"SAPBR","0","1",NULL);
			g_sim808_localInfo.GPRSStatus = SIM808_GPRS_NOTOPEN;
			g_sim808_status = 5;
		}
		
		
	}
}





double gps_info_sort(char *gpsinfostr,int order,char *separator)
{
		static double g_sim808_location_value=999;
		int gpsLength = strlen(gpsinfostr);
		
		char* gpsInfoStrHandler = NULL;
		gpsInfoStrHandler = (char*)malloc(gpsLength*sizeof(char));
		strncpy(gpsInfoStrHandler, gpsinfostr, gpsLength);
	
		char* strHandle = strstr(gpsInfoStrHandler,": ") + 2 ; //get '+CGNSINF: ' string.
		char* result = NULL;
		result = strtok(strHandle,separator);
//		memset(gpsSortInfo_storeStr,0,sizeof(gpsSortInfo_storeStr));
//		result = strtok_r(strHandle, separator, &gpsSortInfo_storeStr); /////////////////2015-12-3

		while(order-1>0)
		{
			result = strtok(NULL, separator);
			if(result)
			{
				g_sim808_location_value = atof(result);
			}
			order--;
		}
		free(gpsInfoStrHandler);
		gpsInfoStrHandler = NULL;
		return g_sim808_location_value;
}

void sim808_UpdateGPSStatus(void)
{	
//	flushBuffer();
	
//	setCommandWaitOk(5000,"CGNSINF",NULL);  //2015-12-3. when we annotate the two lines of code below, our program runs regularly, don't know why.
																					// function setCommandWaitOk() just need 'OK',and this echo is ensured by line 472 in 'main.c',where we printf(".....\r\n");
	printf("AT+CGNSINF\r\n");
	hw_delay(100);
////	getCommand_noAsk(500,"CGNSINF");
//	HAL_Delay(50);
	
	char *str=NULL;
//	int g_sim808_gps_fix_status = 0; // 0 to not fix, 1 to fix!
	
	int cnt = 20;
	while(cnt)
	{
		str=readNewLine();
		if(str && strlen(str)>32)//
		{
//			if(*str == '0')//20151130
			if(strstr(str,"+CGNSINF: ")) //&& *(str+1) == '1')
			{
				int gpsFixStatus = (int)( gps_info_sort(str,2,",") );
				
//				printf("bug here! TEST_WITH_GPS!"); //2015-12-3.
//					printf("bug here11111! g_sim808_gps_fix_status == %d !!!\t\t",g_sim808_gps_fix_status);  //2015-12-3.
				
				if(gpsFixStatus == 0) // have not fixed location.
				{
//					printf("bug here! fix_status == %d !!!\t\t",g_sim808_gps_fix_status);  //2015-12-3.
					
					free(str);  //2015-12-4. !!!before return malloc must free!!!.
					return;
				}
				
				g_sim808_Info.gpsUpdated = true;
				g_sim808_Info.latitude=gps_info_sort(str,4,",");
				g_sim808_Info.longitude=gps_info_sort(str,5,",");
				g_sim808_Info.gpsTime=gps_info_sort(str,3,",");
//				*lat=gps_info_sort(str,4,",");
//				*lon=gps_info_sort(str,5,",");
//				*time=gps_info_sort(str,3,",");
//				*lat=gps_info_sort(str,3,","); //sim908 code.
//				*lon=gps_info_sort(str,2,","); //sim908 code.
//				*time=gps_info_sort(str,5,","); //sim908 code.
				free(str);
//				*lat=111.011;//just to test.
//				*lon=44.011;//just to test.
				return;
			}
			else
			{
				free(str);  //2015-12-4
			}
		}
		else if (str!=NULL)
		{
			free(str);
		}
		else
		{
//			break;
//			continue;
		}
		cnt--;
	}
//	*lat = 0;
//	*lon = 0;
	return;
}


void sim808_GetDeviceId(void)
{
//	hw_delay(10);
	printf("AT+GSN\r\n");
	hw_delay(30);
//	setCommandWait(1,"GSN",NULL); //AT+GSN
	char *IMEIstr = NULL;
	
	int cnt = 20;
	while(cnt)
	{
		IMEIstr = readNewLine();
		if(IMEIstr)
		{
			if((char)(*IMEIstr)>=48 && (char)(*IMEIstr)<=57 && strlen(IMEIstr)==15 )  //first character is between '0' and '9' .
			{
				g_sim808_Info.idUpdated = true;
		#if NORMAL_SERVER
//				unsigned char encrypt[] ="admin";//21232f297a57a5a743894a0e4a801fc3
				unsigned char decrypt[16];    
				MD5_CTX md5;
				MD5Init(&md5);         		
				MD5Update(&md5,(unsigned char *)IMEIstr,strlen((char *)IMEIstr));
				MD5Final(&md5,decrypt);  
//				decrypt[16] = IMEIstr[11];
//				decrypt[17] = IMEIstr[12];
//				decrypt[18] = IMEIstr[13];
//				decrypt[19] = IMEIstr[14];
//				decrypt[20] = '\0';
				sprintf(g_sim808_Info.IMEI_number,"L1-");  //2015-12-7
				for(uint8_t i=0; i<16; i++)
				{
					sprintf(g_sim808_Info.IMEI_number+strlen(g_sim808_Info.IMEI_number),"%02x", decrypt[i]);  //2015-12-7
				}
//				for(uint8_t i=16; i<20; i++)
//				{
//					sprintf(g_sim808_Info.IMEI_number+strlen(g_sim808_Info.IMEI_number),"%c", decrypt[i]);  //2015-12-7
//				}
//				g_sim808_Info.IMEI_number[40] = '\0';
				sprintf(g_sim808_Info.IMEI_number+strlen(g_sim808_Info.IMEI_number),"%s", IMEIstr+11);  //2015-12-7
				
		#else
				sprintf(g_sim808_Info.IMEI_number,"L1-TEST-%s",IMEIstr);
		#endif
	//				g_sim808_IMEI_number = atoll(IMEIstr);
				free(IMEIstr);
				IMEIstr = NULL;
				break;
			}
			free(IMEIstr);
		}
		IMEIstr = NULL;
		cnt--;
	}
//	hw_delay(10);
}

void sim808_UpdateInfo(void)
{
	if(g_sim808_localInfo.GSMStatus == SIM808_GSM_READY)
	{
		g_sim808_Info.gsmOk = true;
	}
	else
	{
		g_sim808_Info.gsmOk = false;
	}
	
	if(g_sim808_localInfo.GPRSStatus == SIM808_GPRS_CONNECTTED)
	{
		g_sim808_Info.gprsConnected = true;
	}
	else
	{
		g_sim808_Info.gprsConnected = false;
	}
	
//	if(!g_sim808_Info.gpsUpdated)
//	{
//		sim808_UpdateGPSStatus();
//	}
	
	if(!g_sim808_Info.idUpdated)
	{
		sim808_GetDeviceId();
	}
}

sim808_Info_t sim808_GetInfo(void)
{
	return g_sim808_Info;
}


void sim808_SetGpsInfo(sim808_Info_t info)
{
	g_sim808_Info.gpsTime = info.gpsTime;
	g_sim808_Info.latitude = info.latitude;
	g_sim808_Info.longitude = info.longitude;
	g_sim808_Info.gpsUpdated = info.gpsUpdated;
}


void sim808_UpdateStatus(void)
{
//	hw_delay(5000);
	sim808_UpdateInfo();
	sim808_UpdateGSMStatus();
	sim808_UpdateGPRSStatus();
}
