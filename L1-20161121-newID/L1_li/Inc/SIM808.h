
#ifndef SIM808_H_
#define SIM808_H_

#include "stdint.h"
#include "CJSON.h"
#include "stm32f0xx_hal.h"
#include "stdbool.h"

#define IMEI_SIZE (32+16)

typedef struct
{
	bool gsmOk;
	uint8_t gsmSigStrenth;
	bool gprsConnected;
	bool gpsUpdated;
	double longitude;
	double latitude;
	double gpsTime;
	bool idUpdated;
	char IMEI_number[IMEI_SIZE];
	
//	Sim900aStatus_t sim900aStatus;
}sim808_Info_t;

//sim808_tick should be call frequently. it would read the RX buffer which collect the bytes WIFI transmited.
//and do the following case.
// * check if the connect has been closed, and reset to re-connect.
// * check if smartlink has been done, and reset to apply it.
// * check http response received. ONLY json data would be stored, and can be get by sim808_getHTTPResponse function.
void sim808_run(void);

//initialization the wifi chip with a giving uart port.
void sim808_init(UART_HandleTypeDef* huart);

void sim808_deinit(void);

//send a http request, path can be "\\" or "\\somepath\\index.jsp".
void sim808_sendHTTPRequest(char* path, cJSON* json);

//get the last response json. last but two will not be strored.
//REMEMBER free(json) when you finish.
cJSON* sim808_getHTTPResponse(void);


void sim808_smart(uint32_t ms);
void sim808_smartStop(void);
//enter smart mode if can't join the default Ap for a while. 
//REMEMBER the wifi would be reset by sim808_resetForCrash function during smart mode. this will quit smart mode.
void sim808_smartNoAp(uint32_t ms);
uint8_t sim808_smarting(void);

//dont sure if the wifi would crash for some unknow reason. so just reset it every few minutes.
uint8_t sim808_resetForCrash(uint32_t ms);


void sim808_test(void);
void sim808_reset(void);
char* sim808_getSSID(void);
uint8_t sim808_tcp(char * str);
uint8_t sim808_udp(char * str, char* address, uint16_t port);
uint8_t sim808_disconncet(uint8_t force);
uint8_t sim808_getStatus(void);
char* sim808_getReceiveString(void);
void sim808_setSmartCallback(void(*fcn)(void));
void sim808_setSsidAndPasswordCallback(void(*fcn)(char*, char*));
uint8_t sim808_setSsidAndPassword(char *ssid, char *password);


void sim808_UpdateGSMStatus(void);
void sim808_UpdateGPRSStatus(void);
void sim808_UpdateGPSStatus(void);
void sim808_GetDeviceId(void);
sim808_Info_t sim808_GetInfo(void);
void sim808_SetGpsInfo(sim808_Info_t info);

void sim808_UpdateStatus(void);


#endif
