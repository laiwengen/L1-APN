#ifndef CONTROL_H_
#define CONTROL_H_

//*********************************read me start******************************************************// //2015-12-10.
//**************About 'fpe()' function, we only use them in 'main.c'**********************************//
//**************We now has used these address: 0x005c, 0x006c, 0x007c, 0x008c*************************//
//*************'0x005c': control var 'resetTimes'*****************************************************//
//*************'0x006c': control var 'g_main_uploadTime'**********************************************//
//*************'0x007c': control var '1117', which is assitant to var 'g_main_uploadTime'*************//
//*************'0x008c': control var 'g_main_start_battery_voltage', may be '3399' or '3550'. ********//
//*************************************'0x001c': save value of g_main_lat. ***************************//  //2016-1-18.
//*************************************'0x002c': save value of g_main_lon. ***************************//  //2016-1-18.
//*************************************'0x011c': save value of g_main_time yy. **************************//  //2016-1-18.
//*************************************'0x013c': save value of g_main_time hh. **************************//  //2016-1-18.
//*************************************'0x004c': contrl every 6h update gps. *************************//  //2016-1-18.
//*********************************read me end********************************************************//  //2015-12-10.

#define DEVICE_TYPE "L1"
//#define TEST_SERVER
#define NORMAL_SERVER 1

//#define PRODUCT_NUMBER "MAI-L1-102206-1202"  //old code. 2015-12-10

//#define SERVER_URL "\"http://api.hwlantian.com/mb/data/upload\""   // normal server.
//#define SERVER_URL "\"test.hwlantian.com\""   // normal server.
#define SERVER_URL "\"hwlantian.com\""   // normal server.
//#define SERVER_URL "\"http://42.62.73.228/mb/data/upload\""         // test server.

/*This define is to confirm we have had new interval_time, which is got from server, binding to 0x007c. */
#define FLAG_FLASH_DATA_ENSURE 0x0a0a
#define SERVER_INTERVAL_MULTIPLE 1000
#define ADDRESS_UPLOAD_INTERVAL_VALUE 0x006c
#define ADDRESS_UPLOAD_INTERVAL_ENSURE 0x007c


#define ADDRESS_CNT_GPS_NOUPDATE_TIME 0x004c
#define ADDRESS_GPS_UPDATED_ENSURE 0x005c
#define ADDRESS_VALUE_GPS_LATITUDE 0x001c
#define ADDRESS_VALUE_GPS_LONGITUDE 0x002c
#define ADDRESS_VALUE_GPS_GPSTIME1 0x011c
#define ADDRESS_VALUE_GPS_GPSTIME2 0x013c


#define FPE_FACTORY_UID 0x014c
#define FPE_FACTORY_UID_STRING 0x024c


#define FPE_CUSTOM_APN 0x034c
#define FPE_CUSTOM_USER 0x044c
#define FPE_CUSTOM_PASS 0x054c


#define RANGE_RUNTIME_BATTERY_LOW  3150
#define RANGE_START_BATTERY_LOW  3400
#define RANGE_START_BATTERY_NORMAL  3550
#define ADDRESS_BATTERY_RANGE  0x008c

#define NORMAL_SERVER 1


/*This define is to confirm size of char[] array to store 'g_sim808_IMEI_number', which locates in 'sim808.c' at line 12 and 'main.c' at line 259. */
//#define IMEI_SIZE 32

/*This define is to let upload info (where you see on html page locate) at first page, we use this define because before 2015-12-2, the html page can't goto next page!*/
#define TEST_IMEI_STARTWITH_11 1

/*These two defines locate in function 'bool sim808_getGPSInfo' in sim808.c*/
#define TEST_WITH_GPS 1

/*These three defines locate in function 'void pm25_update(void)' in main.c*/
#define PM25_STABLE_ACCURRACY 0.3
#define STRICT_PM25_STABLE_VALUE
//#define WITHOUT_PM25_VALUE

#define MY_RTC_SYNCH_PREDIV 0x139

//MAX_WAKE_MS必须考虑到MAX_GPS_MS获取完毕，数据上传失败一次（花费）HTTP_REQUEST_MS的时间！！！

//#define GPS_INTERVAL_MS    86400000               //24 h         //GPS强制更新间隔
//#define UPLOAD_DATA_INTERVAL_MS   180000//900000         //3 min       //上传间隔 test 3min (180000 ms)
//#define MAX_WAKE_MS        150000//660000         //11 min       //单次最大工作时长 test 2.5min (150000 ms)
//#define MAX_GPS_MS         75000//600000          //10 min       //GPS单次更新时长 test 1.5min (90000 ms)
//#define MAX_PM25_MS        60000                  //2 min        //PM2d5稳定时间 test 1.5min (90000 ms)
//#define HTTP_REQUEST_MS    35000                  //20 s         //得不到200时，强制发送数据的间隔.

//#define GPS_INTERVAL_MS    86400000               //24 h         //GPS强制更新间隔.
//#define UPLOAD_DATA_INTERVAL_MS   1800000//900000         //30 min       //上传间隔 test 3min (180000 ms)
//#define MAX_WAKE_MS        240000//660000                //单次最大工作时长 test 2.5min (150000 ms)
//#define MAX_GPS_MS         180000//600000                //GPS单次更新时长 test 1.5min (90000 ms)
//#define MAX_PM25_MS        90000                          //PM2d5稳定时间 test 1.5min (90000 ms)
//#define HTTP_REQUEST_MS    45000                          //得不到200时，强制发送数据的间隔.  

//#define GPS_INTERVAL_MS    600000               //24 h         //GPS强制更新间隔.

#define GPS_INTERVAL_MS    86400000               //24 h         //GPS强制更新间隔.
#define UPLOAD_DATA_INTERVAL_MS   300000		      //5 min       //上传间隔 test 15min (900000 ms)
#define MAX_WAKE_MS        360000//660000         //单次最大工作时长 test 2.5min (150000 ms)
#define MAX_GPS_MS         180000//600000         //GPS单次更新时长 test 1.5min (90000 ms)
#define MAX_PM25_MS        90000                          //PM2d5稳定时间 test 1.5min (90000 ms)
#define HTTP_REQUEST_MS    45000                          //得不到200时，强制发送数据的间隔.  

#define DIVED_DAY					 8

//#define GPS_INTERVAL_MS    86400000               //24 h         //GPS强制更新间隔
//#define UPLOAD_DATA_INTERVAL_MS   60000//900000         //15 min       //上传间隔 test 30min
//#define MAX_WAKE_MS        210000//660000         //11 min       //单次最大工作时长 test 3.5min
//#define MAX_GPS_MS         30000//600000          //10 min       //GPS单次更新时长 test 2.5min
//#define MAX_PM25_MS        60000                  //2 min        //PM2d5稳定时间 test 1.5min (90000 ms)
//#define HTTP_REQUEST_MS    45000                  //20 s         //得不到200时，强制发送数据的间隔.

// GPS_INTERVAL_MS>>UPLOAD_DATA_INTERVAL_MS>MAX_WAKE_MS>(MAX_GPS_MS,MAX_PM25_ MS,HTTP_REQUEST_MS)

#endif

