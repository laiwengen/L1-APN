#include "Console.h"
#include "stdlib.h"
#include "string.h"
#include "stdio.h"
#include "stdarg.h"

console_t* g_console_start = NULL; 
//HARDWARE
static void hw_startDMA(console_t* c)
{
//	memset((void*)c->buffer,UINT16_MAX,c->bufferSize*sizeof(char));
	c->pWrite = &(c->huart->hdmarx->Instance->CNDTR);
	HAL_UART_Receive_DMA(c->huart, (uint8_t *)c->buffer, c->bufferSize);
}
//HARDWARE
static inline void hw_printString(console_t* c, char* str)
{
	HAL_UART_Transmit(c->huart,(uint8_t*)str,strlen(str),100);
}
//HARDWARE
static inline void hw_printChar(console_t*c, char ch)
{
	HAL_UART_Transmit(c->huart,(uint8_t*)&ch,1,100);
}

void console_printHex(console_t*c, uint32_t value, uint8_t bits)
{
	for(uint8_t i = 0; i<bits; i+=4)
	{
		uint8_t ch = (((uint32_t)value<<i>>(bits-4))&0x0f);
		if(ch>9)
		{
			ch+='A'-'0'-10;
		}
		hw_printChar(c,(char)('0'+ch));
	}
}

void console_printf(console_t*c, char* fmt,...)
{
	va_list argptr;
	va_start(argptr, fmt);
#if 0
	hw_printString(c,fmt);
#else
	char buffer[256+1];
	vsnprintf(buffer,256,fmt,argptr);
	hw_printString(c, buffer);
#endif
	va_end(argptr);
}


console_t* console_init(uartHandle huart, volatile uint8_t* buffer, uint16_t bufferSize)
{
	assert_param((bufferSize&(bufferSize-1))==0);                      //a regulation that buferSize must be 2 positive integer power, else false.
	console_t* c = (console_t*)malloc(sizeof(console_t));
	c->huart = huart;
	c->lock = 0;
	c->echo = 1;
	c->firstCommand = NULL;
	c->firstListener = NULL;
	c->buffer = buffer;
	c->readIndex = 0;
	c->echoIndex = 0;
	c->bufferSize = bufferSize;
	c->bufferMask = bufferSize - 1;
	c->next = g_console_start;
	g_console_start = c;
	hw_startDMA(c);
	return c;
}

void console_deinit(uartHandle huart)
{
}

static inline uint16_t getWriteIndex(console_t* console)
{
	volatile uint32_t* pw = (volatile uint32_t*)console->pWrite;
	volatile uint16_t remain = *pw;
	return ((console->bufferSize) - remain);
}

static uint16_t hasNewLine(console_t* c)                                    // ����Ƿ����µ���. 
{
	for(uint16_t i = 0; i<c->bufferSize; i++)
	{
		uint16_t index = ((c->readIndex)+i)&c->bufferMask;
		if (index == getWriteIndex(c))
		{
			break;
		}
		uint16_t ch = *(c->buffer+index);
		if (ch == '\n' || ch == '\0')
		{
			return i+1;
		}
	}
	return 0;
}

static inline uint16_t getChar(console_t* c)
{
	uint16_t ch;
	if (c->readIndex != getWriteIndex(c))
	{
		ch = *(c->buffer+(c->readIndex));
		c->readIndex = (c->readIndex+1)&c->bufferMask;
	}
	else
	{
		ch = UINT16_MAX;                        //65535
	}
	return ch;
}

static inline uint16_t getEchoChar(console_t* c)
{
	uint16_t ch;
	volatile uint16_t a = c->echoIndex;
	volatile uint16_t b = getWriteIndex(c);
	if (c->echoIndex != getWriteIndex(c))
	{
		ch = *(c->buffer+(c->echoIndex));
		c->echoIndex = (c->echoIndex+1)&c->bufferMask;
	}
	else
	{
		ch = UINT16_MAX;
	}
	return ch;
}

//REMEMBER TO Free(return pointer);
static char* readNewLine(console_t* c)
{
	uint16_t lineLength = hasNewLine(c);
	if (lineLength)
	{
		char *str = (char*)malloc(lineLength);
		for (uint16_t i = 0; i<lineLength; i++)
		{
			*(str+i) = (char)getChar(c);
		}
		*(str+lineLength-1) = '\0';
		if (lineLength > 1)
		{
			if(*(str+lineLength-2) == '\r')
			{
				*(str+lineLength-2) = '\0';
			}
		}
		return str;
	}
	return NULL;
}

const char* g_console_headString = "AT+";

static char* substr(char* str, char s, char e)       //���غ�������str��ȡs��e֮����ַ��������ַ�����ʽ���أ���������ȡ�ַ����ĵ�ַ��
{
	char* si;
	if (s == '\0')                                     //'\0'��ͷ��飬�� char e Ϊֹ .
	{
		si = str-1;
	}
	else
	{
		si= strchr(str,s);
	}
	if (si)
	{
		char* ei = strchr(si+1,e);                       //��� char s ���� \0 ���������һλ��ʼѰַ char e������ʹ�ͷѰַ char e . 
		if(ei==NULL)                                     // û�ҵ�
		{
			ei = str + strlen(str);                        // û�ҵ����βֱ��str��ĩβ
		}
		if(ei>si)                                        // ���ei �� si ǰ��
		{
			char* sub = malloc(ei-si);                     // ������һ���ڴ�
			strncpy(sub,si+1,ei-si-1);                     // �Ѵ� si ֮��ĵ�ei ���ַ�������Ϊ ei-si-1 �� ��Ϊ������ si �� ei�� ������ sub �� .
			*(sub+(ei-si)-1) = '\0';                       // ͬʱ�� sub дΪ�ַ���
			return sub;
		}
	}
	return NULL;
}

static stringList_t* newStr(char* str)
{
	if (str == NULL)
	{
		return NULL;
	}
	stringList_t* sl = malloc(sizeof(stringList_t));
	sl->string = str;
	sl->next = NULL;
	return sl;
}

static uint8_t paresLine(console_t* console, char* str)
{
	uint16_t headLength = strlen(g_console_headString);           //g_console_headString="AT+".
	if (strncasecmp(g_console_headString,str,headLength) == 0)    //compare str with "AT+" in a safe mode by function--strncasecmp.
	{
		str += headLength;                                          // str��ַ���� headLength��������str�ַ�����ͷ�ġ�AT+��
		char* sub = substr(str,'\0','=');                           //sub == a part of string between '\0' with '='. ������AT+�� ֮��ȡ��=��֮ǰ���ַ���
 		if (sub)                                                    //sub exist, thus there are some chars between "AT+" with "="
		{
			if(strlen(sub) == 0)                                      // just empty space. ��AT+��֮�����ַ���û������
			{
				free(sub);
				return 1;
			}
			consoleCommand_t* cc = console_getCommandOut(console,sub);       //��console������ȡ��sub�ַ�������consoleCommand_t* ���͸���cc
			if (cc)
			{
				console_deleteCommand(cc);
			}
			cc = malloc(sizeof(consoleCommand_t));
			cc->next = console->firstCommand;
			console->firstCommand = cc;
			stringList_t* sl = newStr(sub);                                  //��sub ����sl�½ڵ�
			cc->first = sl;
			
			str+=strlen(sub);                                               //str������AT+�� ֮�������� sub�����ӡ�=�� ����
			char* para;
			char startChar = '=';
			while(1)
			{
				para = substr(str,startChar,',');                             //ȡ��=�� ֱ�������� ֮���ַ�������para
				if (para == NULL)
				{
					break;
				}
				stringList_t* sp = newStr(para);                              //��para ����sp�½ڵ�
				sl->next = sp;
				sl = sp;
				startChar = ',';
				str+=strlen(para) + 1;                                        //str�ٴӡ����� ����ȡ��ֱ��para==NULL������ѭ��
			}
			return 1;
		}
		return 0;
	}
	else
	{
		return 0;
	}
}

static void reOrgString(char* str)                             //�˸���
{
	uint16_t i = 0, j = 0;
	for(; i< strlen(str)+1; i++)
	{
		if (*(str + i) == '\b')
		{
			if(j>0)
			{
				j--;
			}
		}
		else
		{
			*(str+j) = *(str+i);
			j++;
		}
	}
}

void console_tick(void)
{
	console_t* c = g_console_start;
	while(c)
	{
		//echo stuff
		while(1)
		{
			uint16_t ch= getEchoChar(c);
			if (ch&0xff00)
			{
				break;
			}
			if (c->echo)
			{
				hw_printChar(c,(char)ch);
			}
		}
		//check if new line
		if(c->lock == 0)
		{
			char* str = readNewLine(c);
			if (str)
			{
				reOrgString(str);
				if (paresLine(c,str))
				{
					hw_printString(c,"\r\nAT+ok\r\n");
				}
				else
				{
					hw_printString(c,"\r\nAT+error\r\n");
				}
				free(str);
			}
		}
		//listener
		consoleListener_t* cl = c->firstListener;
		while(cl)
		{
			if (cl->priority == 1)
			{
				consoleCommand_t* cc = console_getCommandOut(c,cl->command);                //**************************************************//
				if(cc)
				{
					cl->function(cc);
				}
			}
			cl = cl->next;
		}
		c=c->next;
	}
}

void console_run(void)
{	
	console_t* c = g_console_start;
	while(c)
	{
		consoleListener_t* cl = c->firstListener;
		while(cl)
		{
			if (cl->priority == 0)
			{
				consoleCommand_t* cc = console_getCommandOut(c,cl->command);
				if(cc)
				{
					cl->function(cc);
				}
			}
			cl = cl->next;
		}
		c=c->next;
	}
}

uint8_t console_clearAllCommand(console_t* console)
{
	consoleCommand_t *cur = NULL;
	uint8_t has = 0;
	
//	__HAL_UART_CLEAR_FLAG(console->huart, UART_IT_ORE);
	__HAL_UART_CLEAR_OREFLAG(console->huart);
	while(console->firstCommand)
	{
		cur = console->firstCommand;
		console->firstCommand = console->firstCommand->next;
		console_deleteCommand(cur);
		has++;
	}
	return has;
}

void console_deleteCommand(consoleCommand_t* command)
{
	if(command == NULL)
	{
		return;
	}
	stringList_t * sl = command->first,*next;
	while(sl)
	{
		next = sl->next;
		free(sl->string);
		free(sl);
		sl = next;
	}
	free(command);
}

consoleCommand_t* console_getCommand(console_t* console, const char* command)
{
	consoleCommand_t* cc = console->firstCommand;
	while(cc)
	{
		if(strcasecmp(command,cc->first->string) ==0)
		{
			return cc;
		}
		cc = cc->next;
	}
	return NULL;
}

consoleCommand_t* console_getCommandOut(console_t* console, const char* command)
{
	consoleCommand_t* cc = console->firstCommand,*pre = NULL;
	while(cc)
	{
		if(strcasecmp(command,cc->first->string) ==0)
		{
			if(pre)
			{
				pre->next = cc->next;
			}
			else
			{
				console->firstCommand = cc->next;
			}
			return cc;
		}
		pre = cc;
		cc = cc->next;
	}
	return NULL;
}


void console_addListener(console_t* console, const char* command, consoleFunction_t function, uint8_t priority)
{
	consoleListener_t* cl = (consoleListener_t*) malloc(sizeof(consoleListener_t));
	cl->command = command;
	cl->function = function;
	cl->priority = priority;
	cl->next = console->firstListener;
	console->firstListener = cl;
}
