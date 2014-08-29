/*
////////////////
	LED	analog clock
////////////////
*/

/*
////////////////
	Work log

*	Add some LEDs on portA to indicate status.
	Set Time : Set Alarm : Alarm active
!	Done : 19 Apr 07, also made time set & alarm set active only on delayed hold down.

!	Sort out reading/writing to the RTC.
	Might need to hard wire this to make I2C work ok.
	Also might need to slow down I2C, which might effect over all display
!	Done (most of) 22 Apr 07, reads for RTC, but doesn't set it yet. i2c at 100khz, seems ok.

!	Need to sort out writing time to RTC (dec2bcd)
!	Also need to sort out odd slownes of blinking for mins.
!	Compare alarm to current time.
!	Buzz or something for 10 mins
!	Just worked out why was having probs with RTC & GPIO on i2c bus
	Was due to the interrupt routine interrupting a main() i2c access
	Rememeber to either only i2c from isr or disable ints whilst i2c's in main()
!	Need to work out some sort of snooze mode for the alarm (hit any button other than alarm set)
	and wire up some sort of buzzer
!	Buzzer driven using PWM out on RB3.
!	Snooze and alarm to be working ok.  Alarm sounds for 25 mins with a 5 min snooze capability
!	Added LDR to RA0 to change global current of GPIO dependant on ambient light.
*	Want to re-write convertTime2light() to allow for...
	!	More realistic Hour display eg, use the LEDs inbetween the hours points to show like a real analog clock
	!	Simple animations on certain events.  Eg. every hours chase a dot around the clock or similar.
!	Buzzer swapped for active type, no need for pwm out still on rb3
!	Need to re-write main text (if) in alarm_test()
!	Indicator LED swapped for RGB LED to create multi colors
	PWM of RGB LED provided by using RA1
!	Animation capability added using effect_run() and associated funcs
!	MAX6956 individual LED current now used to balance the red/blue intensity.
!	PWM on RC0,1,2 now used to dim LEDs rather than global current of MAX6956

!	18 Nov 07
	Rewrite to use LAT for output rather than PORT
////////////////
*/


#include <p18cxxx.h>
#include <delays.h>
#include <usart.h>
#include <i2c.h>
#include <stdio.h>
#include <timers.h>
#include <adc.h>
#include <portb.h>
#pragma	config WDT = OFF

#define _SIM

#define	LoByte(i)	(	(unsigned	char)	i	)
#define	HiByte(i)	(	(unsigned	char)	(	((unsigned int)	i) >>	8) )
#define	MidByte(i)	(	(unsigned	char)	(	((unsigned short long) i)	>> 16) )
#define	TopByte(i)	(	(unsigned	char)	(	((unsigned long) i)	>> 24) )

//PortA
//outputs
#define	LDR						PORTAbits.RA0	//porta.0
#define	LED_PWM			LATAbits.LATA1	//porta.1
#define	ALARMOUT			LATAbits.LATA2	//porta.2
#define	LED_ALERT			LATAbits.LATA4	//active low (Green)
#define	LED_ALARM			LATAbits.LATA5	//active low (Red)
#define	LED_AMPM			LATAbits.LATA3	//active low (Blue)

//Port B
//inputs
#define	rtcin					PORTBbits.RB0	//Pulse input for RTC
#define	btn0					PORTBbits.RB1	//Alarm on/off/set
#define	btn1					PORTBbits.RB2	//Time set/run
#define	btn2					PORTBbits.RB3	//Snooze/sleep
#define	btn3					PORTBbits.RB4	//min inc
#define	btn4					PORTBbits.RB5	//hour
//outputs
#define	RB6	PORTBbits.RB6
#define	RB7	PORTBbits.RB7

//Masks for buttons
#define	BTN_ALARMSET		0b00100000
#define	BTN_TIMESET			0b00010000
#define	BTN_SNOOZE			0b00001000
#define	BTN_HOURADV		0b00000100
#define	BTN_MINADV			0b00000010
#define	DBOUNCEMASK		0b00111110 //only bother to count these pins in dbouncing
#define 	DLY_LONG				70	// A long delay for detecting a held down button (time and alarm set)
#define 	DLY_LONG_RST		75	// Ensures no repeat	
#define 	DLY_SHORT				33 	// A short delay for detecting a held down button (mins + hour advance)
#define	DLY_SHORT_RST	30	// Makes a nice repeat rate for advance
#define	LIGHTMAX				15	//max value for gbl_light
#define	LIGHTCEIL				512 //1024 - 512 //When light levels are below this level just use pwm of 0
#define	LIGHTDIV					(LIGHTCEIL / (LIGHTMAX+1))	//(1024 - 512) / 15

//Port C
#define	RC0		PORTCbits.RC0	//sector0 sel
#define	RC1		PORTCbits.RC1	//sector1 sel
#define	RC2		PORTCbits.RC2	//sector2 sel
#define	SCL		PORTCbits.RC3	//I2C clock,	PORTC	pin	3
#define	SDA		PORTCbits.RC4	//I2C data
#define	RC5		PORTCbits.RC5	
#define	TX		PORTCbits.RC6
#define	RX		PORTCbits.RC7

#define	MAX6956ADDR 0x80 //address of 6956 GPIO
#define	RTCADDR 0b11010000 //address of DS1307 RTC
#define	RTCLEN 3 //number of bytes to read from RTC (zero based)
#define	ALARMLIMIT 30 //max time for alarm
#define	SNOOZELIMIT 9 //max time for alarm

unsigned char gbl_ledBuffer_written; //used to flag that we have new data, not allowed to write until unset
unsigned char gbl_ledBuffer_bank; //[0|1] bank we are currently writing to. (isr reads other)
unsigned char	gbl_ledBuffer[2][3][3]; //gbl_ledBuffer :2 banks of 3 sectors of 3 bytes representing status of the	60 leds	(last	4	bits of	byte [x][3]	unused)
//unsigned short long gbl_effect.buffer[3]; //3 sectors of 24bits representing effects display(last 4 bits of byte [x][3]	unused)
struct{
		unsigned char sector; 	// Store which bank the effect is using. Or sometimes just a spare
		unsigned char ctr;			// Counter for effects so they can top themselves
		unsigned char mode;		// Effects mode
		unsigned char temp;		// Temp...
		union buffer_tmp{
			unsigned short long full;
			struct{
				unsigned char low;
				unsigned char mid;
				unsigned char high;
			};
		} buffer[3];
} gbl_effect;

rom	char sectorSelBits[] = {0x00,0x01,0x02};//Bank selects. AND this bits of port c (LATC)
rom	char sectorAddr[]	=	{0x4c,0x54,0x5c	};//addresses of GPOI ports 12-19,20-27,28-31

unsigned char	gbl_sector;						//used by isr to keep track of current sector
unsigned char	gbl_blinkOff_Min;			//should we blink the mins at all?
unsigned char	gbl_blinkStatus_Min;		//display mins (blink)
unsigned char gbl_blinkStatus_Sec;		//display seconds (blink), used to make seconds a quick pulse
unsigned char	gbl_display_secs;			//display seconds
unsigned char gbl_timeSelect;				//Set [0=time|1=alarm]
unsigned char	gbl_bRun;						//[run/set] time
unsigned char gbl_bTimeChanged;		//Set [1=time has changed]
unsigned char gbl_bAlarmSet;				//Set [1=alarm is armed]
unsigned char gbl_bAlarmSnd;				//Set [1=alarm is currently sounding!]
unsigned char gbl_bSnooze;					//Set [1=we are snoozing!]
unsigned char gbl_bPm;							//Set [0=am,1=pm]
unsigned char gbl_blinky;						//Used as a simple counter for blinking seconds and alarm
unsigned char	gbl_light;							//Light lever 0 - 24
unsigned char	gbl_PartialHour;				//Bool : 1 = show hours as partial, 0 = show hours as whole numbers

//Config words for GPIO {length (hex), start address (hex), data (hex)}
rom	char gpio_start_0[]			= {0x02,0x04,0b01000001};		//config byte
rom	char gpio_start_1[]			= {0x2,0x02,0x00001111};		//global current control : Unnecesary now we are using per seg current to balance brightness
rom	char gpio_start_2[]			= {0x08,0x09,0x00,0x00,0x00,0x00,0x00,0x00,0x00};	//config all ports as LED const cur	driver
//Current per segment
rom	char gpio_start_3[]			= {0x0c,0x16,0xf6,0xff,0x6f,0xff,0xff,0xf6,0xff,0x6f,0xff,0xff};
rom	char gpio_testModeOn[] 	= {0x02,0x07,0x01};
rom	char gpio_testModeOff[]	= {0x02,0x07,0x00};

unsigned char	gbl_temp[10];	//used as a temp buffer for communication

//Debounce stuff
#define	DBOUNCE_LEN 12//length of debounce buffer
unsigned char gbl_dbounce_buff[DBOUNCE_LEN];	//ring buffer for debouncing
unsigned char gbl_dbounce_idx;	//current byte to write
unsigned char gbl_status;	//current byte to write

//time holder : it's the same order as the RTC
union time_t{
	unsigned char arr[RTCLEN];
	struct{
		unsigned char second;
		unsigned char minute;
		unsigned char hour;
/*
		unsigned char day;
		unsigned char date;
		unsigned char month;
		unsigned char year;
*/
	};
} times[3];
//Indexes of time holders
#define	TNOW 0 //now
#define	TALMON 1 //Alarm on
#define	TSNOOZE 2 //Alarm sleep

#define	SEC		0 //position in time_t.arr
#define	MIN		1 //position in time_t.arr
#define	HOUR	2 //position in time_t.arr

//func prototypes
void boot_up(	void );
void gpio_init(void);
void rtc_init(void);
void convertTime2light();
void effect_run();
void set_dbounce();
void time_inc(union time_t *theTime, unsigned char part, unsigned char amount);
void effect_next();
void gpio_readConfig(unsigned char addr, unsigned char length);
signed char	I2C_sendRomStr(unsigned	char addr, unsigned	rom	char *wrptr	);
signed char	gpio_drawSector(unsigned char sector);
signed char	readRTC();
signed char	setRTC();
signed char	alarm_storeTime();
unsigned char	time_dec(unsigned	char part,unsigned char	amount);
unsigned char get_dbounce();
unsigned char bcd2dec(unsigned char bcd);
unsigned char dec2bcd(unsigned char dec);
unsigned int timeDiff(unsigned char timeIdx1, unsigned char timeIdx2);


void timer_isr (void);
#pragma	code low_vector=0x18
void
low_interrupt	(void){
	_asm GOTO	timer_isr	_endasm
}
#pragma	code

#pragma	interruptlow timer_isr
void timer_isr (void){
	//int
	signed char rtn;
	unsigned char tmp;
	
	if(INTCONbits.INT0IF){
		//RTC seconds ticker
		time_inc(&times[TNOW],SEC,1);
		gbl_blinkStatus_Min = !gbl_blinkStatus_Min;
		gbl_bTimeChanged = 1;//flag that time has changed (alarm test uses this)
		INTCONbits.INT0IF = 0;	//reset int
	}

	tmp = gbl_sector;
	
	if(INTCONbits.TMR0IF){
		//Timer 0 int
		
		//PWM of Indicator LEDS. 6bit period (63), max duty of 16
		if(
			((gbl_blinky & 0b00111111) <= gbl_light)
			||
			gbl_bAlarmSnd
		){
			//ON
			//pwm for center led
			LED_PWM = 1;
		}else{
			//OFF
			LED_PWM = 0;
		}

		//PWM of Sectors. 5bit period (31), max duty of 16
		if(
			((gbl_blinky & 0b00011111) <= gbl_light)
			||
			gbl_bAlarmSnd
		){
			//ON
			//pwm for sectors already set by code below
		}else{
			//OFF
			LATC &= 0b11111000;		
		}

		//de-Bounce inputs (portB). Every 32th int
		if((gbl_blinky & (unsigned char) 0b00000111) == (unsigned char) 0b00000111){
			set_dbounce();
		}
		
		//Write to MAX6956
		if((gbl_blinky & (unsigned char) 0b00011111) == (unsigned char) 0b00011111){
			//every 16th int			
			if(gbl_sector > (unsigned char) 2){
				gbl_sector = 0;
				if(gbl_ledBuffer_written){
					//new data on other buffer so toggle it, but only after all 3 sectors have been displayed
					if(gbl_ledBuffer_bank) gbl_ledBuffer_bank = 0;
					else gbl_ledBuffer_bank = 1;
					gbl_ledBuffer_written = 0;//flag main() that it can write to buffer again.
				}//if
			}//if
	
			rtn = gpio_drawSector(gbl_sector);	// Write to the GPIO
			if(rtn) fprintf	(_H_USART, "(%#i) ",rtn); //error writing
			//sector counter
			gbl_sector++;
		}
		gbl_blinky++;
		INTCONbits.TMR0IF	=	0;	//reset	timer 0
	}
}//func

void alarm_updateOff(){
	//copies alarm time to alarm off time + 15 mins
	fprintf(_H_USART, "\r\nalarm_updateOff(");
	times[TSNOOZE] = times[TALMON]; //snooze defaults to alarm on
	fprintf(_H_USART, "Snooze %.2u:%.2u:%.2u)\r\n",times[TSNOOZE].hour,times[TSNOOZE].minute,0);
}

void alarm_setSnooze(){
	//set snooze time times[TSNOOZE] and flag
	fprintf(_H_USART, "\r\nalarm_setSnooze(");
	if(gbl_bAlarmSnd){
		//set snooze time
		gbl_bSnooze = 1;
		times[TSNOOZE] = times[TNOW];
		time_inc(&times[TSNOOZE],MIN,SNOOZELIMIT);
		fprintf(_H_USART, "%.2u:%.2u:%.2u",times[TSNOOZE].hour,times[TSNOOZE].minute,times[TSNOOZE].second);
	}else{
		//not alarm sounding, no point
		gbl_bSnooze = 0;
		fprintf(_H_USART, "nada");
	}
	fprintf(_H_USART, ")\r\n");
}

/*
	calculates the difference between time 1 and time 2 (index's supplied)
	timeIdx1 = now, timeIdx2 = then
	Returns result in mins always positive cos could be tomorrow (unsigned int) 
*/
unsigned int timeDiff(unsigned char timeIdx1, unsigned char timeIdx2){
	unsigned int timeDiff;
	unsigned int timeNow;
	unsigned int timeThen;
	timeNow 		= ((unsigned int) times[timeIdx1].hour * 60) + times[timeIdx1].minute; //minutes
	timeThen 	= ((unsigned int) times[timeIdx2].hour * 60) + times[timeIdx2].minute; //minutes
	if(timeNow <= timeThen){
		//then is in the future within the same day.
		timeDiff = timeThen - timeNow;
	}else{
		//then is in the past, so could be tommorrow.
		timeDiff = timeThen + 1440 - timeNow;
	}//if
	//fprintf	(_H_USART, "Now[%#.1u] %#.2u:%#.2u (%u), then[%#.1u] %#.2u:%#.2u (%u), diff:%#.4u\r\n", timeIdx1, times[timeIdx1].hour, times[timeIdx1].minute, timeNow, timeIdx2, times[timeIdx2].hour, times[timeIdx2].minute, timeThen, timeDiff);
	return timeDiff;
}


void alarm_test(){
	//checks current time against alarm
	/*
	fprintf	(_H_USART, "\r\nTest alarm. %b\r\n",gbl_bAlarmSet);
	fprintf	(_H_USART, "Time : %#.2u:%#.2u\r\n", times[TNOW].hour,times[TNOW].minute);
	fprintf	(_H_USART, "Alon : %#.2u:%#.2u\r\n", times[TALMON].hour,times[TALMON].minute);
	fprintf	(_H_USART, "Snz : %#.2u:%#.2u\r\n", times[TSNOOZE].hour,times[TSNOOZE].minute);
	*/
	if(
		gbl_bAlarmSet
		&
		timeDiff((unsigned char) TALMON,(unsigned char) TNOW) < (unsigned char) ALARMLIMIT //diff between now and alarm tigger <= ALARMLIMIT
	){
		gbl_bAlarmSnd = 1; //Alarm sounding (or at least it might be)
		if(timeDiff((unsigned char) TSNOOZE, (unsigned char) TNOW) < (unsigned char) SNOOZELIMIT){
			//within the alarm time and not snoozing, sound alarm
			fprintf	(_H_USART, "\r\nAlarm!\r\n");
			if(times[TNOW].second & 0b00000001) {
				ALARMOUT = 1;  //sound alarm
			}else{
				ALARMOUT = 0;  //un-sound alarm
			}//if
		}//if
	}else{
		gbl_bAlarmSnd = 0;
		ALARMOUT = 0;  //un-sound alarm
	}//if
}//func

void gpio_setGlobalCurrent(unsigned char current){
	unsigned char	rtn;
	//fprintf(_H_USART, "\r\ngpio_setGlobalCurrent(");
	INTCONbits.GIE = 0; //disable ints for i2c write
	rtn = EEByteWrite(MAX6956ADDR,0x02,current & 0b00001111);
	INTCONbits.GIE = 1;
	//fprintf(_H_USART, "%u : %.8b)\r\n",current,current);
}// end gpio_init(void)


void gpio_setSegCurrent(unsigned char current){
	unsigned char	rtn;
	unsigned char idx ;
	unsigned char cur[16];
	unsigned char tmp;
	current &= 0b00001111; //truncate to < 16
	tmp = current << 4;
	for(idx=0;idx < 10;idx++){
		cur[idx] = 0xff;
		switch (idx){
			case 0 : 
			case 5 :
				cur[idx] = current | (cur[idx] & 0b11110000);
				break;
			case 2 :
			case 7 :
				cur[idx] = tmp | (cur[idx] & 0b00001111);
				break;
			default :
				cur[idx] = 0xff;
		}//switch
		fprintf(_H_USART, "\r\n%2.2x",cur[idx]);
	}//for
	fprintf(_H_USART, "\r\n gpio_setSegCurrent(");
	INTCONbits.GIE = 0; //disable ints for i2c write
	//rtn = I2C_sendRomStr(MAX6956ADDR,gpio_start_3);
	rtn = EEPageWrite(MAX6956ADDR,0x16,cur);
	INTCONbits.GIE = 1;
	fprintf(_H_USART, "%2.2x : %u)\r\n",tmp,rtn);
}// end gpio_init(void)

void main(){
	unsigned char	ctr				= 0;	//gp counter
	unsigned char	tmp				= 0;	//temp var
	unsigned char prv_status	= 0b00000000; //toggle watcher
	unsigned char chg_status	= 0b00000000; //toggle watcher
	unsigned char	btnWait0		= 0;	//wait before btnRepeat
	unsigned char	btnWait1		= 0;	//wait before btnRepeat
	unsigned char	btnWait2		= 0;	//wait before btnRepeat
	unsigned char	btnWait3		= 0;	//wait before btnRepeat
	unsigned char	btnWait4		= 0;	//wait before btnRepeat
	signed 		char	rtn				= 0;	//temp var
	unsigned char bDispSecs	= 0; //hold status of gbl_display_secs whilst we temporaily turn it off
	unsigned char	gotADC		=	0;	//have an ADC value to read
	unsigned int 	light				=	0;	//ADC value
	
	boot_up();	//inti the peripherals
	Delay1KTCYx(10);
	gpio_init();	//init the GPIO

	ctr										=	0;// general purpose counter;
	tmp										=	0;
	gbl_bRun								= 1; //Run?
	gbl_timeSelect					= 0; //work with current time times[TNOW]
	gbl_blinkOff_Min					= 0;//allow mins to blink
	gbl_blinkStatus_Min			= 1;//mins currently displayed
	gbl_blinkStatus_Sec			=	1;//secs currently on (if gbl_display_secs == 1)
	gbl_ledBuffer_bank				= 0;//start using bank 0;
	gbl_display_secs					= 1;//display seconds
	gbl_bAlarmSet						= 0;//is the alarm set?
	gbl_ledBuffer_written			= 0;//convertTime2light sets high when new data avalable. isr notices and flips buffer, then sets low. cT2L cannot write more until low again.
	gbl_bAlarmSnd					= 0;//alarm not sounding
	gbl_bSnooze						= 0;//not snoozing
	gbl_PartialHour					= 1;//show partial hours
	gbl_effect.sector					=	0;//start sector for global effects
	while(ctr<3){
		//writeoutput	to buffer
		gbl_effect.buffer[ctr].full = 0;
		gbl_effect.buffer[ctr].full = 0;
		gbl_effect.buffer[ctr].full = 0;
		ctr++;
	}//for
	gbl_effect.mode		= 0; //
	gbl_effect.ctr 		= 0; //dev

	
	//All LED's off
	LED_ALERT				= 1;
	LED_ALARM				= 1;
	LED_AMPM				= 1;
	Delay1KTCYx(0);

	rtn = readRTC();//read RTC current time and alarmtime
	//fprintf	(_H_USART, "\r\nTime  : %#.2u:%#.2u:%#.2u\r\n", times[TNOW].hour,times[TNOW].minute,times[TNOW].second);
	//fprintf	(_H_USART, "\r\nAlarm : %#.2u:%#.2u:%#.2u\r\n", times[TALMON].hour,times[TALMON].minute,times[TALMON].second);
	alarm_updateOff();//update alarm off time

	INTCONbits.GIE 					= 1;	//Start ints

	while(1){
		prv_status = gbl_status; //Store previous status
		gbl_status = get_dbounce();
		chg_status = gbl_status ^ prv_status; //xor current and prev status to flag changes

		//tmp = ctr & 0b00010000;
		//fprintf	(_H_USART, "\r\n%#.3u : %#.3u : %#.3u\r\n",gbl_bAlarmSet,gbl_bAlarmSnd,tmp);

		if(gbl_bRun){
			//LED Status whilst time is running
			//alarm led
			if(gbl_bAlarmSet)	LED_ALARM = 0; //light alarm set LED
			else							LED_ALARM = 1; //led is active low
	
			//alert LED
			if(gbl_bAlarmSnd){
				if((ctr & (unsigned char) 0b00100000) > (unsigned char) 0) LED_ALERT = !LED_ALERT;//blink alarm led if alarm is sounding
			}else LED_ALERT = 1;
		}

		//pm led : Always processed
		if(gbl_bPm) 				LED_AMPM = 0; //light AM/PM LED
		else 							LED_AMPM = 1; //led is active low	
		
		//gbl_blinkStatus_Sec = 0;
		
		if(gbl_bTimeChanged){
			//only check if time has changed. ie every second, but not so time critical as to need an interrupt
			alarm_test();
			gbl_blinkStatus_Sec = 1;
			if(gbl_bAlarmSnd){
				//max brightnes, alarm is sounding
				gbl_light = 24;
				//Cycle effects every other second
				if((times[TNOW].second & 0b00000111) == 0b00000111){
					gbl_effect.ctr 	= 2;
					gbl_effect.mode++;
				}
			}else{
				if(times[TNOW].minute == 0 & times[TNOW].second == 0){
					//spin effect on the hour
					gbl_effect.ctr 	= 1;
					gbl_effect.mode = 3;
				}
				if(!BusyADC()){
					if(!gotADC){
						//rintf	(_H_USART, "Start ADC\r\n");
						ConvertADC(); // Start conversion if not busy
						gotADC = 1;
					}else{
						light = ReadADC(); // Read result
						/*
							Assume range of 0(light) to 1024(dark)
							We have a cut off point of LIGHTCEIL that sets where we start to increase LED brightness
							This allows us to tweak things to ensure our LEDs are not too bright in dimly lit bedrooms.
						*/
						if(light < LIGHTCEIL){
							//Start to increase brightness
							gbl_light = LIGHTMAX - (unsigned char) (light / LIGHTDIV); //end up with value between 0 and 15			
							//fprintf	(_H_USART, "\r\n %2.1u - ( %4.1i / %2.1u ) = %2.1u ",LIGHTMAX, light, LIGHTDIV, gbl_light);
						}else{
							//Minimum brightness
							gbl_light = 0;
						}
						//fprintf(_H_USART, "\r\nL%4.4i:%2.2u",light, gbl_light);
						gotADC = 0;
					}//if
				}//if
			}
			
			//set global brightness once a second
			gbl_bTimeChanged = 0;
		}

		if(chg_status > (unsigned char) 0){
			//There has been a change in status
			//fprintf	(_H_USART, "chng : %.8b\r\n",chg_status);
			if(!gbl_bRun){
				//if not running
				if(chg_status & BTN_ALARMSET){
					//Alarm btn changed status
					if(!(gbl_status & BTN_ALARMSET)){
						//Btn released
						gbl_timeSelect 		= TNOW; //select current time
						gbl_bRun 				= 1; //go run
						btnWait0 				= 0;	//reset button wait
						gbl_display_secs 	= bDispSecs; //restore previous value
						gbl_PartialHour 		= 1; //hours are partial
						gbl_blinkOff_Min 		= 0; //blink minutes
						LED_ALERT 				= 1; //Alert off.
						gbl_effect.ctr 		= 4;
						gbl_effect.mode 	= 5;
						alarm_storeTime();//store alarm time in rtc
						alarm_updateOff();//update alarm off time
					}//if
				}//if
				if(chg_status & BTN_TIMESET){
					//Time btn changed status
					if(!(gbl_status & BTN_TIMESET)){
						//Btn released
						gbl_timeSelect 		= TNOW;//select current time
						gbl_bRun 				= 1; //go run
						gbl_PartialHour 		= 1; //hours are partial
						btnWait1					= 0; //reset button wait
						gbl_blinkOff_Min		= 0; //blink minutes
						gbl_effect.ctr 		= 4;
						gbl_effect.mode 	= 5;
						setRTC();
						fprintf	(_H_USART, "r");
					}//if
				}//if
				if(gbl_status & BTN_HOURADV){
					//Hour++ if not running
					time_inc(&times[gbl_timeSelect],HOUR,1); //inc hour
					//fprintf	(_H_USART, "H");
					btnWait2 = 0;
				}//if
				if(gbl_status & BTN_MINADV){
					//Min++ if not running
					time_inc(&times[gbl_timeSelect],MIN,1); //inc mins
					//fprintf	(_H_USART, "M");
					btnWait3 = 0;
				}//if
			}else{
				//running
				if(chg_status & BTN_ALARMSET){
					//Alarm btn changed status
					if(!(gbl_status & BTN_ALARMSET)){
						//Btn released quickly pressed
						if(gbl_bAlarmSet){
							gbl_bAlarmSet = 0;
							fprintf(_H_USART, "\r\nAlarm off!");
						}else{
							gbl_bAlarmSet = 1;
							fprintf(_H_USART, "\r\nAlarm on!");
						}
					}//if
				}//if
				if(chg_status & BTN_MINADV){
					//min btn changed status
					if(!(gbl_status & BTN_MINADV)){
						//Mins btn released quickly pressed
						gbl_display_secs = !gbl_display_secs;
						fprintf(_H_USART, "\r\nToggle seconds display %u",gbl_display_secs);
					}//if
				}//if
				if(chg_status & BTN_HOURADV){
					//min btn changed status
					if(!(gbl_status & BTN_HOURADV)){
						//Mins btn released quickly pressed
						effect_next();
					}//if
				}//if
				if(chg_status & BTN_SNOOZE){
					//min btn changed status
					if(!(gbl_status & BTN_SNOOZE)){
						//Snooze btn released quickly pressed
						gbl_effect.ctr 	= 2;
						alarm_setSnooze();
					}//if
				}//if
			}//if
		}else{
			//No change
			
			//Alarm set btn changed status
			if(gbl_status & BTN_ALARMSET){
				if(btnWait0 > DLY_LONG){
					//Only trigger after a small delay
					//Btn pressed and time running (ie, not alarm set)
					gbl_timeSelect 		= TALMON;//select alarm
					//gbl_blinkOff_Min	= 1;//don't blink minutes
					if(gbl_bRun){
						fprintf(_H_USART, "Alarm set");
						//blink
						gbl_effect.ctr 		= 2;
						gbl_effect.mode 	= 5;
					}
					LED_ALARM				= 0; //Alarm led on
					gbl_PartialHour 		= 0;//hours are whole
					bDispSecs 				= gbl_display_secs;//store current setting
					gbl_display_secs 	= 0;
					gbl_bRun 				= 0; //Stop run
					btnWait0 				= DLY_LONG_RST;			
				}else{
					btnWait0++;
				}//if
			}//if

			//Time set btn changed status
			if(gbl_status & BTN_TIMESET){
				if(btnWait1 > DLY_LONG){
					//Only trigger after a small delay
					//Btn pressed and time running (ie, not alarm set)
					gbl_timeSelect 		= TNOW;//select time
					if(gbl_bRun){
						fprintf(_H_USART, "Set time");
						//blink
						gbl_effect.ctr 		= 2;
						gbl_effect.mode 	= 5;
					}
					LED_ALERT				= 0; //Alarm led on
					gbl_PartialHour 		= 0;//hours are whole
					gbl_bRun 				= 0; //Stop run
					btnWait1 				= DLY_LONG_RST;
				}else{
					btnWait1++;
				}//if
			}//if

			if(!gbl_bRun){
				//We're in set mode
				if(gbl_status & BTN_HOURADV){
					//Hours++ if not running
					if(btnWait2 > DLY_SHORT){
						//Only trigger after a small delay
						time_inc(&times[gbl_timeSelect],HOUR,1); //inc hours
						fprintf	(_H_USART, "h");
						btnWait2 = DLY_SHORT_RST;//reset to an even small delay
					}else{
						btnWait2++;
					}//if
				}//if
				if(gbl_status & BTN_MINADV){
					//Mins++ if not running
					gbl_blinkOff_Min		= 1;//no blink whilst moving
					gbl_blinkStatus_Min	= 1;//display mins
					//fprintf	(_H_USART, " %3.u\r\n",btnWait3);
					if(btnWait3 > DLY_SHORT){
						//Only trigger after a small delay
						time_inc(&times[gbl_timeSelect],MIN,1); //inc mins
						//fprintf	(_H_USART, "m");
						btnWait3 = DLY_SHORT_RST;//reset to an even small delay
					}else{
						btnWait3++;
					}//if
				}//if
			}else{

			}//if
		}//if
		
		if(gbl_effect.ctr > 0){
			//Efects
			effect_run();
			Delay1KTCYx(5);
		}else{
			//Just the time
			convertTime2light();
			Delay10KTCYx(3);
		}
		
		//seconds LED (set to off, set on briefly each second)
		//if((ctr & 0b00000001) > 0) gbl_blinkStatus_Sec = 0;
		ctr++;
		//Delay10KTCYx(1);
	}//end while
}

signed char	gpio_drawSector(unsigned char sector){
	//send 3 bytes to	the	GPIO (one	complete sector)
	char rtn;
	char bank;
	//we always read from the other buffer
	if(gbl_ledBuffer_bank) bank = 0;
	else bank = 1;
	LATC	&= 0b11111000; //zero the first three bits of portc (sector selects)
	rtn = EEByteWrite(MAX6956ADDR,sectorAddr[0],gbl_ledBuffer[bank][sector][0]);
	rtn = EEByteWrite(MAX6956ADDR,sectorAddr[1],gbl_ledBuffer[bank][sector][1]);
	rtn = EEByteWrite(MAX6956ADDR,sectorAddr[2],gbl_ledBuffer[bank][sector][2]);
	LATC	|= (1 <<	sectorSelBits[sector]);
	return rtn;
}

signed char	readRTC(){
	unsigned char tmp[RTCLEN];
	unsigned char ctr;
	//Reads the time and also the alarm
	fprintf(_H_USART, "\r\nRead time\r\n");
	INTCONbits.GIE = 0;
	EESequentialRead(RTCADDR,0x00,tmp,7);
	INTCONbits.GIE = 1;
	for(ctr=0;ctr<RTCLEN;ctr++){
		if(ctr == 2) 	times[TNOW].arr[ctr] = bcd2dec(tmp[ctr] & 0b00111111);	//hours, uses special mask
		else					times[TNOW].arr[ctr] = bcd2dec(tmp[ctr]);	//everyone else
		fprintf(_H_USART, "%#u : %.8b : %#.2u \r\n",ctr,times[TNOW].arr[ctr],times[TNOW].arr[ctr]);
	}
	fprintf(_H_USART, "\r\nRead alarm\r\n");
	INTCONbits.GIE = 0;
	EESequentialRead(RTCADDR,0x08,tmp,3);
	INTCONbits.GIE = 1;
	for(ctr=0;ctr<RTCLEN;ctr++){
		if(ctr == 2) 	times[TALMON].arr[ctr] = bcd2dec(tmp[ctr] & 0b00111111);	//hours, uses special mask
		else					times[TALMON].arr[ctr] = bcd2dec(tmp[ctr]);	//everyone else
		fprintf(_H_USART, "%#u : %.8b : %#.2u \r\n",ctr,times[TALMON].arr[ctr],times[TALMON].arr[ctr]);
	}
	fprintf(_H_USART, "End\r\n");
	return ( 0 );										// return with no error
}

signed char	setRTC(){
	signed int tmp;
	fprintf(_H_USART, "\r\nsetRTC()\r\n");
	INTCONbits.GIE = 0;
	tmp = EEByteWrite(RTCADDR,0x07,0b00010000);	//set control reg. sqw out 1hz
	fprintf(_H_USART, "Set CTRL : %u \r\n",tmp);
	tmp = EEByteWrite(RTCADDR,0x02,dec2bcd(times[TNOW].hour) & 0b00111111);	//set hours and 24hr mode (bit6 = 0)
	fprintf(_H_USART, "Set HRS : %u \r\n",tmp);
	tmp = EEByteWrite(RTCADDR,0x01,dec2bcd(times[TNOW].minute));	//set mins
	fprintf(_H_USART, "Set MIN : %u \r\n",tmp);
	tmp = EEByteWrite(RTCADDR,0x00,0x00);	//set seconds and run clock (bit7 = 0)
	fprintf(_H_USART, "Set SEC : %u \r\n",tmp);
	INTCONbits.GIE = 1;
	fprintf(_H_USART, "End\r\n");
	return ( 0 );										// return with no error
}

signed char	alarm_storeTime(){
	signed int tmp;
	//Stores alarm time in RTC eeprom space also stores alarm off time
	fprintf(_H_USART, "\r\nalarm_storeTime(");
	INTCONbits.GIE = 0;
	tmp = EEByteWrite(RTCADDR,0x08,0x00);	//set seconds always 0
	tmp = EEByteWrite(RTCADDR,0x09,dec2bcd(times[TALMON].minute));	//set mins
	tmp = EEByteWrite(RTCADDR,0x0a,dec2bcd(times[TALMON].hour) & 0b00111111);	//set hours and 24hr mode (bit6 = 0)
	INTCONbits.GIE = 1;
	fprintf(_H_USART, "%.2u:%.2u:%.2u)\r\n",times[TALMON].hour,times[TALMON].minute,0);
	return ( 0 );										// return with no error
}

unsigned char dec2bcd(unsigned char dec){
	unsigned int temp;
	//just two digits.
	temp = dec / 10;
	temp <<= 4;
	dec %= 10;
	return (temp | dec);
}

void effect_copyTimeBuffer(){
	//Copies the current time buffer to the Effects buffer. For efects that involve flinging the time around.
	gbl_effect.buffer[0].low 		= gbl_ledBuffer[gbl_ledBuffer_bank][0][0];
	gbl_effect.buffer[0].mid 		= gbl_ledBuffer[gbl_ledBuffer_bank][0][1];
	gbl_effect.buffer[0].high		= gbl_ledBuffer[gbl_ledBuffer_bank][0][2];
	gbl_effect.buffer[1].low 		= gbl_ledBuffer[gbl_ledBuffer_bank][1][0];
	gbl_effect.buffer[1].mid 		= gbl_ledBuffer[gbl_ledBuffer_bank][1][1];
	gbl_effect.buffer[1].high		= gbl_ledBuffer[gbl_ledBuffer_bank][1][2];
	gbl_effect.buffer[2].low 		= gbl_ledBuffer[gbl_ledBuffer_bank][2][0];
	gbl_effect.buffer[2].mid 		= gbl_ledBuffer[gbl_ledBuffer_bank][2][1];
	gbl_effect.buffer[2].high		= gbl_ledBuffer[gbl_ledBuffer_bank][2][2];
}


effect_clearBuffer(){
	//zeros the effects buffer
	gbl_effect.buffer[0].full = 0;
	gbl_effect.buffer[1].full = 0;
	gbl_effect.buffer[2].full = 0;
}

void effect_dotForward(){
	if(gbl_effect.buffer[gbl_effect.sector].full == 0){
		gbl_effect.buffer[gbl_effect.sector].full = 0x000001;
	}else{
		gbl_effect.buffer[gbl_effect.sector].full	<<= 1;
		if(gbl_effect.buffer[gbl_effect.sector].full > 0x080000){
			//end of sector
			gbl_effect.buffer[gbl_effect.sector].full = 0;
			gbl_effect.sector ++;
			//gbl_effect.buffer[gbl_effect.sector] = 0;
			if(gbl_effect.sector > 2){
				//one complete rev completed.
				gbl_effect.sector = 0;
				gbl_effect.ctr --;		
			}
		}
	}
}

void effect_dotBackward(){
	unsigned char notSector = 2 - gbl_effect.sector; //the opposite
	if(gbl_effect.buffer[notSector].full == 0x0){
		gbl_effect.buffer[notSector].full = 0x080000;
	}else{
		gbl_effect.buffer[notSector].full	>>= 1;		
		if(gbl_effect.buffer[notSector].full == 0){
			gbl_effect.buffer[notSector].full = 0;
			gbl_effect.sector ++;
			if(gbl_effect.sector > 2){
				//one complete rev completed.
				gbl_effect.sector = 0;
				gbl_effect.ctr --;		
			}
		}
	}
}

void effect_rotateCW(){
	unsigned char cary = 0; //my carry bit
	//get the highest bit
	cary = gbl_effect.buffer[0].high & 0b00001000;
	cary >>= 1;
	//rotate
	gbl_effect.buffer[0].full <<= 1;

	//get the highest bit
	cary |= gbl_effect.buffer[1].high & 0b00001000;
	cary >>= 1;
	//rotate
	gbl_effect.buffer[1].full <<= 1;

	//get the highest bit
	cary |= gbl_effect.buffer[2].high & 0b00001000;
	//rotate
	gbl_effect.buffer[2].full <<= 1;

	cary >>= 1;

	//put the carried bits back
	gbl_effect.buffer[1].low |= (cary & 0b00000001);
	cary >>= 1;
	gbl_effect.buffer[2].low |= (cary & 0b00000001);
	cary >>= 1;
	gbl_effect.buffer[0].low |= (cary & 0b00000001);


	gbl_effect.sector--;
	if(gbl_effect.sector == 0){
		//one complete rev completed.
		gbl_effect.sector = 60;
		gbl_effect.ctr --;	
	}
}

void effect_flood(){
	//Fills in the face from top to bottom.
	unsigned short long buf0	= 0x0; //my carry bit
	unsigned short long buf1	= 0x0; //my carry bit





	if(gbl_effect.temp == 0){
		//gbl_effect.buffer[1].mid |= 0b00000100;
	}
	//if(gbl_effect.temp <= 31) {
		//bottom
		buf0 = gbl_effect.buffer[1].full & 0b000000000000011111111111;
		buf1 = gbl_effect.buffer[1].full & 0b000011111111110000000000;
		buf0 >>= 1;
		buf1 <<= 1;
		gbl_effect.buffer[1].full = buf0 | buf1;
	//}
	if(gbl_effect.temp > 11) {
		gbl_effect.buffer[0].full >>= 1;
		gbl_effect.buffer[2].full <<= 1;
		if(gbl_effect.temp < 40){
			gbl_effect.buffer[0].full |= 0b000010000000000000000000;
			gbl_effect.buffer[2].full |= 0b000000000000000000000001;
		}
	}
	if(gbl_effect.temp < 31){
		//fill bottom dot
		gbl_effect.buffer[1].mid |= 0b00000100;
	}

	gbl_effect.temp++;

	if(gbl_effect.temp > 61){
		//one complete rev completed.
		gbl_effect.temp = 0;
		gbl_effect.ctr --;	
	}
}

void effect_flash(){
	if((gbl_effect.ctr & 0b00000001) == 0b00000001){
		//all off
		gbl_effect.buffer[0].full = 0x0;
		gbl_effect.buffer[1].full = 0x0;
		gbl_effect.buffer[2].full = 0x0;
		fprintf(_H_USART, "\r\nFlash Off (%u)",gbl_effect.temp);
	}else{
		//all on
		gbl_effect.buffer[0].full = 0xffffff;
		gbl_effect.buffer[1].full = 0xffffff;
		gbl_effect.buffer[2].full = 0xffffff;
		fprintf(_H_USART, "\r\nFlash On (%u)",gbl_effect.temp);
	}	
	gbl_effect.ctr--;
/*
	if(gbl_effect.temp == 0){
		//one complete rev completed.
		gbl_effect.ctr --;	
	}
*/
}


void effect_next(){
	//Used to demo effects.
	//Important to ensure that the limit matches the numer of available effects in effect_run()
	gbl_effect.mode++;
	gbl_effect.ctr = 2;
	fprintf(_H_USART, "\r\nEffect mode %u ",gbl_effect.mode);
	if(gbl_effect.mode >= 10){
		gbl_effect.mode -= 10;
	}
	fprintf(_H_USART, "\r\nEffect mode %u ",gbl_effect.mode);
}

void effect_run(){
	//Creates special effects for particular events
	unsigned char	s			=	0;	//sector count
	if(gbl_ledBuffer_written) return;//only run when not already written

/*
	fprintf(_H_USART
		, "\r\nc%2.1u m%2.1u r%2.1u "
		, gbl_effect.ctr
		, gbl_effect.mode
		, gbl_effect.sector
	);
*/
	
	switch(gbl_effect.mode) {
		case 0 :
			//Setup : single dot forward
			gbl_effect.sector = 0;
			gbl_effect.buffer[gbl_effect.sector].full = 0x000000;
			gbl_effect.mode += 10; //switch to run mode	
		case 10 :
			//single dot forward
			effect_dotForward();
			break;

		case 1 :
			//Setup : single dot reverse
			gbl_effect.sector = 0;
			gbl_effect.buffer[gbl_effect.sector].full = 0x000000;
			gbl_effect.mode += 10; //switch to run mode	
		case 11 :
			//single dot reverse
			effect_dotBackward();
			break;
	
		case 2 :
			//Setup : bounce
			gbl_effect.sector = 0;
			gbl_effect.buffer[gbl_effect.sector].full = 0x000000;
			gbl_effect.mode += 10; //switch to run mode	
		case 12 :
			//bounce
			if(gbl_effect.ctr & 0b00000001){
				effect_dotForward();
			}else{
				effect_dotBackward();
			}
			break;
			
		case 3 :
			//setup : rotate Clockwise
			effect_copyTimeBuffer();
			gbl_effect.sector	= 60; //count for one rotation
			gbl_effect.temp 		= 0; //count for one rotation
			gbl_effect.mode += 10; //switch to run mode	
			effect_copyTimeBuffer();
		case 13 :
			//rotate
			effect_rotateCW();
			break;
			
		case 4 :
			//Flood/Drain : Drips in from top to bottom. Alternate sides.
			gbl_effect.sector 	= 30; //count for a complete fill/empty
			gbl_effect.mode += 10; //switch to run mode	
			effect_clearBuffer();
			break;
		case 14 :
			//Flood
			effect_flood();
			break;

		case 5 :
			//Flash
			effect_clearBuffer();
			gbl_effect.temp	 	= 4; //count for a complete fill/empty
			gbl_effect.mode 	+= 10; //switch to run mode	
			break;
		case 15 :
			//Flash
			effect_flash();
			Delay1KTCYx(70);//wait for stable
			break;

		case 7 :
			//All on
			gbl_effect.buffer[0].full = 0xffffff;
			gbl_effect.buffer[1].full = 0xffffff;
			gbl_effect.buffer[2].full = 0xffffff;
			break;
			
		default :
			//catches any unknowns and resets to effect 0
			gbl_effect.mode = 0;
			break;	
			
	}//switch

	
/*
	fprintf(_H_USART
		, " %.8b%.8b%.8b"
		, gbl_effect.buffer[gbl_effect.sector].high
		, gbl_effect.buffer[gbl_effect.sector].mid
		, gbl_effect.buffer[gbl_effect.sector].low
	);
*/
	while(s<3){
		//writeoutput	to buffer
		gbl_ledBuffer[gbl_ledBuffer_bank][s][0]	=	gbl_effect.buffer[s].low;
		gbl_ledBuffer[gbl_ledBuffer_bank][s][1]	=	gbl_effect.buffer[s].mid;
		gbl_ledBuffer[gbl_ledBuffer_bank][s][2]	=	gbl_effect.buffer[s].high;
		s++;
	}//for
	gbl_ledBuffer_written = 1;//flag to isr that new data is available
}

void convertTime2light(){
	//Converts current time	to correct bits	in gbl_ledBuffer[x][x]
	//three	times, sec,	min, hour
	//sets flag to say that we have changed data in gbl_ledBuffer
	//need to	either mod	the	logic	to bracket the test	for	min/sec/hr or	add	some flag	to say that	min/sec/hr has been	accoutned	for	already.
	unsigned char	s			=	0;	//sector count
	unsigned char	tmp		=	0;
	unsigned char	tmp2	=	0;
	unsigned char	ubnd	=	0;
	unsigned char	lbnd		=	0;
	unsigned short long	tmp24bit	=	0xa0e010;	//24bit	temp var for leds	(only	use	20 Lsbs)
	unsigned short long	out24bit	=	0x000000;	//24bit	temp var for leds	(only	use	20 Lsbs)
	if(gbl_ledBuffer_written) return;//only run when not already written
	//calculate hour LED position including mins
	while(s<3){
		lbnd	=	20*s;
		ubnd	=	lbnd + 20;

		//blank	all	leds in	temp current sector
		out24bit		=	0x000000;//reset out
		gbl_ledBuffer[gbl_ledBuffer_bank][s][0]	=	0x00;
		gbl_ledBuffer[gbl_ledBuffer_bank][s][1]	=	0x00;
		gbl_ledBuffer[gbl_ledBuffer_bank][s][2]	=	0x00;

		//seconds
		if(gbl_display_secs && gbl_blinkStatus_Sec){
			if((times[gbl_timeSelect].second >=	lbnd &&	times[gbl_timeSelect].second	<	ubnd)){
				tmp24bit		=	0x000001;//reset temp
				tmp					=	times[gbl_timeSelect].second	-	lbnd;
				tmp24bit <<= tmp;	//shift	the	lit	bit	into the correct pos.
				//Write	temp to	out
				out24bit |=	tmp24bit;
			}//if
		}//if

		//minutes
		if(gbl_blinkOff_Min | gbl_blinkStatus_Min){
			//only process (and therefor display) if blink is on or we're not blinking
			if(times[gbl_timeSelect].minute >= lbnd && times[gbl_timeSelect].minute < ubnd){
				tmp24bit	=	0x000001;//reset temp
				tmp			=	times[gbl_timeSelect].minute - lbnd;
				tmp24bit 	<<= tmp;	//shift	the	lit	bit	into the correct pos.
				//Write	temp output	to buffer	(logical OR	so as	not	to overwite	the	secs)
				out24bit 	|= tmp24bit;
			}//if
		}//if

		//hours	(multiply	by 5)
		if(
			(
				(times[gbl_timeSelect].hour < 12 && ((times[gbl_timeSelect].hour *5) >= lbnd && (times[gbl_timeSelect].hour *5) < ubnd))
				||
				(times[gbl_timeSelect].hour >= 12 && (((times[gbl_timeSelect].hour-12)*5) >= lbnd && ((times[gbl_timeSelect].hour-12) *5) < ubnd))
			)
		){
			tmp24bit	=	0x000001;//reset temp
			tmp			=	(times[gbl_timeSelect].hour*5) - lbnd;
			if(times[gbl_timeSelect].hour	>= 12) tmp -= 60;
			if(gbl_PartialHour) tmp += times[gbl_timeSelect].minute / 12; //add a few dots for fraction of hour if we are showing partials
		
			tmp24bit <<= tmp;	//shift the lit bit into the correct pos.
			//Write	temp output to buffer (logical OR so as not to overwite the mins)
			out24bit |=	tmp24bit;
		}//if
		//writeoutput	to buffer
		gbl_ledBuffer[gbl_ledBuffer_bank][s][0]	=	LoByte(out24bit);
		gbl_ledBuffer[gbl_ledBuffer_bank][s][1]	=	HiByte(out24bit);
		gbl_ledBuffer[gbl_ledBuffer_bank][s][2]	=	MidByte(out24bit);
		s++;
	}//for
	if(times[gbl_timeSelect].hour > 11)	gbl_bPm = 1;
	else gbl_bPm = 0;
	gbl_ledBuffer_written = 1;//flag to isr that new data is available
	//RC1 = gbl_ledBuffer_written;
}

void boot_up(void){
	//Setup Ports
	CMCON			= 0b00000111;	//Turn off comparator on RA port ]
	LATA				= 0b00000000;
	PORTB				= 0b00000000;
	LATC				= 0b00000000;
	TRISA				= 0b00000001;	// 0= out, 1 = in
	TRISB				= 0b00111111;	// 0= out, 1 = in
	TRISC				= 0b00011000;	// 0= out, 1 = in
	ADCON1			= 0b00001111;	// set portA bits0-3 to digital IO
	SCL					= 1;	// Ensure SCL is high
	DDRCbits.RC3	= 1;	// Configure SCL as Input
	DDRCbits.RC4	= 1;	// Configure SDA as Input
	//Open the USART configured as 8N1, 115200 baud, in polled mode
	OpenUSART	(USART_TX_INT_OFF	&
						 USART_RX_INT_ON &
						 USART_ASYNCH_MODE &
						 USART_EIGHT_BIT &
						 USART_CONT_RX &
						 USART_BRGH_HIGH, 10);
	//setup i2c :
	OpenI2C(MASTER,	SLEW_OFF);
	SSPADD = 49;	//49 = 100KHz at 20MHz : 15 is nice for GPIO

	//setup Timer0 : Use for GPIO output
	OpenTimer0(TIMER_INT_ON & T0_8BIT & T0_SOURCE_INT & T0_PS_1_4);
	//OpenTimer0(TIMER_INT_ON & T0_8BIT & T0_SOURCE_INT & T0_PS_1_64);
	
	//setup ADC
	//OpenADC(ADC_FOSC_64 & ADC_RIGHT_JUST & ADC_20_TAD, ADC_CH0 & ADC_INT_OFF & ADC_VREFPLUS_VDD & ADC_VREFMINUS_VSS,14);
OpenADC(ADC_FOSC_64 & ADC_RIGHT_JUST & ADC_20_TAD, ADC_CH0 & ADC_INT_OFF & ADC_REF_VDD_VSS,14);


	//Disable ints for the mo
	INTCONbits.GIE = 0;	//Stop	ints
	Delay10KTCYx(10);//wait for stable
	putrsUSART ((const far rom char	*)"\r\n\r\n\r\n\r\n////////////////\r\nBootup completed\r\n");
}

//Reads MAX6956 config
void gpio_readConfig(unsigned char addr, unsigned char length){
	unsigned char tmp[20];
	unsigned char idx = 0;
	INTCONbits.GIE 					= 0;	//No ints
	EESequentialRead(MAX6956ADDR,addr,tmp,length);
	INTCONbits.GIE 					= 1;	//Ints
	fprintf(_H_USART, "\r\nGPIO config from %#2.2x", addr);
	
	if(length > 1)
		fprintf(_H_USART, " to %#2.2x \r\n", addr + length);
	else
		fprintf(_H_USART, "\r\n");
		
	for(idx=0;idx < length;idx++){
		fprintf	(_H_USART, "  %#2.2x = %#2.2x %8.8b\r\n",addr + idx,tmp[idx],tmp[idx]);
	}
}

void gpio_init(void){
	unsigned char	rtn;
	//INTCONbits.GIE 					= 0;	//No ints
	
	putrsUSART ((const far rom char	*)"\r\nGPIO	Init : Start\r\n");
	//setup
	rtn	=	I2C_sendRomStr(MAX6956ADDR,gpio_start_0);
	fprintf	(_H_USART, "rtn	%#u\r\n",	rtn);
	
	//set global current to 100%
	rtn	=	I2C_sendRomStr(MAX6956ADDR,gpio_start_1);
	fprintf	(_H_USART, "rtn	%#u\r\n",	rtn);

	//set all outs as LED
	rtn	=	I2C_sendRomStr(MAX6956ADDR,gpio_start_2);
	fprintf	(_H_USART, "rtn	%#u\r\n",	rtn);
	Delay10KTCYx(100);
	
	//set segment currrents
	rtn	=	I2C_sendRomStr(MAX6956ADDR,gpio_start_3);
	fprintf	(_H_USART, "rtn	%#u\r\n",	rtn);
	
	//Test mode
	rtn	=	I2C_sendRomStr(MAX6956ADDR,gpio_testModeOn);
	fprintf	(_H_USART, "rtn	%#u\r\n",	rtn);
	LATC	|= 0b00000111;
	Delay10KTCYx(255);
	rtn	=	I2C_sendRomStr(MAX6956ADDR,gpio_testModeOff);
	LATC	&= 0b11111000;
	Delay10KTCYx(10);
/*
	gpio_readConfig(0x04,1);
	gpio_readConfig(0x09,7);
	gpio_readConfig(0x02,1);
	gpio_readConfig(0x16,10);
*/

	putrsUSART ((const far rom char	*)"\r\nGPIO Init : End\r\n");
}// end gpio_init(void)

// Write to the GPIO
signed char	I2C_sendRomStr(unsigned	char addr, unsigned	rom	char *wrptr	){
	//sends a rom string, first char is len of str-1s
	char sLen	=*wrptr++;
	//INTCONbits.GIE = 0;							//disable int whilst using i2c
	IdleI2C();											// ensure	module is	idle
	StartI2C();											// initiate START condition
	while	(SSPCON2bits.SEN);				// wait	until	start condition is over
	if(WriteI2C(addr)) return	(-3);		// Write	address
	do{
		//Output to	i2c
		IdleI2C();
		if(WriteI2C(*wrptr))		return ( -3	);	// return with write collision error
		if (SSPCON2bits.ACKSTAT	)	return (-4);
		if (PIR2bits.BCLIF)	return ( -1	);	// return with Bus Collision error
		//Inc counters
		wrptr++;
		sLen --;
	}	while	(sLen	>	0);
	IdleI2C();											// ensure	module is	idle
	StopI2C();											// send STOP condition
	while	(	SSPCON2bits.PEN	);			// wait	until	stop condition is	over
	//INTCONbits.GIE = 1;							// re-enable int whilst using i2c
	return ( 0 );										// return with no error
}

void set_dbounce(){
	//de-Bounce inputs (portB.1.2.3.4)
	gbl_dbounce_buff[gbl_dbounce_idx] = ~PORTB; //put not portb state into the ring buffer : We only really need 1st 4 bits
	gbl_dbounce_idx++;
	if(gbl_dbounce_idx >= DBOUNCE_LEN) gbl_dbounce_idx = 0; //reset de-bounce ctr
}

unsigned char get_dbounce(){
	unsigned char ctr,tmp;
	tmp = 0xff;
	//fprintf (_H_USART, "\r\n");
	for(ctr=0; ctr<DBOUNCE_LEN;ctr++){
		tmp &= gbl_dbounce_buff[ctr];
		//fprintf (_H_USART, "%.8B\r\n", gbl_dbounce_buff[ctr]);
	}
	tmp &= DBOUNCEMASK; //we only want certain bits
	//fprintf (_H_USART, "%.8B\r\n", tmp);
	return tmp;
}

void time_inc(union time_t *theTime, unsigned char part, unsigned char amount){
	//Increments time part by amount
	//part [0] seconds, [1] minutes, [2], hours
	unsigned char	temp;
	//fprintf (_H_USART, "time_inc(%#u,%#u)\r\n",	part,amount);
	temp = (unsigned char) theTime->arr[part];
	temp +=	amount;
	if(part	== 2){
		//hours
		if(temp >	23)	temp -=	24;
	}else{
		//minutes	and seconds
		if(temp	>	59){
			temp -=	60;
			if(gbl_bRun) time_inc(theTime, part+1, 1);	//inc	next part
		}//if
	}//if
	theTime->arr[part] = (unsigned char) temp;
}//func

unsigned char bcd2dec(unsigned char bcd){
	unsigned char dec;
	dec = bcd & 0b00001111;	//units
	bcd = (bcd >> 4) * 10;	//tens
	return(bcd+dec);
}//func
