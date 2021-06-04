/*
*	An example project implementing a simple modbus slave device using an
*	ATmega328P running at 8MHz.
*	Baudrate: 9600, 8 data bits, 1 stop bit, no parity
*	Your busmaster can read/write the following data:
*	coils: 0 to 7
*	discrete inputs: 0 to 7
*	input registers: 0 to 3
*	holding registers: 0 to 3
*	LCD via I2C, BMP via I2C, RTC via I2C
*	DS18b20, buttons
*	0- sensor group - BMP180
*	1-3 sensors group - DS18b20 1-6
*	PD2 - button
*/

#include <stdio.h>
#include <stdint.h>
#include <avr/io.h>
#include <util/delay.h>
#include <stdbool.h>
#include <avr/interrupt.h>
#include <string.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
//#include <avr/power.h>
//#include <avr/sleep.h> 

#include "yaMBSiavr.h"
#include "i2c_n.h"
#include "rtcds1307.h"
#include "lcdpcf8574/lcdpcf8574.h"	//i2c addr 3F 
#include "pcf8574_1.h"
#include "ds18b20.h"
#include "bmp180.h"

// address DS1307 7-bit 1101000 R/W - 0x68
#define DS1307_R 0xD1	
#define DS1307_W 0xD0	//8-bit address
#define MAX_STRLEN 10
//for i2c bus
#define MAXDEVICEID 7
#define clientAddress 0x0B
#define	HOLDING_REG	30					//size of holding reg

#define ADR_thermostat1	 	0x02		//adr in EEPROM
#define ADR_thermostat2	 	0x04		//adr in EEPROM
#define ADR_delay_read		0x06		//adr in EEPROM
#define	MAX_TEMP	950					// max temperature for thermostat - decicelsius

#define	IN_TEMP		7
#define	IN_PRESS	6
#define	GRP_SEN		10					// change view of group sensors
#define Therm1_EE	12					// adr of holding reg
#define Therm2_EE	13					// adr of holding reg

#define wdt_reset()   __asm__ __volatile__ ("wdr")
#define 	WDTO_2S   7
#define 	WDTO_4S   8
#define 	WDTO_8S   9



//global var
volatile uint8_t instate = 0;
volatile uint8_t outstate = 0;
volatile uint16_t inputRegisters[4];
volatile uint16_t holdingRegisters[HOLDING_REG];
uint8_t second,minute,hour,day,date,month,year,rtc_present=1;
unsigned char chyba1=0,chyba2=0,chyba3=0,chyba4=0,chyba5=0,chyba6=0;
unsigned char ErrFlag=0;
uint8_t Device1Presence=1,Device2Presence=1,Device3Presence=1,Device4Presence=1,Device5Presence=1,Device6Presence=1;
int CurrentTemp1=0,CurrentTemp2=0,CurrentTemp3=0,CurrentTemp4=0,CurrentTemp5=0,CurrentTemp6=0;
int32_t temperature = 0,pressure = 0; 
uint8_t led = 0; //display ON
volatile uint8_t tick=0,once=1,LCDledtick=0;
uint8_t thermostat1_eeprom,thermostat2_eeprom,delay_read_eeprom; //24dec18hex,28dec1C,6dec6hex
uint8_t termostat2, termostat1;

// declaration functions prototypes
void timer0100us_start(void);
void SetOuts(volatile uint8_t in);
uint8_t ReadIns(void);
void io_conf(void);
void modbusGet(void);
void rtc_read_clock(char read_clock_address);
void rtc_read_cal(char read_calendar_address);
void rtc_clock_write(char set_hour, char set_minute);
void rtc_hour_w(uint8_t set_hour);
void rtc_minute_w(uint8_t set_min);
void rtc_dow_w(uint8_t set_dow);
void rtc_year_w(uint8_t set_year);
void rtc_day_w(uint8_t set_date);
void rtc_month_w(uint8_t set_month);
void rtc_calendar_write(char set_date, char set_month, char set_year);
void timer_tick();
void ds18b20readtemperature(int snimac);
void ds18b20presence(void);
void ds18b20ports(void);
void print_row_LCD(uint16_t num_sensors_group);

void Ftermostat1(void);
void Ftermostat2(void);


// interrupts
ISR(TIMER0_OVF_vect) { //this ISR is called 9765.625 times per second
	modbusTickTimer();
}

ISR (TIMER1_OVF_vect){
	// citac/casovac1 - 16bit
  	// nastavenie poèiatoènej hodnoty poèítadla
  	//TCNT1 = 34286;
  	//TCNT1 = 3036; // 8sekund, pre delicku 1024
  	//TCNT1 = 57724;	// 1 sekunda pre 1024
 	TCNT1 = 49910; //2 sec
	//TCNT1 = 26473; //5 sec
	tick++;LCDledtick++;
	once = 1;
	// 	preteèenie registra TCNT1
	// 	preddelièku èítaèa/èasovaèa1 nastav na 256 (1/8MHz=0.125->0.125*256=32us)
	// 	1s/32us = 31250 impulzov,65536 – 31250 = 34286  
	//	1s/128us = 7812 impulzov, 65536 – 7812 = 57723
	//	2s/128us = 15625 imp, 65536 - 15625 = 49910
	//	5s/0.000128 = 39062, 65536 – 39062 = 26473 
	//	6s/128us =  46875,	65536 – 46875 = 18661
	//	8s/128us = 62500, 	65536 –62500 = 3036
	//	PORTD ^= (1 << PD7); //neguj PD7  
}  


int main(void)
{
	//int j;
	
	
	char CharBuffer[21];
	uint8_t errorCode = 0;
	

	io_conf();
	i2cSetup();
	ds18b20ports();
	bmpCalibrate(&errorCode);
	sei();
	//init lcd
    lcd_init(LCD_DISP_ON);
    //lcd go home
    lcd_home();
	lcd_led(led); //set led
	
	lcd_clrscr();
    lcd_gotoxy( 0, 0);	//column stlpec, row  riadok
	sprintf( CharBuffer, "modbus RTU, adr %d",clientAddress);
	lcd_puts(CharBuffer);
	lcd_gotoxy( 0, 1);
	sprintf( CharBuffer, "9600_8_N_1");
	lcd_puts(CharBuffer);
	lcd_gotoxy( 0, 2);	//column stlpec, row  riadok
	lcd_puts_P( "i2c RTC BMP DS");
	
	if(rtc_present==1){
			rtc_read_clock(0);	//read hours,min,sec.
			rtc_read_cal(3);	// read date
			}
		sprintf( CharBuffer, "%02x:%02x %02x/%02x/20%02x",(hour & 0b00011111),minute,date,month,year);
		lcd_gotoxy( 0, 3);	//column, row
		lcd_puts(CharBuffer);

	thermostat1_eeprom = eeprom_read_word((uint16_t*)ADR_thermostat1);   
	thermostat2_eeprom = eeprom_read_word((uint16_t*)ADR_thermostat2);   
	delay_read_eeprom = eeprom_read_word((uint16_t*)ADR_delay_read);
	termostat1 = thermostat1_eeprom;
	holdingRegisters[Therm1_EE]= thermostat1_eeprom;
	termostat2 = thermostat2_eeprom;
	holdingRegisters[Therm2_EE]= thermostat2_eeprom;
	delay_read_eeprom = delay_read_eeprom;

	modbusSetAddress(clientAddress);
	modbusInit();
		
    wdt_enable(WDTO_4S);
	timer0100us_start();
	
	timer_tick();
	ds18b20presence();

	ds18b20readtemperature(1);
	ds18b20readtemperature(2);
	ds18b20readtemperature(3);
	ds18b20readtemperature(4);
	ds18b20readtemperature(5);
	ds18b20readtemperature(6);

	holdingRegisters[0]= CurrentTemp1;
	holdingRegisters[1]= CurrentTemp2;
	holdingRegisters[2]= CurrentTemp3;
	holdingRegisters[3]= CurrentTemp4;
	holdingRegisters[4]= CurrentTemp5;
	holdingRegisters[5]= CurrentTemp6;
	bmpComputePressureAndTemperature(&temperature, &pressure, &errorCode);
	holdingRegisters[IN_PRESS]=pressure/100;
	holdingRegisters[IN_TEMP]=temperature;
	
	print_row_LCD(0);	//default view
	lcd_led(1);					// switch off LCD

    while(1)
    {
		
		if((PIND & (1<<PIND2))){led=0;lcd_led(led);LCDledtick=0;}
		
		//if(LCDledtick==1){lcd_led(1);}
		if(LCDledtick>3){led=1;lcd_led(led);LCDledtick=0;}
		
		modbusGet();
		Ftermostat1(); 		//check the change of termostat value and write to EEprom
		Ftermostat2();
				
		switch(tick)
			{
		case 1:{if(once == 1){
			
				print_row_LCD(holdingRegisters[GRP_SEN]);
				holdingRegisters[IN_PRESS]= pressure/100; 
				holdingRegisters[IN_TEMP]= temperature;
				once = 0;
				}
				}break;
		case 3:{if(rtc_present==1 && once==1){
		
				rtc_read_clock(0);	//read hours,min,sec.
				rtc_read_cal(3);	// read date
				sprintf( CharBuffer, "%02x:%02x %02x/%02x/20%02x   ",(hour & 0b00011111),minute,date,month,year);
				lcd_gotoxy( 0, 0);	//column, row
				lcd_puts(CharBuffer);once=0;}
				}break;

		case 4:{if(once == 1){
		
					print_row_LCD(holdingRegisters[GRP_SEN]);
					
					holdingRegisters[0]= CurrentTemp1;
					holdingRegisters[1]= CurrentTemp2;
					holdingRegisters[2]= CurrentTemp3;
					holdingRegisters[3]= CurrentTemp4;
					holdingRegisters[4]= CurrentTemp5;
					holdingRegisters[5]= CurrentTemp6;
					once=0;}
				}break;
		case 6:{if(once == 1){
	
					print_row_LCD(holdingRegisters[GRP_SEN]);
					lcd_gotoxy( 0, 0);	//column, row
					lcd_puts(CharBuffer);
					holdingRegisters[0]= CurrentTemp1;
					holdingRegisters[1]= CurrentTemp2;
					holdingRegisters[2]= CurrentTemp3;
					holdingRegisters[3]= CurrentTemp4;
					holdingRegisters[4]= CurrentTemp5;
					holdingRegisters[5]= CurrentTemp6;
					once=0;}
				}break;
		case 8:{tick = 0;}break;
	    } //switch

	wdt_reset();
    }//while
} 

//////////////////////////---------------------------
void modbusGet(void) {
	if (modbusGetBusState() & (1<<ReceiveCompleted))
	{
		switch(rxbuffer[1]) {
			case fcReadCoilStatus: {
				modbusExchangeBits(&outstate,0,8);
			}
			break;
			
			case fcReadInputStatus: {
				volatile uint8_t inps = ReadIns();
				modbusExchangeBits(&inps,0,8);
			}
			break;
			
			case fcReadHoldingRegisters: {
				modbusExchangeRegisters(holdingRegisters,0,HOLDING_REG);
			}
			break;
			
			case fcReadInputRegisters: {
				modbusExchangeRegisters(inputRegisters,0,4);
			}
			break;
			
			case fcForceSingleCoil: {
				modbusExchangeBits(&outstate,0,8);
				SetOuts(outstate);
			}
			break;
			
			case fcPresetSingleRegister: {
				modbusExchangeRegisters(holdingRegisters,0,HOLDING_REG);
			}
			break;
			
			case fcForceMultipleCoils: {
				modbusExchangeBits(&outstate,0,8);
				SetOuts(outstate);
			}
			break;
			
			case fcPresetMultipleRegisters: {
				modbusExchangeRegisters(holdingRegisters,0,HOLDING_REG);
			}
			break;
			
			default: {
				modbusSendException(ecIllegalFunction);
			}
			break;
		}
	}
}

void timer0100us_start(void) {
	TCCR0B|=(1<<CS01); //prescaler 8
	TIMSK0|=(1<<TOIE0); //logical shift left for the assembler (NOT to the cpu).
						//enable Timer  (set BIT "TOIE0" in TIMSK)
}

void timer_tick()
{
	TCCR1B |= (1 << CS12) | (1 << CS10); //preddelièka 1024 (128us)    
    TIMSK1 |= (1 << TOIE1);  // prerušenie pri preteèení TCNT1     
    //OSCCAL = 0xA1;    // nastavenie kalibracneho bajtu interneho RC oscilatora
}

/*
*   Modify the following 3 functions to implement your own pin configurations...
*/
void SetOuts(volatile uint8_t in) {
	PORTD|= (((in & (1<<3))<<4) | ((in & (1<<4))<<1) | ((in & (1<<5))<<1));
	PORTB|= (((in & (1<<0))<<2) | ((in & (1<<1))) | ((in & (1<<2))>>2));
	in=~in;
	PORTB&= ~(((in & (1<<0))<<2) | ((in & (1<<1))) | ((in & (1<<2))>>2));
	PORTD&= ~(((in & (1<<3))<<4) | ((in & (1<<4))<<1) | ((in & (1<<5))<<1));
}

uint8_t ReadIns(void) {
	uint8_t ins=0x00;
	ins|=(PINC&((1<<0)|(1<<1)|(1<<2)|(1<<3)|(1<<4)|(1<<5)));
	ins|=(((PIND&(1<<4))<<2)|((PIND&(1<<3))<<4));
	return ins;
}

void io_conf(void) { 
	/*
	 Outputs: PB2,PB1,PB0,PD5,PD6, PD7
	 Inputs: PC0, PC1, PC2, PC3, PC4, PC6, PD4, PD3
	*/
	DDRD=0x00;
	DDRB=0x00;
	DDRC=0x00;
	PORTD=0x00;
	PORTB=0x00;
	PORTC=0x00;
	PORTD|=(1<<0);
	DDRD |= (1<<5)|(1<<6)|(1<<7);
	DDRB |= (1<<0)|(1<<1)|(1<<2)|(1<<3);
	
	DDRD = (1 << DDD2); //- input PD2
	PORTD |= (1 << PD2);   //pull up resistor
    //PORTD &= ~(1 << PD2); // input for button
	ACSR |= (1<<ACD); // vypnut komparator
}


void rtc_read_clock(char read_clock_address)
{
	rtc_present=i2crtc_start(DS1307_W);		/* Start I2C communication with RTC */
	i2crtc_write(read_clock_address);		/* Write address to read */
	i2crtc_repeated_start(DS1307_R);		/* Repeated start with device read address */

	second = i2crtc_read_ack();				/* Read second */
	minute = i2crtc_read_ack();				/* Read minute */
	hour = i2crtc_read_nack();				/* Read hour with Nack */
	i2crtc_stop();							/* Stop i2C communication */
}


void rtc_read_cal(char read_calendar_address)
{
	i2crtc_start(DS1307_W);
	i2crtc_write(read_calendar_address);
	i2crtc_repeated_start(DS1307_R);

	day = i2crtc_read_ack();				/* Read day */ 
	date = i2crtc_read_ack();				/* Read date */
	month = i2crtc_read_ack();				/* Read month */
	year = i2crtc_read_nack(); 				/* Read the year with Nack */
	i2crtc_stop();							/* Stop i2C communication */
}

void rtc_clock_write(char set_hour, char set_minute)
{
	// you need convert input data - bcd decimal to hex 	
	char bcd_hour,bcd_minute;
	bcd_hour = (((set_hour/10) *16) + (set_hour % 10));
	bcd_minute = (((set_minute/10)*16) + (set_minute % 10));
		
	i2crtc_start(DS1307_W);			// Start I2C communication with RTC 
	i2crtc_write(1);				// Write on 0 location for second value, 1 for minute 
	//i2crtc_write(5);				// Write second value on 00 location 
	i2crtc_write(bcd_minute);		// Write minute value on 01 location 
	i2crtc_write(bcd_hour);			// Write hour value on 02 location 
	i2crtc_stop();					// Stop I2C communication 
}

void rtc_hour_w(uint8_t set_hour)
{
	char bcd_hour;
	bcd_hour = (((set_hour/10) *16) + (set_hour % 10));
			
	i2crtc_start(DS1307_W);			// Start I2C communication with RTC 
	i2crtc_write(2);				// Write on 0 location for second value, 1 for minute 
	//i2crtc_write(5);				// Write second value on 00 location 
	i2crtc_write(bcd_hour);			// Write hour value on 02 location 
	i2crtc_stop();
}

void rtc_minute_w(uint8_t set_min)
{
	char bcd_minute;
	bcd_minute = (((set_min/10)*16) + (set_min % 10));
			
	i2crtc_start(DS1307_W);			// Start I2C communication with RTC 
	i2crtc_write(1);				// Write on 0 location for second value, 1 for minute 
	
	i2crtc_write(bcd_minute);			// Write value on 01 location 
	i2crtc_stop();
}

void rtc_dow_w(uint8_t set_dow)
{
	char bcd_dow;
	bcd_dow = (((set_dow/10)*16) + (set_dow % 10));
			
	i2crtc_start(DS1307_W);			// Start I2C communication with RTC 
	i2crtc_write(3);				// Write on 3 location for second value, 3 day of week 
	
	i2crtc_write(bcd_dow);			// Write value on 01 location 
	i2crtc_stop();
}

void rtc_day_w(uint8_t set_date)
{
	char bcd_date;
	bcd_date = (((set_date/10) *16) + (set_date % 10));
	i2crtc_start(DS1307_W);			// Start I2C communication with RTC 
	i2crtc_write(4);				// Write on 3 location for day value 
	i2crtc_write(bcd_date);			// Write date value on 04 location 
	i2crtc_stop();					// Stop I2C communication 
}

void rtc_month_w(uint8_t set_month)
{
	char bcd_month;
	bcd_month = (((set_month/10) *16) + (set_month % 10));
	i2crtc_start(DS1307_W);			// Start I2C communication with RTC 
	i2crtc_write(5);				// Write on 5 location for  value 
	i2crtc_write(bcd_month);			// Write value on location 
	i2crtc_stop();					// Stop I2C communication 
}

void rtc_year_w(uint8_t set_year)
{
	char bcd_year;
	
	bcd_year = (((set_year/10) *16) + (set_year % 10));
	i2crtc_start(DS1307_W);			// Start I2C communication with RTC 
	i2crtc_write(6);				// Write on 3 location for day value 
	i2crtc_write(bcd_year);			// Write year value on 06 location 
	i2crtc_stop();					// Stop I2C communication 
}

//void rtc_calendar_write(char set_day, char set_date, char set_month, char set_year)	// function for calendar 
void rtc_calendar_write(char set_date, char set_month, char set_year)
{
	char bcd_date,bcd_month,bcd_year;
	//bcd_day = (((set_day/10) *16) + (set_day % 10));
	bcd_date = (((set_date/10) *16) + (set_date % 10));
	bcd_month = (((set_month/10) *16) + (set_month % 10));
	bcd_year = (((set_year/10) *16) + (set_year % 10));
	
	i2crtc_start(DS1307_W);			// Start I2C communication with RTC 
	i2crtc_write(4);				// Write on 3 location for day value 
	//i2crtc_write(bcd_day);			// Write day value on 03 location 
	i2crtc_write(bcd_date);			// Write date value on 04 location 
	i2crtc_write(bcd_month);		// Write month value on 05 location 
	i2crtc_write(bcd_year);			// Write year value on 06 location 
	i2crtc_stop();					// Stop I2C communication 
}

void ds18b20ports(void)
{
// Port C initialization DS18b20 //
	DDRC &= ~(1 << PC0);   // input
    PORTC &= ~(1 << PC0);   
	DDRC &= ~(1 << PC1);   
    PORTC &= ~(1 << PC1);   
	DDRC &= ~(1 << PC2);   
    PORTC &= ~(1 << PC2);   
	DDRC &= ~(1 << PC3);   
    PORTC &= ~(1 << PC3);
	DDRD &= ~(1 << PD3);   
    PORTD &= ~(1 << PD3);   
	DDRD &= ~(1 << PD4);   
    PORTD &= ~(1 << PD4);  
}

void ds18b20presence(void)
{
 	
	Device1Presence = ds18b20_reset(&PORTC,PC0);
	if(Device1Presence)
			{chyba1=0;}
		else { chyba1=1;}
	if(chyba1==0)
		{lcd_gotoxy( 0, 0);read_ROM_CODE(&PORTC,PC0);_delay_ms(200);}
		else {lcd_puts_P( "   ");}
	
	//---------------------------------
	Device2Presence = ds18b20_reset(&PORTC,PC1);
	if(Device2Presence)
		{chyba2=0;}
		else { chyba2=1;}
	if(chyba2==0)
		{lcd_gotoxy( 0, 1);read_ROM_CODE(&PORTC,PC1);_delay_ms(200);}
		else {lcd_puts_P( "   ");}
	
	//---------------------------------
	Device3Presence = ds18b20_reset(&PORTC,PC2);
	if(Device3Presence)
			{chyba3=0;}
		else { chyba3=1;}
	if(chyba3==0)
		{lcd_gotoxy( 0, 2);read_ROM_CODE(&PORTC,PC2);_delay_ms(200);}
		else {lcd_puts_P( "   ");}
	
	//---------------------------------
	Device4Presence = ds18b20_reset(&PORTC,PC3);
	if(Device4Presence)
			{chyba4=0;}
		else { chyba4=1;}
	if(chyba4==0)
		{lcd_gotoxy( 0, 3);read_ROM_CODE(&PORTC,PC3);_delay_ms(200);}
		else {lcd_puts_P( "   ");}
	//---------------------------------
	Device5Presence = ds18b20_reset(&PORTD,PD3);
	if(Device5Presence)
			{chyba5=0;}
		else {  chyba5=1;}
	if(chyba5==0)
		{lcd_gotoxy( 0, 1);read_ROM_CODE(&PORTD,PD3);_delay_ms(100);}
		else {lcd_puts_P( "   ");}
	//---------------------------------
	Device6Presence = ds18b20_reset(&PORTD,PD4);
	if(Device6Presence)
			{chyba6=0;}
		else { chyba6=1;}
	if(chyba6==0)
		{lcd_gotoxy( 0, 2);read_ROM_CODE(&PORTD,PD4);_delay_ms(100);}
		else {lcd_puts_P( "   ");}
}

void ds18b20readtemperature(int snimac)
{
	switch (snimac){
		
		case 1: {if(chyba1==0)CurrentTemp1 = ds18b20_gettemp(&PORTC,PC0); // decicelsius
			break;}
			//----------------------------------------------------	
		case 2:	{if(chyba2==0){CurrentTemp2 = ds18b20_gettemp(&PORTC,PC1);	//decicelsius
			}break;}
			//----------------------------------------------------	
		case 3: {if(chyba3==0){CurrentTemp3 = ds18b20_gettemp(&PORTC,PC2);	//decicelsius
				}break;}
			//----------------------------------------------------	
		case 4: {if(chyba4==0){CurrentTemp4 = ds18b20_gettemp(&PORTC,PC3);	//decicelsius
			}break;}
			//----------------------------------------------------			
		case 5:	{if(chyba5==0){CurrentTemp5 = ds18b20_gettemp(&PORTD,PD3);	//decicelsius
			}break;}
			//----------------------------------------------------			
		case 6:	{if(chyba6==0){CurrentTemp6 = ds18b20_gettemp(&PORTD,PD4);	//decicelsius
			}break;}
	}

}

// rows 1 - 4, groups 0 - 3
void print_row_LCD(uint16_t num_sensors_group)
{
	char Buffer[21];
	// able position 1234,1342,1432, 1243,1324,1423,  
switch (num_sensors_group){
	case 1234:{	lcd_gotoxy( 0, 0);
				sprintf( Buffer, "Ti:%ld C P:%ld Pa",temperature,pressure);
				lcd_puts(Buffer);
				lcd_gotoxy( 0, 1);
				sprintf( Buffer, "T1:%3d C   T2:%3d C",CurrentTemp1,CurrentTemp2);
				lcd_puts(Buffer);
				lcd_gotoxy( 0, 2);
				sprintf( Buffer, "T3:%3d C   T4:%3d C",CurrentTemp3,CurrentTemp4);
				lcd_puts(Buffer);
				lcd_gotoxy( 0, 3);
				sprintf( Buffer, "T5:%3d C   T6:%3d C",CurrentTemp5,CurrentTemp6);
				lcd_puts(Buffer);
				}break;
	
	case 1342:{	lcd_gotoxy( 0, 0);
				sprintf( Buffer, "Ti:%ld C P:%ld Pa",temperature,pressure);
				lcd_puts(Buffer);
				lcd_gotoxy( 0, 2);
				sprintf( Buffer, "T1:%3d C   T2:%3d C",CurrentTemp1,CurrentTemp2);
				lcd_puts(Buffer);
				lcd_gotoxy( 0, 3);
				sprintf( Buffer, "T3:%3d C   T4:%3d C",CurrentTemp3,CurrentTemp4);
				lcd_puts(Buffer);
				lcd_gotoxy( 0, 1);
				sprintf( Buffer, "T5:%3d C   T6:%3d C",CurrentTemp5,CurrentTemp6);
				lcd_puts(Buffer);
				}break;

	case 1432:{	lcd_gotoxy( 0, 0);
				sprintf( Buffer, "Ti:%ld C P:%ld Pa",temperature,pressure);
				lcd_puts(Buffer);
				lcd_gotoxy( 0, 1);
				sprintf( Buffer, "T1:%3d C   T2:%3d C",CurrentTemp1,CurrentTemp2);
				lcd_puts(Buffer);
				lcd_gotoxy( 0, 2);
				sprintf( Buffer, "T3:%3d C   T4:%3d C",CurrentTemp3,CurrentTemp4);
				lcd_puts(Buffer);
				lcd_gotoxy( 0, 3);
				sprintf( Buffer, "T5:%3d C   T6:%3d C",CurrentTemp5,CurrentTemp6);
				lcd_puts(Buffer);
				}break;

	case 1243:{	lcd_gotoxy( 0, 0);
				sprintf( Buffer, "Ti:%ld C P:%ld Pa",temperature,pressure);
				lcd_puts(Buffer);
				lcd_gotoxy( 0, 1);
				sprintf( Buffer, "T1:%3d C   T2:%3d C",CurrentTemp1,CurrentTemp2);
				lcd_puts(Buffer);
				lcd_gotoxy( 0, 3);
				sprintf( Buffer, "T3:%3d C   T4:%3d C",CurrentTemp3,CurrentTemp4);
				lcd_puts(Buffer);
				lcd_gotoxy( 0, 2);
				sprintf( Buffer, "T5:%3d C   T6:%3d C",CurrentTemp5,CurrentTemp6);
				lcd_puts(Buffer);
				}break;

	case 1324:{	lcd_gotoxy( 0, 0);
				sprintf( Buffer, "Ti:%ld C P:%ld Pa",temperature,pressure);
				lcd_puts(Buffer);
				lcd_gotoxy( 0, 2);
				sprintf( Buffer, "T1:%3d C   T2:%3d C",CurrentTemp1,CurrentTemp2);
				lcd_puts(Buffer);
				lcd_gotoxy( 0, 1);
				sprintf( Buffer, "T3:%3d C   T4:%3d C",CurrentTemp3,CurrentTemp4);
				lcd_puts(Buffer);
				lcd_gotoxy( 0, 3);
				sprintf( Buffer, "T5:%3d C   T6:%3d C",CurrentTemp5,CurrentTemp6);
				lcd_puts(Buffer);
				}break;

	case 1423:{	lcd_gotoxy( 0, 0);
				sprintf( Buffer, "Ti:%ld C P:%ld Pa",temperature,pressure);
				lcd_puts(Buffer);
				lcd_gotoxy( 0, 3);
				sprintf( Buffer, "T1:%3d C   T2:%3d C",CurrentTemp1,CurrentTemp2);
				lcd_puts(Buffer);
				lcd_gotoxy( 0, 1);
				sprintf( Buffer, "T3:%3d C   T4:%3d C",CurrentTemp3,CurrentTemp4);
				lcd_puts(Buffer);
				lcd_gotoxy( 0, 2);
				sprintf( Buffer, "T5:%3d C   T6:%3d C",CurrentTemp5,CurrentTemp6);
				lcd_puts(Buffer);
				}break;

	
	default: {	lcd_gotoxy( 0, 0);
				sprintf( Buffer, "Ti:%ld C P:%ld Pa",temperature,pressure);
				lcd_puts(Buffer);
				lcd_gotoxy( 0, 1);
				sprintf( Buffer, "T1:%3d C   T2:%3d C",CurrentTemp1,CurrentTemp2);
				lcd_puts(Buffer);
				lcd_gotoxy( 0, 2);
				sprintf( Buffer, "T3:%3d C   T4:%3d C",CurrentTemp3,CurrentTemp4);
				lcd_puts(Buffer);
				lcd_gotoxy( 0, 3);
				sprintf( Buffer, "T5:%3d C   T6:%3d C",CurrentTemp5,CurrentTemp6);
				lcd_puts(Buffer);
				}
		
	} //switch
		
}


void Ftermostat1(void){
		
		if( (holdingRegisters[Therm1_EE] > 0) && (holdingRegisters[Therm1_EE] < MAX_TEMP) && (holdingRegisters[Therm1_EE] != termostat1) ){
				termostat1 = holdingRegisters[Therm1_EE];
				thermostat1_eeprom = termostat1;
				eeprom_write_word((uint16_t*)ADR_thermostat1,thermostat1_eeprom); // works
				lcd_puts_P( " zapisujem EE1 ");
				}
				else {
					holdingRegisters[Therm1_EE]= termostat1;
				}
}

void Ftermostat2(void){		
		
		if( (holdingRegisters[Therm2_EE] > 0) && (holdingRegisters[Therm2_EE] < MAX_TEMP) && (holdingRegisters[Therm2_EE] != termostat2) ){
				termostat2 = holdingRegisters[Therm2_EE];
				thermostat2_eeprom = termostat2;
				eeprom_write_word((uint16_t*)ADR_thermostat2,thermostat2_eeprom); // works
				lcd_puts_P( " zapisujem EE2 ");
				}
				else {
					holdingRegisters[Therm2_EE]= termostat2;
				}
}
