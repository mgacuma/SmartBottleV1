//*****************************************************************************
//*****************************************************************************

//*****************************************************************************
//
//			PIN CONFIGURATIONS
//
//	GPIO PORTF PIN 	0 	= LCD RS
//	GPIO PORTE PIN 	0 	= LCD EN
//	GPIO PORTB PIN	7-0	= LCD D7-0
//	GPIO PORTE PIN 	2 	= HX711 DATA
//	GPIO PORTE PIN 	3 	= HX711 CLK
//
//******************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "SmartBottleV1.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom_map.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "inc/tm4c123gh6pm.h"


//*****************************************************************************
char inst = 'i';
char data = 'd';
int i, j;
float currentMass;
char buffer[32];
float modulate = -0.00111991147068;
float offset = 9769.96880117;
float drank = 0;
int time;


void PortFunctionInit(void) {
	//
	// Enable Peripheral Clocks 
	//
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);



	//Enable pin PE2 for GPIOInput
	MAP_GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_2);


	//Enable pin PE3 for GPIOOutput
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_3);


	// Enable pin PB0 for GPIOOutput
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_0);

	// Enable pin PB1 for GPIOOutput
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_1);

	// Enable pin PB2 for GPIOOutput
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_2);

	// Enable pin PB3 for GPIOOutput
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_3);

	// Enable pin PB4 for GPIOOutput
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_4);

	// Enable pin PB5 for GPIOOutput
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_5);

	// Enable pin PB6 for GPIOOutput
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_6);

	// Enable pin PB7 for GPIOOutput
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_7);


	// Enable pin PE0 for GPIOOutput
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_0);

	//First open the lock and select the bits we want to modify in the GPIO commit register.
	HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
	HWREG(GPIO_PORTF_BASE + GPIO_O_CR) = 0x1;

	//Now modify the configuration of the pins that we unlocked.
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0);
}

//Globally enable interrupts 
void IntGlobalEnable(void) {
	__asm("    cpsie   i\n");
}

//Globally disable interrupts 
void IntGlobalDisable(void) {
	__asm("    cpsid   i\n");
}

void Interrupt_Init(void) {
	NVIC_EN0_R |= 0x40000000;  		// enable interrupt 30 in NVIC (GPIOF)
	NVIC_PRI7_R &= 0x00E00000; 		// configure GPIOF interrupt priority as 0
	GPIO_PORTF_IM_R |= 0x11;   		// arm interrupt on PF0 and PF4
	GPIO_PORTF_IS_R &= ~0x11;     // PF0 and PF4 are edge-sensitive
	GPIO_PORTF_IBE_R &= ~0x11;   	// PF0 and PF4 not both edges trigger 
	GPIO_PORTF_IEV_R &= ~0x11;  	// PF0 and PF4 falling edge event
	IntGlobalEnable();        		// globally enable interrupt
}
void Timer0A_Init(unsigned long period) {
	//
  // Enable Peripheral Clocks 
  //
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
	TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC); 		// configure for 32-bit timer mode
	TimerLoadSet(TIMER0_BASE, TIMER_A, period - 1);      //reload value
	IntPrioritySet(INT_TIMER0A, 0x00);  	 // configure Timer0A interrupt priority as 0
	IntEnable(INT_TIMER0A);    				// enable interrupt 19 in NVIC (Timer0A)
	TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);      // arm timeout interrupt
	TimerEnable(TIMER0_BASE, TIMER_A);      // enable timer0A
}

void waitms(float ms) {
	float del = ((ms * 10000000) / 3) / 1000;
	SysCtlDelay(del);
}

long signd(uint32_t in) {
	long out = 0x0 | in;
	return out;
}

void lcd_en(int n) {
	GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0, n);
}
void lcd_rs(char c) {
	if (c == 'i') {
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, 0x0);
	}
	else if (c == 'd') {
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, 0x1);
	}
}
void lcd_clear() {
	lcd_en(0x0);
	lcd_rs(inst);
	lcd_en(0x01);
	GPIO_PORTB_DATA_R = 0x01;
	waitms(1);
	lcd_en(0x0);
}
void lcd_entryDefaultSet() {
	lcd_en(0x0);
	lcd_rs(inst);
	lcd_en(0x01);
	GPIO_PORTB_DATA_R = 0x06;
	waitms(1);
	lcd_en(0x0);
}
void lcd_home() {
	lcd_en(0x0);
	lcd_rs(inst);
	lcd_en(0x1);
	GPIO_PORTB_DATA_R = 0x02;
	waitms(1);
	lcd_en(0x0);
}
void lcd_displayOff() {
	lcd_en(0x0);
	lcd_rs(inst);
	lcd_en(0x1);
	GPIO_PORTB_DATA_R = 0x08;
	waitms(1);
	lcd_en(0x0);
}
void lcd_displayOn() {
	lcd_en(0x0);
	lcd_rs(inst);
	lcd_en(0x1);
	GPIO_PORTB_DATA_R = 0x0c;
	waitms(1);
	lcd_en(0x0);
}
void lcd_cursorOn() {
	lcd_en(0x0);
	lcd_rs(inst);
	lcd_en(0x1);
	GPIO_PORTB_DATA_R = 0x0f;
	waitms(1);
	lcd_en(0x0);
}
void lcd_cursorOff() {
	lcd_en(0x0);
	lcd_rs(inst);
	lcd_en(0x1);
	GPIO_PORTB_DATA_R = 0xc;
	waitms(1);
	lcd_en(0x0);
}
void lcd_shiftRight() {
	lcd_en(0x0);
	lcd_rs(inst);
	lcd_en(0x1);
	GPIO_PORTB_DATA_R = 0x14;
	waitms(1);
	lcd_en(0x0);
}
void lcd_shiftLeft() {
	lcd_en(0x0);
	lcd_rs(inst);
	lcd_en(0x1);
	GPIO_PORTB_DATA_R = 0x16;
	waitms(1);
	lcd_en(0x0);
}
void lcd_functionset() {
	lcd_en(0x0);
	lcd_rs(inst);
	GPIO_PORTB_DATA_R = 0x38;
	waitms(1);
	lcd_en(0x1);
	waitms(1);
	lcd_en(0x0);
}
void lcd_setAddr(int n) {
	lcd_en(0x0);
	lcd_rs(inst);
	lcd_en(0x1);
	GPIO_PORTB_DATA_R = 0x80 + n;
	waitms(1);
	lcd_en(0x0);
}
void lcd_write(char c) {
	int ascii = c;
	lcd_en(0x0);
	lcd_rs(data);
	lcd_en(0x1);
	GPIO_PORTB_DATA_R = ascii;
	waitms(1);
	lcd_en(0x0);
}
void lcd_type(char arr[32]) {
	lcd_clear();
	waitms(1);
	lcd_home();

	for (i = 0; i < 32; i++) {
		if (i == 16 || arr[i] == '\n') {
			lcd_setAddr(0x40);
		}

		if (arr[i] != '\0') {
			lcd_write(arr[i]);
		}
		else break;
	}

}
void lcd_init() {
	lcd_functionset();
	waitms(1);
	lcd_entryDefaultSet();
	waitms(1);
}

void pulseUp() {
	GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_3, GPIO_PIN_3);
}
void pulseDown() {
	GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_3, 0x0);
}
void pulse() {
	pulseUp();
	pulseDown();
}
void hx_reset() {
	pulseUp();
	waitms(10);
	pulseDown();
}
bool hx_ready() {
	if (GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_2) == 0x0) {
		return true;
	}
	else return false;
}

uint32_t hx_readMass() {
	IntGlobalDisable();
	uint32_t count = 0;
	uint32_t div = 0x800000;

	hx_reset();

	while (!(hx_ready())) {}

	for (i = 0; i < 24; i++) {
		pulse();
		if (GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_2) == GPIO_PIN_2) count++;
		count = count << 1;
	}

	pulse();

	count ^= div;

	/*
	if ((count & div) == div) {
		count = count | 0xFF000000;
	}
	*/

	IntGlobalEnable();
	return count;
}

void hx_showMass(uint32_t in) {
	int mod = 10000000;
	long conv = signd(in);
	uint32_t div = 0x80000000;
	uint32_t div2 = 0x800000;

	///*
	if ((in & div) == div) {
		buffer[0] = '-';
	}
	else buffer[0] = ' ';

	for (i = 1; i < 8; i++) {
		buffer[i] = '0' + (conv % mod) / (mod / 10);
		mod = mod / 10;
	}
	buffer[8] = '\0';
	lcd_type(buffer);
	//*/

	/*
	for (i = 0; i < 24; i++) {
		if ((in & div2) != 0) {
			buffer[i] = '1';
		}
		else buffer[i] = '0';
		div2 = div2 / 2;
	}
	lcd_type(buffer);
	*/

	/*
	if(in < 0){
		lcd_type("negative");
	}
	else if (in > 0){
		lcd_type("positive");
	}
	else lcd_type("zero");
	*/

	waitms(100);
}

uint32_t hx_findAvg() {
	uint32_t avg[7];
	uint32_t favg = hx_readMass();

	for (i = 0; i < 7; i++) {
		favg = (favg + hx_readMass()) / 2;
	}

	return favg;
}

float hx_findGrams(uint32_t in){
	return (float)(modulate * in) + offset;
}

void printMenu(){
	//H2O 10 Chars, start at 11
	lcd_type("H2O Taken:                   ago");
}

void addTime(){
	time = time + 3;
}

void resetTime(){
	time = 0;
}

void printTime(){
	int mins, secs;
	char timeArr[5];
	
	mins = time / 60;
	secs = time % 60;
	
	timeArr[0] = '0' + (mins / 10);
	timeArr[1] = '0' + (mins % 10);
	timeArr[2] = ':';
	timeArr[3] = '0' + (secs / 10);
	timeArr[4] = '0' + (secs % 10);
	
	lcd_home();
	for(i = 0; i < 11; i++){
		lcd_shiftRight();
	}
	for(i = 0; i < 5; i++){
		lcd_write(timeArr[i]);
	}
}

void printDrank(float in){
	int integer = (int) in;
	float fraction = in - integer;
	char massArr[10];
	massArr[4] = '.';
	massArr[7] = ' ';
	massArr[8] = 'm';
	massArr[9] = 'L';
	
	if((integer / 1000) >= 0){
		massArr[0] = '0' + integer / 1000;
	}
	else massArr[0] = ' ';
	
	if((integer % 1000 / 100) >= 0){
		massArr[1] = '0' + integer % 1000 / 100;
	}
	else massArr[1] = ' ';
	
	if((integer % 100 / 10) >= 0){
		massArr[2] = '0' + integer % 100 / 10;
	}
	else massArr[2] = ' ';
	
	massArr[3] = '0' + integer % 10;
	
	massArr[5] = '0' + (int) fraction * 10;
	massArr[6] = '0' + (int) fraction * 100 % 10;
	
	lcd_home();
	for(i = 0; i < 40; i++){
		lcd_shiftRight();
	}
	for(i = 0; i < 10; i++){
		lcd_write(massArr[i]);
	}
}

//interrupt handler for Timer0A
void Timer0A_Handler(void) {
	addTime();
	printTime();
	
	float readMass = hx_findGrams(hx_findAvg());
	
	if(currentMass == 0){
		currentMass = readMass;
	}
	
	
	if(readMass < currentMass){
		if(readMass < 20){
		}
		else if ((currentMass - readMass) > 5){
			drank = drank + currentMass - readMass;
			currentMass = readMass;
			resetTime();
		}
	}
		
	else {
		currentMass = readMass;
	}
	
	printDrank(drank);
	
	TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
}

int main(void)
{
	float secs = 5;
	unsigned long period = secs * 10000000;

	SysCtlClockSet(SYSCTL_SYSDIV_20 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

	//initialize the GPIO ports	
	PortFunctionInit();
	
	
	lcd_init();
	waitms(1000);
	

	lcd_type("Initializing...");
	waitms(100);
	
	printMenu();
	hx_reset();
	
	
	
	Timer0A_Init(period);

	while (1)
	{
	}
}
