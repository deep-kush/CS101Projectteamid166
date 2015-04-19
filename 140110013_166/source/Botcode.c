/*
 * License Plate Scanning Bot
 * Program this code into FireBird V using AVR Atmel Studio
 * Created: 3/29/2015 3:29:21 PM
 *  Author: Deepanshu, SHivam, rishabh, milind
 */ 


#define F_CPU 14745600
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h> 
#include "lcd.h"

unsigned char data; //to store received data from UDR0
unsigned char ADC_Conversion(unsigned char);
unsigned char ADC_Value;
unsigned char sharp, distance, adc_reading;
unsigned int value;
float BATT_Voltage, BATT_V;
unsigned char flag = 0;
unsigned char Left_white_line = 0;
unsigned char Center_white_line = 0;
unsigned char Right_white_line = 0;

void buzzer_pin_config (void)
{
	DDRC = DDRC | 0x08;		//Setting PORTC 3 as output
	PORTC = PORTC & 0xF7;		//Setting PORTC 3 logic low to turnoff buzzer
}

void buzzer_on (void)
{
	unsigned char port_restore = 0;
	port_restore = PINC;
	port_restore = port_restore | 0x08;
	PORTC = port_restore;
}

void buzzer_off (void)
{
	unsigned char port_restore = 0;
	port_restore = PINC;
	port_restore = port_restore & 0xF7;
	PORTC = port_restore;
}
void motion_pin_config (void)
{
	DDRA = DDRA | 0x0F; //set direction of the PORTA 3 to PORTA 0 pins as output
	PORTA = PORTA & 0xF0; // set initial value of the PORTA 3 to PORTA 0 pins to logic 0
	DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
	PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM
}

//Function to configure LCD port
void lcd_port_config (void)
{
	DDRC = DDRC | 0xF7; //all the LCD pin's direction set as output
	PORTC = PORTC & 0x80; // all the LCD pins are set to logic 0 except PORTC 7
}

//ADC pin configuration
void adc_pin_config (void)
{
	DDRF = 0x00; //set PORTF direction as input
	PORTF = 0x00; //set PORTF pins floating
	DDRK = 0x00; //set PORTK direction as input
	PORTK = 0x00; //set PORTK pins floating
}

//Function to initialize ports
void port_init()
{
	motion_pin_config();
	lcd_port_config();
	adc_pin_config();

}

void motion_set (unsigned char Direction)
{
	unsigned char PortARestore = 0;

	Direction &= 0x0F; 			// removing upper nibble as it is not needed
	PortARestore = PORTA; 			// reading the PORTA's original status
	PortARestore &= 0xF0; 			// setting lower direction nibble to 0
	PortARestore |= Direction; 	// adding lower nibble for direction command and restoring the PORTA status
	PORTA = PortARestore; 			// setting the command to the port
}
//Function for velocity control
void velocity (unsigned char left_motor, unsigned char right_motor)
{
	OCR5AL = (unsigned char)left_motor;
	OCR5BL = (unsigned char)right_motor;
}

void forward (void) //both wheels forward
{
	motion_set(0x06);
}

void back (void) //both wheels backward
{
	motion_set(0x09);
}

void stop (void) //hard stop
{
	motion_set(0x00);
}

//Function to Initialize ADC
void adc_init()
{
	ADCSRA = 0x00;
	ADCSRB = 0x00;		//MUX5 = 0
	ADMUX = 0x20;		//Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
	ACSR = 0x80;
	ADCSRA = 0x86;		//ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}

void timer5_init()
{
	TCCR5B = 0x00;	//Stop
	TCNT5H = 0xFF;	//Counter higher 8-bit value to which OCR5xH value is compared with
	TCNT5L = 0x01;	//Counter lower 8-bit value to which OCR5xH value is compared with
	OCR5AH = 0x00;	//Output compare register high value for Left Motor
	OCR5AL = 0xFF;	//Output compare register low value for Left Motor
	OCR5BH = 0x00;	//Output compare register high value for Right Motor
	OCR5BL = 0xFF;	//Output compare register low value for Right Motor
	OCR5CH = 0x00;	//Output compare register high value for Motor C1
	OCR5CL = 0xFF;	//Output compare register low value for Motor C1
	TCCR5A = 0xA9;	/*{COM5A1=1, COM5A0=0; COM5B1=1, COM5B0=0; COM5C1=1 COM5C0=0}
 					  For Overriding normal port functionality to OCRnA outputs.
				  	  {WGM51=0, WGM50=1} Along With WGM52 in TCCR5B for Selecting FAST PWM 8-bit Mode*/
	
	TCCR5B = 0x0B;	//WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=64)
}


//Function To Initialize UART0
// desired baud rate:9600
// actual baud rate:9600 (error 0.0%)
// char size: 8 bit
// parity: Disabled
void uart0_init(void)
{
	UCSR0B = 0x00; //disable while setting baud rate
	UCSR0A = 0x00;
	UCSR0C = 0x06;
	UBRR0L = 0x5F; //set baud rate lo
	UBRR0H = 0x00; //set baud rate hi
	UCSR0B = 0x98;
}


SIGNAL(SIG_USART0_RECV) 															// ISR for receive complete interrupt
{
	data = UDR0; 																	//making copy of data from UDR0 in 'data' variable

	UDR0 = data; 																	//echo data back to PC
	
	if(data == 0x41)																//ASCII value of A
	{
		buzzer_on();																//function to turn on the buzzer
		lcd_cursor(1,1);															// Takes the cursor to the first row and first column
		lcd_string("Stolen Vehicle");
		lcd_cursor(2,1);															// Takes the cursor to the first row and first column
		lcd_string("Number: 0");
		_delay_ms(2000);
		//lcd_reset();
		buzzer_off();
		lcd_cursor(1,1);															// making the LCD screen blank
		lcd_string("              ");
		lcd_cursor(2,1);
		lcd_string("              ");
		data = 0x6E;																// the value of the register reset to 0x6E which was the initially set value (NOTE: as long as the value sent by the register is 0x6E,
																					//																				 no instruction is sent to Laptop)
	}
	
	
	
	if(data == 0x42)																//ASCII value of B
	
{	buzzer_on();																//function to turn on the buzzer
	lcd_cursor(1,1);															// Takes the cursor to the first row and first column
	lcd_string("Stolen Vehicle");
	lcd_cursor(2,1);															// Takes the cursor to the first row and first column
	lcd_string("Number: 0");
	_delay_ms(2000);
	//lcd_reset();
	buzzer_off();
	lcd_cursor(1,1);															// making the LCD screen blank
	lcd_string("              ");
	lcd_cursor(2,1);
	lcd_string("              ");
	data = 0x6E;																// the value of the register reset to 0x6E which was the initially set value (NOTE: as long as the value sent by the register is 0x6E,
																				//																				 no instruction is sent to Laptop)
}			

																	
	if(data == 0x43) //ASCII value of C
	{
		
		buzzer_on();																//function to turn on the buzzer
		lcd_cursor(1,1);															// Takes the cursor to the first row and first column
		lcd_string("Stolen Vehicle");
		lcd_cursor(2,1);															// Takes the cursor to the first row and first column
		lcd_string("Number: 0");
		_delay_ms(2000);
		//lcd_reset();
		buzzer_off();
		lcd_cursor(1,1);															// making the LCD screen blank
		lcd_string("              ");
		lcd_cursor(2,1);
		lcd_string("              ");
		data = 0x6E;																// the value of the register reset to 0x6E which was the initially set value (NOTE: as long as the value sent by the register is 0x6E,
																					//																				 no instruction is sent to Laptop)}
}	
	if(data == 0x44) //ASCII value of D
	{
		
		buzzer_on();																//function to turn on the buzzer
		lcd_cursor(1,1);															// Takes the cursor to the first row and first column
		lcd_string("Stolen Vehicle");
		lcd_cursor(2,1);															// Takes the cursor to the first row and first column
		lcd_string("Number: 0");
		_delay_ms(2000);
		//lcd_reset();
		buzzer_off();
		lcd_cursor(1,1);															// making the LCD screen blank
		lcd_string("              ");
		lcd_cursor(2,1);
		lcd_string("              ");
		data = 0x6E;																// the value of the register reset to 0x6E which was the initially set value (NOTE: as long as the value sent by the register is 0x6E,
																					//																				 no instruction is sent to Laptop)
	}
	if(data == 0x45) //ASCII value of E
	{
		
		buzzer_on();																//function to turn on the buzzer
		lcd_cursor(1,1);															// Takes the cursor to the first row and first column
		lcd_string("Stolen Vehicle");
		lcd_cursor(2,1);															// Takes the cursor to the first row and first column
		lcd_string("Number: 0");
		_delay_ms(2000);
		//lcd_reset();
		buzzer_off();
		lcd_cursor(1,1);															// making the LCD screen blank
		lcd_string("              ");
		lcd_cursor(2,1);
		lcd_string("              ");
		data = 0x6E;																// the value of the register reset to 0x6E which was the initially set value (NOTE: as long as the value sent by the register is 0x6E,
																					//																				 no instruction is sent to Laptop)
	}
	if(data == 0x46) //ASCII value of F
	{
		
		buzzer_on();																//function to turn on the buzzer
		lcd_cursor(1,1);															// Takes the cursor to the first row and first column
		lcd_string("Stolen Vehicle");
		lcd_cursor(2,1);															// Takes the cursor to the first row and first column
		lcd_string("Number: 0");
		_delay_ms(2000);
		//lcd_reset();
		buzzer_off();
		lcd_cursor(1,1);															// making the LCD screen blank
		lcd_string("              ");
		lcd_cursor(2,1);
		lcd_string("              ");
		data = 0x6E;																// the value of the register reset to 0x6E which was the initially set value (NOTE: as long as the value sent by the register is 0x6E,
																					//																				 no instruction is sent to Laptop)
	}
	if(data == 0x47) //ASCII value of G
	{
		
		buzzer_on();																//function to turn on the buzzer
		lcd_cursor(1,1);															// Takes the cursor to the first row and first column
		lcd_string("Stolen Vehicle");
		lcd_cursor(2,1);															// Takes the cursor to the first row and first column
		lcd_string("Number: 0");
		_delay_ms(2000);
		//lcd_reset();
		buzzer_off();
		lcd_cursor(1,1);															// making the LCD screen blank
		lcd_string("              ");
		lcd_cursor(2,1);
		lcd_string("              ");
		data = 0x6E;																// the value of the register reset to 0x6E which was the initially set value (NOTE: as long as the value sent by the register is 0x6E,
																					//																				 no instruction is sent to Laptop)
	}
	if(data == 0x48) //ASCII value of H
	{
		
		buzzer_on();																//function to turn on the buzzer
		lcd_cursor(1,1);															// Takes the cursor to the first row and first column
		lcd_string("Stolen Vehicle");
		lcd_cursor(2,1);															// Takes the cursor to the first row and first column
		lcd_string("Number: 0");
		_delay_ms(2000);
		//lcd_reset();
		buzzer_off();
		lcd_cursor(1,1);															// making the LCD screen blank
		lcd_string("              ");
		lcd_cursor(2,1);
		lcd_string("              ");
		data = 0x6E;																// the value of the register reset to 0x6E which was the initially set value (NOTE: as long as the value sent by the register is 0x6E,
																					//																				 no instruction is sent to Laptop)
	}
	if(data == 0x49) //ASCII value of I
	{
		
		buzzer_on();																//function to turn on the buzzer
		lcd_cursor(1,1);															// Takes the cursor to the first row and first column
		lcd_string("Stolen Vehicle");
		lcd_cursor(2,1);															// Takes the cursor to the first row and first column
		lcd_string("Number: 0");
		_delay_ms(2000);
		//lcd_reset();
		buzzer_off();
		lcd_cursor(1,1);															// making the LCD screen blank
		lcd_string("              ");
		lcd_cursor(2,1);
		lcd_string("              ");
		data = 0x6E;																// the value of the register reset to 0x6E which was the initially set value (NOTE: as long as the value sent by the register is 0x6E,
																					//																				 no instruction is sent to Laptop)
	}
	if(data == 0x4A) //ASCII value of J
	{
		
		buzzer_on();																//function to turn on the buzzer
		lcd_cursor(1,1);															// Takes the cursor to the first row and first column
		lcd_string("Stolen Vehicle");
		lcd_cursor(2,1);															// Takes the cursor to the first row and first column
		lcd_string("Number: 0");
		_delay_ms(2000);
		//lcd_reset();
		buzzer_off();
		lcd_cursor(1,1);															// making the LCD screen blank
		lcd_string("              ");
		lcd_cursor(2,1);
		lcd_string("              ");
		data = 0x6E;																// the value of the register reset to 0x6E which was the initially set value (NOTE: as long as the value sent by the register is 0x6E,
																					//																					 no instruction is sent to Laptop)
	}
	
	if (data == 0x61) //ASCII value of a
	{
		
		buzzer_on();																//function to turn on the buzzer
		lcd_cursor(1,1);															// Takes the cursor to the first row and first column
		lcd_string("Stolen Vehicle");
		lcd_cursor(2,1);															// Takes the cursor to the first row and first column
		lcd_string("Number: 0");
		_delay_ms(2000);
		//lcd_reset();
		buzzer_off();
		lcd_cursor(1,1);															// making the LCD screen blank
		lcd_string("              ");
		lcd_cursor(2,1);
		lcd_string("              ");
		data = 0x6E;																// the value of the register reset to 0x6E which was the initially set value (NOTE: as long as the value sent by the register is 0x6E,
																					//																				 no instruction is sent to Laptop)
	}
	if (data == 0x62) //ASCII value of b
	{
		
		buzzer_on();																//function to turn on the buzzer
		lcd_cursor(1,1);															// Takes the cursor to the first row and first column
		lcd_string("Stolen Vehicle");
		lcd_cursor(2,1);															// Takes the cursor to the first row and first column
		lcd_string("Number: 0");
		_delay_ms(2000);
		//lcd_reset();
		buzzer_off();
		lcd_cursor(1,1);															// making the LCD screen blank
		lcd_string("              ");
		lcd_cursor(2,1);
		lcd_string("              ");
		data = 0x6E;																// the value of the register reset to 0x6E which was the initially set value (NOTE: as long as the value sent by the register is 0x6E,
																					//																				 no instruction is sent to Laptop)
		
	}
	if (data == 0x63) //ASCII value of c
	{
		
		buzzer_on();																//function to turn on the buzzer
		lcd_cursor(1,1);															// Takes the cursor to the first row and first column
		lcd_string("Stolen Vehicle");
		lcd_cursor(2,1);															// Takes the cursor to the first row and first column
		lcd_string("Number: 0");
		_delay_ms(2000);
		//lcd_reset();
		buzzer_off();
		lcd_cursor(1,1);															// making the LCD screen blank
		lcd_string("              ");
		lcd_cursor(2,1);
		lcd_string("              ");
		data = 0x6E;																// the value of the register reset to 0x6E which was the initially set value (NOTE: as long as the value sent by the register is 0x6E,
																					//																				 no instruction is sent to Laptop)
		
	}
	if (data == 0x64) //ASCII value of d
	{
		
		buzzer_on();																//function to turn on the buzzer
		lcd_cursor(1,1);															// Takes the cursor to the first row and first column
		lcd_string("Stolen Vehicle");
		lcd_cursor(2,1);															// Takes the cursor to the first row and first column
		lcd_string("Number: 0");
		_delay_ms(2000);
		//lcd_reset();
		buzzer_off();
		lcd_cursor(1,1);															// making the LCD screen blank
		lcd_string("              ");
		lcd_cursor(2,1);
		lcd_string("              ");
		data = 0x6E;																// the value of the register reset to 0x6E which was the initially set value (NOTE: as long as the value sent by the register is 0x6E,
																					//																				 no instruction is sent to Laptop)
		
	}
	if (data == 0x65) //ASCII value of e
	{
		
		buzzer_on();																//function to turn on the buzzer
		lcd_cursor(1,1);															// Takes the cursor to the first row and first column
		lcd_string("Stolen Vehicle");
		lcd_cursor(2,1);															// Takes the cursor to the first row and first column
		lcd_string("Number: 0");
		_delay_ms(2000);
		//lcd_reset();
		buzzer_off();
		lcd_cursor(1,1);															// making the LCD screen blank
		lcd_string("              ");
		lcd_cursor(2,1);
		lcd_string("              ");
		data = 0x6E;																// the value of the register reset to 0x6E which was the initially set value (NOTE: as long as the value sent by the register is 0x6E,
																					//																				 no instruction is sent to Laptop)
		
	}
	if (data == 0x66) //ASCII value of f
	{
		
		buzzer_on();																//function to turn on the buzzer
		lcd_cursor(1,1);															// Takes the cursor to the first row and first column
		lcd_string("Stolen Vehicle");
		lcd_cursor(2,1);															// Takes the cursor to the first row and first column
		lcd_string("Number: 0");
		_delay_ms(2000);
		//lcd_reset();
		buzzer_off();
		lcd_cursor(1,1);															// making the LCD screen blank
		lcd_string("              ");
		lcd_cursor(2,1);
		lcd_string("              ");
		data = 0x6E;																// the value of the register reset to 0x6E which was the initially set value (NOTE: as long as the value sent by the register is 0x6E,
																					//																				 no instruction is sent to Laptop)
		
	}
	if (data == 0x67) //ASCII value of g
	{
		
		buzzer_on();																//function to turn on the buzzer
		lcd_cursor(1,1);															// Takes the cursor to the first row and first column
		lcd_string("Stolen Vehicle");
		lcd_cursor(2,1);															// Takes the cursor to the first row and first column
		lcd_string("Number: 0");
		_delay_ms(2000);
		//lcd_reset();
		buzzer_off();
		lcd_cursor(1,1);															// making the LCD screen blank
		lcd_string("              ");
		lcd_cursor(2,1);
		lcd_string("              ");
		data = 0x6E;																// the value of the register reset to 0x6E which was the initially set value (NOTE: as long as the value sent by the register is 0x6E,
																					//																				 no instruction is sent to Laptop)
	}
	if (data == 0x68) //ASCII value of h
	{
		
		buzzer_on();																//function to turn on the buzzer
		lcd_cursor(1,1);															// Takes the cursor to the first row and first column
		lcd_string("Stolen Vehicle");
		lcd_cursor(2,1);															// Takes the cursor to the first row and first column
		lcd_string("Number: 0");
		_delay_ms(2000);
		//lcd_reset();
		buzzer_off();
		lcd_cursor(1,1);															// making the LCD screen blank
		lcd_string("              ");
		lcd_cursor(2,1);
		lcd_string("              ");
		data = 0x6E;																// the value of the register reset to 0x6E which was the initially set value (NOTE: as long as the value sent by the register is 0x6E,
																					//																				 no instruction is sent to Laptop)
		
	}
	if (data == 0x69)																//ASCII value of i
	{
		
		buzzer_on();																//function to turn on the buzzer
		lcd_cursor(1,1);															// Takes the cursor to the first row and first column
		lcd_string("Stolen Vehicle");
		lcd_cursor(2,1);															// Takes the cursor to the first row and first column
		lcd_string("Number: 0");
		_delay_ms(2000);
		//lcd_reset();
		buzzer_off();
		lcd_cursor(1,1);															// making the LCD screen blank
		lcd_string("              ");
		lcd_cursor(2,1);
		lcd_string("              ");
		data = 0x6E;																// the value of the register reset to 0x6E which was the initially set value (NOTE: as long as the value sent by the register is 0x6E,
																					//																				 no instruction is sent to Laptop)
		
	}
	if (data == 0x6A)																//ASCII value of j
	{
		
		buzzer_on();																//function to turn on the buzzer
		lcd_cursor(1,1);															// Takes the cursor to the first row and first column
		lcd_string("Stolen Vehicle");
		lcd_cursor(2,1);															// Takes the cursor to the first row and first column
		lcd_string("Number: 0");
		_delay_ms(2000);
		//lcd_reset();
		buzzer_off();
		lcd_cursor(1,1);															// making the LCD screen blank
		lcd_string("              ");
		lcd_cursor(2,1);
		lcd_string("              ");
		data = 0x6E;																// the value of the register reset to 0x6E which was the initially set value (NOTE: as long as the value sent by the register is 0x6E,
																					//																				 no instruction is sent to Laptop)
	}
	
	if (data == 0x75)
	{
		stop();
	}
	
}

void init_devices (void)
{
	cli();																				 //Clears the global interrupts
	port_init();
	adc_init();
	timer5_init();
	uart0_init();																		 //Initialize UART1 for serial communication
	sei();																				//Enables the global interrupts
}


unsigned char ADC_Conversion(unsigned char Ch)             // to convert analog signal received by senor to digital
{
	unsigned char a;
	if(Ch>7)
	{
		ADCSRB = 0x08;
	}
	Ch = Ch & 0x07;
	ADMUX= 0x20| Ch;
	ADCSRA = ADCSRA | 0x40;		//Set start conversion bit
	while((ADCSRA&0x10)==0);	//Wait for ADC conversion to complete
	a=ADCH;
	ADCSRA = ADCSRA|0x10; //clear ADIF (ADC Interrupt Flag) by writing 1 to it
	ADCSRB = 0x00;
	return a;
}



// This Function prints the Analog Value Of Corresponding Channel No. at required Row
// and Coloumn Location.
void print_sensor(char row, char coloumn,unsigned char channel)                // to print value of a sensor on the LCD
{
	ADC_Value = ADC_Conversion(channel);
	lcd_print(row, coloumn, ADC_Value, 3);
}

unsigned int Sharp_GP2D12_estimation(unsigned char adc_reading)          // function to convert data received from sharp IR sensor to the distance of obeject (in mm)
{
	float distance;
	unsigned int distanceInt;
	distance = (int)(10.00*(2799.6*(1.00/(pow(adc_reading,1.1546)))));
	distanceInt = (int)distance;
	if(distanceInt>800)
	{
		distanceInt=800;
	}
	return distanceInt;
}




int main(void)
{
	
	unsigned int value;
	init_devices();										// initialising the various devices and ports
	int numberofcars = 4;								// total number of cars on the side; can be set as per need
	lcd_set_4bit();
	lcd_init();											//// initialising the LCD
    
	UDR0 = 0x6E;										//ASCII value of 'n', which stands for 'nothing' (NOTE: this register is what is used for communication via XBee)
	
	
	while(1)
    {
		sharp = ADC_Conversion(9);						//Stores the Analog value of front sharp connected to ADC channel 9 into variable "sharp"
		value = Sharp_GP2D12_estimation(sharp);			//Stores Distance calculated in a variable "value".
		
		
		//lcd_print(2,14,value,3);
		
		
					
		if (value<200)									// If the distance of the object is less than 200mm.
		{	
			numberofcars = numberofcars - 1;
			stop();
			_delay_ms(1000);
			UDR0 = 0x76;								//ASCII value of 'v', which stands for 'verify'  (NOTE: this register is what is used for communication via XBee and now it instructs the
														//													 laptop to start the image processing module)
														
			_delay_ms(2000);
			forward();									// Done so that the bot moves for sometime without taking note of any object on its side
			velocity(200,200);
			_delay_ms(1000);
			
		}		
         
		forward();
		velocity(200,200);
		
		if(numberofcars == 0)							//goes out of the loop once the given number of cars have been checked
		{
			break;
		}
		
    }
	
	stop();												// Stops the bot.
	_delay_ms(0);	
}
