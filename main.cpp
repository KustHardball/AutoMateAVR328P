/*
 * AutoNut.cpp
 *
 * Created: 26.05.2019 13:48:46
 * Author : Michalich
 * Core : ATMega328P
 */ 
#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <stdlib.h>

#define BAUD 115200  //�������� �������� ������ �� USART
#define MYUBRR 16 //(16000000/(BAUD*16)-1)/1  ������� �� ������������� �� ����������

#define solinoid 7
#define led 5
#define buffersize 32

//����������, ���������� � EEPROM
uint8_t set EEMEM = 0;
uint32_t leng EEMEM = 3;
uint32_t imp EEMEM = 800;
uint32_t rat EEMEM = 9000;
uint32_t loimp EEMEM = 200;
uint32_t spleng EEMEM = 5;

bool flag = true;//���� ��������� ����� 1 ��������
unsigned long impulse;
unsigned long lowimpulse;
unsigned long rate;
int splength; //���������� ��������� ��� ������ ������������ ���������������� ��� ������������� ��������
uint8_t settings;   //���� � ���������� ������� ������ ����� 8-������� �������, 7-����� ������������ ����������������, 6-����� ������������� ��������, ��������� �������������
int state; //���� ����������
bool licen =false;  //���� ����������
int count;  //���������� ��� �������� ���������� ��������� � �������
int length;  //����� ������� � ������ �������
char buffer[buffersize]; //����� ������ ��� USART
bool done = false; //�������� � ������� � ������ USART ������ ��� ���������
volatile unsigned char IDX;  //������ ���������� ������� ������ USART
int powervoltage; //����������, ���������� �� ����� �������� ����������
int pressure;
char strbuffer[20];
int conti;
bool trig=false;

int ReadVolt();
int ReadTemperature();
void USORT_Init(unsigned int ubrr);
void USORT_Transmit(unsigned char data);
void timerini ();
void pinini();
void acpini();
void settingsread();
void USARTWrite(char string[]);
void parse();
void settingssave();
void writesettings();
char* tostring(int num);
void sendstat();
void shot();

ISR (USART_RX_vect)  //������ ���������� ��������� ������ �� USART
{
	if (done==false)
	{
	char bf= UDR0;
	buffer[IDX]=bf;
	IDX++;
	//USORT_Transmit(bf);	
	if (bf == '^' || IDX >= buffersize)
	{
		IDX=0;
		done=true;		
	}
		if (bf == '\n' || bf == ' ') 
		{
			IDX=0;
			for (int i=0; i<buffersize; i++) buffer[i]=NULL;
			done=false;
		}	
	
	unsigned char dummy; 
    while ( UCSR0A & (1<<RXC0) ) dummy = UDR0;
	}
}
ISR (TIMER0_COMPA_vect) //������ ���������� ������� 0
	{
	static int iter = 0;
	iter++;
	if (iter==300){ //�� ����� �������� �������� ����������
		//PORTB |= 1<<1;
		trig = true;
		//powervoltage = ReadVolt();
		//PORTB &= ~(1<<1);
	}
	if (iter==impulse+impulse){  //���� � ������� �������� ������ ������� �� �������, �������� ���������� �� USART
		//PORTB |= 1<<1;
		sendstat();
		//PORTB &= ~(1<<1);
	}
	if (iter==impulse){
	PORTD &= ~(1<<solinoid);
	PORTB &= ~(1<<1);
	 count++;
	if ((!(~settings & 0b10000000) & count >= conti) | conti ==1) {
		licen = false;
	}
	}
	else if (iter==rate){	// ��������� ���� ��������
		iter=0;
		flag = true;
		//cli();  
		TCCR0B = 0;  //������������� ������
		UCSR0B = (1<<RXEN0)|(1<<TXEN0) |(1<<RXCIE0);// ����� ��������� ���������� �� USART
	}
}



int main(void)
{
	pinini(); //������������ ������ � ����������� ��������� ��������    
	acpini();//������������ ��� ��� �������� ���������� �������
	USORT_Init(MYUBRR);  //������������� USART ��� ������ ������� �� Bluetooth	
	settingsread();//������ ��������� �� ����������������� ������
	timerini(); //������������� ������� 0
	int contcount = 0;
	
    while (1) {//�������� ����!!!!!!!!!!!!!!!
	   if(!(~PIND & 0b00100000)& flag == true){  // �������� ������ � ��������� ����� �� ��������� ��������� �� ����� ����� ��������
		   conti=1;	 
	   }
	   else if (~PIND & 0b00100000) {
		   if (flag == true){
		    conti = length;
		   }
	   }	   	     
	   for(int i; i<1;i++); //�������� ����, ��� ���� ������ �� ������������ � ��������� �����
	   
		if (!(~PIND & 0b00001000)& flag == true & licen == true){
			shot();				
		}
		else if (~PIND & 0b00001000){
			 licen= true;
			 count =0;
			 }				 
		if (done)
		{
			parse();
			for (int i=0; i<buffersize; i++) buffer[i]=NULL; // ������� �����
			done=false;
		}
		if(trig){  //�������� ������ � �������� � �������� ����, � �� � ����������
			powervoltage = ReadVolt();
			trig = false;
			//PORTB &= ~(1<<1);
		}
		if (count != contcount){
			//if (count!=0) sendstat();
			contcount = count;
		}
    }
}



void shot()  //�������
{
	flag = false;
	PORTD |= 1<<solinoid;
	PORTB |= 1<<1;
	sei();
	TCCR0B=(1<<CS01);
	UCSR0B &= ~(1<<RXCIE0);	// ��������� ���������� USART ��� ������������� ����� �������� �� �� ����������� ���������
}
int ReadVolt(){   //����� ����������
	ADMUX=0b01000100;  //����� ����� ��� �������� ���������� �����
	ADCSRA |= 0b01000000;  //������ ������ ���������� �� ����
	while ((ADCSRA & (1 << ADIF)) == 0);  //�������� ���������� ������ ����������
	return ADCW;
}
int ReadTemperature(){   //����� ����������
	ADMUX=0b01001000;  //����� ����� ��� ������ �����������
	ADCSRA |= 0b01000000;  //������ ������ ���������� �� ����
	while ((ADCSRA & (1 << ADIF)) == 0);  //�������� ���������� ������ ����������
	return ADCW;
}
void USORT_Init(unsigned int ubrr){ //������������� USART
	UBRR0L = (unsigned char)ubrr;
	UCSR0B = (1<<RXEN0)|(1<<TXEN0) |(1<<RXCIE0); //��������� �������� � ��������� ������, ���������� �� ��������� �����
	UCSR0C = (1<<USBS0)|(3<<UCSZ00); //�������� ������ �� 8 ��� � 2-�� ����-������
}
void USORT_Transmit( unsigned char data )
{
	/* Wait for empty transmit buffer */
	while ( !( UCSR0A & (1<<UDRE0)) );
	/* Put data into buffer, sends the data */
	UDR0 = data;
}
void timerini (){  //������������� �������
	  TCCR0A |= (1<<WGM01); //set ctc bit
	  OCR0A |= 20; // 10 ��� ����������� �� https://eleccelerator.com/avr-timer-calculator/
	  TIMSK0 |= (1<<OCIE0A);
	  sei();
	  //TCCR0B=(1<<CS02)|(1<<CS00);
}
void pinini(){  //������������� ������
	 DDRB = 0b00100010;  //����������� ��� PB5 � PB1 �� �����
	 DDRD = 0b10000000;  //����������� ��� PD7 �� �����
	 PORTB |= 1<<led;  //�������� ��������� ������ �����
	 PORTD &= ~(1<<solinoid);  //��������� ��������
}
void acpini(){  //������������� ���
	ACSR=0x80;  //��������� �����������
	ADCSRA=0b10000101;  //��������� ���
}
void settingsread(){
	impulse = eeprom_read_dword(&imp);  //����� �������� ��������� � ���
	rate= eeprom_read_dword(&rat);  //����� ����� ���������� � ��
	settings = eeprom_read_byte(&set);  //���� �������� ������� ������ (����� � ������ 0)
	length= eeprom_read_dword(&leng);  //����� ������� (����� � ������ 10)
	lowimpulse = eeprom_read_dword(&loimp);  //����� �������� ��������� � ���
	splength= eeprom_read_dword(&spleng);
}
void USARTWrite(char * string){  //�������� ����� �� USART
	while(*string != '\0')
	{
		USORT_Transmit(*string);
		string++;
	}
}
void parse()
{
	USARTWrite(buffer);
	char ch [10];
	int val;
	switch (buffer[0]){
		case 'A':   // ���������� �������� ��������
		for(int i =0;i<10; i++) ch[i]=buffer[i+1];
		val =atoi(ch);
		if ((val>=200) & (val<=2000)){
		impulse = val;
		USARTWrite("OK");
		USARTWrite(tostring(impulse));
		USARTWrite("*");
		USARTWrite("\n");
		}
		else USARTWrite("ERR\n");
		break;
		
		case 'P':   // ���������� �������� ������� ��������
		for(int i =0;i<10; i++) ch[i]=buffer[i+1];
		val =atoi(ch);
		if ((val>=200) & (val<=2000)){
			lowimpulse = val;
			USARTWrite("OK");
			USARTWrite(tostring(lowimpulse));
			USARTWrite("*");
			USARTWrite("\n");
		}
		else USARTWrite("ERR\n");
		break;
		
		case 'B':   // ���������� �������� �������� ����� ����������
		for(int i =0;i<10; i++) ch[i]=buffer[i+1];
		val =atoi(ch);
		if ((val>=5000) & (val<=20000)){
		rate = val;
		USARTWrite("OK");
		USARTWrite(tostring(rate));
		USARTWrite("*");
		USARTWrite("\n");
		}
		else USARTWrite("ERR\n");
		break;
		
		case 'X':   // ���������� �������� �������
		for(int i =0;i<10; i++) ch[i]=buffer[i+1];
		val =atoi(ch);
		if ((val>=2) & (val<=250)){
		length = val;
		USARTWrite("OK");
		USARTWrite(tostring(length));
		USARTWrite("\n");
		}
		else USARTWrite("ERR\n");
		break;
		
		case 'Z':   // ���������� �������� ����� ������������ �������
		for(int i =0;i<10; i++) ch[i]=buffer[i+1];
		val =atoi(ch);
		if ((val>=2) & (val<=30)){
		splength = val;
		USARTWrite("OK");
		USARTWrite(tostring(splength));
		USARTWrite("\n");
		} 
		else USARTWrite("ERR\n");
		break;
		
		case 'R':   // ������ ���������
		writesettings();
		break;
		
		case 'S':  //���������� ��������
		settingssave();  // ��������� ��������� �� ����������� ������ � EEPROM
		break;
		
		case 'C':   // ������������ ������ ������� 
		if (!(~settings & 0b10000000)){
			 settings &= ~(1<<7);
			 USARTWrite("OK0\n");
		}
		else if (~settings & 0b10000000) {
			settings |=1<<7;
			USARTWrite("OK1\n");
		}
		break;
		
		case 'D':   // ������������ ������ ������������ ����������������
		if (!(~settings & 0b01000000)){
			settings &= ~(1<<6);
			USARTWrite("OK0\n");
		}
		else if (~settings & 0b01000000) {
			settings |=1<<6;
			USARTWrite("OK1\n");
		}
		break;
		
		case 'O':   // ������������ ������ ������������� ��������
		if (!(~settings & 0b00100000)){
			settings &= ~(1<<5);
			USARTWrite("OK0\n");
		}
		else if (~settings & 0b00100000) {
			settings |=1<<5;
			USARTWrite("OK1\n");
		}
		break;
		
		case 'K':  //�������������� �����
		USARTWrite("AutoNut\n");
		break;
		
		case '|':  //�������� 10��� � ��������
		if (impulse < 2000){
			impulse +=1;
			USARTWrite("OK");
			USARTWrite(tostring(impulse));
			USARTWrite("*");
			USARTWrite("\n");
		}
		else USARTWrite("ERR\n");
		break;
		
		case '-':  //������� 10��� � ��������
		if (impulse > 200){
			impulse -=1;
			USARTWrite("OK");
			USARTWrite(tostring(impulse));
			USARTWrite("*");
			USARTWrite("\n");
		}
		else USARTWrite("ERR\n");
		break;
		
		case '<':  //�������� 10��� � ��������
		if (impulse < 2000){
			impulse +=1;
			USARTWrite("OK");
			USARTWrite(tostring(impulse));
			USARTWrite("*");
			USARTWrite("\n");
		}
		else USARTWrite("ERR\n");
		break;
		
		case 'H':  //��������� �������� impulse lowimpulse
		lowimpulse = impulse;
		USARTWrite("OK Lowimpulse = impulse\n");
		break;
		case 'Q':  //���
		USARTWrite("*");
		USARTWrite("\n");
		break;
			
		case 'F':  //���������� �������
		if (flag == true & licen == true){
			shot();
		}
		break;	
			
		default:
		USARTWrite("IC\n");
		break;
	}
}
void settingssave() //���� ���������� �������� ����� �� �������������� ���������� (���������� �������� ����� ����� ������� � ��������� ������ ������)
{
	int val=0;
	if (eeprom_read_dword(&imp)!=impulse){
		 eeprom_write_dword(&imp,impulse);
		 val++;
	}
	if (eeprom_read_dword(&rat)!=rate){
		 eeprom_write_dword(&rat,rate);
		 val++;
	 }
	if (eeprom_read_byte(&set)!=settings) {
		eeprom_write_byte(&set,settings);
		val++;
	}
	if (eeprom_read_dword(&leng)!=length){
		 eeprom_write_dword(&leng,length);
		 val++;
	 }
	if (eeprom_read_dword(&loimp)!=lowimpulse){
		 eeprom_write_dword(&loimp,lowimpulse);
		 val++;
	 }
	if (eeprom_read_dword(&spleng)!=splength){
		 eeprom_write_dword(&spleng,splength);
		 val++;
	 }
	 if (val>0) USARTWrite("S^OK\n");
	 else USARTWrite("ERR\n");
	
}
void writesettings()
{
	//USARTWrite("Impulse = ");
	USARTWrite(tostring(impulse));
	USARTWrite("*");
	//USARTWrite(" Lowimpulse = ");
	USARTWrite(tostring(lowimpulse));
	USARTWrite("*");
	//USARTWrite(" Rate = ");
	USARTWrite(tostring(rate));
	USARTWrite("*");
	USARTWrite(tostring(length));
	USARTWrite("*");
	USARTWrite(tostring(splength));
	int ind =0;
	if (!(~settings & 0b10000000)){ 
		 ind++;
		 }
	if (!(~settings & 0b01000000)){ 
		ind+=2;
	}
	if (!(~settings & 0b00100000)){
		ind+=4;
	}
	if (settings ==0){
		//USARTWrite("normal");
		ind=0;
	}
	USARTWrite("*");
	USARTWrite(tostring(ind));
	USARTWrite("*");
	//USARTWrite(" Voltage = ");
	USARTWrite(tostring(ReadVolt()));
	USARTWrite("*");
	//USARTWrite(" Pressure = ");
	USARTWrite("55");
	USARTWrite("*");
	//USARTWrite(" Speed = ");
	USARTWrite("160");
	USARTWrite("*");
	USARTWrite("\n");
}
char* tostring(int num)
{
	return itoa(num,strbuffer,10);
}
void sendstat()  //������� ��� �������� ���������� ��� ������ �������� (���� ��������)
{
	USARTWrite("LT");
	USARTWrite(tostring(impulse));
	USARTWrite("*");
	USARTWrite(tostring(powervoltage));
	USARTWrite("*");
	USARTWrite(tostring(150 + rand() % ((600 + 1) - 150))); // ������
	USARTWrite("*");
	USARTWrite(tostring(132 + rand() % ((195 + 1) - 132)));
	USARTWrite("*");
	USARTWrite("\n");	
}