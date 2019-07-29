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

#define BAUD 115200  //скорость передачи данных по USART
#define MYUBRR 16 //(16000000/(BAUD*16)-1)/1  формула из доукументации на микросхему

#define solinoid 7
#define led 5
#define buffersize 32

//переменные, хрянящиеся в EEPROM
uint8_t set EEMEM = 0;
uint32_t leng EEMEM = 3;
uint32_t imp EEMEM = 800;
uint32_t rat EEMEM = 9000;
uint32_t loimp EEMEM = 200;
uint32_t spleng EEMEM = 5;

bool flag = true;//флаг окончания цикла 1 импульса
unsigned long impulse;
unsigned long lowimpulse;
unsigned long rate;
int splength; //количество импульсов для режима динамической скорострельности или динамического импульса
uint8_t settings;   //байт с регистрами режимов работы платы 8-наличие отсечки, 7-режим динамической скорострельности, 6-режим динамического импульса, остальные резервированы
int state; //флаг разрешения
bool licen =false;  //флаг разрешения
int count;  //переменная для подсчета количества импульсов в очереди
int length;  //длина очереди в режиме отсечки
char buffer[buffersize]; //буфер данных для USART
bool done = false; //сообщает о наличии в буфере USART пакета для обработки
volatile unsigned char IDX;  //индекс заполнения массива буфера USART
int powervoltage; //напряжение, замеряемое во время нагрузки соленоидом
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

ISR (USART_RX_vect)  //вектор прерывания получения данных по USART
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
ISR (TIMER0_COMPA_vect) //вектор прерывания таймера 0
	{
	static int iter = 0;
	iter++;
	if (iter==300){ //во время импульса замеряем напряжение
		//PORTB |= 1<<1;
		trig = true;
		//powervoltage = ReadVolt();
		//PORTB &= ~(1<<1);
	}
	if (iter==impulse+impulse){  //если с момента импульса прошло столько же времени, отсылаем статистику по USART
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
	else if (iter==rate){	// завершаем цикл импульса
		iter=0;
		flag = true;
		//cli();  
		TCCR0B = 0;  //останавливаем таймер
		UCSR0B = (1<<RXEN0)|(1<<TXEN0) |(1<<RXCIE0);// снова разрешаем прерывания по USART
	}
}



int main(void)
{
	pinini(); //конфигурация портов и выставление начальных значений    
	acpini();//конфигуряция АЦП для контроля напряжения питания
	USORT_Init(MYUBRR);  //инициализация USART для обмена данными по Bluetooth	
	settingsread();//читаем настройки из энергонезависимой памяти
	timerini(); //инициализация таймера 0
	int contcount = 0;
	
    while (1) {//ОСНОВНОЙ ЦИКЛ!!!!!!!!!!!!!!!
	   if(!(~PIND & 0b00100000)& flag == true){  // проверка режима с проверкой флага во избежание изменений во время цикла импульса
		   conti=1;	 
	   }
	   else if (~PIND & 0b00100000) {
		   if (flag == true){
		    conti = length;
		   }
	   }	   	     
	   for(int i; i<1;i++); //холостой цикл, без него таймер не сбрасывается и программа сбоит
	   
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
			for (int i=0; i<buffersize; i++) buffer[i]=NULL; // очищаем буфер
			done=false;
		}
		if(trig){  //получаем данные с датчиков в основном коде, а не в прерывании
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



void shot()  //выстрел
{
	flag = false;
	PORTD |= 1<<solinoid;
	PORTB |= 1<<1;
	sei();
	TCCR0B=(1<<CS01);
	UCSR0B &= ~(1<<RXCIE0);	// запрещаем прерывания USART для предохранения цикла импульса от не желательных процессов
}
int ReadVolt(){   //замер напряжения
	ADMUX=0b01000100;  //выбор порта для контроля напряжения платы
	ADCSRA |= 0b01000000;  //запуск замера напряжения на ноге
	while ((ADCSRA & (1 << ADIF)) == 0);  //ожидание завершения замера напряжения
	return ADCW;
}
int ReadTemperature(){   //замер напряжения
	ADMUX=0b01001000;  //выбор порта для замера температуры
	ADCSRA |= 0b01000000;  //запуск замера напряжения на ноге
	while ((ADCSRA & (1 << ADIF)) == 0);  //ожидание завершения замера напряжения
	return ADCW;
}
void USORT_Init(unsigned int ubrr){ //инициализация USART
	UBRR0L = (unsigned char)ubrr;
	UCSR0B = (1<<RXEN0)|(1<<TXEN0) |(1<<RXCIE0); //разрешаем передачу и получение данных, прерывания на получение байта
	UCSR0C = (1<<USBS0)|(3<<UCSZ00); //выбираем пакеты по 8 бит с 2-мя стоп-битами
}
void USORT_Transmit( unsigned char data )
{
	/* Wait for empty transmit buffer */
	while ( !( UCSR0A & (1<<UDRE0)) );
	/* Put data into buffer, sends the data */
	UDR0 = data;
}
void timerini (){  //инициализация таймера
	  TCCR0A |= (1<<WGM01); //set ctc bit
	  OCR0A |= 20; // 10 мкС расчитываем на https://eleccelerator.com/avr-timer-calculator/
	  TIMSK0 |= (1<<OCIE0A);
	  sei();
	  //TCCR0B=(1<<CS02)|(1<<CS00);
}
void pinini(){  //инициализация портов
	 DDRB = 0b00100010;  //настраиваем пин PB5 и PB1 на выход
	 DDRD = 0b10000000;  //настраиваем пин PD7 на выход
	 PORTB |= 1<<led;  //включаем индикатор работы платы
	 PORTD &= ~(1<<solinoid);  //отключаем соленоид
}
void acpini(){  //инициализация АЦП
	ACSR=0x80;  //настройка компаратора
	ADCSRA=0b10000101;  //настройка АЦП
}
void settingsread(){
	impulse = eeprom_read_dword(&imp);  //длина импульса соленоида в мкс
	rate= eeprom_read_dword(&rat);  //время между импульсами в мс
	settings = eeprom_read_byte(&set);  //байт настроек режимов работы (адрес в памяти 0)
	length= eeprom_read_dword(&leng);  //длина отсечки (адрес в памяти 10)
	lowimpulse = eeprom_read_dword(&loimp);  //длина импульса соленоида в мкс
	splength= eeprom_read_dword(&spleng);
}
void USARTWrite(char * string){  //передача стоки по USART
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
		case 'A':   // установить значение импульса
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
		
		case 'P':   // установить значение нижнего импульса
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
		
		case 'B':   // установить значение задержки между импульсами
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
		
		case 'X':   // установить значение отсечки
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
		
		case 'Z':   // установить значение длины динамических режимов
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
		
		case 'R':   // читать настройки
		writesettings();
		break;
		
		case 'S':  //сохранение настроек
		settingssave();  // сохраняем настройки из оперативной памяти в EEPROM
		break;
		
		case 'C':   // переключение режима отсечки 
		if (!(~settings & 0b10000000)){
			 settings &= ~(1<<7);
			 USARTWrite("OK0\n");
		}
		else if (~settings & 0b10000000) {
			settings |=1<<7;
			USARTWrite("OK1\n");
		}
		break;
		
		case 'D':   // переключение режима динамической скорострельности
		if (!(~settings & 0b01000000)){
			settings &= ~(1<<6);
			USARTWrite("OK0\n");
		}
		else if (~settings & 0b01000000) {
			settings |=1<<6;
			USARTWrite("OK1\n");
		}
		break;
		
		case 'O':   // переключение режима динамического импульса
		if (!(~settings & 0b00100000)){
			settings &= ~(1<<5);
			USARTWrite("OK0\n");
		}
		else if (~settings & 0b00100000) {
			settings |=1<<5;
			USARTWrite("OK1\n");
		}
		break;
		
		case 'K':  //аутентификация платы
		USARTWrite("AutoNut\n");
		break;
		
		case '|':  //добавить 10мкС к импульсу
		if (impulse < 2000){
			impulse +=1;
			USARTWrite("OK");
			USARTWrite(tostring(impulse));
			USARTWrite("*");
			USARTWrite("\n");
		}
		else USARTWrite("ERR\n");
		break;
		
		case '-':  //убавить 10мкС у импульса
		if (impulse > 200){
			impulse -=1;
			USARTWrite("OK");
			USARTWrite(tostring(impulse));
			USARTWrite("*");
			USARTWrite("\n");
		}
		else USARTWrite("ERR\n");
		break;
		
		case '<':  //добавить 10мкС к импульсу
		if (impulse < 2000){
			impulse +=1;
			USARTWrite("OK");
			USARTWrite(tostring(impulse));
			USARTWrite("*");
			USARTWrite("\n");
		}
		else USARTWrite("ERR\n");
		break;
		
		case 'H':  //присвоить значение impulse lowimpulse
		lowimpulse = impulse;
		USARTWrite("OK Lowimpulse = impulse\n");
		break;
		case 'Q':  //эхо
		USARTWrite("*");
		USARTWrite("\n");
		break;
			
		case 'F':  //произвести выстрел
		if (flag == true & licen == true){
			shot();
		}
		break;	
			
		default:
		USARTWrite("IC\n");
		break;
	}
}
void settingssave() //ищем измененные значения чтобы не перезаписывать одинаковые (перезапись занимает очень много времени и уменьшает ресурс памяти)
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
void sendstat()  //функция для передачи статистики при каждом импульсе (пока заглушка)
{
	USARTWrite("LT");
	USARTWrite(tostring(impulse));
	USARTWrite("*");
	USARTWrite(tostring(powervoltage));
	USARTWrite("*");
	USARTWrite(tostring(150 + rand() % ((600 + 1) - 150))); // рандом
	USARTWrite("*");
	USARTWrite(tostring(132 + rand() % ((195 + 1) - 132)));
	USARTWrite("*");
	USARTWrite("\n");	
}