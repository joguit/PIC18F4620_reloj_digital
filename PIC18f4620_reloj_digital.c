/*
 *
 *dificultades superadas:
 *daba error atoi , se añadió el caracter final de linia '\0' para evitar el fallo
 *uso caracteres extendidos en linea 147
 *
 *
 *
 *
 
 */
 

// set configuration words

//#define  led_rojo PORTDbits.RD0//PORTDbits.RD0
#define _XTAL_FREQ 8000000

#pragma config OSC=INTIO67,MCLRE=ON,WDT=OFF,LVP=OFF //INTIO67->0xF8	Internal oscillator block, port function on RA6 and RA7.

#include <xc.h>
#include <stdint.h>        // include stdint header
#include <stdio.h> 
#include <usart.h>
#include <delays.h>
#include <stdlib.h>
#include <time.h>

//definicion de variables

//#define  interruptor_1 PORTDbits.RD3//PORTDbits.RD1
//#define  interruptor_2  PORTDbits.RD2//PORTDbits.RD0
#define  led_ambar  PORTCbits.RC3//output=0
#define  led_rojo PORTDbits.RD0//PORTDbits.RD0
#define  interruptor_2  PORTDbits.RD2//PORTDbits.RD0
#define  pulsador_2 PORTDbits.RD4//PORTDbits.RD4  inputs=1 trisd 00011100
#define  zumbador PORTDbits.RD7
#define puente_1 PORTCbits.RC2


unsigned int CO2[21]={395,415,440,470,500,506,533,547,560,572,700,738,1012,1217,1276,1506,1905,2240,3325,4700,5700};
float volt[21]={0.5,0.60,0.70,0.8,0.90,0.91,0.96,0.98,0.99,1.01,1.04,1.05,1.09,1.1,1.115,1.15,1.2,1.22,1.29,1.33,1.37};

int nueva_hora, nueva_min,nuevo_mes,nuevo_dia_del_mes,nuevo_anio;
unsigned int sensor_CO2;
uint16_t raw_temp;
char  temp[] = "000.0000 C";
char texto[6]=" ";
float volts;
float interpol=0;
unsigned short numero=45000;  //límites unsigned short: 0 a 65535 -límite c02
//unsigned short numero=0xFFFF;  //límites: 0 a 65535
unsigned char address_00 = 0x00;//direccion memoria eeprom nueva hora
unsigned char address_01 = 0x01;//diceecion memoria eeprom nuevos minutios
unsigned char address_02 = 0x02;//direccion memoria eeprom nuevos dia mes
unsigned char address_03 = 0x03;//direccion memoria eeprom nuevo mes
unsigned char address_04 = 0x04;//direccion memoria eeprom nuevo año-1
unsigned char address_05 = 0x05;//direccion memoria eeprom nuevo año-2
//alarmas
unsigned short MAX_CO2=1500;
char estado_zumbador='n';
//no usado
struct datos_eprom {
    int MAX_CO2;
    char alarma;
};

//funciones
unsigned int read_analog_channel(unsigned int); 
unsigned char getch();
uint8_t UART_GetC();
void UART_Read_Text(char *, unsigned int );
int  atoi( const char * s ); 

void setup(void);


char FuncionMenuMaxCo(void);
char FuncionLeeAlarmamOnOff(void);
unsigned short FuncionSeleccion(char letra);
unsigned short LeeMaxCo2Eeprom(void);
int i,j=0;



//funciones de entrada hora
int lectura_hora(void);
int lectura_min(void);
int lectura_dia_del_mes(void);
int lectura_mes(void);
int lectura_anio(void);


/*************************** main function *********************/

//
//struct tm {
//int tm_sec;       /*seconds after the minute ( 0 to 61 )*/
                  /*allows for up to two leap seconds*/
//int tm_min;       /*minutes after the hour ( 0 to 59 )*/
//int tm_hour;      /*hours since midnight ( 0 to 23 )*/
//int tm_mday;      /*day of month ( 1 to 31 )*/
//int tm_mon;       /*month ( 0 to 11 where January = 0 )*/
//int tm_year;      /*years since 1900*/
//int tm_wday;      /*day of week ( 0 to 6 where Sunday = 0 )*/
//int tm_yday;      /*day of year ( 0 to 365 where January 1 = 0 )*/
//int tm_isdst;     /*Daylight Savings Time flag*/
//}

//cambio horario:
//noche del sábado 26 de marzo al domingo 27 de marzo de 2022 de 2.00 a 3.00
//sábado 30 al domingo 31 de octubre. a las 03.00 horas del día 31, cuando los relojes se atrasan a las 2.00
 //una hora y volverán a ser las 02.00 horas 
 

//variable de tipo tiempo 
time_t timer, whattime;
struct tm *newtime;
const char * wday[] = {
 "Sunday", "Monday", "Tuesday", "Wednesday",
"Thursday", "Friday", "Saturday"};


 struct tm when;
	
void main(void)
{

    OSCCON = 0x70;   // set internal oscillator to 8MHz
    setup(); // Configure the PIC
	led_ambar=0;
	
  
    printf("\r\rInicio programa....\n");
    struct datos_eprom var1_eprom;
	for (int i=0;i<256;i++)
		printf("%d,%x=%c  ",i,i,i);
	
	
if( pulsador_2==0 )
	printf("\npulsado");
else
  	printf("\nno pulsado");


	printf("\nCaracteres extendidos ejemplo: %c+%c+%c",'a',0XF1,0XBA);
	printf("\nejemplo1: a\xF1o, ejemplo2: %c",'p');
	lectura_hora();
    lectura_min();
	lectura_dia_del_mes();
	lectura_mes();
    lectura_anio();
	
	
//guardar lecturas en eeprom:
//prueba para escribir unsigned short en eeprom
	//char admite de 0 a 255
	//convertimos numero en dos char
/*	volatile unsigned char nu1=numero>>8;
	unsigned char address_2a = 0x02;//;
	eeprom_write(address_2a, nu1); 
	unsigned char p=eeprom_read (address_2a); 
	printf("\reeprom read adress_2a :%x ",p);
	
	volatile unsigned char nu2=numero & 0xFF;
	unsigned char address_2b = 0x03;//;
	eeprom_write(address_2b, nu2); 
*/	
	eeprom_write(address_00, nueva_hora);
	unsigned char p=eeprom_read (address_00); 
    printf("\n memoria eeprom hora:%u",p);//%u para decimal sin signo
	
	eeprom_write(address_01, nueva_min);
	unsigned char p=eeprom_read (address_01); 
    printf("\n memoria eeprom min:%u",p);//%u para decimal sin signo
	
	eeprom_write(address_02, nuevo_mes);
	unsigned char p=eeprom_read (address_02); 
    printf("\n memoria eeprom mes:%u",p);//%u para decimal sin signo
	
	eeprom_write(address_03, nuevo_dia_del_mes);
	unsigned char p=eeprom_read (address_03); 
    printf("\n memoria eeprom dia del mes:%u",p);//%u para decimal sin signo
	
	//para el año hay que partir el numero:
	volatile unsigned char nu1=nuevo_anio>>8; 
	eeprom_write(address_04, nu1); 
	
	volatile unsigned char nu2=nuevo_anio & 0xFF;
	eeprom_write(address_05, nu2);
	
	
    printf("\rnu1 y nu2 :%x ,%x",nu1,nu2);
	//convertimos 2 char en numero decimal sin signo
	unsigned short resultado9=(nu1<<8 )+ nu2;
	printf("\r memoria eeprom a%co :%u",0XF1,resultado9);//%u para decimal sin signo
  	

	
	
//asignamos los valores al reloj actual:	
	
	when.tm_sec = 30;
    when.tm_min = nueva_min;
    when.tm_hour = nueva_hora;
    when.tm_mday = nuevo_dia_del_mes;
    when.tm_mon = nuevo_mes;
    when.tm_year = nuevo_anio-1900;
	 //mktime convierte los datos de extructura en tipo time_t (UNIX TIMESTAMP)
	whattime = mktime(&when);
	//asctime convierte la extructura en una cadena para visualizar:
    printf("\n\nDia y hora es: %s\n", asctime(&when));
	
	// whattime = mktime(newtime);
    printf("Calendar UNIX TIMESTAMP: time_t = %ld\n", whattime);
	
	//Creamos nueva variable tipo UNIX TIMESTAMP:
	timer=whattime; //
	//timer =1635628680; /* Mon Oct 20 16:43:02 2003 */
	
     
    //asignamos horario veran o invierno:(debe precisarse mas)
    if ( (newtime->tm_mon >2) && (newtime->tm_mon <10))
      {
        printf("horario verano\n");
        timer= timer+(2*3600);//horario verano UTC+2
      }
	else 
	  {
        printf("horario invierno\n");
	    timer= timer+(1*3600);//horario verano UTC+1
	  }
	
	
	
	while(1)
	{
	//timer = 1066668182; /* Mon Oct 20 16:43:02 2003 */
	
  /* localtime allocates space for struct tm */
  newtime = localtime(&timer);
  printf("\nLocal time = %s", asctime(newtime));
  printf("hoy es  = %s\n",wday[newtime->tm_wday]);
  printf("la hora es  = %d h\n",newtime->tm_hour);
  printf("los minutos son = %d \n",newtime->tm_min);
  printf("el mes es  = %d \n",newtime->tm_mon);
  printf("el a%co es  = %d \n",0XF1,newtime->tm_year+1900);//newtime->tm_year=121 (+1900=2021)
  printf("los segundos son = %d \n",newtime->tm_sec);
  int segundos=newtime->tm_sec;
  //condicion tarea1:
  if(segundos>15 && segundos< 30)
	  led_ambar=1;
  else
	  led_ambar=0;
   
   Delay10KTCYx(200);//pausa un segundo
   ++timer;
   led_rojo=~led_rojo;
   //led_ambar=~led_ambar;
   }
	
	
//fin while
}

/*************************** end main function ********************************/
void setup(void)
{
   //contador=0;
    //enable_count=0;
	
    // Set clock frequency (section 2 of the PIC18F4620 Data Sheet)
    // Set Fosc = 8MHz, which gives Tcy = 0.5us
    //OSCCON = 0b01110000;//original
    //  OSCCON = 0b01100000; //0110=4mhz
    // Set up ADC (section 19 of the PIC18F4620 Data Sheet)
    // Enable AN0-7 as analog inputs
    ADCON1 = 0b00000111;
    // Left justified result, manual acquisition time,
    // AD clock source = 8 x Tosc
    ADCON2 = 0b00000001;
  
    // Set up Port B (section 9.2 of the PIC18F4620 Data Sheet)
    // RB0-5 outputs, RB6-7 inputs
    LATB = 0b00000000;
    TRISB = 0b11000000;
    TRISCbits.RC2=1; //input puente_1
	TRISCbits.RC3=0;//output led_ambar
    // Set up Port D (section 9.4 of the PIC18F4620 Data Sheet)
    //------------CAMBIAR --> RD0-3 digital outputs, RD4-7 digital inputs
    LATD = 0b00000000;
    //TRISD = 0b11110000;
    TRISD = 0b01111100;// 0b11111100
	TRISDbits.RD0=0; //0=output led rojo
    //led_rojo=0;//inicilmente apagar led
    
    
    // Set up PWM (section 15 of the PIC18F4620 Data Sheet)
    // Set PWM frequency=36kHz and duty cycle=50% on both channels
    CCP1CON = 0b00001100;   // PWM on CCP1
    CCP2CON = 0b00001100;   // PWM on CCP2
    TRISC = 0b11110001;     // CCP1, CCP2 as outputs
    T2CON = 0b00000100;     // Enable TMR2 with prescale = 1:1
    PR2 = 55;               // period = (PR2+1) * Tcy * prescale
    CCPR1L = 27;            // Ch 1 duty cycle = CCPR1L / PR2
    CCPR2L = 27;            // Ch 2 duty cycle = CCPR1L / PR2
     
     
    // Set up USART (section 18 of the PIC18F4620 Data Sheet)
    // baud_rate = Fosc/(16*(spbrg+1))
    //           = 8000000/(16*(207+1)) = 2403.846 bits/s
    OpenUSART(USART_TX_INT_OFF & USART_RX_INT_OFF
        & USART_ASYNCH_MODE & USART_EIGHT_BIT
        & USART_CONT_RX & USART_BRGH_HIGH, 51);//valor 207 para 2400 baud, 51 para 9600, 25 para 9600 y 4000000
}

//funciones para puerto serie
void putch(char data) {    
     while (!TXIF)    
         continue;    
     TXREG = data;    
 }    
uint8_t UART_GetC()
   {     
         while  (RCIF == 0) ; //ESPERA RECIBIR DATO
        if (OERR)	//SI HAY ERROR
        {  //LIMPIA OVERRRUN BIT DE ERROR
			CREN=0;
			CREN=1;
		}	
       		
		return RCREG;
   }

 unsigned char getch(void)
 {/*retrieve one byte*/
    while (!RCIF)  /*SET WHEN REGISTER IS NOT EMPTY*/
	//_delay(100);
	continue;
    return RCREG;
 }
 
 void UART_Read_Text(char *Output, unsigned int length)
{
  unsigned int i;
  for(int i=0;i<length;i++)
  Output[i] = UART_GetC();
}
 
 // Read voltage from the specified channel.
// This function takes approximately 35us.
unsigned int read_analog_channel(unsigned int n)
{
    unsigned int voltage;
  
    ADCON0 = n << 2;
    ADCON0bits.ADON = 1;
    Delay10TCYx(3); // 15us charging time
    ADCON0bits.GO = 1;
    while (ADCON0bits.GO); // Await result (11us approx)
  
    // Return the result (a number between 0 and 1023)
    voltage = ADRESH;
    voltage = (voltage << 2) + (ADRESL >> 6);
    return voltage;
}

//funcion para obtener hora;
int lectura_hora(void)
{//Obtener nueva hora del reloj
    printf("\nIntroduce nueva hora GTM(2 digitos)");
	printf("\nla hora local es GTM+1 en invierno");
	printf("\nla hora local es GTM+2 en verano:");
	UART_Read_Text(texto,2);
	printf("nueva hora=%c%c\n",texto[0],texto[1]);
	texto[3]='\0';
	//printf("##%d,%d##\n",atoi(texto[0]),atoi(texto[1]));
	nueva_hora = atoi( texto ); 
	printf("\suma+1=%d\n",nueva_hora+1);
	return nueva_hora;
}

int lectura_min(void)
{//Obtener nueva min del reloj
    printf("\nintroduce nuevos min(2 digitos):");
	UART_Read_Text(texto,2);
	texto[3]='\0';
	printf("nuevos min=%s\n",texto);
	nueva_min = atoi( texto ); 
	printf("\suma+1=%d\n",nueva_min+1);
	return nueva_min;
}

int lectura_mes(void)
{//Obtener nueva min del reloj
    printf("\nintroduce nuevo mes(2 digitos) -enero es 0-:");
	UART_Read_Text(texto,2);
	texto[3]='\0';
	printf("nuevo mes=%s\n",texto);
	nuevo_mes = atoi( texto ); 
	printf("\suma+1=%d\n",nuevo_mes+1);
	return nuevo_mes;
}

int lectura_dia_del_mes(void)
{//Obtener nueva min del reloj
    printf("\nintroduce dia del mes (2 digitos):");
	UART_Read_Text(texto,2);
	texto[3]='\0';
	printf("nuevo dia del mes=%s\n",texto);
	nuevo_dia_del_mes = atoi( texto ); 
	printf("\suma+1=%d\n",nuevo_dia_del_mes+1);
	return nuevo_dia_del_mes;
}

int lectura_anio(void)
{//Obtener nueva min del reloj
    printf("\nintroduce nuevo a%co(4 digitos):",0XF1);
	UART_Read_Text(texto,4);
	texto[5]='\0';
	printf("nuevo a%co=%s\n",0XF1,texto);
	nuevo_anio = atoi( texto ); 
	printf("\suma+1=%d\n",nuevo_anio+1);
	return nuevo_anio;
}


