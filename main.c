#include "LPC17xx.h"
#include "stdio.h"
#include "math.h"
#include "system_LPC17xx.h"
#include "lcddriver.h"
#include "uart.h"
#include "stdlib.h"
#include "I2C.c"
#define N_muestras 100
#define pi 3.1416
#define F_pclk 25e6

//variables para modo manual y automatico
uint8_t num_pulso=0, key1=1, key2=1, pulsado=0,vuelta=0,retardo=0,modo,unavez=0,cambio=0,aux,modo_trabajo; 
uint16_t contmanual=0, contauto=0, stop=1, rep_man_LCD=0,contador=0;
const uint32_t step[]={1<<20,3<<20,2<<20,6<<20,4<<20,0xC<<20,8<<20,9<<20}; //Secuencia de excitacion motor

//Variables para el ADC
float ADCant=0, mVsharp=0; 
uint16_t distancia=0,time_ADC=0,umbral=100;

//Variables para el DAC
uint16_t frecuenciaDAC=100, muestras[N_muestras];

//Variables de la UART
char buffer[30];// Buffer de recepción de 30 caracteres
char bufferLCD[30]; //Buffer para guargar lo que se deba represerntar en el Display
char bufferUART[30]; //Buffer para guardar lo que se deba representar en la UART
char *ptr_rx;// puntero de recepción
char rx_completa;// Flag de recepción de cadena que se activa a "1" al recibir la tecla return CR(ASCII=13)
char *ptr_tx;// puntero de transmisión
char tx_completa;// Flag de transmisión de cadena que se activa al transmitir el caracter null (fin de cadena)
char opcion=0,modoUART=1;

//Variables para I2C
char precision,signo;
uint16_t time_temp=0; //tiempo para la representacion de la temperatura
float temperatura=0;

void delay(void){
uint32_t k;
	for(k=0;k<825000;k++);	
}
void ConfigPines(void){
	LPC_GPIO1->FIODIR|=(1<<20)|(1<<21)|(1<<22)|(1<<23);	//MOTOR: pines P1.20 a P1.23 como salida
	LPC_PINCON->PINSEL4 |=(1<<20); //EINT0
	NVIC_SetPriorityGrouping(4);
	NVIC_SetPriority(EINT0_IRQn,0); //Le damos la mayor prioridad a EINT0
	LPC_SC->EXTMODE=0x1;//por flanco
	LPC_SC->EXTPOLAR=0x0;//de subida
	NVIC_EnableIRQ(EINT0_IRQn);//habilitamos la interrupcion del ISP
}
void genera_muestras(uint8_t muestras_ciclo){ //Bien
	uint8_t i;
	for(i=0;i<muestras_ciclo;i++)
		muestras[i]=(uint16_t)1023*(0.5 + 0.5*sin(2*pi*i/N_muestras));

}
void Config_ADC(void){ 
	LPC_SC->PCONP|=(1<<12); 				//Encender el ADC
	LPC_PINCON->PINSEL1|=(1<<14);		//P0.23 (AD0.0)
	LPC_PINCON->PINMODE1|=(2<<14); 	//Deshabilitar pull-up/pull-down
	LPC_SC->PCLKSEL0 &=~(3<<24); 		//CCLK/4= 25MHz
	LPC_ADC->ADCR=(1<<0)						//Elegimos AD0.0
								|(1<<8)						//CLKDIV=2 para tener 12.5 MHz
								|(1<<21)					//PDN=1
								|(6<<24);					//MAT1.0
	LPC_ADC->ADINTEN =(1<<8); 			//Habilita interrupcion fin de conversion en el canal 0 (DONE global 0)
	NVIC_EnableIRQ(ADC_IRQn);
	NVIC_SetPriority(ADC_IRQn,1);
}	
void DistanciaADC(void){
	
	mVsharp=(ADCant*3300)/4096;	//Le pasamos el valor del ADC y lo convertimos en mV
	
	if((mVsharp<2750) && (mVsharp>=1500)){ //Distacia entre 15 y 40 cm
		distancia=(((mVsharp-2750)/(-50))+15);
	}
	else if((mVsharp<1500) && (mVsharp>=900)){ //Distacia entre 40 y 70 cm
		distancia=((mVsharp-1500)/(-20))+40;
	}
	else if((mVsharp<900) && (mVsharp>=660)){ //Distacia entre 70 y 100 cm
		distancia=((mVsharp-900)/(-8))+70;
	}
	else if((mVsharp<660) && (mVsharp>=460)){ //Distacia entre 100 y 150 cm
		distancia=((mVsharp-660)/(-4))+100;
	}
	frecuenciaDAC=5000-(distancia*10); //Frecuencia del pitido
	LPC_TIM0->MR0 =(F_pclk/N_muestras/frecuenciaDAC)-1; //Se actualiza la frecuencia del TIMER0
	LPC_TIM0->TC = 0; //Para que empiece a contar desde 0
	if(distancia>umbral){
		LPC_TIM0->TCR =0; //se apaga el TIMER0 para que el DAC no funcione
	}	 
	else
		LPC_TIM0->TCR =0x1;//El DAC sigue funcionando
}	 

void Config_DAC(void){ 
	LPC_PINCON->PINSEL1|=(2<<20); //P0.26 como AOUT
	LPC_PINCON->PINMODE1|=(2<<20); //Deshabilita pull-up/pull-down
	LPC_DAC->DACCTRL=0;
}
void Config_TIMER0(void){ //Timer para el funcionamiento del DAC
	LPC_SC->PCONP|=(1<<1);				//Enciendo el TIMER0(aunque esta encendido de forma predeterminada)
	LPC_TIM0->MCR = (3<<0); 			//Reset y Interrupme on MR0
	LPC_TIM0->MR0 =( F_pclk/N_muestras/frecuenciaDAC)-1; 		//Asignacion del valor del match 
	LPC_TIM0->EMR = (1<<0);				//External Match 1, do nothing
	LPC_TIM0->TCR = 0x01;					//Counter Enable
	NVIC_EnableIRQ(TIMER0_IRQn);
	NVIC_SetPriority(TIMER0_IRQn,2);
	}
void Config_TIMER1(void){ //Timer para el funcionamiento del ADC
	LPC_SC->PCONP|=(2<<1);				//Enciendo el TIMER1
	LPC_TIM1->MCR = (1<<1); 			//Reset on MR0
	LPC_TIM1->MR0 =(625000)-1; 		//Asignacion del valor del match cada 0.25s
	LPC_TIM1->EMR = (3<<4);				//External Match 0, do nothing
	LPC_TIM1->TCR = 0x01;					//Counter Enable
	}
void Config_I2C(void){ 
	LPC_SC->PCONP|=(1<<7);
	I2CSendAddr(0x48,0);
	I2CSendByte(0xAC);
	I2CSendByte(0x02);
	I2CSendStop();
	delay();
	I2CSendAddr(0x48,0);
	I2CSendByte(0xEE);
	I2CSendStop();
}
int I2C_Temperatura(void){ 
	uint16_t temp=0;
	I2CSendAddr(0x48,0);
	I2CSendByte(0xAA);
	I2CSendAddr(0x48,1);
	temp=I2CGetByte(0);
	precision=(I2CGetByte(1) & 0x80);
	I2CSendStop();
	
	if(((temp>>7) & 0x1)==1){
		
		//temp=((~(temp-1)) & 0xFF); //pasamos a valor absoluto
		temp=abs(temp-1); 
		signo=1; //negativo
	}
	else
		signo=0;
	return temp; //positivo
}
void TIMER0_IRQHandler(void){ 
	static uint8_t indice_muestra;
	
	LPC_TIM0->IR|=(1<<0);
	LPC_DAC->DACR=muestras[indice_muestra++]<<6;
	if(indice_muestra==N_muestras-1)
		indice_muestra=0;
}
void ADC_IRQHandler(void){ 
	ADCant=((LPC_ADC->ADGDR>>4) & 0xFFF); //con 12 bits de resolucion 
	DistanciaADC();
}

void ModoAuto(void){
	
	if(contauto%116==0 && contauto!=0){ //se han hecho 10 grados
			retardo=1; //variable que controla el giro del motor
			contador++; //temporizador que cuenta 50ms
		if(contador==500){	//si han pasado 50ms, entonces vuelve a girar el motor
				contador=0;
				retardo=0;
		}
	}
	if(retardo==0){
	if(stop==1){ // Si el boton ISP no esta pulsado el motor gira 
		if(vuelta==0){ // Gira de 0º a 360º
			
			LPC_GPIO1->FIOPIN=step[num_pulso++]; 	//Sentencia que excita al motor, giro a derechas
			contauto++;
	
		if(num_pulso==8)// Si ha recorrido todo el array de excitacion
			num_pulso=0;	
		
		if(contauto==4095){ //Si a llegado a 360º
			vuelta=1;
			num_pulso=8;
		}
		}

		if(vuelta==1){ //Gira de 360º a 0º
			LPC_GPIO1->FIOPIN=step[(num_pulso--)-1];	//Sentencia que excita al motor, giro a izquierdas
			contauto--;
		
		if(num_pulso==0)// Si ha recorrido todo el array de excitacion
			num_pulso=8;
		
		if(contauto==0){
			vuelta=0;
			num_pulso=0;
		}
		}	
	}
	}
}
void ModoManualDcha(void){	
			if(num_pulso==8){
			num_pulso=0;
			}
		LPC_GPIO1->FIOPIN=step[num_pulso++];//Sentencia que excita al motor, giro a derechas
			contmanual++;
		
		if(num_pulso>=8)// Si ha recorrido todo el array de excitacion
				num_pulso=0;
				
		if(contmanual==116){ //Si ha hecho 10º
			contmanual=0;
			num_pulso=0;
			pulsado=0;
		}
}
void ModoManualIzq(void){	
		if(num_pulso==0){
			num_pulso=8;
		}
	LPC_GPIO1->FIOPIN=step[(num_pulso--)-1];//Sentencia que excita al motor, giro a izquierdas
		contmanual++;
	
	if(num_pulso==0)// Si ha recorrido todo el array de excitacion
		num_pulso=8;

	if(contmanual==116){	//Si ha hecho 10º
			contmanual=0;
			num_pulso=0;
			pulsado=0;
	}
}

void EINT0_IRQHandler(void){ //ISP (marcha/paro)
		LPC_SC->EXTINT|=0x00000001;
		if(modo_trabajo==1){
			stop^=1; //en modo automatico, cuando pulsas ISP, se para el motor, o por el contrario, si estaba parado, arranca
		}
		else{ //si esta en modo manual, cuando pulses ISP se hara la medida y transferencia al LCD y UART
			rep_man_LCD=1; //representacion en modo manual al LCD
			LPC_ADC->ADCR &=~(7<<24); //Se limpian las posiciones 24,25 y 26
			LPC_ADC->ADCR |=(1<<24); //Cada vez que se pulsa ISP en modo manual, se obtiene el valor del ADC (START NOW)
		}
}
void SysTick_Handler(void){
	
	time_temp++; //cada 2 segundos se representa en el display la temperatura

	modo=(LPC_GPIO1->FIOPIN &(1<<0));
	
	if(aux!=modo){ //si es diferente del anterior, se cambia de modo de trabajo
		cambio=0;
		modo_trabajo=modo;
	}
	if(cambio==1){ // si se ha producido un cambio por la UART, se cambia de modo de trabajo
		modo_trabajo=modoUART;
	}
	if(modo_trabajo==1){ 
		time_ADC++;//en modo automatico se representa en el display la medida
		LPC_ADC->ADCR &=~(7<<24); //Se limpian las posiciones 24,25 y 26
		LPC_ADC->ADCR|=(6<<24); //En modo automatico, se hace la conversion cada vez que hay un flanco de subida en MAT1.0 
		ModoAuto();
	}
	
	else{
		if(pulsado==0){
			key1=(LPC_GPIO2->FIOPIN & (1<<11))>>11; //Leemos el boton Key 1 y lo guardamos en key1
			key2=(LPC_GPIO2->FIOPIN & (1<<12))>>12;	//Leemos el boton Key 2 y lo guardamos en key2
			pulsado=1;
		}
		
		if(key1==0 && ((LPC_GPIO2->FIOPIN & (1<<11))!=0)){  //KEY1 pulsado, giro a izquierdas 10º
			LPC_ADC->ADCR &=~(7<<24); //Se limpian las posiciones 24,25 y 26
			ModoManualIzq();
		}
		else if(key2==0 && ((LPC_GPIO2->FIOPIN & (1<<12))!=0)){	//KEY2 pulsado, giro a derechas 10º
			LPC_ADC->ADCR &=~(7<<24); //Se limpian las posiciones 24,25 y 26
			ModoManualDcha();
		}
		if(key1==1 && key2==1) //Si ningun boton esta pulsado
			pulsado=0; // se vuelve a comprobar si estan pulsados los pulsadores
	}
	aux=modo;
}
void config_UART(){
	tx_cadena_UART0("Modo de funcionamiento:");while(tx_completa==0); tx_completa=0;
	tx_cadena_UART0("\n\r Manual(1) o automatico(2)");while(tx_completa==0); tx_completa=0;
	tx_cadena_UART0("\n\r configuracion del umbral(3)");while(tx_completa==0); tx_completa=0;
	tx_cadena_UART0("\n\r Se visualiza la medida cada 0,5s");while(tx_completa==0); tx_completa=0;
	tx_cadena_UART0(" en modo automatico\n\r");while(tx_completa==0); tx_completa=0;
}
int main(){
	Config_ADC();
	ConfigPines();
	genera_muestras(N_muestras);
	Config_DAC();
	Config_TIMER0();
	Config_TIMER1();
	Config_I2C();
	//lcdInitDisplay();
	//fillScreen(BLACK);
	//drawString(10,10,"Escaner de distancias",WHITE,GREEN,MEDIUM);
	ptr_rx=bufferUART;
	uart0_init(19200);
	config_UART();
	
	SysTick->LOAD=100000; //1ms	-> 100.000/100 MHz = 1ms
	NVIC_SetPriority(SysTick_IRQn,3); 
  SysTick->VAL=0;           
  SysTick->CTRL=0x7; //habilitamos

	while(1){
		
		if(time_ADC>=500){ //en modo automatico cada 0.5s se toma una medida y se representa en el Display
			time_ADC=0;
			if(distancia>umbral){
				sprintf(bufferLCD,"Medida: Fuera de umbral");
				//drawString(10,40,bufferLCD,WHITE,BLUE,0);
			}
			else{
				sprintf(bufferLCD,"Medida: %4.4d cm           ",distancia);
				sprintf(buffer,"%d",distancia);
				//drawString(10,40,bufferLCD,WHITE,BLUE,0);
				tx_cadena_UART0(buffer);while(tx_completa==0); tx_completa=0;
				tx_cadena_UART0(" cm \n\r");while(tx_completa==0); tx_completa=0;
			}
		}
		
		if(rep_man_LCD==1){ //en modo manual cada vez que se pulsa ISP se representa en el Display
			rep_man_LCD=0;
			sprintf(bufferLCD,"Medida: %d cm",distancia);
			//drawString(10,40,bufferLCD,WHITE,BLUE,0);
			tx_cadena_UART0(buffer);while(tx_completa==0); tx_completa=0;
			tx_cadena_UART0(" cm \n\r");while(tx_completa==0); tx_completa=0;
		}
		
		if(time_temp>=2000){ //Representacion de la temperatura cada 2 segundos
			time_temp=0;
			
			if(precision==0)
				temperatura=(float)I2C_Temperatura();
			else
				temperatura=(float)I2C_Temperatura()+0.5;
			
			if(signo==0)//positivo
				{sprintf(bufferLCD,"Temperatura: %2.2f C",temperatura);}
			else //negativo
				{sprintf(bufferLCD,"Temperatura: -%2.2f C",temperatura);}
			
				//drawString(10,60,bufferLCD,WHITE,BLUE,0);
		}
//UART0
			if(rx_completa){
				if(opcion>3){
					opcion=0;
					sprintf(bufferUART,"Elige bien la opcion\n\r");
					tx_cadena_UART0(bufferUART);while(tx_completa==0); tx_completa=0;
					rx_completa=0;
				}
				if(opcion==0){
					opcion=atoi(bufferUART);
				}
				if(opcion==1){ //Modo manual
						rx_completa=0;
						opcion=0;
						cambio=1;
						modoUART=0;
						sprintf(bufferUART,"Has elegido modo manual\n\r");
						tx_cadena_UART0(bufferUART);while(tx_completa==0); tx_completa=0;
				}
				if(opcion==2){ //Modo automatico
						rx_completa=0;
						opcion=0;
						cambio=1;
						modoUART=1;
						sprintf(bufferUART,"Has elegido modo automatico\n\r");
						tx_cadena_UART0(bufferUART);while(tx_completa==0); tx_completa=0;
				}		
				if(opcion==3){ //configuracion del umbral
					if(unavez==0){
						unavez=1;
						sprintf(bufferUART,"Indica nuevo umbral:\n\r");
						tx_cadena_UART0(bufferUART);while(tx_completa==0); tx_completa=0;
					}
					rx_completa=0;
					umbral=atoi(bufferUART);
					
						if(umbral){
							unavez=0;
							opcion=0;
							rx_completa=0;
							sprintf(bufferUART,"Has elegido umbral:%d\n\r",umbral);
							tx_cadena_UART0(bufferUART);while(tx_completa==0); tx_completa=0;
						}
				}
			}
}
}

