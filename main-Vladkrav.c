#include "LPC17xx.h"
#include "stdio.h"
#include "math.h"
#include "system_LPC17xx.h"

uint8_t num_pulso=0, key1pendiente=0, key2pendiente=0, key1=1, key2=1,pulsado=0; 
uint16_t contmanual=0;
const uint32_t step[]={1<<20,3<<20,2<<20,6<<20,4<<20,0xC<<20,8<<20,9<<20}; //Secuencia de excitacion motor

void ConfigPines(void){
	
	LPC_GPIO1->FIODIR|=(1<<20)|(1<<21)|(1<<22)|(1<<23);	//MOTOR: pines P1.20 a P1.23 como salida
	//LPC_GPIO2->FIODIR&=~((1<<10)|(1<<11)|(1<<12)); //ISP, KEY1, KEY2 como entrada ¿Necesario?
}
void Config_ADC(void){
	LPC_SC->PCONP|=(1<<12); 				//Encender el ADC
	LPC_PINCON->PINSEL1|=(1<<14);		//P0.23
	LPC_PINCON->PINMODE1|=(2<<14); 	//Deshabilitar pull-up/pull-down
	LPC_SC->PCLKSEL0 &=~(3<<24); 		//CCLK/4= 25MHz
	LPC_ADC->ADCR=(1<<0)						//Elegimos AD0.0
								|(1<<8)						//CLKDIV=2 para tener 12.5 MHz
								|(1<<21)					//PDN=1
								|(5<<24);					//MAT0.3
	LPC_ADC->ADINTEN|=(1<<8); 			// Habilita interrupcion fin de conversion en el canal 0 (DONE global 0)
	NVIC_EnableIRQ(ADC_IRQn);
	NVIC_SetPriority(ADC_IRQn,8);

}	
void ModoAuto(void){
	
	LPC_GPIO1->FIOPIN=step[num_pulso];
	num_pulso++;
	if(num_pulso==8)
	{
		num_pulso=0;
	}
}
void ModoManualDcha(void){	
		if(num_pulso>=7)
		num_pulso=0;
		
		LPC_GPIO1->FIOPIN=step[num_pulso];
			num_pulso++;
			contmanual++;
		
		if(num_pulso==8)
			num_pulso=0;
			
		if(contmanual==113){
			contmanual=0;
			pulsado=0;
			}
}
void ModoManualIzq(void){	
	if(num_pulso>=0)
		num_pulso=8;
	
	LPC_GPIO1->FIOPIN=step[num_pulso-1];
			num_pulso--;
			contmanual++;
	
	if(num_pulso==0)
				num_pulso=8;
			

	if(contmanual==113){
		contmanual=0;
		pulsado=0;
	}
}


void SysTick_Handler(void){
	if((LPC_GPIO1->FIOPIN &(1<<0))==1)
	{
		ModoAuto();
	}
	
	else
	{
		if(pulsado==0){
		key1=(LPC_GPIO2->FIOPIN & (1<<11))>>11;
		key2=(LPC_GPIO2->FIOPIN & (1<<12))>>12;
		pulsado=1;
		}
		
		if(key1==0)  //KEY1 pulsado, giro a derecha 10º
		{
			ModoManualDcha();
		}
		else if(key2==0)
		{
			ModoManualIzq();	//KEY2 pulsado, giro a izquierda 10º
		}
	}
	
}

int main(){
	
	ConfigPines();
	//ConfigInterrupts();
	SysTick->LOAD=100000;  //10ms X/100 MHz=10 ms	
	NVIC_SetPriority(SysTick_IRQn,0); //le damos la mayor prioridad al SysTick
  SysTick->VAL=0;           
  SysTick->CTRL=0x7; //habilitamos

	while(1);
}
