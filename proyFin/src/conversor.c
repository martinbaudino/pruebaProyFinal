#include "conversor.h"
#include "chip.h"

void ISR_RIT_Handler(void){
	adc_convertir();
}

uint8_t main(void){
	uint32_t ret=10;
	uint16_t prom=0,suma=0,lect[10]={0},n=0;
	uint8_t i;

	leds_init();
	adc_init(CANAL0);
	base_tiempo_init(ret);

	while(1){
		lect[n]=adc_pool(CANAL0);
		if(n==9){
			n=0;
		}
		else{
			n++;
		}
		for(i=0;i<10;i++){
			suma+=lect[i];
		}
		prom=suma/n;
	}

	return 0;
}
