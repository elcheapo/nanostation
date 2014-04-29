/*
 * adc.h
 *
 *  Created on: 3 mars 2011
 *      Author: florrain
 */

#ifndef ADC_H_
#define ADC_H_


typedef enum {disable, analog5, analog4, digital5, digital4, digital3, fem5, fem4} t_adc;

typedef struct adc_channel_s {
	uint16_t value;
	int adc_data_flag:1;
	int enabled:1;
} adc_channel;

#define NB_CHANNEL 4

void adc_init(void);

inline void start_adc(void){
	// Start next conversion, enable interrupt
	ADCSRA |= (1<<ADSC);
}
void adc_enable_channel(uint8_t channel, t_adc mode);
void adc_disable_channel(uint8_t channel);
int16_t get_adc(uint8_t channel);
void process_adc(void);
#endif /* ADC_H_ */
