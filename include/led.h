#ifndef LEDS_H
#define LEDS_H

extern struct A_Object      LED_Setup;
extern struct A_Object      SPI_Config_Ob;

extern struct A_Object_Cfg      LED_Setup_Cfg;
extern struct A_Object_Cfg      SPI_Config_Cfg;

struct SPI_Data {
    bool ch;
    bool type; 
    uint16_t val; 
}; 

void led_set(void *p1, void *p2, void *p3); 
void spi_config_thread(void *p1, void *p2, void *p3);

extern struct k_msgq led_msgq; 

#endif

