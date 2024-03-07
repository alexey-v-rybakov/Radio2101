#ifndef _INDICATION_H_
#define _INDICATION_H_


void init_indication();

//
// режимы работы радио
typedef enum MODE
 {
   RADIO_SEARCH   = 0,
   RADIO_FIX         ,
   RADIO_BLUETOOTH   ,
   NUM_OF_MODE
 };

 void set_mode_color(MODE md);

void set_led_brightness(int br);

void test_led(float d1, float d2, float d3, float d4);


void show_led(int enc_state);

#endif
