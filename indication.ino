#include "indication.h"

#define NUM_LEDS 4
#include "FastLED.h"
#define PIN 10
CRGB leds[NUM_LEDS];


static int  g_cur_br      = 0;

void init_indication()
{

    FastLED.addLeds<WS2812, PIN, GRB>(leds, NUM_LEDS).setCorrection( TypicalPixelString );
    FastLED.setBrightness(g_cur_br);

    return;
}

void set_led_brightness(int br)
{
  if (br > 100) br  = 100;
  if (br < 0)   br  = 0;
  g_cur_br = br;
  FastLED.setBrightness(g_cur_br);
  FastLED.show();
}

static int cled = 0;


CRGB clr[] = 
{
CRGB::AliceBlue ,
CRGB::Aquamarine,
CRGB::Black,
CRGB::Brown,
CRGB::Chocolate,
CRGB::Crimson,
CRGB::DarkGoldenrod,
CRGB::DarkKhaki,
CRGB::DarkOrchid,
CRGB::DarkSlateBlue,
CRGB::DarkViolet,
CRGB::DimGrey,
CRGB::ForestGreen,
CRGB::Gold,
CRGB::Green,
CRGB::IndianRed,
CRGB::Lavender,
CRGB::LightBlue,
CRGB::LightGreen,
CRGB::LightSeaGreen ,
CRGB::LightSteelBlue ,
CRGB::Linen ,
CRGB::MediumBlue ,
CRGB::MediumSlateBlue ,
CRGB::MidnightBlue ,
CRGB::NavajoWhite ,
CRGB::OliveDrab ,
CRGB::PaleGoldenrod ,
CRGB::PapayaWhip ,
CRGB::Plaid ,
CRGB::Red ,
CRGB::Salmon ,
CRGB::Sienna ,
CRGB::SlateGray ,
CRGB::SteelBlue ,
CRGB::Tomato ,
CRGB::White ,
CRGB::FairyLight };


CRGB color;

 void set_mode_color(MODE md)
  {
    if (md == RADIO_SEARCH) color = CRGB(255,161,0);//CRGB::LightSeaGreen; 
    if (md == RADIO_FIX) color = CRGB::Red; 
    if (md == RADIO_BLUETOOTH) color = CRGB::AliceBlue; 
  }

void test_led(float d1, float d2, float d3, float d4)
 {
   
   
  

  
     for (int i = 0; i < NUM_LEDS; i++)
      {
      leds[i] = color;//CRGB::CRGB::MediumSlateBlue;//clr[cled];
      //leds[i].setRGB(255, 255, 255); // LED 0 full brightness white
     // if (i == 0)
     // leds[i].setRGB(40, 40, 40);
      //leds[i].fadeLightBy(255);     
       
       
       
       /*if (i > 27)
        leds[i] = CRGB::AliceBlue;
        else
        {leds[i] = CRGB::AliceBlue ;
        
        }*/
      }
  //leds[cled%NUM_LEDS] = CRGB::Green; 
  leds[0].fadeLightBy(255*(1 - d1));


  leds[1].fadeLightBy(255*(1 - d2));
  leds[2].fadeLightBy(255*(1 - d3));
  leds[3].fadeLightBy(255*(1 - d4));
  cled++;
  if (cled >= (sizeof(clr)/sizeof(CRGB)))
    cled = 0;
  FastLED.show();
 }

void show_led(int enc_state)
 {
  
  leds[0] = CRGB::AliceBlue; 
  leds[0].fadeLightBy(0);

leds[1] = CRGB::AliceBlue; 
  
  leds[1].fadeLightBy(0);

  leds[2] = CRGB::AliceBlue; 
  
  leds[2].fadeLightBy(0);
  leds[3] = CRGB::AliceBlue; 
  
  leds[3].fadeLightBy(0);
  FastLED.show();
 }



/* count++;
  
  if (on_off == 0)
    leds[led_count] = CHSV(0, 0, 0);
  else 
    {
      //if (count%3 == 0)
     //  leds[led_count] = CRGB::Blue; //CHSV(255, 0, 255);
      //  if (count%3 == 1)
      //  leds[led_count] = CRGB::Green; //CHSV(255, 0, 255);
      //   if (count%3 == 2)
     //   leds[led_count] = CRGB::Red; //CHSV(255, 0, 255);
        leds[led_count] = CRGB::White;
    }
    
  led_count++;
  if (led_count >= NUM_LEDS)
   {
    led_count = 0;
    if (on_off == 0) on_off = 1;
    else             on_off = 0;
   }
    
  FastLED.show();
  
  delay(50);         // скорость движения радуги*/
