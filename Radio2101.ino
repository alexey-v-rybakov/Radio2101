#include <TEA5767.h>
#include <Wire.h>
#include <EEPROM.h>
#include "indication.h"
#include <Button.h>

#define LOW_EC_STATE   0
#define HIGH_EC_STATE  263


#define EEPROM_SIGNATURE  0xA5
#define EEPROM_BASE       0x0
#define BT_BUTTON_HOLD  100



//
// состояние работы системы, которое сохраняем в EEPROM
typedef struct RADIO_STATE
 {
   MODE         mode;
   double       search_freq;  // последняя частота в режиме поиска
   unsigned int fix_freq;     // номер частоты из таблицы radio_piter
 };

 //
 // Что обновляем в состоянии радио
 typedef enum UPDATE_MODE
  {
    FULL_UPDATE = 0, // полное обновление
    ENCODER_UPDATE,   // изменилось состояние энкодера
    NUM_OF_UPDATE
  };

#define NUM_OF_STATION (sizeof(radio_piter)/sizeof(float))

//
// Список радиостанций Петербурга
float radio_piter[] = 
 {
  87.5,
  88.0,
  88.4,
  88.9,
  89.3,
  89.7,
  90.1,
  90.6,
  91.1,
  91.5,
  92.0,
  92.4,
  92.9,	
  93.3,
  94.1,
  95.0,
  95.5,
  95.9,
  97.0,
  98.6,
  99.0,
  100.5,
  100.9,
  101.4,
  102.0,
  102.4,
  102.8,
  103.4,
  103.7,
  104.0,
  104.4,
  104.8,
  105.3,
  105.9,
  106.3,
  107.0,
  107.4,
  107.8
 };
TEA5767 Radio;
RADIO_STATE radio_state;

int encoder_state = 0;
int old_encoder_state = 0;
int show_light;

//
// Запись состояния радио в EEPROM
//
void save_state_to_eeprom()
 {
    Serial.println("Save state to EEPROM");
    EEPROM.write(EEPROM_BASE, 0x0);
    byte* r_state = (byte*)&radio_state; 
    for (int i = 0; i < sizeof(radio_state); i++)
      EEPROM.write(EEPROM_BASE + i + 1, r_state[i]);
    EEPROM.write(EEPROM_BASE, EEPROM_SIGNATURE);
 }

//
// Чтение состояния радил из EEPROM
// 
 void read_state_from_eeprom()
  {
    if (EEPROM.read(EEPROM_BASE) != EEPROM_SIGNATURE)
     {
       Serial.println("No EEPROM signature. Loading default state");
       radio_state.mode        = MODE::RADIO_SEARCH;
       radio_state.search_freq = radio_piter[0];
       radio_state.fix_freq    = 0;
       save_state_to_eeprom();
     }
    else
     {
       Serial.println("Here is EEPROM signature. Loading state from EEPROM");
       byte* r_state = (byte*)&radio_state; 
       for (int i = 0; i < sizeof(radio_state); i++)
        r_state[i] = EEPROM.read(EEPROM_BASE + i + 1);

     }
  }





 
  unsigned long last_pressed,time;
  unsigned char buf[5];
  int stereo,signal_level,search_mode = 0,search_direction,i,f_h,f_l,f,w;
  double current_freq;
  float f_new = (EEPROM.read(0)*256 +  EEPROM.read(1));
  byte a1[8]={0b00000,0b00000,0b00000,0b00000,0b00000,0b00000,0b00000,0b11111};
  byte a2[8]={0b00000,0b00000,0b00000,0b00000,0b00000,0b00000,0b11111,0b11111};
  byte a3[8]={0b00000,0b00000,0b00000,0b00000,0b00000,0b11111,0b11111,0b11111};
  byte a4[8]={0b00000,0b00000,0b00000,0b00000,0b11111,0b11111,0b11111,0b11111};
  byte a5[8]={0b00000,0b00000,0b00000,0b11111,0b11111,0b11111,0b11111,0b11111};
  byte a6[8]={0b00000,0b00000,0b11111,0b11111,0b11111,0b11111,0b11111,0b11111};
  byte a7[8]={0b00000,0b11111,0b11111,0b11111,0b11111,0b11111,0b11111,0b11111};
  byte a8[8]={0b11111,0b11111,0b11111,0b11111,0b11111,0b11111,0b11111,0b11111};
 
// 1. Записать состояние через 5 сек после того как изменилось

void update_radio_state(UPDATE_MODE um, int enc_delta = 0);




Button btn_mode(9); // Connect your button between pin 2 and GND
Button btn_pp(8); // Connect your button between pin 3 and GND


void setup() 
{ 
   Serial.begin(9600); 
  //
  // Инициализация состояния девайса
  read_state_from_eeprom();
  // инициализация кнопок
  //pinMode(8, INPUT_PULLUP);
  //pinMode(9, INPUT_PULLUP);
  // инициализация переключателя аудивхода
  pinMode(7, OUTPUT);
  // инициализация переключателя аудивхода
  pinMode(6, OUTPUT);
  digitalWrite(6, LOW); // управление паузой
  pinMode(4, OUTPUT);
  digitalWrite(4, LOW); // трек вперед
  pinMode(2, OUTPUT);
  digitalWrite(2, LOW); // трек назад
  
  //
  //
  btn_mode.begin(); // Connect your button between pin 2 and GND
  btn_pp.begin(); // Connect your button between pin 3 and GND
 
  Wire.begin();
  Radio.init();
 // Radio.set_frequency(/*f_new/10*/87.5); 
 // Serial.begin(9600);

  init_indication();
  set_led_brightness(100);
  
  update_radio_state(UPDATE_MODE::FULL_UPDATE);




  // для энкодера
  pinMode(3,INPUT_PULLUP); // ENC-A
  pinMode(5,INPUT_PULLUP); // ENC-B
  
  PCICR =  0b00000100; // PCICR |= (1<<PCIE1); Включить прерывание PCINT2
  PCMSK2 = 0b00101100; // Разрешить прерывание для  A1, A2, A3
}
 
const float mu    = 1.0;
const float sigma = 0.4;  

float gausse(float x)
 {
   float p1 = sqrt(2.0 * M_PI);
   p1 = p1 * sigma;
   p1 = 1.0 / p1;

   float degree = square(x - mu);
   degree = degree/ (2.0 * square(sigma));

   float p2 = exp(-degree);

   return (p1 * p2);
 }


float step = 0.02;//0.02;
float base = 0;
float offset = 1.2;

unsigned long last_update     = 0;
unsigned long save_state_time = 0;







 void update_radio_state(UPDATE_MODE um, int enc_delta = 0)
  {
    save_state_time = millis();

    set_mode_color(radio_state.mode);


    // 1. Выбираем аудио выход
    if (radio_state.mode != MODE::RADIO_BLUETOOTH)
      digitalWrite(7, LOW);
    else
      digitalWrite(7, HIGH); 
    // 2. Ставим нужную частоту и состояние энкодера
    if (radio_state.mode == MODE::RADIO_SEARCH)
     {
       // если обновление энкодера - вычислить частоту
      if (um == UPDATE_MODE::ENCODER_UPDATE)
        {
          // держим энкодер в рамках приличия
          if (old_encoder_state < LOW_EC_STATE) 
           {
            noInterrupts();
            encoder_state = old_encoder_state = LOW_EC_STATE;
            interrupts();
            return;
           }
          if (old_encoder_state >= HIGH_EC_STATE) 
           {
            noInterrupts();
            encoder_state = old_encoder_state = HIGH_EC_STATE;
            interrupts();
            return;
           }
          radio_state.search_freq = radio_piter[0] + 0.1*old_encoder_state; 
        }  
      // если полное обновление, то надо вычислить состояние энкодера
       if (um == UPDATE_MODE::FULL_UPDATE)
          {
            old_encoder_state = (int)((radio_state.search_freq - radio_piter[0])/0.1);
            noInterrupts();
            encoder_state = old_encoder_state;
            interrupts();
          }
       // частота
       Radio.set_frequency(radio_state.search_freq); 
       Serial.print("FM: ");
       Serial.print(radio_state.search_freq, 2);
       Serial.println(" MHz ");
       show_light = old_encoder_state;
     }  
    // 2. Ставим нужную частоту и состояние энкодера
    if (radio_state.mode == MODE::RADIO_FIX)
     {
       // если обновление энкодера - вычислить частоту
      if (um == UPDATE_MODE::ENCODER_UPDATE)
        {
          // держим энкодер в рамках приличия
          if (old_encoder_state < 0) 
           {
            noInterrupts();
            encoder_state = old_encoder_state = 0;
            interrupts();
            return;
           }
          if (old_encoder_state >= NUM_OF_STATION) 
           {
            noInterrupts();
            encoder_state = old_encoder_state = NUM_OF_STATION-1;
            interrupts();
            return;
           }
           
        }  
      // если полное обновление, то надо вычислить состояние энкодера
       if (um == UPDATE_MODE::FULL_UPDATE)
          {
            old_encoder_state = radio_state.fix_freq;
            noInterrupts();
            encoder_state = old_encoder_state;
            interrupts();
          }


      radio_state.fix_freq = old_encoder_state;
       // частота
       Radio.set_frequency(radio_piter[radio_state.fix_freq]); 
       Serial.print("FM: ");
       Serial.print(radio_piter[radio_state.fix_freq], 2);
       Serial.println(" MHz ");
       show_light = (int)((radio_piter[radio_state.fix_freq] - radio_piter[0])/0.1);
     }     
    //============================================================================================================
    bool n_set = false;
    if (radio_state.mode == MODE::RADIO_BLUETOOTH)
     {
        show_led(1000); n_set = true;
       //save_state_time = millis();
       if (enc_delta > 0)
        {
          save_state_time = 0;
          Serial.println("Bluetooth next track");
          digitalWrite(4, HIGH);
          delay(BT_BUTTON_HOLD); 
          digitalWrite(4, LOW);
        }
       if (enc_delta < 0) 
        {
          save_state_time = 0;
          Serial.println("Bluetooth previous track"); 
          digitalWrite(2, HIGH);
          delay(BT_BUTTON_HOLD); 
          digitalWrite(2, LOW);
        }
     }

    if (!n_set) {
    float x = base + step*show_light; 
    float g = gausse(x);
    test_led(gausse(x-offset*3.), gausse(x-offset*2.), gausse(x-offset), g);}
    
  } 

//
// Главный цикл
//
void loop() 
{
  bool upd = false;

  if (save_state_time != 0)
   {
     if ((millis() - save_state_time) > 3000)
      {
         save_state_to_eeprom();  
         save_state_time = 0;
         //Serial.println("Save state to EEPROM");
      }

   }

/*
  noInterrupts();
  if (encoder_state != old_encoder_state)
    {
      old_encoder_state = encoder_state;
      upd = true;
    }
  interrupts();
  if (upd == true)
   {
    Serial.print("encoder state: ");
    Serial.println(old_encoder_state, DEC);
    show_led(old_encoder_state);
   }
  
  return;

*/



  if (btn_mode.toggled()) 
  	if (btn_mode.read() == Button::RELEASED)
      {
        radio_state.mode = radio_state.mode + 1; 
        if (radio_state.mode >= MODE::NUM_OF_MODE)
          radio_state.mode = 0;
        Serial.print("mode : "); Serial.println(radio_state.mode, DEC);   
        update_radio_state(UPDATE_MODE::FULL_UPDATE);
      }

  if (btn_pp.toggled()) 
  	if (btn_pp.read() == Button::PRESSED)
      {
        if (radio_state.mode == MODE::RADIO_BLUETOOTH)
        {
          Serial.println("Bluetooth play-pause"); 
          digitalWrite(6, HIGH);
          delay(BT_BUTTON_HOLD); 
          digitalWrite(6, LOW); 
        }
      }

  
  int enc_delta;
  noInterrupts();
  if (encoder_state != old_encoder_state)
  {
    enc_delta = encoder_state - old_encoder_state;
    old_encoder_state = encoder_state;
    upd = true;
    }
  interrupts();
  if (upd == true)
   {
   
  
    update_radio_state(UPDATE_MODE::ENCODER_UPDATE, enc_delta);
    Serial.print("encoder state: ");
    Serial.println(old_encoder_state, DEC);
    
    
    }

}




uint8_t last_comb = -1;
int count = 0;
uint8_t int_data[] = {5,5,5};

ISR (PCINT2_vect) //Обработчик прерывания от пинов A1, A2, A3
{
 
  uint8_t comb = /*bitRead(PIND, 5) << 2 |*/ bitRead( PIND, 5)<<1 | bitRead(PIND, 3); //считываем состояние пинов энкодера и кнопки
 
  if (last_comb == comb)
    return;

  last_comb = comb;

  int_data[0] = int_data[1];
  int_data[1] = int_data[2];
  int_data[2] = comb;

  if (comb == 3)
    {
      if ((int_data[0] == 0)&&(int_data[1] == 1)) encoder_state++;
      if ((int_data[0] == 1)&&(int_data[1] == 0)) encoder_state--;
    }
  return;
}