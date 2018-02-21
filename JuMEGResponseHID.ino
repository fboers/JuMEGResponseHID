/*---------------------------------------------
 * Arduino Due
 * sending event eventcode TTL signals
 * --------------------------------------------
 * @author   F.Boers (f.boers@fz-juelich.de)
 * FZJ-INM4-MEG
 * --- v1.00  
 * update 06.11.2017 
 *---------------------------------------------
 * debouncing input 4 key response -> TTL Output 
 * --> response input 1-4 
 *     Port D0-3  Pin 25,26,27,28 
 * --> response code out 1-4 
 *     Port C1-8  Pin 33,34,35,36,37,38,39,40
 *     HID Keyboard letters: 1,2,3,4
 *---------------------------------------------    
 * http://forum.arduino.cc/index.php?topic=260731.0
 * https://www.arduino.cc/en/Hacking/PinMappingSAM3X
 *---------------------------------------------
 *ToDo:
 * --> implement IRQ MR Trigger Pulse 5V 50us
 *--------------------------------------------- 
*/
    
//#include <DueTimer.h>
//#define TIMER5_INTERVALL_US 100

#include <Keyboard.h>

#define BAUDRATE 115200

#define MAX_OUTPUT_PINS   8
#define LED_PIN 13
#define INPUT_PIN_PORT  PIOD
#define OUTPUT_PIN_PORT PIOC

#define BEBOUNCE_US 200

#define MAX_RESPONSE_KEYS  8
#define USED_RESPONSE_KEYS 4

// irqmask defines which of the inputpin[] can generate events
// related to USED_RESPONSE_KEYS
byte irqmask = B00001111; // first 4 keys
//byte irqmask = B11111111; // first 8 keys


//--- I/O pins
//--- OUT Port C  PC1- PC8 ;PC19,18,17,16
int  outputpin[MAX_OUTPUT_PINS] = {33,34,35,36,37,38,39,40}; //44,45,46,47 };

//--- IN PortD PD0,PD1,PD2,PD3,PD6,PD9,PD7,PD8
int  inputpin[MAX_RESPONSE_KEYS] = {25,26,27,28,29,30,11,12 };

char HIDKey[MAX_RESPONSE_KEYS]   = {'1','2','3','4','5','6','7','8'};
unsigned long input_and_mask     = 975;// 1+2+4+8+2**6+2**9+2**7+2**8
unsigned long led_toggle_time_us          = 0;
unsigned long led_toggle_time_expander_us = 1000000; // 1sec 
bool          led_is_on          = false;
// ----------------------------------------------------
// End of user defined section
//-----------------------------------------------------
 
//--- variables used in interrupt routines
volatile bool          key_state[MAX_RESPONSE_KEYS];  //on off
volatile uint8_t       key_input_port_shift[MAX_RESPONSE_KEYS]={0,1,2,3,6,9,7,8};
volatile uint8_t       key_output_port_shift[MAX_OUTPUT_PINS] ={1,2,3,4,5,6,7,8};
volatile uint8_t       key_apply_update[MAX_RESPONSE_KEYS]; //needs update
volatile bool          key_isOnUpdate[MAX_RESPONSE_KEYS];
volatile unsigned long debounce_us[MAX_RESPONSE_KEYS];


//------------------------------------------------------
// Interrupt service routines (ISR)
// ------------------------------------------------------

void KeyOnChange0() { update_debounce_mu(0); }
void KeyOnChange1() { update_debounce_mu(1); }
void KeyOnChange2() { update_debounce_mu(2); }
void KeyOnChange3() { update_debounce_mu(3); }
void KeyOnChange4() { update_debounce_mu(4); }
void KeyOnChange5() { update_debounce_mu(5); }
void KeyOnChange6() { update_debounce_mu(6); }
void KeyOnChange7() { update_debounce_mu(7); }



// test Direct I/O
inline void digitalWriteDirect(int pin, boolean val){
  if(val) g_APinDescription[pin].pPort -> PIO_SODR = g_APinDescription[pin].ulPin;
  else    g_APinDescription[pin].pPort -> PIO_CODR = g_APinDescription[pin].ulPin;
}

inline int digitalReadDirect(int pin){
  return !!(g_APinDescription[pin].pPort -> PIO_PDSR & g_APinDescription[pin].ulPin);
}


//------------------------------------------------------  
// initialisation routine executed once after program start
//------------------------------------------------------
extern void setupResponseIO() {
  uint8_t i = 0;
//--- initialize input/output pins
  Serial.print("DUE Start setupResponse");
  
  pinMode(LED_PIN,OUTPUT);
  digitalWrite(LED_PIN,HIGH);
  
  for (i=0; i < MAX_OUTPUT_PINS; i++) { pinMode(outputpin[i],OUTPUT); }       

  for (i=0; i < MAX_RESPONSE_KEYS; i++){ pinMode(inputpin[i],INPUT_PULLUP); } //,INPUT_PULLUP);} // invertiert
 
     
//--- define interrupt routines
  if (bitRead (irqmask, 0)) {attachInterrupt(digitalPinToInterrupt(inputpin[0]), KeyOnChange0,CHANGE);}
  if (bitRead (irqmask, 1)) {attachInterrupt(digitalPinToInterrupt(inputpin[1]), KeyOnChange1,CHANGE);}
  if (bitRead (irqmask, 2)) {attachInterrupt(digitalPinToInterrupt(inputpin[2]), KeyOnChange2,CHANGE);}
  if (bitRead (irqmask, 3)) {attachInterrupt(digitalPinToInterrupt(inputpin[3]), KeyOnChange3,CHANGE);}
  if (bitRead (irqmask, 4)) {attachInterrupt(digitalPinToInterrupt(inputpin[4]), KeyOnChange4,CHANGE);}             
  if (bitRead (irqmask, 5)) {attachInterrupt(digitalPinToInterrupt(inputpin[5]), KeyOnChange5,CHANGE);}
  if (bitRead (irqmask, 6)) {attachInterrupt(digitalPinToInterrupt(inputpin[6]), KeyOnChange6,CHANGE);}
  if (bitRead (irqmask, 7)) {attachInterrupt(digitalPinToInterrupt(inputpin[7]), KeyOnChange7,CHANGE);}
    
  Serial.println(" -> Done setupResponse");

  for (i=0; i < MAX_RESPONSE_KEYS; i++)
   {
    debounce_us[i]      = 0;
    key_state[i]        = false;
    key_apply_update[i] = false;
    key_isOnUpdate[i]   = false;
   }        
} // end setupResponseIO

//-------------------------------------------------------------
// setup
//---------------------------------------------------------------
void setup() {
  Serial.begin(BAUDRATE); 
  Serial.flush();
 
  Keyboard.begin();
  delay(100);
  setupResponseIO();
  
  //if ((PMC->PMC_PCSR0 & (0x01u << ID_PIOD)) != (0x01u << ID_PIOD)) 
  //  { PMC->PMC_PCER0 = PMC->PMC_PCSR0 | 0x01u << ID_PIOD;}   // Enable PIO D clock 

} // end of setup

//---------------------------------------------
// MAIN LOOP  
//---------------------------------------------
void loop() {
     update_response_state();
      
} // end of loop


//---------------------------------------------
//  update_debounce_mu  
//---------------------------------------------
void update_debounce_mu(uint8_t i)
{
  key_isOnUpdate[i] = true;
  debounce_us[i]    = (unsigned long) micros() + BEBOUNCE_US;
//--- response ->rising edge key pressed 
 // key_state[i]    = digitalRead(inputpin[i]);
  //key_state[i]      = ( ( INPUT_PIN_PORT->PIO_PDSR & (1<< key_input_port_shift[i]) ) == (1<< key_input_port_shift[i]) );
  key_state[i]        = ( INPUT_PIN_PORT->PIO_PDSR & (1<< key_input_port_shift[i]) );
//  key_state[i]      = PIOD->PIO_PDSR & 1<< key_input_port_shift[i];
  key_apply_update[i]= true;
  key_isOnUpdate[i]  = false;
}


//---------------------------------------------
//  update_response_state  
//---------------------------------------------
void update_response_state()
{
 //unsigned long dt_u = (unsigned long) micros();
 
 bool input_status = false;
   
 for (uint8_t i=0; i < USED_RESPONSE_KEYS; i++)
  { 
    
    if (!key_isOnUpdate[i])
      { 
       if ( ( (key_apply_update[i] == true) && ( (unsigned long) micros() > debounce_us[i]) ) )
         {
         //--- write key
           if ( key_state[i] > 0 )
            { 
             OUTPUT_PIN_PORT->PIO_SODR |= (1<< key_output_port_shift[0]);  // set output pin 33 HIGH
             Keyboard.write( HIDKey[i] );
            
             /*Serial.print("key "); 
              Serial.print(HIDKey[i]); Serial.print(" -> "); 
              Serial.println( input_status); //INPUT_PIN_PORT->PIO_PDSR );
             */
            }
                     
         //--- reset
          debounce_us[i] = 0;
          key_apply_update[i]= false;
         } // if update and dt
       }// if not OnUpdate   
   } // for 
   
 //-- if no button is pressed 
 /*
   Serial.println("-> "); 
   for (uint8_t i=0; i < MAX_RESPONSE_KEYS; i++)
   {Serial.print(i); Serial.print(" -> ");Serial.print( digitalRead( inputpin[i]) );Serial.print(" | ");}
 */  
 
   for (uint8_t i=0; i < USED_RESPONSE_KEYS; i++)
    { 
      if  ( INPUT_PIN_PORT->PIO_PDSR & (1<< key_input_port_shift[i]) ) {input_status = true;break;}
       
    }
  // Serial.print(" status: "); Serial.println( input_status);
   if ( !input_status ) { OUTPUT_PIN_PORT->PIO_CODR |= 1<< key_output_port_shift[0]; }      

} // end of update_response_state

