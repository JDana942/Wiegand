#include "pins_arduino.h"
volatile long b;
bool readerTest_1;
bool readerTest_2;
int readerCONN_LED = 9;

/* Pin to interrupt map:
 * D0-D7 = PCINT 16-23 = PCIR2 = PD = PCIE2 = pcmsk2
 * D8-D13 = PCINT 0-5 = PCIR0 = PB = PCIE0 = pcmsk0
 * A0-A5 (D14-D19) = PCINT 8-13 = PCIR1 = PC = PCIE1 = pcmsk1
 */

volatile uint8_t *port_to_pcmask[] = {
  &PCMSK0,
  &PCMSK1,
  &PCMSK2
};

typedef void (*voidFuncPtr)(void);

volatile static voidFuncPtr PCintFunc[24] = { 
  NULL };

volatile static uint8_t PCintLast[3];

 void PCattachInterrupt(uint8_t pin, void (*userFunc)(void), int mode) {
  
  uint8_t bit = digitalPinToBitMask(pin);
  uint8_t port = digitalPinToPort(pin);
  uint8_t slot;
  volatile uint8_t *pcmask;

  if (mode != CHANGE) {
    return;
  }
  // map pin to PCIR register
  if (port == NOT_A_PORT) {
    return;
  } 
  else {
    port -= 2;
    pcmask = port_to_pcmask[port];
  }
  slot = port * 8 + (pin % 8);
  PCintFunc[slot] = userFunc;
  // set the mask
  *pcmask |= bit;
  // enable the interrupt
  PCICR |= 0x01 << port;
}

void PCdetachInterrupt(uint8_t pin) {
  
  uint8_t bit = digitalPinToBitMask(pin);
  uint8_t port = digitalPinToPort(pin);
  volatile uint8_t *pcmask;

  // map pin to PCIR register
  if (port == NOT_A_PORT) {
    return;
  } 
  else {
    port -= 2;
    pcmask = port_to_pcmask[port];
  }

  // disable the mask.
  *pcmask &= ~bit;
  if (*pcmask == 0) {
    PCICR &= ~(0x01 << port);
  }
}

static void PCint(uint8_t port) {
  uint8_t bit;
  uint8_t curr;
  uint8_t mask;
  uint8_t pin;

  // get the pin states for the indicated port.
  curr = *portInputRegister(port+2);
  mask = curr ^ PCintLast[port];
  PCintLast[port] = curr;
  // mask is pins that have changed. screen out non pcint pins.
  if ((mask &= *port_to_pcmask[port]) == 0) {
    return;
  }
  // mask is pcint pins that have changed.
  for (uint8_t i=0; i < 8; i++) {
    bit = 0x01 << i;
    if (bit & mask) {
      pin = port * 8 + i;
      if (PCintFunc[pin] != NULL) {
        PCintFunc[pin]();
      }
    }
  }
}

SIGNAL(PCINT0_vect) {
  PCint(0);
}
SIGNAL(PCINT1_vect) {
  PCint(1);
}
SIGNAL(PCINT2_vect) {
  PCint(2);
}

// End of interrupts code and start of the reader code

volatile long reader1 = 0,reader2 = 0;
volatile int reader1Count = 0, reader2Count = 0;

void reader1One(void) {
  if(digitalRead(4) == LOW){
  reader1Count++;
  reader1 = reader1 << 1;
  reader1 |= 1;
  }
}

void reader1Zero(void) {
  if(digitalRead(5) == LOW){
  reader1Count++;
  reader1 = reader1 << 1;  
  }
}

void reader2One(void) {
  if(digitalRead(6) == LOW){
  reader2Count++;
  reader2 = reader2 << 1;
  reader2 |= 1;
  }
}

void reader2Zero(void) {
  if(digitalRead(7) == LOW){
  reader2Count++;
  reader2 = reader2 << 1;  
  }
}

void setup()
{
  Serial.begin(9600);
  // Attach pin change interrupt service routines from the Wiegand RFID readers
  PCattachInterrupt(4, reader1One, CHANGE);
  PCattachInterrupt(5, reader1Zero, CHANGE);
  PCattachInterrupt(6, reader2One, CHANGE);
  PCattachInterrupt(7, reader2Zero, CHANGE);
  delay(10);
  for(int i = 4; i<8; i++){
    pinMode(i, INPUT_PULLUP);
  }
  delay(10);
  // put the reader input variables to zero
  reader1 = reader2 = 0;
  reader1Count = reader2Count = 0;
}

void loop() {
  readerTest_1 = digitalRead(4);
  readerTest_2 = digitalRead(6);
  if(readerTest_1 == 1 && readerTest_2 == 1){
    digitalWrite(readerCONN_LED, HIGH);
  }
  else{
    digitalWrite(readerCONN_LED,LOW);
  }
  if(reader1Count >= 26){
  //Serial.print("Binary  Data = ");Serial.println(reader1,BIN);
  //b = ~reader1;
  //b >>= 1;
  //Serial.print("Edited = "); Serial.println(b, BIN);
  Serial.print("Reader_1  Data = ");Serial.println(reader1,DEC);
  reader1 = 0;
  reader1Count = 0;
     }
     
  if(reader2Count >= 26){
  Serial.print("Reader_2  Data = ");Serial.println(reader2,DEC);
  reader2 = 0;
  reader2Count = 0;
  }
}
