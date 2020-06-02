#include "grbl.h"
#include "pen_servo.h"
#include <Arduino.h>

#define MIN_PULSE_WIDTH       544     // the shortest pulse sent to a servo  
#define MAX_PULSE_WIDTH      2400     // the longest pulse sent to a servo 
#define DEFAULT_PULSE_WIDTH  1500     // default pulse width when servo is attached
#define REFRESH_INTERVAL    20000     // minumim time to refresh servos in microseconds 

#define SERVO_PIN A9

typedef struct {
  uint8_t _isActive;
  volatile unsigned int ticks;
} servo_t;


#define usToTicks(_us)    (( clockCyclesPerMicrosecond()* _us) / 8)     // converts microseconds to tick (assumes prescale of 8)  // 12 Aug 2009
#define ticksToUs(_ticks) (( (unsigned)_ticks * 8)/ clockCyclesPerMicrosecond() ) // converts from ticks back to microseconds

#define TRIM_DURATION  2                                   // compensation ticks to trim adjust for digitalWrite delays
static servo_t servo;
static int servoPos;
// static volatile int8_t currentServoIndex;// index for the servo being pulsed for each timer (or -1 if refresh interval)
static volatile int8_t pulse_state;
// convenience macros
/************ static functions common to all instances ***********************/

static inline void handle_interrupts(volatile uint16_t *TCNTn, volatile uint16_t* OCRnA){
  if(pulse_state == 0){
     digitalWrite( SERVO_PIN,HIGH); // its an active channel so pulse it high
//     digitalWrite( 13,HIGH);
    *TCNTn = 0; // channel set to -1 indicated that refresh interval completed so reset the timer  
     *OCRnA = servo.ticks - TRIM_DURATION;
     pulse_state =  1;
  }else if( pulse_state == 1 ){
     *OCRnA = (unsigned int)usToTicks(REFRESH_INTERVAL) - *TCNTn - TRIM_DURATION;  
      digitalWrite(SERVO_PIN,LOW); // pulse this channel low if activated
//      digitalWrite( 13,LOW);
      pulse_state =  0;
  }
}

ISR(TIMER5_COMPA_vect){
  handle_interrupts( &TCNT5, &OCR5A);
}



void initISR(){
    servo._isActive = 1;
    pulse_state = 0;
    TCCR5A = 0;             // normal counting mode
    TCCR5B = _BV(CS51);     // set prescaler of 8
    TCNT5 = 0;              // clear the timer count
    TIFR5 = _BV(OCF5A);     // clear any pending interrupts;
    TIMSK5 =  _BV(OCIE5A) ; // enable the output compare interrupt
//     sei();  
 // printString("INIT ISR\n");
}

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
void finISR(){

    TIMSK5 &= ~_BV(OCIE5A) ; // enable the output compare interrupt
    servo._isActive = 0;
// printString("FIN ISR\n");
}
/****************** end of static functions ******************************/

#define PEN_UP 1
#define PEN_DOWN 0
static uint8_t pen_pos;//0 down, 1 up
void servo_init(){
  servo._isActive = 0;
  servoPos = -1;
  pen_pos =PEN_DOWN;
}

void servo_attach(){
  if (servo._isActive == 0) {
    // printString("SERVO ATTACH\n");

    servo.ticks = usToTicks(DEFAULT_PULSE_WIDTH);   // store default values
    pinMode(SERVO_PIN, OUTPUT);                                   // set servo pin to output
    pinMode(13, OUTPUT);
      initISR();
    }
}

void servo_detach(){
  
  if(servo._isActive){
    finISR();  
  }
}

void servo_writeMicroseconds(int value){
    if (value < MIN_PULSE_WIDTH)       
      value = MIN_PULSE_WIDTH;
    else if (value > MAX_PULSE_WIDTH)
      value = MAX_PULSE_WIDTH;

    value = value - TRIM_DURATION;
    value = usToTicks(value);  // convert to ticks after compensating for interrupt overhead
    servo.ticks = value;
}

void servo_write(int value){
  // treat values less than 544 as angles in degrees (valid values in microseconds are handled as microseconds)
  if (value < MIN_PULSE_WIDTH)
  {
    if (value < 0)
      value = 0;
    else if (value > 180)
      value = 180;

    value = map(value, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  }
  servo_writeMicroseconds(value);
}


int servo_readMicroseconds()
{
  return  ticksToUs(servo.ticks) + TRIM_DURATION;
}

int servo_read() // return the value as degrees
{
  return map(servo_readMicroseconds()+1, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH, 0, 180);
}

void pen_up() {
  if(pen_pos != PEN_UP){
  servo_attach();
  servo_write(PEN_SERVO_UP);
  pen_pos = PEN_UP;
  // printString("SERVO UP\n");
  }
}

void pen_down(){
   if(pen_pos != PEN_DOWN){
    servo_attach();
    servo_write(PEN_SERVO_DOWN);
    pen_pos = PEN_DOWN;
  // printString("SERVO DOWN\n");
   }
}

void servo_stop(){
  servo_detach();
}

void set_pen_pos()
{ 
  float z = system_convert_axis_steps_to_mpos(sys_position, Z_AXIS) - gc_state.coord_system[Z_AXIS];  // get the machine Z in mm 

    if(z < 0.0){
     z = 0.0; 
     }else if(z > 1.0){
      z = 1.0;
    }
    

    int p = z*(PEN_SERVO_UP - PEN_SERVO_DOWN) + PEN_SERVO_DOWN ;
    if(servoPos != p){
      servo_attach();
      servo_write( p);
      servoPos = p;
      // printString("servoPos: ");   
      // printInteger(p);
    }
    
// if(wpos_z != sys_position[Z_AXIS]){

//    printInteger(wpos_z);
//    printString("  sys_position: ");   
//    printInteger(sys_position[Z_AXIS]);
//    printString("\n");   
//    wpos_z = sys_position[Z_AXIS];
//    printString("z pos: ");   
//        printFloat(z, 5);
//  if(pen_pos != PEN_UP && z > 0.0){
//   pen_up();
// }else  if(pen_pos != PEN_DOWN && z < 1.0){
//   pen_down();
// } 
//}
}

