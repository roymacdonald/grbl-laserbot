#ifndef pen_servo_h
#define pen_servo_h

#define PEN_SERVO_DOWN     60      
#define PEN_SERVO_UP       0        



void servo_init();
void servo_stop();
void pen_up();
void pen_down();
void set_pen_pos();


#endif
