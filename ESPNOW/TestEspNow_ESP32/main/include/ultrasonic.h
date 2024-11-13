#define TRIG_PIN GPIO_NUM_5      
#define ECHO_PIN GPIO_NUM_18    
#define SOUND_SPEED 0.0343       

void init_ultrasonic_sensor(void);

float measure_distance(void);

float measure_distance(int trig_pin, int echo_pin);