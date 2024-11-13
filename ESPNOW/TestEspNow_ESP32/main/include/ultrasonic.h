#define TRIG_PIN_1 4
#define ECHO_PIN_1 5
#define TRIG_PIN_2 18
#define ECHO_PIN_2 19
#define CRITICAL_DISTANCE_CM 20
#define SOUND_SPEED 0.034
#define TIMEOUT 30000


void init_ultrasonic_sensor(void);

float measure_distance(int trig_pin, int echo_pin);