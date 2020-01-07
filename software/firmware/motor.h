/* Low throttle is the minimum throttle for the ESC initialization */
#define MOTOR_LO_THROTTLE   126
#define MOTOR_MIN_THROTTLE  (MOTOR_LO_THROTTLE + 1)
/* High throttle is the maximum throttle for the ESC initialization */
#define MOTOR_HI_THROTTLE   255
#define MOTOR_MAX_THROTTLE  (MOTOR_HI_THROTTLE - 1)
/* MOTOR_MIN_VALUE / MOTOR_MAX_VALUE are user value */
#define MOTOR_MIN_VALUE     0
#define MOTOR_MAX_VALUE     (MOTOR_MAX_THROTTLE - MOTOR_MIN_THROTTLE)

typedef enum {
    MOTOR_A_PIN = 6,
    MOTOR_B_PIN = 7,
    MOTOR_C_PIN = 8,
    MOTOR_D_PIN = 9,
    MOTOR_PIN_MAX
} MotorPin;
