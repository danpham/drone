/* Low throttle is the minimum throttle for the ESC initialization */
#define MOTOR_LO_THROTTLE   2046
#define MOTOR_MIN_THROTTLE  (MOTOR_LO_THROTTLE + 1)
/* High throttle is the maximum throttle for the ESC initialization */
#define MOTOR_HI_THROTTLE   4095
#define MOTOR_MAX_THROTTLE  (MOTOR_HI_THROTTLE - 20)
/* MOTOR_MIN_VALUE / MOTOR_MAX_VALUE are user value */
#define MOTOR_MIN_VALUE     0
#define MOTOR_MAX_VALUE     (MOTOR_MAX_THROTTLE - MOTOR_MIN_THROTTLE)

typedef enum {
    MOTOR_A,
    MOTOR_B,
    MOTOR_C,
    MOTOR_D,
    MOTOR_MAX
} MotorName;

typedef enum {
    MOTOR_A_PIN = 6,
    MOTOR_B_PIN = 7,
    MOTOR_C_PIN = 8,
    MOTOR_D_PIN = 9,
    MOTOR_PIN_MAX
} MotorPin;

typedef struct {
    U8 motor_pin;
    U16 motor_offset;
    float motor_gain;
} Motor;

extern void setup_motor(void);
extern U16 getMotorOffset(const U8 motorId);
extern void setMotorOffset(const U8 motorId, const U16 offset);
extern float getMotorGain(const U8 motorId);
extern void setMotorGain(const U8 motorId, const float value);
extern void printMotorData(void);
void setMotorValue(const U8 motorId, const short value);