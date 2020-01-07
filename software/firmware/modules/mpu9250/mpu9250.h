void setup_driver();
/* Define an angle */
typedef struct gyro_struct {
  float x = 0;
  float y = 0;
  float z = 0;
} gyro_t;

extern gyro_t gyro_sum;

/* Define an acceleration */
typedef struct accel_struct {
  float x = 0;
  float y = 0;
  float z = 0;
} accel_t;
