typedef struct {
  float x;
  float y;
  float z;
} angle_errors;

typedef struct {
  short int motor_1_value;
  short int motor_2_value;
  short int motor_3_value;
  short int motor_4_value;
} quad_motors;

typedef struct {
  short int motor_A_value;
  short int motor_B_value;
} dual_motors;


typedef struct {
  float coeff_p;
  float coeff_i;
  float coeff_d;
  float sum_error;
  float last_error;
} dpid_t;

extern quad_motors quadcopter;

void regulation_init();
void regulation_loop(angle_errors values);
void setPidx_P(float p);
void setPidx_I(float i);
void setPidx_D(float d);
void print_Pidx();