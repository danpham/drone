/******************************************************************
 * 1. Included files (microcontroller ones then user defined ones)
******************************************************************/

/******************************************************************
 * 2. Define declarations (macros then function macros)
******************************************************************/
#define P_DEFAULT_GAIN    8.0
#define I_DEFAULT_GAIN    0.0
#define D_DEFAULT_GAIN    0.0

/******************************************************************
 * 3. Typedef definitions (simple typedef, then enum and structs)
******************************************************************/

/******************************************************************
 * 4. Variable definitions (static then global)
******************************************************************/
static dpid_t pid_x;
static dpid_t pid_y;
//static dpid_t pid_z;
quad_motors quadcopter;

/******************************************************************
 * 5. Functions prototypes (static only)
******************************************************************/
static void pid(float angle_error_x, float angle_error_y, dpid_t * values_x, dpid_t * values_y, quad_motors * motors);

void setPidx_P(float p) {
  pid_x.coeff_p = p;
  pid_y.coeff_p = p;
}


void setPidx_I(float i) {
  pid_x.coeff_i = i;
  pid_y.coeff_i = i;
}


void setPidx_D(float d) {
  pid_x.coeff_d = d;
  pid_y.coeff_d = d;
}


void regulation_init() {
  pid_x = {P_DEFAULT_GAIN, I_DEFAULT_GAIN, D_DEFAULT_GAIN, 0.0, 0.0};
  pid_y = {P_DEFAULT_GAIN, I_DEFAULT_GAIN, D_DEFAULT_GAIN, 0.0, 0.0};
  quadcopter = {0, 0, 0, 0};
}


void print_Pidx() {
    SerialUSB.print("Coeff P\t");
    SerialUSB.print(pid_x.coeff_p);
    SerialUSB.print("\tCoeff I\t");
    SerialUSB.print(pid_x.coeff_i);
    SerialUSB.print("\tCoeff D\t");
    SerialUSB.println(pid_x.coeff_d);
}

void regulation_loop(angle_errors values /* consigne */) {
  pid(values.x, values.y, &pid_x, &pid_y, &quadcopter);
}

static void pid(float angle_error_x, float angle_error_y, dpid_t * values_x, dpid_t * values_y, quad_motors * motors)
{
  short int command_x = 0;
  short int command_y = 0;
  static float last_angle_error_x = 0;
  static float last_angle_error_y = 0;

  /* Add the error to the sum */
  values_x->sum_error += angle_error_x;
  values_y->sum_error += angle_error_y;

  /* Check sum error does not grow */
  if ((values_x->sum_error > 100) || (values_x->sum_error < -100))
  {
    values_x->sum_error = 0;
  }

  /* Check sum error does not grow */
  if ((values_y->sum_error > 100) || (values_y->sum_error < -100))
  {
    values_y->sum_error = 0;
  }

  command_x = (short int)(values_x->coeff_p * angle_error_x + values_x->coeff_i * values_x->sum_error + values_x->coeff_d * (angle_error_x - last_angle_error_x));
  command_y = (short int)(values_y->coeff_p * angle_error_y + values_y->coeff_i * values_y->sum_error + values_y->coeff_d * (angle_error_y - last_angle_error_y));

  last_angle_error_x = angle_error_x;
  last_angle_error_y = angle_error_y;

  motors->motor_1_value = command_x;
  motors->motor_2_value = -1 * command_y;
  motors->motor_3_value = -1 * command_x;
  motors->motor_4_value = command_y;
}
