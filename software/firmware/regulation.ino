/******************************************************************
 * 1. Included files (microcontroller ones then user defined ones)
******************************************************************/

/******************************************************************
 * 2. Define declarations (macros then function macros)
******************************************************************/
#define P_GAIN    0.25
#define I_GAIN    0.1
#define D_GAIN    0.0

/******************************************************************
 * 3. Typedef definitions (simple typedef, then enum and structs)
******************************************************************/

/******************************************************************
 * 4. Variable definitions (static then global)
******************************************************************/
static dpid_t pid_x;
static dpid_t pid_y;
static dpid_t pid_z;
static dual_motors s_1_and_2;
static dual_motors s_3_and_4;
static dual_motors s_1_and_2y;
static dual_motors s_3_and_4y;
quad_motors quadcopter;

/******************************************************************
 * 5. Functions prototypes (static only)
******************************************************************/

void regulation_init() {
  pid_x = {P_GAIN, I_GAIN, D_GAIN, 0.0, 0.0};
  pid_y = {P_GAIN, I_GAIN, D_GAIN, 0.0, 0.0};
  pid_z = {P_GAIN, I_GAIN, D_GAIN, 0.0, 0.0};
  quadcopter = {0, 0, 0, 0};
}

static void pidx(float angle_error, dpid_t * values, dual_motors * motors_A_B, dual_motors * motors_C_D)
{
  short int command = 0;

  if (angle_error > 1) {
    /* Add the error to the sum */
    values->sum_error += angle_error;
  }
  /* Check sum error does not grow */
  if ((values->sum_error > 100) || (values->sum_error < -100)) {
    values->sum_error = 0;
  }

  command = (short int)(values->coeff_p * angle_error + values->coeff_i * values->sum_error);

  motors_A_B->motor_A_value = command;
  motors_A_B->motor_B_value = command;
  motors_C_D->motor_A_value = -1 * command;
  motors_C_D->motor_B_value = -1 * command;
}

static void pidy(float angle_error, dpid_t * values, dual_motors * motors_A_B, dual_motors * motors_C_D) {

  short int command = 0;

  /* Add the error to the sum */
  values->sum_error += angle_error;

  /* Check sum error does not grow */
  if ((values->sum_error > 100) || (values->sum_error < -100)) {
    values->sum_error = 0;
  }

  command = (short int)(values->coeff_p * angle_error + values->coeff_i * values->sum_error);

  motors_A_B->motor_A_value = -1 * command;
  motors_A_B->motor_B_value = command;
  motors_C_D->motor_A_value = command;
  motors_C_D->motor_B_value = -1 * command;
}

void regulation_loop(angle_errors values /* consigne */) {
  /* X regulation */
  pidx(values.angle_error_x, &pid_x, &s_1_and_2, &s_3_and_4);
  /* Y regulation */
  pidy(values.angle_error_y, &pid_y, &s_1_and_2y, &s_3_and_4y);

  quadcopter.motor_1_value = (127 * (s_1_and_2.motor_A_value)) / 512; // + s_1_and_3_for_lacet.motor_A_value;
  quadcopter.motor_2_value = (127 * (s_1_and_2.motor_B_value)) / 512;
  quadcopter.motor_3_value = (127 * (s_3_and_4.motor_A_value)) / 512; // + s_1_and_3_for_lacet.motor_B_value;
  quadcopter.motor_4_value = (127 * (s_3_and_4.motor_B_value)) / 512;
}
