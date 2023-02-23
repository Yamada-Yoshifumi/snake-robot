#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <webots/connector.h>
#include <webots/motor.h>
#include <webots/robot.h>

#define CONTROL_STEP 32
#define N 6
#define pi M_PI

static double t = 0.0; /* time elapsed since simulation start [s] */
static int id = -1;    /* this module's ID */

/* each module is equipped with a single motormotor and 2 connectors */
static WbDeviceTag motor, rear_connector, front_connector;

/* oscillation parameters */
static const double A = 0.9; /* amplitude */
static const double T = 1.0; /* time period of oscillation */
static const double F = 2.0 * pi / T; /* frequency */

static void forward() {
  double angle_p = A*sin((F*t) + (id * (2.0 * pi / 8)));
  double angle_y = 0.0;
  double angle_head = (A/4)*sin((F*t) + (id * (2.0 * pi / 8)));

  if (id == 1) {
    wb_motor_set_position(motor, angle_head);
  } else if (id == 3 || id ==5 || id == 7) {
    wb_motor_set_position(motor, angle_p);
  } else if (id == 2 || id == 4 || id == 6) {
    wb_motor_set_position(motor, angle_y);
  } else {
    //do_nothing();
  }
}

static void backward() {
  double angle_p = A*sin((F*t) - (id * (2.0 * pi / 8)));
  double angle_y = 0.0;
  double angle_head = (A/3)*sin((F*t) - (id * (2.0 * pi / 8)));

  if (id == 1) {
    wb_motor_set_position(motor, angle_head);
  } else if (id == 3 || id ==5 || id == 7) {
    wb_motor_set_position(motor, angle_p);
  } else if (id == 2 || id == 4 || id == 6) {
    wb_motor_set_position(motor, angle_y);
  } else {
    //do_nothing();
  }
}

static void rotate_cw() {
  double angle_p = A*sin(F*t);
  double angle_y = A*sin(F*t + pi*(id-2)/2 - pi/2);

  printf("Rotate clockwise");
  if (id == 1 || id == 3 || id ==5 || id == 7) {
    wb_motor_set_position(motor, angle_p);
  } else if (id == 2 || id == 4 || id == 6) {
    wb_motor_set_position(motor, angle_y);
  } else {
    //do_nothing();
  }
}

static void rotate_acw() {
  double angle_p = A*sin(F*t);
  double angle_y = A*sin(F*t + pi*(id-2)/2 + pi/2);

  printf("Rotate anticlockwise");
  if (id == 1 || id == 3 || id ==5 || id == 7) {
    wb_motor_set_position(motor, angle_p);
  } else if (id == 2 || id == 4 || id == 6) {
    wb_motor_set_position(motor, angle_y);
  } else {
    //do_nothing();
  }
}

static void sidewinding_left() {
  double angle_p = A*sin(F*t + (-2*pi/3)*(id-1)/2);
  double angle_y = A*sin(F*t + (-pi/3)*(id-2)/2 - pi/9);

  printf("Sidewinding left");
  if (id == 1 || id == 3 || id ==5 || id == 7) {
    wb_motor_set_position(motor, angle_p);
  } else if (id == 2 || id == 4 || id == 6) {
    wb_motor_set_position(motor, angle_y);
  } else {
    //do_nothing();
  }
}

static void sidewinding_right() {
  double angle_p = A*sin(F*t + (2*pi/3)*(id-1)/2);
  double angle_y = A*sin(F*t + (pi/3)*(id-2)/2 + pi/9);

  printf("Sidewinding right");
  if (id == 1 || id == 3 || id ==5 || id == 7) {
    wb_motor_set_position(motor, angle_p);
  } else if (id == 2 || id == 4 || id == 6) {
    wb_motor_set_position(motor, angle_y);
  } else {
    //do_nothing();
  }
}

static void disconnect_all() {
  static int done = 0;

  if (done)
    return;

  /* detach everything */
  wb_connector_unlock(front_connector);
  wb_connector_unlock(rear_connector);
  done = 1;
}

static void do_nothing() {
}




/* demo steps and durations */
struct {
  void (*func)();
  float duration;
} states[] = {{forward, 10}, {backward, 10}, {do_nothing, 2}, {rotate_cw, 7}, {rotate_acw, 7}, {do_nothing, 2}, {sidewinding_left, 7},
              {sidewinding_right, 7}, {do_nothing, 10}, {NULL, 1}};

static int state = 0;
static float deadline;

int main() {
  /* necessary to initialize Webots */
  wb_robot_init();

  /*
   * Find module id from robot name
   * The robot name is used to identify each module
   */
  const char *name = wb_robot_get_name();

  id = atoi(name + 7) - 1;

  /* find hardware devices */
  motor = wb_robot_get_device("motor");
  rear_connector = wb_robot_get_device("rear_connector");
  front_connector = wb_robot_get_device("front_connector");

  deadline = states[0].duration;

  while (wb_robot_step(CONTROL_STEP) != -1) {
    /* change state */
    if (t >= deadline) {
      state++;
      deadline += states[state].duration;
    }

    /* finished ? */
    if (!states[state].func)
      continue;

    /* call current state's function */
    (*states[state].func)();

    /* computed elapsed time */
    t += CONTROL_STEP / 1000.0;
  }

  wb_robot_cleanup();

  return 0;
}