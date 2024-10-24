/*

  CSC C85 - Embedded Systems - Project # 1 - EV3 Robot Localization
   
 This file provides the headers for the EV3 localization starter code. Please have a look to
see what is provided by the starter. The corresponding .c file contains the implementation
as well as clearly defined *** TO DO *** sections that correspond to the parts of the 
code that you have to complete in order to implement the localization algorithms.
 
 In a nutshell, the starter code provides:
 
 * Reading a map from an input image (in .ppm format). The map is bordered with red, 
   must have black streets with yellow intersections, and buildings must be either
   blue, green, or be left white (no building).
   
 * Setting up an array with map information which contains, for each intersection,
   the colours of the buildings around it in ** CLOCKWISE ** order from the top-left.
   
 * Initialization of the EV3 robot (opening a socket and setting up the communication
   between your laptop and your bot)
   
 What you must implement:
 
 * All aspects of robot control:
   - Finding and then following a street
   - Recognizing intersections
   - Scanning building colours around intersections
   - Detecting the map boundary and turning around or going back - the robot must not
     wander outside the map (though of course it's possible parts of the robot will
     leave the map while turning at the boundary)

 * The histogram-based localization algorithm that the robot will use to determine its
   location in the map - this is as discussed in lecture.

 * Basic robot exploration strategy so the robot can scan different intersections in
   a sequence that allows it to achieve reliable localization
   
 * Basic path planning - once the robot has found its location, it must drive toward a 
   user-specified position somewhere in the map.

 What you need to understand thoroughly in order to complete this project:
 
 * The histogram localization method as discussed in lecture. The general steps of
   probabilistic robot localization.

 * Sensors and signal management - your colour readings will be noisy and unreliable,
   you have to handle this smartly
   
 * Robot control with feedback - your robot does not perform exact motions, you can
   assume there will be error and drift, your code has to handle this.
   
 * The robot control API you will use to get your robot to move, and to acquire 
   sensor data. Please see the API directory and read through the header files and
   attached documentation
   
 Starter code:
 F. Estrada, 2018 - for CSC C85 
 
*/


#ifndef __localization_header
#define __localization_header

#include<stdio.h>
#include<stdlib.h>
#include<math.h>
#include<malloc.h>
#include "./EV3_RobotControl/btcomm.h"

#ifndef HEXKEY
	#define HEXKEY "00:16:53:56:07:8a"	// <--- SET UP YOUR EV3's HEX ID here
#endif

typedef struct {
  double Kp;
  double Ki;
  double Kd;

  /* Output limits */
  double limMin;
	double limMax;

  /* Controller "memory" */
	// double integrator;
  int prevError;
	int prevErrorArr[10];
	// double differentiator;
	int prevMeasurement;
  int arr_size;

	/* Controller output */
	double out;

} PIDController;

/*
  The ports associated with whatever sensors/motors
*/

#define MOTOR_LEFT   MOTOR_D
#define MOTOR_RIGHT  MOTOR_A
#define MOTOR_MIDDLE MOTOR_B
#define MOTOR_GHOST  MOTOR_C
#define COLOUR_PORT  PORT_2
#define GYRO_PORT    PORT_3
#define ULTRASONIC_PORT PORT_4

/*
  Other Constants
*/

#define COLOUR_DATA_FILE "colour_data.txt"
#define DATA_PTS_PER_COLOUR 10
#define READS_PER_DATA_PT 100

void wait_ready_to_scan(void);
int wait_colour_change(int* coloursArray, int initialColour);
int wait_until_get_colours(int* coloursArray, int wantedColours[], int numColours);
int wait_colour_consistent(int* coloursArray);
int wait_turn_angle(int turn_angle);

int turn(int* coloursArray, int turn_angle, int checkRightAway);
int* get_colour_dataPoint(int*coloursArray, int colour, int dataPoint);
void reading_colour_data(int* coloursArray);
void read_colour_sensor(int repetitions, int* R, int* G, int* B);
int detect_and_classify_colour(int* coloursArray);
void scan_colours(int* coloursArray, int coloursDetected[3]);

int parse_map(unsigned char *map_img, int rx, int ry);
int robot_localization(int *coloursArray, int *robot_x, int *robot_y, int *direction);
int find_street(int* coloursArray);
int go_to_target(int* coloursArray, int robot_x, int robot_y, int direction, int target_x, int target_y);
int drive_along_street(int *colorArr);
int scan_intersection(int* coloursArrary, int *tl, int *tr, int *br, int *bl);
int turn_at_intersection(int* coloursArray, int turn_direction);
void calibrate_sensor(void);
unsigned char *readPPMimage(const char *filename, int *rx, int*ry);
void pid_straight_init(PIDController *pid);
double pid_controller_update(PIDController *pid, int error, int measurement);

void align_street(int *colorArr);
void normalize_beliefs(void);
void print_beliefs(void);
void test_localization(void);
void update_beliefs(int tl, int tr, int br, int bl);
void update_facing_beliefs(int colour, int position);
void rotate_beliefs(int direction);
int is_localized(int *x_pos, int *y_pos, int *direction);
#endif
