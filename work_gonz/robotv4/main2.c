/*
 *  TU/e 5EID0 - Venus Project -- ROBOT 2 entry point.
 *
 *  Robot 2 is placed right next to robot 1 at the shared start point but facing
 *  the OPPOSITE way: robot 1 looks along +y, robot 2 looks along -y. Both share
 *  the same world origin (0,0) and axes, so by baking robot 2's fixed start
 *  heading (theta = -pi/2) in here, both robots integrate odometry in ONE common
 *  frame. That makes their cube coordinates directly comparable -- which is what
 *  lets the UI map overlay both robots and the cross-robot dedup / six-cube stop
 *  line up, with NO runtime coordinate transform.
 *
 *  This file is just main.c with robot 2's identity + start heading. Compile it
 *  INSTEAD OF main.c for the second robot (never both in the same binary):
 *     gcc main2.c -o robot2 $(pkg-config --cflags --libs libpynq) -lm
 *
 *  Everything else (autopilot, sensors, mission logic) lives in main.c.
 */

#define ROBOT_ID "r2"
/* Start position defaults to (0,0) in main.c (shared origin). Only the heading
 * differs: -pi/2 so "forward" is -y. Literal (not M_PI) because main.c's M_PI is
 * not defined until after it is #included below. */
#define START_THETA_RAD (-1.5707963267948966)   /* -pi/2: robot 2 looks along -y */

#include "main.c"
