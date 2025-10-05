

/*
 * Copyright 1996-2024 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * File: youbot_hanoi_solver.c
 * Date: October 5, 2025
 * Description: An intelligent and recursive Tower of Hanoi solver for the YouBot robot.
 * The robot uses its Kinect sensor to dynamically locate disks for precise
 * manipulation, solving the puzzle for a user-configurable number of disks.
 * Author: Prince Sharma
 */


/*
1. 𝐑𝐞𝐜𝐮𝐫𝐬𝐢𝐯𝐞 𝐒𝐨𝐥𝐯𝐞𝐫 : Streamlined Tower of Hanoi logic using recursion.

2. 𝐕𝐢𝐬𝐢𝐨𝐧-𝐁𝐚𝐬𝐞𝐝 𝐃𝐢𝐬𝐤 𝐃𝐞𝐭𝐞𝐜𝐭𝐢𝐨𝐧 : Kinect-driven object localization for accurate grip movement. 

3. 𝐂𝐨𝐧𝐟𝐢𝐠𝐮𝐫𝐚𝐛𝐥𝐞 𝐃𝐢𝐬𝐤 𝐂𝐨𝐮𝐧𝐭 : Dynamic disk configuration via controllerArgs—no rebuild needed.

4. 𝐒𝐩𝐞𝐞𝐝 𝐌𝐮𝐥𝐭𝐢𝐩𝐥𝐢𝐞𝐫 : Real-time motion tuning using G_SPEED_MULTIPLIER.

5. 𝐀𝐮𝐝𝐢𝐨 𝐂𝐮e : Speaker-triggered sound once the puzzle is solved.

6. 𝐄𝐫𝐫𝐨𝐫 𝐑𝐞𝐬𝐢𝐥𝐢𝐞𝐧𝐜𝐞 : Patched runtime issues, enhanced vision range, and added fail-safe handling.

7. 𝐖𝐨𝐫𝐥𝐝 𝐒𝐞𝐭𝐮𝐩 : Integrated a YouBot “Speaker” device into the Webots scene tree.  

*/


#include <webots/camera.h>
#include <webots/range_finder.h>
#include <webots/robot.h>
#include <webots/speaker.h> 

#include <arm.h>
#include <base.h>
#include <gripper.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#define TIME_STEP 32
#define MAX_DISKS 3 


typedef enum {
  SRC_PEG = 0,
  AUX_PEG = 1,
  DST_PEG = 2
} Peg;

enum {
  X = 0,
  Y = 1,
  ANGLE = 2
};


static double GOTO_INFO[3][3];
static int DISK_LEVELS[3] = {0};
static int MOVE_COUNT = 0;
static double G_SPEED_MULTIPLIER = 1.0; 


static void step();
static void passive_wait(double sec);
static void navigate_to_peg(Peg peg);
static bool find_top_disk_position(WbDeviceTag kinect_range, WbDeviceTag kinect_camera, double *x_pos, double *y_pos);
static void execute_grip_action(Peg peg, bool grip, WbDeviceTag kinect_range, WbDeviceTag kinect_camera);
static void initialize_peg_locations();
static void move_disk(Peg source, Peg destination, WbDeviceTag kinect_range, WbDeviceTag kinect_camera);
static void solve_tower_of_hanoi(int n, Peg source, Peg destination, Peg auxiliary, WbDeviceTag kinect_range, WbDeviceTag kinect_camera);




int main(int argc, char **argv) {
  wb_robot_init();

  
  base_init();
  base_goto_init(TIME_STEP);
  arm_init();
  gripper_init();

  
  WbDeviceTag kinect_color = wb_robot_get_device("kinect color");
  WbDeviceTag kinect_range = wb_robot_get_device("kinect range");
  wb_camera_enable(kinect_color, TIME_STEP);
  wb_range_finder_enable(kinect_range, TIME_STEP);

  
  WbDeviceTag speaker = wb_robot_get_device("speaker");

  
  passive_wait(1.0);
  arm_set_height(ARM_HANOI_PREPARE);
  passive_wait(2.0);

  
  printf("--- YouBot Tower of Hanoi Solver ---\n");
  int num_disks = 3; 
  if (argc > 1) { 
    int arg_disks = atoi(argv[1]); 
    if (arg_disks > 0 && arg_disks <= MAX_DISKS) {
      num_disks = arg_disks;
      printf("Received argument to solve for %d disk(s).\n", num_disks);
    } else {
      printf("Invalid argument '%s'. Must be between 1 and %d. Using default.\n", argv[1], MAX_DISKS);
    }
  } else {
    printf("No argument provided. Using default of %d disk(s).\n", num_disks);
  }

  
  initialize_peg_locations();
  DISK_LEVELS[SRC_PEG] = num_disks;
  DISK_LEVELS[AUX_PEG] = 0;
  DISK_LEVELS[DST_PEG] = 0;

  
  solve_tower_of_hanoi(num_disks, SRC_PEG, DST_PEG, AUX_PEG, kinect_range, kinect_color);

  
  printf("Puzzle solved in %d moves!\n", MOVE_COUNT);
  printf("Resetting robot to initial position.\n");
  if (speaker) 
    wb_speaker_play_sound(speaker, speaker, "sounds/success.wav", 1.0, 1.0, 0, false);
  arm_reset();
  navigate_to_peg(SRC_PEG); 
  navigate_to_peg(-1);      

  wb_robot_cleanup();
  return EXIT_SUCCESS;
}



static void solve_tower_of_hanoi(int n, Peg source, Peg destination, Peg auxiliary, WbDeviceTag kinect_range, WbDeviceTag kinect_camera) {
  if (n == 1) {
    move_disk(source, destination, kinect_range, kinect_camera);
    return;
  }
  solve_tower_of_hanoi(n - 1, source, auxiliary, destination, kinect_range, kinect_camera);
  move_disk(source, destination, kinect_range, kinect_camera);
  solve_tower_of_hanoi(n - 1, auxiliary, destination, source, kinect_range, kinect_camera);
}




static void move_disk(Peg source, Peg destination, WbDeviceTag kinect_range, WbDeviceTag kinect_camera) {
  MOVE_COUNT++;
  printf("Move #%d: Disk from Peg %d to Peg %d\n", MOVE_COUNT, source + 1, destination + 1);

  
  navigate_to_peg(source);
  execute_grip_action(source, true, kinect_range, kinect_camera);
  DISK_LEVELS[source]--;

  
  navigate_to_peg(destination);
  execute_grip_action(destination, false, kinect_range, kinect_camera);
  DISK_LEVELS[destination]++;
}

static void initialize_peg_locations() {
  const double distance_arm0_platform = 0.2;
  const double distance_arm0_robot_center = 0.189;
  const double distance_origin_platform = 1.0;
  const double angles[3] = {0.0, 2.0 * M_PI / 3.0, -2.0 * M_PI / 3.0};
  double delta = distance_origin_platform - distance_arm0_platform - distance_arm0_robot_center;
  for (int i = 0; i < 3; ++i) {
    GOTO_INFO[i][X] = delta * sin(angles[i]);
    GOTO_INFO[i][Y] = delta * cos(angles[i]);
    GOTO_INFO[i][ANGLE] = -angles[i];
  }
}

/**
 * @brief Uses the Kinect to find the precise (x, y) coordinates of the top disk.
 */
static bool find_top_disk_position(WbDeviceTag kinect_range, WbDeviceTag kinect_camera, double *x_pos, double *y_pos) {
  int width = wb_range_finder_get_width(kinect_range);
  int height = wb_range_finder_get_height(kinect_range);
  
  double fov = wb_camera_get_fov(kinect_camera);
  const float *image = wb_range_finder_get_range_image(kinect_range);

  if (!image) {
    printf("ERROR: Cannot get range image.\n");
    return false;
  }

  
  int center_pixel_idx = width * (height / 2) + (width / 2);
  *y_pos = image[center_pixel_idx];

  if (*y_pos < 0.1 || *y_pos > 0.3) {
    printf("WARNING: No disk found in operational range.\n");
    return false;
  }

  
  int y_pixel = height / 2;
  int left_edge_px = -1, right_edge_px = -1;

  for (int x_pixel = width / 2; x_pixel < width; ++x_pixel) {
    if (fabs(image[width * y_pixel + x_pixel] - *y_pos) > 0.02) {
      right_edge_px = x_pixel;
      break;
    }
  }
  for (int x_pixel = width / 2; x_pixel >= 0; --x_pixel) {
    if (fabs(image[width * y_pixel + x_pixel] - *y_pos) > 0.02) {
      left_edge_px = x_pixel;
      break;
    }
  }

  if (left_edge_px == -1 || right_edge_px == -1) {
    printf("WARNING: Could not determine disk edges. Using center.\n");
    *x_pos = 0.0;
    return true;
  }

  
  double center_px = (left_edge_px + right_edge_px) / 2.0;
  double pixel_offset_from_center = center_px - (width / 2.0);
  *x_pos = (pixel_offset_from_center / (width / 2.0)) * *y_pos * tan(fov / 2.0);

  printf("Disk detected at (x=%.3f, y=%.3f)\n", *x_pos, *y_pos);
  return true;
}

/**
 * @brief Moves the robot's arm and performs a grip/release, using vision for precision.
 */
static void execute_grip_action(Peg peg, bool grip, WbDeviceTag kinect_range, WbDeviceTag kinect_camera) {
  const double h_per_step = 0.002;
  const double box_length = 0.05;
  const double platform_height = 0.01;
  const double security_offset = 0.01;
  const double arm_start_height = 0.2;
  
  double target_x = 0.0;
  double target_y = 0.2; 

  
  arm_set_sub_arm_rotation(ARM5, M_PI_2);
  arm_ik(target_x, target_y, arm_start_height);
  if (grip)
    gripper_release();
  passive_wait(1.0);

  
  if (grip) {
    printf("Scanning for disk...\n");
    if (!find_top_disk_position(kinect_range, kinect_camera, &target_x, &target_y)) {
      printf("ERROR: Disk detection failed. Aborting grip.\n");
      arm_set_orientation(ARM_FRONT);
      return;
    }
    target_y -= 0.01; 
  }

  
  int target_level = grip ? (DISK_LEVELS[peg] - 1) : DISK_LEVELS[peg];
  double target_z = security_offset + platform_height + (target_level + 1) * box_length;
  if (!grip)
    target_z += security_offset;

  
  for (double h = arm_start_height; h > target_z; h -= h_per_step) {
    arm_ik(target_x, target_y, h);
    step();
  }
  passive_wait(0.2);

  
  if (grip)
    gripper_set_gap(0.04);
  else
    gripper_release();
  passive_wait(1.0);

  
  for (double h = target_z; h < arm_start_height; h += h_per_step) {
    arm_ik(target_x, target_y, h);
    step();
  }
  arm_set_orientation(ARM_FRONT);
}

static void navigate_to_peg(Peg peg) {
  double x, y, a;
  if (peg == -1) {
    x = 0.0; y = 0.0; a = 0.0;
  } else {
    x = GOTO_INFO[peg][X];
    y = GOTO_INFO[peg][Y];
    a = GOTO_INFO[peg][ANGLE];
  }
  base_goto_set_target(x, y, a);
  while (!base_goto_reached()) {
    base_goto_run();
    step();
  }
  base_reset();
}


static void step() {
  if (wb_robot_step(TIME_STEP) == -1) {
    wb_robot_cleanup();
    exit(EXIT_SUCCESS);
  }
}

static void passive_wait(double sec) {
  double start_time = wb_robot_get_time();
  do {
    step();
  } while (start_time + (sec * G_SPEED_MULTIPLIER) > wb_robot_get_time());
}
