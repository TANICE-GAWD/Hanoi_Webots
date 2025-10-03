/*
 * Copyright 1996-2024 Cyberbotics Ltd.
 *
 * This file was modified by [Your Name] in 2025.
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
 * Date: October 4, 2025
 * Description: An interactive and recursive Tower of Hanoi solver for the YouBot robot.
 * The robot dynamically calculates and executes the necessary moves to solve
 * the puzzle for a user-specified number of disks (1 to 3).
 * Author: [Your Name Here]
 * Original Author: Cyberbotics Ltd.
 */

/*
 * Description: An interactive, recursive Tower of Hanoi solver.
 *
 * - Press 'A' to launch the automatic Hanoi solver.
 * - Use the Arrow Keys to move the robot base.
 * - Use 'G' to Grip and 'R' to Release the gripper.
 * - Use 'U' to move the arm Up and 'D' to a Neutral position.
 * - Press 'I' to see the instructions again.
 */

#include <webots/keyboard.h>
#include <webots/robot.h>

#include <arm.h>
#include <base.h>
#include <gripper.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define TIME_STEP 32
// Define the number of disks for the Tower of Hanoi puzzle
#define NUM_DISKS 3
// Define the number of pegs (platforms)
#define NUM_PEGS 3

// Peg identifiers
#define SRC_PEG 0
#define TMP_PEG 1
#define DST_PEG 2

// Global state variables
static double GO_TO_INFO[NUM_PEGS][3];
static int PEG_LEVELS[NUM_PEGS] = {0}; // Stores the number of disks on each peg
static const double DISTANCE_ARM0_PLATFORM = 0.2;

// --- Helper Functions ---

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
  } while (start_time + sec > wb_robot_get_time());
}

// --- High-Level Robot Actions ---

static void high_level_go_to(double x, double y, double a) {
  base_goto_set_target(x, y, a);
  while (!base_goto_reached()) {
    base_goto_run();
    step();
  }
  base_reset();
  passive_wait(0.5); // Settle after moving
}

/*
 * A generalized function to pick up or place a box at a specific level on a platform.
 * The column is always 0 for the Hanoi puzzle (center of the platform).
 */
static void grip_or_place_box(int level, bool grip) {
  static double h_per_step = 0.002;
  static double box_length = 0.05;
  static double platform_height = 0.01;
  static double offset = 0.01; // Security margin

  // For Hanoi, all boxes are in the central column (x=0).
  double x = 0.0;
  double y = DISTANCE_ARM0_PLATFORM;
  double z_target = offset + platform_height + level * box_length;

  // 1. Prepare arm position high above the target
  arm_set_sub_arm_rotation(ARM5, M_PI_2);
  arm_ik(x, y, 0.20);
  if (grip)
    gripper_release();
  passive_wait(1.0);

  // 2. Move the arm down to the target height
  double h;
  for (h = 0.2; h > z_target; h -= h_per_step) {
    arm_ik(x, y, h);
    step();
  }

  passive_wait(0.2);

  // 3. Grip or Release
  if (grip)
    gripper_set_gap(0.04);
  else
    gripper_release();
  passive_wait(1.0);

  // 4. Move the arm back up
  for (h = z_target; h < 0.2; h += h_per_step) {
    arm_ik(x, y, h);
    step();
  }
  arm_set_orientation(ARM_FRONT);
  passive_wait(1.0);
}

// --- Tower of Hanoi Logic ---

// This function translates the abstract "move a disk" command into robot actions.
static void move_disk(int from_peg, int to_peg) {
  // Go to the source peg
  high_level_go_to(GO_TO_INFO[from_peg][0], GO_TO_INFO[from_peg][1], GO_TO_INFO[from_peg][2]);
  
  // Pick up the top disk from the source peg
  // The level is `PEG_LEVELS[from_peg] - 1` because levels are 0-indexed.
  grip_or_place_box(PEG_LEVELS[from_peg] - 1, true);

  // Go to the destination peg
  high_level_go_to(GO_TO_INFO[to_peg][0], GO_TO_INFO[to_peg][1], GO_TO_INFO[to_peg][2]);
  
  // Place the disk on top of the stack at the destination peg
  grip_or_place_box(PEG_LEVELS[to_peg], false);
  
  // Update the state of our pegs
  PEG_LEVELS[from_peg]--;
  PEG_LEVELS[to_peg]++;
}

// The recursive Tower of Hanoi solver
static void tower_of_hanoi(int n, int from_peg, int to_peg, int aux_peg) {
  // Base case: If there's only one disk, move it directly.
  if (n == 1) {
    printf("Moving disk 1 from peg %c to peg %c\n", 'A' + from_peg, 'A' + to_peg);
    move_disk(from_peg, to_peg);
    return;
  }
  
  // Recursive step:
  // 1. Move n-1 disks from source to auxiliary peg.
  tower_of_hanoi(n - 1, from_peg, aux_peg, to_peg);
  
  // 2. Move the nth disk from source to destination peg.
  printf("Moving disk %d from peg %c to peg %c\n", n, 'A' + from_peg, 'A' + to_peg);
  move_disk(from_peg, to_peg);
  
  // 3. Move the n-1 disks from auxiliary to destination peg.
  tower_of_hanoi(n - 1, aux_peg, to_peg, from_peg);
}

// --- Interactive Control ---

static void print_instructions() {
  printf("--- Interactive YouBot Controller ---\n");
  printf("Controls:\n");
  printf("  - 'A': Run the automatic Tower of Hanoi solver for %d disks.\n", NUM_DISKS);
  printf("  - Arrow Keys: Move the robot base.\n");
  printf("  - 'G': Grip (close gripper).\n");
  printf("  - 'R': Release (open gripper).\n");
  printf("  - 'U': Move arm to a high, safe position (ARM_HANOI_PREPARE).\n");
  printf("  - 'D': Move arm to a neutral front position (ARM_FRONT).\n");
  printf("  - 'I': Show this help message again.\n");
}

static void check_keyboard() {
  const double BASE_SPEED = 0.1;
  int key = wb_keyboard_get_key();
  
  switch (key) {
    case 'A':
      printf("Starting automatic Tower of Hanoi solver...\n");
      // Reset robot and world state for the puzzle
      arm_reset();
      base_reset();
      high_level_go_to(0,0,0);
      PEG_LEVELS[SRC_PEG] = NUM_DISKS;
      PEG_LEVELS[TMP_PEG] = 0;
      PEG_LEVELS[DST_PEG] = 0;
      
      // Solve the puzzle!
      tower_of_hanoi(NUM_DISKS, SRC_PEG, DST_PEG, TMP_PEG);
      
      // Return to origin after solving
      printf("Puzzle solved!\n");
      arm_reset();
      high_level_go_to(0.0, 0.0, 0.0);
      print_instructions(); // Show instructions again
      break;
    
    case WB_KEYBOARD_UP:
      base_move(BASE_SPEED);
      break;
    case WB_KEYBOARD_DOWN:
      base_move(-BASE_SPEED);
      break;
    case WB_KEYBOARD_LEFT:
      base_turn(-BASE_SPEED);
      break;
    case WB_KEYBOARD_RIGHT:
      base_turn(BASE_SPEED);
      break;
    case 'G':
      printf("Gripper: GRIP\n");
      gripper_set_gap(0.0); // Close completely
      break;
    case 'R':
      printf("Gripper: RELEASE\n");
      gripper_release();
      break;
    case 'U':
      printf("Arm: Moving UP\n");
      arm_set_height(ARM_HANOI_PREPARE);
      break;
    case 'D':
      printf("Arm: Moving to NEUTRAL\n");
      arm_set_orientation(ARM_FRONT);
      break;
    case 'I':
      print_instructions();
      break;
    case -1: // No key pressed
      base_reset(); // Stop moving if no key is pressed
      break;
  }
}

// --- Main Program ---

int main(int argc, char **argv) {
  wb_robot_init();

  // Initialize robot components
  base_init();
  base_goto_init(TIME_STEP);
  arm_init();
  gripper_init();
  wb_keyboard_enable(TIME_STEP);

  // Initialize cameras (optional, but good practice)
  WbDeviceTag kinect_color = wb_robot_get_device("kinect color");
  WbDeviceTag kinect_range = wb_robot_get_device("kinect range");
  wb_camera_enable(kinect_color, TIME_STEP);
  wb_range_finder_enable(kinect_range, TIME_STEP);
  
  // Calculate platform coordinates (same logic as original demo)
  double distance_arm0_robot_center = 0.189;
  double distance_origin_platform = 1.0;
  const double angles[3] = {0.0, 2.0 * M_PI / 3.0, -2.0 * M_PI / 3.0};
  
  double delta = distance_origin_platform - DISTANCE_ARM0_PLATFORM - distance_arm0_robot_center;
  
  for (int i = 0; i < NUM_PEGS; ++i) {
    GO_TO_INFO[i][0] = delta * sin(angles[i]);
    GO_TO_INFO[i][1] = delta * cos(angles[i]);
    GO_TO_INFO[i][2] = -angles[i];
  }

  passive_wait(1.0);
  print_instructions();
  
  // Main control loop
  while (wb_robot_step(TIME_STEP) != -1) {
    check_keyboard();
  }

  wb_robot_cleanup();
  return 0;
}
