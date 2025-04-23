#pragma once
#include "robot-config.h"

vex::brain Brain;
vex::controller con;

// ================ INPUTS ================
// Digital sensors
vex::inertial imu(vex::PORT12);
vex::distance clamper_sensor(vex::PORT18);

// ================ OUTPUTS ================
// Motors
vex::motor left_back_bottom(vex::PORT3, vex::gearSetting::ratio6_1, true);
vex::motor left_center_bottom(vex::PORT7, vex::gearSetting::ratio6_1, true);
vex::motor left_front_top(vex::PORT9, vex::gearSetting::ratio6_1, true);
vex::motor left_back_top(vex::PORT20, vex::gearSetting::ratio6_1, true);
vex::motor_group left_drive_motors({left_back_bottom, left_center_bottom, left_back_top, left_front_top});

vex::motor right_back_bottom(vex::PORT5, vex::gearSetting::ratio6_1, false);
vex::motor right_center_bottom(vex::PORT8, vex::gearSetting::ratio6_1, false);
vex::motor right_front_top(vex::PORT10, vex::gearSetting::ratio6_1, false);
vex::motor right_back_top(vex::PORT19, vex::gearSetting::ratio6_1, false);
vex::motor_group right_drive_motors({right_back_bottom, right_center_bottom, right_back_top, right_front_top});

vex::motor conveyor(vex::PORT4, vex::gearSetting::ratio6_1, true);
vex::motor intake_motor(vex::PORT6, vex::gearSetting::ratio6_1, false);

// pnematices
vex::digital_out goal_grabber_sol{Brain.ThreeWirePort.H};
vex::digital_out goal_rush_sol{Brain.ThreeWirePort.G};

// Button Definitions
const vex::controller::button &goal_grabber = con.ButtonY;
const vex::controller::button &goal_rush_arm = con.ButtonLeft;
const vex::controller::button &conveyor_button = con.ButtonR1;
const vex::controller::button &conveyor_button_rev = con.ButtonR2;

// ================ SUBSYSTEMS ================
PID::pid_config_t drive_pid_cfg{
  .p = 0.5,
  .i = 0,
  .d = 0.03,
  .deadband = 0.5,
  .on_target_time = 0.1,
};

PID drive_pid{drive_pid_cfg};

PID::pid_config_t turn_pid_cfg{
  .p = 0.05,
  .i = 0.0,
  .d = 0.001,
  .deadband = 1.0,
  .on_target_time = 0.1,
  .error_method = PID::ERROR_TYPE::LINEAR,

};

PID::pid_config_t correction_pid_cfg{
  .p = 0.05,
  .i = 0.0,
  .d = 0.001,
  .deadband = 1.0,
};

FeedForward::ff_config_t drive_ff_cfg{.kS = 0.01, .kV = 0.015, .kA = 0.002, .kG = 0};

PID turn_pid{turn_pid_cfg};
// ======== SUBSYSTEMS ========

robot_specs_t robot_cfg = {
  .robot_radius = 10,
  .odom_wheel_diam = 1.75,
  .odom_gear_ratio = 0.75,
  .dist_between_wheels = 12.4,

  .drive_correction_cutoff = 10,

  .drive_feedback = &drive_pid,
  .turn_feedback = &turn_pid,
  .correction_pid = correction_pid_cfg,
};

ClamperSys clamper_sys{};
IntakeSys intake_sys{};

Pose2d zero{0, 0, from_degrees(0)};
Pose2d blue_r_test{124.6, 101.6, from_degrees(180)};

OdometryTank odom(left_drive_motors, right_drive_motors, robot_cfg, &imu);

TankDrive drive_sys(left_drive_motors, right_drive_motors, robot_cfg, &odom);

// A global instance of vex::brain used for printing to the V5 brain screen
void print_multiline(const std::string &str, int y, int x);

// ================ UTILS ================

/**
 * Main robot initialization on startup. Runs before opcontrol and autonomous are started.
 */
void robot_init() {
    odom.set_position(blue_r_test);
    while (imu.isCalibrating()) {
        vexDelay(10);
    }
    screen::start_screen(Brain.Screen, {new screen::PIDPage(turn_pid, "turnpid")});
    printf("started!\n");
}