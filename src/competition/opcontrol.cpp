#pragma once
#include "competition/opcontrol.h"
#include "robot-config.h"
#include "vex.h"

void testing();
/**
 * Main entrypoint for the driver control period
 */
bool enableDrive = true;
void opcontrol() {
    // autonomous();
    testing();

    goal_grabber.pressed([]() { clamper_sys.toggle_clamp(); });

    conveyor_button.pressed([]() {
        intake_sys.intake();
        intake_sys.conveyor_in();
    });
    conveyor_button_rev.pressed([]() {
        intake_sys.outtake();
        intake_sys.conveyor_out();
    });

    conveyor_button.released([]() {
        intake_sys.intake_stop();
        intake_sys.conveyor_stop();
    });

    conveyor_button_rev.released([]() {
        intake_sys.intake_stop();
        intake_sys.conveyor_stop();
    });

    goal_rush_arm.pressed([]() { clamper_sys.toggle_rush_arm(); });

    // ================ INIT ================

    while (true) {
        if (enableDrive) {

            if (!conveyor_button.pressing() && !conveyor_button_rev.pressing()) {
                intake_sys.intake_stop();
                intake_sys.conveyor_stop();
            }
            OdometryBase *odombase = &odom;
            Pose2d pos = odombase->get_position();

            double left = (double)con.Axis3.position() / 100;
            double right = (double)con.Axis1.position() / 100;
            // printf("ODO X: %.2f, Y: %.2f, R:%.2f\n", pos.x(), pos.y(), pos.rotation().degrees());
            drive_sys.drive_arcade(left, right, 1, TankDrive::BrakeType::None);
        }
        vexDelay(20);
    }

    // ================ PERIODIC ================
}

void testing() {

    con.ButtonUp.pressed([]() {
        printf("resetting position");
        enableDrive = false;
        CommandController cc{
          new Async(new FunctionCommand([]() {
              while (true) {
                  //   printf(
                  //     "ODO X: %f ODO Y: %f, ODO ROT: %f, TurnPID Error: %f\n", odom.get_position().x(),
                  //     odom.get_position().y(), odom.get_position().rotation().degrees(), turn_pid.get_error()
                  //   );
                  vexDelay(100);
              }
              return true;
          })),
          drive_sys.DriveToPointCmd({19.4, 42.4}),
          drive_sys.TurnToHeadingCmd(0),
        };
        cc.run();
        enableDrive = true;
    });
    con.ButtonRight.pressed([]() {
        printf(
          "{%.2f, %.2f}, ODO ROT: %f\n", odom.get_position().x(), odom.get_position().y(),
          odom.get_position().rotation().degrees()
        );
    });

    con.ButtonX.pressed([]() {
        if (!enableDrive) {
            return;
        } else {
            enableDrive = false;
        }
        intake_sys.fixConveyorStalling(true);
        printf("running test");
        CommandController cc{
          new Async(new FunctionCommand([]() {
              while (true) {
                  //   printf(
                  //     "ODO X: %f ODO Y: %f, ODO ROT: %f, TurnPID Error: %f\n", odom.get_position().x(),
                  //     odom.get_position().y(), odom.get_position().rotation().degrees(), turn_pid.get_error()
                  //   );
                  vexDelay(100);
              }
              return true;
          })),
          intake_sys.OuttakeCmd(),
          clamper_sys.RushCmd(ClamperSys::RushState::OUT),
          drive_sys.PurePursuitCmd(
            PurePursuit::Path({{25.97, 41.99}, {36.47, 40.59}, {49.05, 37.84}, {59.86, 34.19}}, 7), vex::forward
          ),
          clamper_sys.RushCmd(ClamperSys::RushState::IN),
          drive_sys.DriveToPointCmd({49.7, 40.9}, vex::reverse),
          intake_sys.IntakeStopCmd(),
          drive_sys.TurnToHeadingCmd(60.5),
          clamper_sys.RushCmd(ClamperSys::RushState::OUT),
          drive_sys.DriveForwardCmd(24),
          clamper_sys.RushCmd(ClamperSys::RushState::IN),
          drive_sys.DriveForwardCmd(20, vex::reverse),
          drive_sys.TurnToPointCmd({60, 23.5}, vex::reverse),
          clamper_sys.AutoClampCmd(true),
          drive_sys.DriveForwardCmd(15, vex::reverse, 0.2),
          clamper_sys.ClampCmd(ClamperSys::ClamperState::CLAMPED),
          drive_sys.TurnDegreesCmd(15),
          drive_sys.TurnDegreesCmd(-15),
          intake_sys.ColorSortCmd(true),
          drive_sys.TurnToPointCmd({48, 19.5}, vex::forward),
          intake_sys.ConveyorInCmd(),
          intake_sys.IntakeCmd(),
          drive_sys.DriveForwardCmd(16, vex::forward, 0.4),
          drive_sys.PurePursuitCmd(PurePursuit::Path({{40, 24}, {24, 24}, {10, 30.5}}, 8), vex::forward, 0.3),
          drive_sys.DriveToPointCmd({24, 24}, vex::reverse, 0.3),
          drive_sys.TurnToHeadingCmd(213.5),
          drive_sys.DriveForwardCmd(24, vex::forward, 0.4)->withTimeout(1),
          new DelayCommand(200),
          drive_sys.DriveForwardCmd(10, vex::reverse, 0.4),
          drive_sys.DriveForwardCmd(12, vex::forward, 0.4)->withTimeout(1),
          new DelayCommand(200),
          drive_sys.DriveForwardCmd(10, vex::reverse, 0.4),
          drive_sys.DriveForwardCmd(12, vex::forward, 0.4)->withTimeout(1),
          new DelayCommand(200),
          drive_sys.DriveForwardCmd(10, vex::reverse, 0.4),
          drive_sys.DriveForwardCmd(12, vex::forward, 0.4)->withTimeout(1),
          new DelayCommand(200),
          drive_sys.DriveForwardCmd(24, vex::reverse, 0.4),

        };
        cc.run();
        intake_sys.fixConveyorStalling(false);
        enableDrive = true;
    });
}
