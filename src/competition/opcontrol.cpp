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
            printf("ODO X: %.2f, Y: %.2f, R:%.2f\n", pos.x(), pos.y(), pos.rotation().degrees());
            drive_sys.drive_arcade(left, right, 1, TankDrive::BrakeType::None);
        }
        vexDelay(20);
    }

    // ================ PERIODIC ================
}

void testing() {

    con.ButtonX.pressed([]() {
        printf("running test");
        enableDrive = false;
        CommandController cc{
          new Async(new FunctionCommand([]() {
              while (true) {
                  printf(
                    "ODO X: %f ODO Y: %f, ODO ROT: %f, TurnPID Error: %f\n", odom.get_position().x(),
                    odom.get_position().y(), odom.get_position().rotation().degrees(), turn_pid.get_error()
                  );
                  vexDelay(100);
              }
              return true;
          })),

          odom.SetPositionCmd({124.6, 101.6, from_degrees(180)}),
          drive_sys.TurnToHeadingCmd(172),
          clamper_sys.RushCmd(ClamperSys::RushState::OUT),
          drive_sys.DriveForwardCmd(40),
          clamper_sys.RushCmd(ClamperSys::RushState::IN),
          new DelayCommand(50),
          drive_sys.DriveForwardCmd(24, vex::reverse),
          drive_sys.TurnToHeadingCmd(231),
          intake_sys.IntakeCmd(),
          intake_sys.ConveyorInCmd(),
          clamper_sys.RushCmd(ClamperSys::RushState::OUT),
          drive_sys.DriveForwardCmd(38),
          clamper_sys.RushCmd(ClamperSys::RushState::IN),
          drive_sys.DriveForwardCmd(24, vex::reverse),
          drive_sys.TurnToPointCmd(97.5, 113, vex::reverse),
          clamper_sys.AutoClampCmd(true),
          drive_sys.DriveToPointCmd(97.5, 113, vex::reverse, 0.3),
          clamper_sys.ClampCmd(ClamperSys::ClamperState::CLAMPED),
          drive_sys.TurnToHeadingCmd(45),
          intake_sys.IntakeCmd(),
          intake_sys.ConveyorInCmd(),
          drive_sys.DriveForwardCmd(23),
          new DelayCommand(10000),
          intake_sys.IntakeStopCmd(),
          intake_sys.ConveyorStopCmd(),

        };
        cc.run();
        enableDrive = true;
    });
}
