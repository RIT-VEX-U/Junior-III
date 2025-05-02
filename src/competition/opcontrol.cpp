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
          clamper_sys.ClampCmd(ClamperSys::ClamperState::UNCLAMPED),
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
        printf(
          "{%.2f, %.2f}, ODO ROT: %f\n", odom.get_position().x(), odom.get_position().y(),
          odom.get_position().rotation().degrees()
        );
        intake_sys.color_to_remove(IntakeSys::RingColor::RED);
        printf("running b+ autonomous\n");
        CommandController cc{
          intake_sys.OuttakeCmd(),
          clamper_sys.RushCmd(ClamperSys::RushState::OUT),
          drive_sys.PurePursuitCmd(
            PurePursuit::Path({{112.37, 41.92}, {98.15, 39.96}, {92.79, 37.97}, {85.06, 32.35}}, 7), vex::forward
          ),
          clamper_sys.RushCmd(ClamperSys::RushState::IN),
          drive_sys.DriveToPointCmd({103.21, 42.37}, vex::reverse),
          drive_sys.TurnDegreesCmd(90),
          drive_sys.TurnToHeadingCmd(146),
          intake_sys.IntakeCmd(),
          clamper_sys.RushCmd(ClamperSys::RushState::OUT),
          drive_sys.DriveForwardCmd(26),
          clamper_sys.RushCmd(ClamperSys::RushState::IN),
          drive_sys.DriveForwardCmd(24, vex::reverse),
          drive_sys.TurnToPointCmd({84.5, 42.5}, vex::reverse),
          clamper_sys.AutoClampCmd(true),
          drive_sys.DriveForwardCmd(15, vex::reverse, 0.2),
          clamper_sys.ClampCmd(ClamperSys::ClamperState::CLAMPED),
          drive_sys.TurnDegreesCmd(15)->withTimeout(1),
          drive_sys.TurnDegreesCmd(-15)->withTimeout(1),
          intake_sys.ColorSortCmd(true),
          drive_sys.TurnToPointCmd({97.5, 16.5}, vex::forward),
          intake_sys.ConveyorInCmd(),
          intake_sys.IntakeCmd(),
          drive_sys.DriveForwardCmd(34, vex::forward, 0.4)->withCancelCondition(drive_sys.DriveStalledCondition(1)),
          drive_sys.TurnToPointCmd({120, 24}),
          drive_sys.DriveToPointCmd({120, 24}, vex::forward, 0.4),
          new DelayCommand(1000),
          drive_sys.TurnToHeadingCmd(303.5),
          intake_sys.OuttakeCmd(),
          drive_sys.DriveForwardCmd(24, vex::forward, 0.4)
            ->withTimeout(1)
            ->withCancelCondition(drive_sys.DriveStalledCondition(0.5)),
          intake_sys.IntakeCmd(),
          intake_sys.FixConveyorStallingCmd(true),
          new DelayCommand(500),
          drive_sys.DriveForwardCmd(10, vex::reverse, 0.4),
          intake_sys.OuttakeCmd(),
          drive_sys.DriveForwardCmd(14, vex::forward, 0.4)
            ->withTimeout(1)
            ->withCancelCondition(drive_sys.DriveStalledCondition(0.5)),
          intake_sys.IntakeCmd(),
          new DelayCommand(500),
          drive_sys.DriveForwardCmd(10, vex::reverse, 0.4),
          intake_sys.OuttakeCmd(),
          drive_sys.DriveForwardCmd(14, vex::forward, 0.4)
            ->withTimeout(1)
            ->withCancelCondition(drive_sys.DriveStalledCondition(0.5)),
          intake_sys.IntakeCmd(),
          new DelayCommand(500),
          drive_sys.DriveForwardCmd(10, vex::reverse, 0.4),
          intake_sys.OuttakeCmd(),
          drive_sys.DriveForwardCmd(14, vex::forward, 0.4)
            ->withTimeout(1)
            ->withCancelCondition(drive_sys.DriveStalledCondition(0.5)),
          intake_sys.IntakeCmd(),
          new DelayCommand(500),
          drive_sys.DriveForwardCmd(24, vex::reverse, 0.4),
          drive_sys.TurnToHeadingCmd(48)->withTimeout(1),
          clamper_sys.ClampCmd(ClamperSys::ClamperState::UNCLAMPED),
          drive_sys.DriveForwardCmd(5),
          clamper_sys.ClampCmd(ClamperSys::ClamperState::CLAMPED),
          drive_sys.DriveForwardCmd(29, vex::reverse)
            ->withTimeout(1)
            ->withCancelCondition(drive_sys.DriveStalledCondition(0.5)),
          drive_sys.DriveForwardCmd(24),
          //   drive_sys.TurnToHeadingCmd(327),
          //   clamper_sys.AutoClampCmd(true),
          //   drive_sys.DriveForwardCmd(33, vex::reverse, 0.2),
          //   clamper_sys.ClampCmd(ClamperSys::ClamperState::CLAMPED),
        };
        cc.run();
        intake_sys.conveyor_stop();
        intake_sys.intake_stop();
        intake_sys.fixConveyorStalling(false);
        intake_sys.stop_color_sort();
        clamper_sys.auto_clamp_off();
        enableDrive = true;
    });
}
