#pragma once
#include "competition/opcontrol.h"
#include "robot-config.h"
#include "vex.h"

void testing();
/**
 * Main entrypoint for the driver control period
 */

void opcontrol() {
    // autonomous();
    // testing();

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
        if (!conveyor_button.pressing() && !conveyor_button_rev.pressing()) {
            intake_sys.intake_stop();
            intake_sys.conveyor_stop();
        }
        OdometryBase *odombase = &odom;
        Pose2d pos = odombase->get_position();
        // printf(
        //   "ODO X: %.2f, Y: %.2f, R:%.2f, PID ERROR: %f\n", pos.x(), pos.y(), drive_pid.get_error(),
        //   conveyor.current()
        // );

        double left = (double)con.Axis3.position() / 100;
        double right = (double)con.Axis1.position() / 100;
        drive_sys.drive_arcade(left, right, 1, TankDrive::BrakeType::None);
        vexDelay(20);
    }

    // ================ PERIODIC ================
}

void testing() {

    con.ButtonX.pressed([]() { printf("running test"); });
}
