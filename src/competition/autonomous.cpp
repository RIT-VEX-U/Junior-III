#include "competition/autonomous.h"
/**
 * Main entrypoint for the autonomous period
 */

AutoCommand *PrintPos() {
    return new FunctionCommand([]() {
        auto pos = odom.get_position();
        printf("Pos: (%.2f, %.2f) %.2f\n", pos.x(), pos.y(), pos.rotation().degrees());
        return true;
    });
}

AutoCommand *RecalGPSOr(Pose2d orelse) {
    return new FunctionCommand([bad_gps_count = 0, orelse]() mutable {
        int thresh = 50;

        if ((!gps_sensor.installed() || gps_sensor.quality() != 100) && bad_gps_count < thresh) {
            bad_gps_count++;

            return false;
        }

        if (bad_gps_count >= thresh) {
            printf("Orelse: %.2f, %.2f    %.2f", orelse.x(), orelse.y(), orelse.rotation().degrees());
            odom.set_position(orelse);
            return true;
        }
        double x = gps_sensor.xPosition(vex::distanceUnits::in) + 71.25;
        double y = gps_sensor.yPosition(vex::distanceUnits::in) + 71.25;
        double heading = deg2rad(wrap_degrees_180(gps_sensor.heading(vex::rotationUnits::deg) + 90));
        printf("Setting to %.2f, %.2f  %.2f\n", x, y, 180 * heading / PI);
        odom.set_position(Pose2d(x, y, from_radians(heading)));
        return true;
    });
}

// Autonomous Paths
void bluebot_redside_pos();
void bluebot_blueside_pos();
void bluebot_redside_neg();
void bluebot_blueside_neg();
void bluebot_blueside_pos_new();

// Main Autonomous Function
void autonomous() { bluebot_blueside_pos_new(); };

// Autonomous Path Implementations
void bluebot_redside_pos() {
    vex::timer tmr;
    tmr.reset();
    // clang-format off
    vexDelay(2000);
    intake_sys.color_to_remove(IntakeSys::RingColor::BLUE);
     intake_sys.start_color_sort();
     intake_sys.fixConveyorStalling(true);
   printf("running r+ autonomous\n");
    CommandController cc{
      // put intake down
      clamper_sys.RushCmd(ClamperSys::RushState::OUT),
      intake_sys.OuttakeCmd(),
      // goal rush 1,
      new Async((new FunctionCommand([&]() {
                    auto pos = odom.get_position();
                    printf("%.2f, Pos: (%.2f, %.2f) %.2f\n", (double)tmr.value(),pos.x(), pos.y(), pos.rotation().degrees());
                    return false;
                })
      )->withTimeout(30000)),
      // drive_sys.DriveForwardCmd(39)->withTimeout(2.5), 
      drive_sys.DriveTankCmd(1,1)->withTimeout(.64),
      new DelayCommand(100),
      clamper_sys.RushCmd(ClamperSys::RushState::IN),
      intake_sys.IntakeStopCmd(),
      drive_sys.DriveForwardCmd(22, vex::reverse)->withTimeout(1.5), 
      drive_sys.TurnToHeadingCmd(54),
      clamper_sys.RushCmd(ClamperSys::RushState::OUT),
      drive_sys.DriveForwardCmd(34, vex::fwd)->withTimeout(1.5), 
      clamper_sys.RushCmd(ClamperSys::RushState::IN),
      drive_sys.DriveForwardCmd(16, vex::reverse)->withTimeout(1.5), 
      drive_sys.TurnToHeadingCmd(100)->withTimeout(1.5),
      RecalGPSOr({52.00, 54.50,  from_degrees(97.82)}),
      RecalGPSOr({52.00, 54.50,  from_degrees(97.82)}),
      RecalGPSOr({52.00, 54.50,  from_degrees(97.82)}),   
      drive_sys.TurnToHeadingCmd(95)->withTimeout(1.5),
      drive_sys.DriveForwardCmd(17, vex::reverse, 0.2),
      clamper_sys.ClampCmd(ClamperSys::CLAMPED),
      new DelayCommand(300), // wait for it to finish grabbing
      intake_sys.IntakeCmd(),
      intake_sys.ConveyorInCmd(),
      drive_sys.TurnToPointCmd({17.58, 27.78})->withTimeout(1.5),
      drive_sys.DriveForwardCmd(30, vex::fwd, 0.6)->withTimeout(3.0),
      drive_sys.TurnToPointCmd({0, 0})->withTimeout(1.5),
      clamper_sys.RushCmd(ClamperSys::RushState::OUT),
      drive_sys.DriveForwardCmd(21.5, vex::fwd, 0.2)->withTimeout(3.0),
      drive_sys.TurnDegreesCmd(90)->withTimeout(1.5),
      clamper_sys.RushCmd(ClamperSys::RushState::IN),
      drive_sys.TurnToHeadingCmd(45)->withTimeout(1.5),
      drive_sys.DriveForwardCmd(10, vex::fwd, 0.3)->withTimeout(1.5),
      clamper_sys.ClampCmd(ClamperSys::UNCLAMPED),
      drive_sys.DriveForwardCmd(25, vex::reverse, 0.3)->withTimeout(1.5),
      drive_sys.DriveForwardCmd(25, vex::fwd, 0.3)->withTimeout(1.5),
      
      new DelayCommand(10000000),
      //   drive_sys.PurePursuitCmd(
      //     PurePursuit::Path(
      //       {
      //         {25.97, 41.99},
      //         {36.47, 40.59},
      //         {49.05, 37.84},
      //         {59.86, 36.19},
      //       },
      //       7
      //     ),
      //     vex::forward, 1
      //   ),
      //   PrintPos(),
      //   // (new DelayCommand(10000000))->withTimeout(100000000),
      //   clamper_sys.RushCmd(ClamperSys::RushState::IN),
      //   drive_sys.DriveToPointCmd({49.7, 38.9}, vex::reverse),
      //   intake_sys.IntakeStopCmd(),
      //   // goal rush 2
      //   drive_sys.TurnToHeadingCmd(65),
      //   clamper_sys.RushCmd(ClamperSys::RushState::OUT),
      //   drive_sys.DriveForwardCmd(30),
      //   clamper_sys.RushCmd(ClamperSys::RushState::IN),
      //   drive_sys.DriveForwardCmd(20, vex::reverse),
      //   // get goal 1
      //   drive_sys.TurnToPointCmd({50, 40}, vex::reverse),
      //   drive_sys.DriveToPointCmd({50, 40}, vex::reverse),
      //   drive_sys.TurnToPointCmd({68, 28}, vex::reverse),
      //   clamper_sys.AutoClampCmd(true),
      //   drive_sys.DriveForwardCmd(7, vex::reverse, 0.2),
      //   drive_sys.TurnToPointCmd({66, 24}, vex::reverse),
      //   drive_sys.DriveForwardCmd(1, vex::forward, 0.2),
      //   drive_sys.DriveForwardCmd(7, vex::reverse, 0.2),
      //   clamper_sys.ClampCmd(ClamperSys::ClamperState::CLAMPED),
      //   // get ring 1
      //   intake_sys.ColorSortCmd(true),
      //   drive_sys.TurnToPointCmd({60, 18}, vex::forward),
      //   intake_sys.ConveyorInCmd(),
      //   intake_sys.IntakeCmd(),
      //   drive_sys.DriveToPointCmd({60, 18}, vex::forward),
      //   drive_sys.DriveTankCmd(.8, 0)->withTimeout(.6),
      //   drive_sys.TurnToPointCmd({24, 12}),
      //   drive_sys.DriveToPointCmd({40, 12}, vex::forward, 0.4),
      //   drive_sys.DriveTankCmd(.8, 0)->withTimeout(.3),
      //   drive_sys.TurnToHeadingCmd(135),
      //   // get ring 2
      //   drive_sys.TurnToPointCmd({24, 24}),
      //   drive_sys.DriveToPointCmd({24, 24}, vex::forward, 0.4),
      //   new DelayCommand(100000000000000),
      //   // corner nightmare nightmare nightmare
      //   drive_sys.TurnToHeadingCmd(213.5),
      //   intake_sys.OuttakeCmd(),
      //   drive_sys.DriveForwardCmd(24, vex::forward, 0.4)
      //     ->withTimeout(1)
      //     ->withCancelCondition(drive_sys.DriveStalledCondition(0.5)),
      //   intake_sys.IntakeCmd(),
      //   intake_sys.FixConveyorStallingCmd(true),
      //   new DelayCommand(500),
      //   drive_sys.DriveForwardCmd(10, vex::reverse, 0.4),
      //   intake_sys.OuttakeCmd(),
      //   drive_sys.DriveForwardCmd(14, vex::forward, 0.4)
      //     ->withTimeout(1)
      //     ->withCancelCondition(drive_sys.DriveStalledCondition(0.5)),
      //   intake_sys.IntakeCmd(),
      //   new DelayCommand(500),
      //   drive_sys.DriveForwardCmd(10, vex::reverse, 0.4),
      //   intake_sys.OuttakeCmd(),
      //   drive_sys.DriveForwardCmd(14, vex::forward, 0.4)
      //     ->withTimeout(1)
      //     ->withCancelCondition(drive_sys.DriveStalledCondition(0.5)),
      //   intake_sys.IntakeCmd(),
      //   new DelayCommand(500),
      //   drive_sys.DriveForwardCmd(10, vex::reverse, 0.4),
      //   intake_sys.OuttakeCmd(),
      //   drive_sys.DriveForwardCmd(14, vex::forward, 0.4)
      //     ->withTimeout(1)
      //     ->withCancelCondition(drive_sys.DriveStalledCondition(0.5)),
      //   intake_sys.IntakeCmd(),
      //   new DelayCommand(500),
      //   drive_sys.DriveForwardCmd(24, vex::reverse, 0.4),
      //   drive_sys.TurnToHeadingCmd(48)->withTimeout(1),
      //   clamper_sys.ClampCmd(ClamperSys::ClamperState::UNCLAMPED),
      //   drive_sys.DriveForwardCmd(5),
      //   clamper_sys.ClampCmd(ClamperSys::ClamperState::CLAMPED),
      //   drive_sys.DriveForwardCmd(29, vex::reverse)
      //     ->withTimeout(1)
      //     ->withCancelCondition(drive_sys.DriveStalledCondition(0.5)),
      //   // get goal 2
      //   // delete if we dont have lidar
      //   drive_sys.DriveForwardCmd(24),
      //   drive_sys.TurnToHeadingCmd(237),
      //   // clamper_sys.AutoClampCmd(true),
      //   drive_sys.DriveForwardCmd(33, vex::reverse, 0.2),
      //   clamper_sys.ClampCmd(ClamperSys::ClamperState::CLAMPED),
    };
    cc.run();
    intake_sys.fixConveyorStalling(false);
    intake_sys.stop_color_sort();
    clamper_sys.auto_clamp_off();

}

void bluebot_blueside_pos_new() {
  vex::timer tmr;
  tmr.reset();
  // clang-format off
  vexDelay(2000);
  intake_sys.color_to_remove(IntakeSys::RingColor::RED);
   intake_sys.start_color_sort();
   intake_sys.fixConveyorStalling(true);
 printf("running r+ autonomous\n");
  CommandController cc{
    // put intake down
    clamper_sys.RushCmd(ClamperSys::RushState::OUT),
    intake_sys.OuttakeCmd(),
    // goal rush 1,
    new Async((new FunctionCommand([&]() {
                  auto pos = odom.get_position();
                  printf("%.2f, Pos: (%.2f, %.2f) %.2f\n", (double)tmr.value(),pos.x(), pos.y(), pos.rotation().degrees());
                  vexDelay(200);
                  return false;
              })
    )->withTimeout(30000)),
    // drive_sys.DriveForwardCmd(39)->withTimeout(2.5), 
    intake_sys.OuttakeCmd(),
    clamper_sys.RushCmd(ClamperSys::RushState::OUT),
    new DelayCommand(100),
    drive_sys.DriveTankCmd(1,1)->withTimeout(.7),
    // new DelayCommand(100),
    clamper_sys.RushCmd(ClamperSys::RushState::IN),
    intake_sys.IntakeStopCmd(),
    drive_sys.DriveForwardCmd(20, vex::reverse)->withTimeout(1.5),
    intake_sys.IntakeCmd(),
    drive_sys.TurnToHeadingCmd(60),
    drive_sys.TurnToHeadingCmd(142),
    clamper_sys.RushCmd(ClamperSys::RushState::OUT),
    drive_sys.DriveForwardCmd(34, vex::fwd)->withTimeout(1.5), 
    clamper_sys.RushCmd(ClamperSys::RushState::IN),
    drive_sys.DriveForwardCmd(16, vex::reverse)->withTimeout(1.5), 
    drive_sys.TurnToHeadingCmd(100)->withTimeout(1.5),
    RecalGPSOr({92.00, 54.50,  from_degrees(82.18)}),
    RecalGPSOr({92.00, 54.50,  from_degrees(82.18)}),
    RecalGPSOr({92.00, 54.50,  from_degrees(82.18)}),   
    drive_sys.TurnToHeadingCmd(85)->withTimeout(1.5),
    drive_sys.DriveForwardCmd(17, vex::reverse, 0.2),
    clamper_sys.ClampCmd(ClamperSys::CLAMPED),
    new DelayCommand(300), // wait for it to finish grabbing
    intake_sys.IntakeCmd(),
    intake_sys.ConveyorInCmd(),
    drive_sys.TurnToPointCmd({134, 24})->withTimeout(1.5),
    drive_sys.DriveToPointCmd({134, 24}, vex::fwd, 0.6)->withTimeout(3.0),
    drive_sys.TurnToPointCmd({148, 0})->withTimeout(1.5),
    clamper_sys.RushCmd(ClamperSys::RushState::OUT),
    drive_sys.DriveForwardCmd(21.5, vex::fwd, 0.2)->withTimeout(3.0),
    drive_sys.TurnDegreesCmd(90)->withTimeout(1.5),
    clamper_sys.RushCmd(ClamperSys::RushState::IN),
    drive_sys.TurnToPointCmd({144, 0}, vex::reverse)->withTimeout(1.5),
    drive_sys.DriveForwardCmd(10, vex::fwd, 0.3)->withTimeout(1.5),
    clamper_sys.ClampCmd(ClamperSys::UNCLAMPED),
    drive_sys.DriveForwardCmd(25, vex::reverse, 1)->withTimeout(1.5),
    drive_sys.DriveForwardCmd(25, vex::fwd, 0.3)->withTimeout(1.5),
    
  };
  cc.run();
  intake_sys.fixConveyorStalling(false);
  intake_sys.stop_color_sort();
  clamper_sys.auto_clamp_off();
}

void bluebot_blueside_pos() {
  intake_sys.start_color_sort();
    intake_sys.color_to_remove(IntakeSys::RingColor::RED);
    printf("running b+ autonomous\n");
    printf(
      "{%.2f, %.2f}, ODO ROT: %f\n", odom.get_position().x(), odom.get_position().y(),
      odom.get_position().rotation().degrees()
    );
    CommandController cc{
      intake_sys.OuttakeCmd(),
      clamper_sys.RushCmd(ClamperSys::RushState::OUT),
      // rush goal 1
      drive_sys.PurePursuitCmd(
        PurePursuit::Path({{112.37, 41.92}, {98.15, 39.96}, {89.15, 39.96}, {88.06, 32.30}}, 7), vex::forward
      ),
      clamper_sys.RushCmd(ClamperSys::RushState::IN),
      drive_sys.PurePursuitCmd(PurePursuit::Path({{87.65, 38.63}, {88.82, 47.30}, {86.24, 51.70}}, 7), vex::reverse),
      // rush goal 2
      drive_sys.TurnDegreesCmd(90),
      drive_sys.TurnToHeadingCmd(144),
      intake_sys.IntakeCmd(),
      clamper_sys.RushCmd(ClamperSys::RushState::OUT),
      drive_sys.DriveForwardCmd(9),
      clamper_sys.RushCmd(ClamperSys::RushState::IN),
      drive_sys.DriveForwardCmd(26, vex::reverse),
      // get goal 1
      drive_sys.TurnToPointCmd({79.5, 33.5}, vex::reverse),
      // clamper_sys.AutoClampCmd(true),
      drive_sys.DriveForwardCmd(15, vex::reverse, 0.2),
      clamper_sys.ClampCmd(ClamperSys::ClamperState::CLAMPED),
      drive_sys.TurnDegreesCmd(15)->withTimeout(1),
      drive_sys.TurnDegreesCmd(-15)->withTimeout(1),
      // get ring 1
      intake_sys.ColorSortCmd(true),
      drive_sys.TurnToPointCmd({96, 24}, vex::forward),
      intake_sys.ConveyorInCmd(),
      intake_sys.IntakeCmd(),
      drive_sys.DriveForwardCmd(32, vex::forward, 0.4)->withCancelCondition(drive_sys.DriveStalledCondition(1)),
      // get ring 2
      drive_sys.TurnToPointCmd({120, 24}),
      drive_sys.DriveToPointCmd({120, 24}, vex::forward, 0.4),
      new DelayCommand(1000),
      // corner nightmare nightmare nightmare
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
      drive_sys.TurnToHeadingCmd(135)->withTimeout(1),
      clamper_sys.ClampCmd(ClamperSys::ClamperState::UNCLAMPED),
      drive_sys.DriveForwardCmd(5),
      clamper_sys.ClampCmd(ClamperSys::ClamperState::CLAMPED),
      drive_sys.DriveForwardCmd(29, vex::reverse)
        ->withTimeout(1)
        ->withCancelCondition(drive_sys.DriveStalledCondition(0.5)),
      clamper_sys.ClampCmd(ClamperSys::ClamperState::UNCLAMPED),
      // get goal 2
      // delete if we dont get lidar
      drive_sys.DriveForwardCmd(24),
      drive_sys.TurnToHeadingCmd(327),
      clamper_sys.AutoClampCmd(true),
      drive_sys.DriveForwardCmd(33, vex::reverse, 0.2),
      clamper_sys.ClampCmd(ClamperSys::ClamperState::CLAMPED),
    };
    cc.run();
    intake_sys.conveyor_stop();
    intake_sys.intake_stop();
    intake_sys.fixConveyorStalling(false);
    intake_sys.stop_color_sort();
    clamper_sys.auto_clamp_off();
}

void bluebot_blueside_neg() {
    printf("running b- autonomous");

    intake_sys.fixConveyorStalling(true);
    CommandController cc{
      // Odometry Logs
      new Async(new FunctionCommand([]() {
          while (true) {
              printf(
                "ODO X: %f ODO Y: %f, ODO ROT: %f, turnPID Error: %f\n", odom.get_position().x(),
                odom.get_position().y(), odom.get_position().rotation().degrees(), turn_pid.get_error()
              );
              vexDelay(100);
          }
          return true;
      })),

      // Goal Rush (Rush and Deposit)
      intake_sys.ConveyorOutCmd(), intake_sys.OuttakeCmd(), intake_sys.ConveyorStopCmd(),
      clamper_sys.RushCmd(ClamperSys::RushState::OUT),
      drive_sys.PurePursuitCmd(PurePursuit::Path({{111.33, 103.18}, {94.2, 107.38}, {85.06, 110.28}}, 7), vex::forward)
        ->withTimeout(1.5),
      clamper_sys.RushCmd(ClamperSys::RushState::IN), drive_sys.DriveForwardCmd(24, vex::reverse),
      drive_sys.TurnDegreesCmd(60),

      // Alliance-side Goal (Ring)
      drive_sys.DriveToPointCmd({118, 96}, vex::reverse, .25), intake_sys.IntakeCmd(),
      drive_sys.TurnToHeadingCmd(90, .75), drive_sys.DriveForwardCmd(20, vex::forward),

      // Alliance-side Goal (Grab Goal and Score)
      drive_sys.DriveForwardCmd(24, vex::reverse, 1, .5), clamper_sys.AutoClampCmd(true),
      drive_sys.DriveToPointCmd({118, 71}, vex::reverse, .25), clamper_sys.ClampCmd(ClamperSys::ClamperState::CLAMPED),
      intake_sys.ConveyorInCmd(), new DelayCommand(1850), intake_sys.ConveyorStopCmd(),

      // Alliance Stake
      drive_sys.TurnToHeadingCmd(0), drive_sys.DriveForwardCmd(8, vex::forward, .5),
      new DelayCommand(3000), // wall stake mech and intake stuff
      drive_sys.DriveForwardCmd(12, vex::reverse, .75),
      new DelayCommand(3000), // wall stake mech stuff
      intake_sys.OuttakeCmd(), intake_sys.ConveyorOutCmd(),

      // Alliance-side Goal (Deposit)
      drive_sys.DriveToPointCmd({130, 48}, vex::reverse), clamper_sys.ClampCmd(ClamperSys::ClamperState::UNCLAMPED),
      new DelayCommand(100), drive_sys.DriveForwardCmd(6, vex::forward),

      // Retrieve Rush Goal
      drive_sys.DriveToPointCmd({120, 96}, vex::forward, .75), drive_sys.TurnToPointCmd({96, 120}, vex::reverse),
      clamper_sys.AutoClampCmd(true), drive_sys.DriveForwardCmd(34, vex::reverse, .5),
      clamper_sys.ClampCmd(ClamperSys::ClamperState::CLAMPED),

      intake_sys.IntakeStopCmd(), intake_sys.ConveyorStopCmd()
      // Score Stack near Goal
      // Score Corner Rings (optional)
      // Touch Hang structure
    };
    intake_sys.fixConveyorStalling(false);

    cc.run();
}