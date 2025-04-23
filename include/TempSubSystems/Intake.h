#pragma once
#include "core/utils/command_structure/auto_command.h"
#include "vex.h"
class IntakeSys {
  public:
    IntakeSys();

    enum IntakeState {
        STOP,
        IN,
        OUT,

    };
    IntakeState get_intake_state() { return intake_state; }
    IntakeState get_conveyor_state() { return conveyor_state; }

    void colorSort();
    void intake(double volts = 12);

    void outtake(double volts = 12);

    void intake_stop();

    void conveyor_in(double volts = 12.0);
    void conveyor_stop();
    void conveyor_out(double volts = 12.0);

    static int thread_fn(void *ptr);

    AutoCommand *IntakeCmd(double amt = 12.0);
    AutoCommand *OuttakeCmd(double amt = 12.0);
    AutoCommand *IntakeStopCmd();

    AutoCommand *ConveyorInCmd(double amt = 12.0);
    AutoCommand *ConveyorOutCmd(double amt = 12.0);
    AutoCommand *ConveyorStopCmd();

  private:
    vex::task task;
    IntakeState intake_state = IntakeState::STOP;
    IntakeState conveyor_state = IntakeState::STOP;
    double intakeVolts = 12;
    double conveyorVolts = 12;
    bool conveyorStarted = false;
    double sortConveyorVolts = 12;
};