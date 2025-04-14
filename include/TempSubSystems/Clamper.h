#pragma once
#include "core/utils/command_structure/auto_command.h"
#include "vex.h"

class ClamperSys {
  public:
    ClamperSys();
    enum ClamperState {
        CLAMPED,
        UNCLAMPED,
    };
    enum RushState {
        OUT,
        IN,
    };

    void toggle_clamp();

    void toggle_rush_arm();

    void clamp();
    void unclamp();

    void rush_out();
    void rush_in();

    bool is_clamped();
    bool is_rush_out();
    AutoCommand *ClampCmd(ClamperState state);
    AutoCommand *RushCmd(ClamperState state);

  private:
    static int thread_fn(void *ptr);

    vex::task task;
    ClamperState clamper_state = ClamperState::UNCLAMPED;
    RushState rush_arm_state = RushState::IN;
};