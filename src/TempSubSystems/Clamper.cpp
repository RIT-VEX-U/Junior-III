#include "TempSubSystems/Clamper.h"
#include "robot-config.h"

ClamperSys::ClamperSys() { task = vex::task(thread_fn, this); }

void ClamperSys::clamp() { clamper_state = ClamperState::CLAMPED; };
void ClamperSys::unclamp() { clamper_state = ClamperState::UNCLAMPED; };

void ClamperSys::rush_out() { rush_arm_state = RushState::OUT; };
void ClamperSys::rush_in() { rush_arm_state = RushState::IN; };

void ClamperSys::toggle_clamp() {
    if (is_clamped()) {
        unclamp();
    } else {
        clamp();
    }
}

void ClamperSys::toggle_rush_arm() {
    if (is_rush_out()) {
        rush_in();
    } else {
        rush_out();
    }
}

// returns true if the piston is clamped down
bool ClamperSys::is_clamped() { return goal_grabber_sol.value(); };
bool ClamperSys::is_rush_out() { return goal_rush_sol.value(); };

AutoCommand *ClamperSys::ClampCmd(ClamperState state) {
    return new FunctionCommand([this, state]() {
        clamper_state = state;
        return true;
    });
}

int ClamperSys::thread_fn(void *ptr) {
    ClamperSys &self = *(ClamperSys *)ptr;
    while (true) {
        if (self.clamper_state == ClamperState::CLAMPED) {
            goal_grabber_sol.set(true);
        } else if (self.clamper_state == ClamperState::UNCLAMPED) {
            goal_grabber_sol.set(false);
        }
        if (self.rush_arm_state == RushState::OUT) {
            goal_rush_sol.set(true);
        } else if (self.rush_arm_state == RushState::IN) {
            goal_rush_sol.set(false);
        }
    }
    return 0;
}