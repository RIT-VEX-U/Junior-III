#include "TempSubSystems/Clamper.h"
#include "robot-config.h"

ClamperSys::ClamperSys() { task = vex::task(thread_fn, this); }

void ClamperSys::clamp() { clamper_state = ClamperState::CLAMPED; };
void ClamperSys::unclamp() { clamper_state = ClamperState::UNCLAMPED; };

void ClamperSys::rush_out() { rush_arm_state = RushState::OUT; };
void ClamperSys::rush_in() { rush_arm_state = RushState::IN; };

void ClamperSys::toggle_clamp() {
    doAutoClamping = false;
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

void ClamperSys::auto_clamp() {
    if (clamper_sensor.objectDistance(vex::distanceUnits::mm) <= 50) {
        clamper_state = ClamperState::CLAMPED;
    } else {
        clamper_state = ClamperState::UNCLAMPED;
    }
}

void ClamperSys::auto_clamp_on() { doAutoClamping = true; }

void ClamperSys::auto_clamp_off() { doAutoClamping = false; }

bool ClamperSys::is_auto_clamping() { return doAutoClamping; }

// returns true if the piston is clamped down
bool ClamperSys::is_clamped() { return goal_grabber_sol.value(); };
bool ClamperSys::is_rush_out() { return goal_rush_sol.value(); };

AutoCommand *ClamperSys::ClampCmd(ClamperState state) {
    return new FunctionCommand([this, state]() {
        doAutoClamping = false;
        clamper_state = state;
        return true;
    });
}

AutoCommand *ClamperSys::RushCmd(RushState state) {
    return new FunctionCommand([this, state]() {
        doAutoClamping = false;
        rush_arm_state = state;
        return true;
    });
}

AutoCommand *ClamperSys::AutoClampCmd(bool do_auto_clamping) {
    return new FunctionCommand([this, do_auto_clamping]() {
        doAutoClamping = do_auto_clamping;
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
        if (self.doAutoClamping) {
            self.auto_clamp();
            printf("Distance: %f\n", clamper_sensor.objectDistance(vex::distanceUnits::mm));
        }
        vexDelay(20);
    }
    return 0;
}