#include "competition/opcontrol.h"
#include "core.h"
#include "robot-config.h"
#include "vex.h"
#include <iostream>

#include "core/utils/math/estimator/unscented_kalman_filter.h"


double control_input(double t) {
    double u = 8 * sin(M_PI * sqrt(2.0) * t) + 6 * sin(M_PI * sqrt(3.0) * t) + 4 * sin(M_PI * sqrt(5.0) * t);

    if (u > 12)
        u = 12;
    if (u < -12)
        u = -12;
    return u;
}

vex::motor motorf(vex::PORT11);

/**
 * Main entrypoint for the driver control period
 */
void opcontrol() {
    std::cout << std::endl << std::endl << "guh" << std::endl << std::endl;

    // motorf.spin(vex::forward, 12, vex::volt);
    std::cout << std::endl << std::endl << "guh" << std::endl << std::endl;
    vexDelay(1000);


    // ---------------------------------------------------------------------
    // 5) Set up the SRUKF.
    //    State dimension: 4  ([p, v, kV, kA])
    //    Input dimension: 1  ([u])
    //    Measurement dimension: 3  ([p, v, I])
    // ---------------------------------------------------------------------

    // Define the process model (f) and measurement model (h) as lambda functions.
    auto f = [](const Eigen::Vector<double, 4> &xhat, const Eigen::Vector<double, 1> &u) -> Eigen::Vector<double, 4> {
        double v = xhat(1);
        double kV = xhat(2);
        double kA = xhat(3);
        double a = (u(0) - kV * v) / kA;
        Eigen::Vector<double, 4> xdot{v, a, 0, 0};
        return xdot;
    };

    auto h = [](const Eigen::Vector<double, 4> &xhat, const Eigen::Vector<double, 1> &u) -> Eigen::Vector<double, 2> {
        double p = xhat(0);
        double v = xhat(1);
        double kV = xhat(2);
        double kA = xhat(3);
        double I = (u(0) - kV * v) / kA;
        Eigen::Vector<double, 2> z{p, v};
        return z;
    };

    Eigen::Vector<double, 4> Q_vec;
    Eigen::Vector<double, 2> Q_vec2;
    Q_vec << 0.001, 0.01, 1e-14, 1e-14;
    Q_vec2 << 0.001, 0.01;
    Eigen::Vector<double, 2> H_vec;
    H_vec << 0.01, 0.023;

    SRUKF<4, 1, 2> srukf(f, h, RK4_with_input<4, 1>, Q_vec, H_vec);

    Eigen::Vector<double, 4> P0{0.1, 0.1, 1E-7, 1E-7};

    srukf.set_xhat(Eigen::Vector<double, 4>{0.0, 0.0, 1.6, 1.3});
    srukf.set_P(P0.asDiagonal());

    long initial = vexSystemHighResTimeGet();
    long next = initial;
    double prev_time = 0;

    // [0      1]
    // [0 -kV/kA]
    EMat<2, 2> A;
    A << 0.0,         1.0,  
           0, -6.164/0.241;

    // [   0]
    // [1/kA]
    EMat<2, 1> B;
    B <<      0, 
         1/0.25;

    // [1 0]   [p]
    // [0 1] * [v]
    EMat<2, 2> C;
    C << 1, 0, 
         0, 1;

    EMat<2, 1> D;
    D << 0, 
         0;

    // LinearSystem<2, 1, 2> plant(A, B, C, D);
    // all the 0.01 are just dt
    // LinearPlantInversionFeedforward<2, 1> piff(plant, 0.01);
    // the first vector is the tolerance of each state, lower = more aggressive
    // the second vector is the tolerance of each input (voltage), lower = less aggressive
    // LinearQuadraticRegulator<2, 1> lqr(plant, EVec<2>{1, 1}, EVec<1> {1}, 0.01);
    // we compensate for the measurement/control latency, 10ms worst case for a vex motor, and 10ms dt
    // lqr.latency_compensate(plant, 0.01, 0.01);

    // Q and H are defined somewhere up above, I found H experimentally by just setting a voltage and calculating stddev
    // Q you kinda just have to guess (it's the amount of noise added to each state per second)
    // KalmanFilter<2, 1, 2> kf(plant, Q_vec2, H_vec);

    // EVec<2> traj[5*100];

    // traj[0] = EVec<2> {0, 0};

    

    // for (int i = 1; i < 500; i++) {
    //     traj[i] = plant.compute_X(traj[i - 1], EVec<1>(control_input(i / 100.0)), 0.01);
    // }

    
    int count = 1;
    // piff.calculate(traj[0])(0);
    EMat<8, 500> save = EMat<8, 500>::Zero();

    while (count < 501) {
        while (vexSystemHighResTimeGet() - next < 9000) {
    
        }
    
        
        // double v = piff.calculate(traj[count])(0) + lqr.calculate(kf.xhat(), traj[count])(0);
        double time = (double) (vexSystemHighResTimeGet() - initial) / 1000000.0;
    
        double v = control_input(time);
        if (v > 12) {
            v = 12;
        } else if (v < -12) {
            v = -12;
        }
    
    
        left_drive_motors.spin(vex::forward, v, vex::volt);
        right_drive_motors.spin(vex::forward, v, vex::volt);
    
    
        // kf.correct(Eigen::Vector<double, 2>{motorf.position(vex::rev), motorf.velocity(vex::rpm) / 60}, EVec<1>{v});
        // kf.predict(EVec<1>{v}, 0.01);
    
        srukf.correct(EVec<1>{v}, EVec<2>{left_drive_motors.position(vex::rev) / 0.75 * 1.75 * M_PI, (left_drive_motors.velocity(vex::rpm) / 0.75 * 1.75 * M_PI / 60)});
        srukf.predict(EVec<1>{v}, 0.01);
        // prev_time = time;
    
        // if (time > 1) {
        //     A << 0, 1, 0, -srukf.xhat(2) / srukf.xhat(3);
        //     B << 0, 1 / srukf.xhat(3);
    
        //     plant = LinearSystem<2, 1, 2>(A, B, C, D);
        //     piff = LinearPlantInversionFeedforward<2, 1>(plant, 0.01);
        //     piff.set_r(traj[count]);
        //     lqr = LinearQuadraticRegulator<2, 1>(plant, EVec<2>{0.01, 0.01}, EVec<1>{40}, 0.01);
        //     // lqr.latency_compensate(plant, 0.01, 0.01);
        // }
    
        save.block<8, 1>(0, count - 1) = EVec<8>{time, v, srukf.xhat(0), srukf.xhat(1), left_drive_motors.position(vex::rev) / 0.75 * 1.75 * M_PI, left_drive_motors.velocity(vex::rpm) / 0.75 * 1.75 * M_PI / 60, srukf.xhat(2), srukf.xhat(3)};
        // std::cout << time << "," << v << "," << srukf.xhat(0) << "," << srukf.xhat(1) << "," << left_drive_motors.position(vex::rev) << ","  << (left_drive_motors.velocity(vex::rpm) / 60) << ","  << ","  << "," << srukf.xhat(2) << "," << srukf.xhat(3) << std::endl;
        // std::cout << left_drive_motors.position(vex::rev) / 0.75 * 1.75 * M_PI << std::endl;
        next = vexSystemHighResTimeGet();
        count++;
    }
    left_drive_motors.stop();
    right_drive_motors.stop();
    // std::cout << std::flush;

    for (int i = 0; i < 500; i++) {
        printf("%f,%f,%f,%f,%f,%f,,,%f,%f\n", save(0, i), save(1, i), save(2, i), save(3, i), save(4, i), save(5, i), save(6, i), save(7, i));
        fflush(stdout);
        // std::cout << save(0, i) << "," << save(1, i) << "," << save(2, i) << "," << save(3, i) << "," << save(4, i) << ","  << save(5, i) << ","  << "," << "," << save(6, i) << ","  << save(7, i) << std::endl;
        vexDelay(100);
    }





}