/*!
 * @brief
 *
 * @file
 *
 * @ingroup     kalman_filter
 */

/*------------------------------------------        Include Files           ------------------------------------------*/
#include "Eigen/Dense"

/*------------------------------------------        Using Statements      `------------------------------------------*/
using namespace std;
using Eigen::Matrix;
using Eigen::MatrixXf;

/*------------------------------------------        Class Declaration       ------------------------------------------*/
#ifndef CPP_ROBOTICS_KALMANFILTER_H
#define CPP_ROBOTICS_KALMANFILTER_H

class KalmanFilter2D {

public:
    KalmanFilter2D();
    ~KalmanFilter2D();
    void predict(void);
    void update(const Eigen::Ref<const MatrixXf>& measurement);
    MatrixXf get_state();
    MatrixXf get_covariance();


private:
    float _delta_t;             // Time step between filter steps (in seconds)
    float _sensor_noise;        // Value of sensor noise.
    float _sv_2;
    float _d;
    MatrixXf _state;            // Initial state (location and velocity) [[pos_x], [pos_y], [vel_x], [vel_y]]
    MatrixXf _ext_motion;       // Known external motion vector
    MatrixXf _covariance;       // Initial uncertainty covariance [cov_pos_x, cov_pos_y, cov_vel_x, cov_vel_y]
    MatrixXf _state_fn;         // Next State Function (State Transition Matrix)
    MatrixXf _measure_noise;    // Measurement uncertainty (Measurement Noise covariance)
    MatrixXf _measure_fn;       // Measurement function: (Maps states to measurements)
    MatrixXf _q_d;
    MatrixXf _process_noise;    // Process Noise covariance
};


#endif //CPP_ROBOTICS_KALMANFILTER_H
