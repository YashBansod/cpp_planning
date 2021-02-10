/*!
 * @brief
 *
 * @file
 *
 * @ingroup     kalman_filter
 */

/*------------------------------------------        Include Files           ------------------------------------------*/
#include "Eigen/Dense"

/*------------------------------------------        Using Statements        ------------------------------------------*/
using namespace std;
using Eigen::Matrix;
using Eigen::MatrixXf;

/*------------------------------------------        Class Declaration       ------------------------------------------*/
#ifndef CPP_ROBOTICS_KALMANFILTER_H
#define CPP_ROBOTICS_KALMANFILTER_H

class KalmanFilter2D {

public:
    KalmanFilter2D();
    void predict();
    void update(const Eigen::Ref<const MatrixXf>& measurement);
    MatrixXf get_state();
    MatrixXf get_covariance();
    MatrixXf get_state_fn();


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


/*------------------------------------------        Class Definitions       ------------------------------------------*/
#ifndef CPP_ROBOTICS_KALMANFILTER_CPP
#define CPP_ROBOTICS_KALMANFILTER_CPP
KalmanFilter2D::KalmanFilter2D() {

    _delta_t = 1;
    _sensor_noise = 0.1;
    _sv_2 = 0.001 * 0.001;
    _d = 0.001;
    _state = Matrix<float, 4, 1>::Zero();
    _ext_motion = Matrix<float, 4, 1>::Zero();
    _covariance = Matrix<float, 4, 4>::Identity() * 1000;
    _state_fn = Matrix<float, 4, 4>::Identity();
    _state_fn.block(0, 2, 2, 2) = Matrix<float, 2, 2>::Identity() * _delta_t;
    _measure_noise = Matrix<float, 2, 2>::Identity() * _sensor_noise;
    _measure_fn = Matrix<float, 2, 4>::Identity();
    Matrix<float, 4, 1> g;
    g << 0.5 * _delta_t * _delta_t, 0.5 * _delta_t * _delta_t, _delta_t, _delta_t;
    _q_d = Matrix<float, 4, 4>::Identity() * (_d * _d);
    _process_noise = g * g.transpose() * _sv_2 + _q_d;

}

void KalmanFilter2D::predict() {
    _state = _state_fn * _state + _ext_motion;
    _covariance = _state_fn * _covariance * _state_fn.transpose() + _process_noise;
}

void KalmanFilter2D::update(const Eigen::Ref<const MatrixXf>& measurement) {
    Matrix<float, 2, 1> measure_error = measurement - (_measure_fn * _state);
    Matrix<float, 2, 2> uncertainity = (_measure_fn * _covariance * _measure_fn.transpose()) + _measure_noise;
    Matrix<float, 4, 2> kalman_gain = (_covariance * _measure_fn.transpose()) * uncertainity.inverse();
    _state = _state + (kalman_gain * measure_error);
    _covariance = (Matrix<float, 4, 4>::Identity() - kalman_gain * _measure_fn) * _covariance;
}

MatrixXf KalmanFilter2D::get_state(){
    return _state;
}

MatrixXf KalmanFilter2D::get_covariance(){
    return _covariance;
}

MatrixXf KalmanFilter2D::get_state_fn(){
    return _state_fn;
}

#endif //CPP_ROBOTICS_KALMANFILTER_CPP

