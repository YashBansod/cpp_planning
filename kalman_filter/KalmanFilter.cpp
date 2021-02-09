/*!
 * @brief
 *
 * @file
 *
 * @ingroup     kalman_filter
 */
/*------------------------------------------        Include Files           ------------------------------------------*/
#include "KalmanFilter.h"

/*------------------------------------------        Class Definitions       ------------------------------------------*/
KalmanFilter2D::KalmanFilter2D() {

    _delta_t = 1;
    _sensor_noise = 0.1;
    _sv_2 = 0.001 * 0.001;
    _d = 0.001;
    _state = Matrix<float, 4, 1>::Zero();
    _ext_motion = Matrix<float, 4, 1>::Zero();
    _covariance = Matrix<float, 4, 4>::Identity() * 1000;
    _state_fn = Matrix<float, 4, 4>::Identity();
    _state_fn(0, 2) = _delta_t;
    _state_fn(1, 3) = _delta_t;
    _measure_noise = Matrix<float, 2, 2>::Identity() * _sensor_noise;
    _measure_fn = Matrix<float, 2, 4>::Identity();
    Matrix<float, 4, 1> g;
    g << 0.5 * _delta_t * _delta_t, 0.5 * _delta_t * _delta_t, _delta_t, _delta_t;
    _q_d = Matrix<float, 4, 4>::Identity() * (_d * _d);
    _process_noise = g * g.transpose() * _sv_2 + _q_d;

}

KalmanFilter2D::~KalmanFilter2D() {

}

void KalmanFilter2D::predict(void) {
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