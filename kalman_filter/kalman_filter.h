/*!
 * @brief
 *
 * @file
 *
 * @ingroup     kalman_filter
 */

/*------------------------------------------        Include Files           ------------------------------------------*/
#include "Eigen/Dense"

/*------------------------------------------        Class Declaration       ------------------------------------------*/
#ifndef KALMAN_FILTER_KALMAN_FILTER_H
#define KALMAN_FILTER_KALMAN_FILTER_H

namespace kf {
    class kalman_filter_2d {

    public:
        kalman_filter_2d();

        void predict();

        void update(const Eigen::Ref<const Eigen::MatrixXf> &measurement);

        Eigen::MatrixXf get_state();

        Eigen::MatrixXf get_covariance();

        Eigen::MatrixXf get_state_fn();


    private:
        float _delta_t;             // Time step between filter steps (in seconds)
        float _sensor_noise;        // Value of sensor noise.
        float _sv_2;
        float _d;
        Eigen::MatrixXf _state;            // Initial state (location and velocity) [[pos_x], [pos_y], [vel_x], [vel_y]]
        Eigen::MatrixXf _ext_motion;       // Known external motion vector
        Eigen::MatrixXf _covariance;       // Initial uncertainty covariance [cov_pos_x, cov_pos_y, cov_vel_x, cov_vel_y]
        Eigen::MatrixXf _state_fn;         // Next State Function (State Transition Matrix)
        Eigen::MatrixXf _measure_noise;    // Measurement uncertainty (Measurement Noise covariance)
        Eigen::MatrixXf _measure_fn;       // Measurement function: (Maps states to measurements)
        Eigen::MatrixXf _q_d;
        Eigen::MatrixXf _process_noise;    // Process Noise covariance
    };

    kalman_filter_2d::kalman_filter_2d() {

        _delta_t = 1;
        _sensor_noise = 0.1;
        _sv_2 = 0.001 * 0.001;
        _d = 0.001;
        _state = Eigen::Matrix<float, 4, 1>::Zero();
        _ext_motion = Eigen::Matrix<float, 4, 1>::Zero();
        _covariance = Eigen::Matrix<float, 4, 4>::Identity() * 1000;
        _state_fn = Eigen::Matrix<float, 4, 4>::Identity();
        _state_fn.block(0, 2, 2, 2) = Eigen::Matrix<float, 2, 2>::Identity() * _delta_t;
        _measure_noise = Eigen::Matrix<float, 2, 2>::Identity() * _sensor_noise;
        _measure_fn = Eigen::Matrix<float, 2, 4>::Identity();
        Eigen::Matrix<float, 4, 1> g;
        g << 0.5 * _delta_t * _delta_t, 0.5 * _delta_t * _delta_t, _delta_t, _delta_t;
        _q_d = Eigen::Matrix<float, 4, 4>::Identity() * (_d * _d);
        _process_noise = g * g.transpose() * _sv_2 + _q_d;

    }

    void kalman_filter_2d::predict() {
        _state = _state_fn * _state + _ext_motion;
        _covariance = _state_fn * _covariance * _state_fn.transpose() + _process_noise;
    }

    void kalman_filter_2d::update(const Eigen::Ref<const Eigen::MatrixXf> &measurement) {
        Eigen::Matrix<float, 2, 1> measure_error = measurement - (_measure_fn * _state);
        Eigen::Matrix<float, 2, 2> uncertainity =
                (_measure_fn * _covariance * _measure_fn.transpose()) + _measure_noise;
        Eigen::Matrix<float, 4, 2> kalman_gain = (_covariance * _measure_fn.transpose()) * uncertainity.inverse();
        _state = _state + (kalman_gain * measure_error);
        _covariance = (Eigen::Matrix<float, 4, 4>::Identity() - kalman_gain * _measure_fn) * _covariance;
    }

    Eigen::MatrixXf kalman_filter_2d::get_state() {
        return _state;
    }

    Eigen::MatrixXf kalman_filter_2d::get_covariance() {
        return _covariance;
    }

    Eigen::MatrixXf kalman_filter_2d::get_state_fn() {
        return _state_fn;
    }
}

#endif //KALMAN_FILTER_KALMAN_FILTER_H
