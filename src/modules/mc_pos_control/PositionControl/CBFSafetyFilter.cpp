/**
 * @file CBFSafetyFilter.cpp
 */
#include <CBFSafetyFilter.hpp>

void CBFSafetyFilter::updateObstacles() {
    // obstacles_s obs;
    // if (_obstacles_sub.update(&obs))
    // {
    //     _obstacles.clear();
    //     for (size_t i=0; i<obs.num_points; ++i)
    //     {
    //         _obstacles.push_back(Vector3f(obs.x[i], obs.y[i], obs.z[i]));
    //     }
    // }
    _obstacles.clear();
}

void CBFSafetyFilter::updateAttitude() {
    if (_vehicle_attitude_sub.updated())
    {
        vehicle_attitude_s vehicle_attitude;
        if (_vehicle_attitude_sub.copy(&vehicle_attitude))
            _attitude = Quatf(vehicle_attitude.q);
    }
}

void CBFSafetyFilter::filter(Vector3f& acceleration_setpoint, const Vector3f& velocity) {
    if  (!_enabled) return;
    uint64_t tic = hrt_absolute_time();

    // pass through if no obstacles are recorded
    updateAttitude();
    updateObstacles();
    const size_t n = _obstacles.size();
    if (n == 0) return;

    // compute local state
    Dcmf R_WB(_attitude);
    Dcmf R_BW = R_WB.transpose();
    Eulerf euler_current(_attitude);
    Eulerf euler_WV(0.f, 0.f, euler_current.psi());
    Dcmf R_WV(euler_WV);

    _body_acceleration_setpoint = R_BW * acceleration_setpoint;
    _body_velocity = R_BW * velocity;

    // low pass acceleration setpoint
    _filtered_input = (1.f - _lp_gain_in) * _filtered_input + _lp_gain_in * _body_acceleration_setpoint;

    // composite collision CBF
    // nu1_i
    for(size_t i = 0; i < n; i++) {
        float nu_i0 = _obstacles[i].norm_squared() - (_epsilon * _epsilon);
        float Lf_nu_i0 = -2.f * _obstacles[i].dot(_body_velocity);
        _nu1[i] = Lf_nu_i0 - _pole0 * nu_i0;
    }

    // h(x)
    float exp_sum = 0.f;
    for(size_t i = 0; i < n; i++) {
        exp_sum += expf(-_kappa * saturate(_nu1[i] / _gamma));
    }
    float h = -(_gamma / _kappa) * logf(exp_sum);

    // L_{f}h(x)
    float Lf_h = 0.f;
    for(size_t i = 0; i < n; i++) {
        float Lf_nu_i1 = 2.f * (_body_velocity + _pole0 * _obstacles[i]).dot(_body_velocity);
        float lambda_i = expf(-_kappa * saturate(_nu1[i] / _gamma)) * saturateDerivative(_nu1[i] / _gamma);
        Lf_h += lambda_i * Lf_nu_i1;
    }
    Lf_h /= exp_sum;

    // L_{g}h(x)
    Vector3f Lg_h(0.f, 0.f, 0.f);
    for(size_t i = 0; i < n; i++) {
        Vector3f Lg_nu_i1 = -2.f * _obstacles[i];
        float lambda_i = expf(-_kappa * saturate(_nu1[i] / _gamma)) * saturateDerivative(_nu1[i] / _gamma);
        Lg_h += lambda_i * Lg_nu_i1;
    }
    Lg_h /= exp_sum;

    // L_{g}h(x) * u, u = k_n(x) = a
    float Lg_h_u = Lg_h.dot(_body_acceleration_setpoint);

    // analytical QP solution from: https://arxiv.org/abs/2206.03568
    float eta = 0.f;
    float Lg_h_mag2 = Lg_h.norm_squared();
    if (Lg_h_mag2 > 1e-5f) {
        eta = -(Lf_h + Lg_h_u + _alpha*h) / Lg_h_mag2;
    }
    Vector3f acceleration_correction = (eta > 0.f ? eta : 0.f) * Lg_h;
    _unfiltered_ouput = _body_acceleration_setpoint + acceleration_correction;

    // clamp and low pass acceleration ouptput
    clampAccSetpoint(_unfiltered_ouput);
    _filtered_ouput = (1.f - _lp_gain_out) * _filtered_ouput + _lp_gain_out * _unfiltered_ouput;

    acceleration_setpoint =  R_WB * _filtered_ouput;

    uint64_t toc = hrt_absolute_time();
    _debug_msg.h = h;
    // _debug_msg.virtual_obstacle = ; // TODO: marvin
    _debug_msg.input[0] = _filtered_input(0);
    _debug_msg.input[1] = _filtered_input(1);
    _debug_msg.input[2] = _filtered_input(2);
    _debug_msg.cbf_duration = toc - tic;
    _debug_msg.output[0] = _filtered_ouput(0);
    _debug_msg.output[1] = _filtered_ouput(1);
    _debug_msg.output[2] = _filtered_ouput(2);
}

void CBFSafetyFilter::clampAccSetpoint(Vector3f& acc) {
    acc(0) = math::constrain(acc(0), -_max_acc_xy, _max_acc_xy);
    acc(1) = math::constrain(acc(1), -_max_acc_xy, _max_acc_xy);
    acc(2) = math::constrain(acc(2), -_max_acc_z, _max_acc_z);
}

float CBFSafetyFilter::saturate(float x) {
    return tanh(x);
}

float CBFSafetyFilter::saturateDerivative(float x) {
    float th = tanh(x);
    return 1.f - (th * th);
}

float CBFSafetyFilter::kappaFunction(float h, float alpha) {
    float a = alpha;
    float b = 1.f/alpha;
    if (h>=0.f) {
        return alpha * h;
    }
    else {
        return a * b * ( h / (b + abs(h)) );
    }
}
