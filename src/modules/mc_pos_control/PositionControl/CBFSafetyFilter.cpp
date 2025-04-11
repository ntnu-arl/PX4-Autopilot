/**
 * @file CBFSafetyFilter.cpp
 */
#include <CBFSafetyFilter.hpp>

void CBFSafetyFilter::updateObstacles() {
    if (_obstacles_sub.update(&_obs_msg))
    {
        _ts_obs = hrt_absolute_time();

        _obstacles.clear();
        for (uint8_t i=0; i<_obs_msg.num_points; ++i)
        {
            _obstacles.push_back(Vector3f(_obs_msg.x[i], _obs_msg.y[i], _obs_msg.z[i]));
        }
    }
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

    // timeout obstacles
    if (tic - _ts_obs > _obstacle_timeout)
    {
        PX4_WARN("CBF hit obstacle timeout, clearing _obstacles");
        _obstacles.clear();
    }

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
    Dcmf R_BV = R_BW * R_WV;

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


    // ========================
    // ========================
    // ========================
    // implement the FoV constraints. This is done in a sub-optima fashoin by projecting the acceleration after the collision filtering onto the feasible subspace.
    // horizontal FoV CBF
    Vector3f e1(sinf(_fov_h), cosf(_fov_h), 0.f);
    Vector3f e2(sinf(_fov_h), -cosf(_fov_h), 0.f);
    e1 = R_BV * e1;
    e2 = R_BV * e2;
    float h1 = (e1).dot(_body_velocity);
    float h2 = (e2).dot(_body_velocity);
    float Lf_h1 = 0.f;
    float Lf_h2 = 0.f;
    Vector3f Lg_h1 = e1;
    Vector3f Lg_h2 = e2;
    float violation1 = Lf_h1 + Lg_h1.dot(_unfiltered_ouput) + _fov_alpha * h1;
    float violation2 = Lf_h2 + Lg_h2.dot(_unfiltered_ouput) + _fov_alpha * h2;


    // Project _unfiltered_output to make both violations non-negative
    // TODO: This simplified version assumes the FoV is 90 degrees or larger
    if (violation1 < 0.f || violation2 < 0.f) {
        if (violation1 < 0.f && violation2 >= 0.f) {
            // Solve for the first constraint
            matrix::Matrix<float, 1, 3> A1;
            A1.setRow(0, Lg_h1);
            matrix::Vector<float, 1> b1;
            b1(0) = -violation1;
            // Analytical solution for single constraint: delta = -(Ax-b)A^T/(AA^T)
            Vector3f A1_T = Lg_h1;
            float A1_A1T = Lg_h1.dot(Lg_h1);
            Vector3f delta1 = (A1_A1T > 1e-6f) ? (-violation1 * A1_T / A1_A1T) : Vector3f();

            _unfiltered_ouput += delta1;

        } else if (violation2 < 0.f && violation1 >= 0.f) {
            // Solve for the second constraint
            matrix::Matrix<float, 1, 3> A2;
            A2.setRow(0, Lg_h2);
            matrix::Vector<float, 1> b2;
            b2(0) = -violation2;

            // Analytical solution for single constraint: delta = -(Ax-b)A^T/(AA^T)
            Vector3f A2_T = Lg_h2;
            float A2_A2T = Lg_h2.dot(Lg_h2);
            Vector3f delta2 = (A2_A2T > 1e-6f) ? (-violation2 * A2_T / A2_A2T) : Vector3f();

            _unfiltered_ouput += delta2;

        } else {
            // Solve for both constraints

            matrix::Matrix<float, 2, 3> A;
            A.setRow(0, Lg_h1);
            A.setRow(1, Lg_h2);
            matrix::Vector<float, 2> b;
            b(0) = -violation1;
            b(1) = -violation2;
            // Direct analytical solution for two constraints using A^T(AA^T)^(-1)b
            matrix::Matrix<float, 2, 2> AAT = A * A.transpose();
            float det = AAT(0,0)*AAT(1,1) - AAT(0,1)*AAT(1,0);

            matrix::Matrix<float, 2, 2> AAT_inv;
            AAT_inv(0,0) = AAT(1,1)/det;
            AAT_inv(0,1) = -AAT(0,1)/det;
            AAT_inv(1,0) = -AAT(1,0)/det;
            AAT_inv(1,1) = AAT(0,0)/det;
            Vector3f delta = A.transpose() * (AAT_inv * b);

            _unfiltered_ouput += delta;
        }
    }
    // ========================
    // ========================
    // ========================

    // clamp and low pass acceleration output
    clampAccSetpoint(_unfiltered_ouput);

    _filtered_ouput = (1.f - _lp_gain_out) * _filtered_ouput + _lp_gain_out * _unfiltered_ouput;

    if (!_filtered_ouput.isAllFinite())
    {
        _filtered_ouput.setZero();
    }

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
