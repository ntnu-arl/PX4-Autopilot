#pragma once

#include <uORB/topics/cbf_debug.h>

#include <matrix/matrix/math.hpp>
#include <mathlib/math/Limits.hpp>

#include <containers/Array.hpp>

#define CBF_MAX_OBSTACLES 50


using namespace matrix;


class CBFSafetyFilter
{
public:

    CBFSafetyFilter();

    void setPosition(const Vector3f& position) { _position = position; }
    void setAttitude(const Quatf& attitude) { _attitude = attitude; }
    void setLinearVelocity(const Vector3f& velocity) {
        Dcmf R_IB(_attitude);
        Dcmf R_BI = R_IB.transpose();
        _local_velocity = R_BI * velocity;
        _velocity = velocity;
    }
    void update(Vector3f& acceleration_setpoint, uint64_t timestamp);

    void setEpsilon(float epsilon) { _epsilon = epsilon; }
    void setPole0(float pole0) { _pole0 = pole0; }
    void setKappa(float kappa) { _kappa = kappa; }
    void setGamma(float gamma) { _gamma = gamma; }
    void setAlpha(float alpha) { _alpha = alpha; }

    px4::Array<Vector3f, CBF_MAX_OBSTACLES>& obstacles() { return _obstacles; }
    void getDebug(cbf_debug_s& debug_msg)
    {
      debug_msg.h = _debug_msg.h;
      memcpy(debug_msg.virtual_obstacle, _debug_msg.virtual_obstacle, 3);
      memcpy(debug_msg.input, _debug_msg.input, 3);
      memcpy(debug_msg.output, _debug_msg.output, 3);
    };

private:
    Vector3f _position;
    Vector3f _local_velocity;
    Vector3f _velocity;
    Quatf _attitude;
    px4::Array<Vector3f, CBF_MAX_OBSTACLES> _obstacles;
    size_t _num_obstacles;
//     std::vector<Vector3f> _rel_pos;
    px4::Array<float, CBF_MAX_OBSTACLES> _nu1;

    cbf_debug_s _debug_msg;

    float _epsilon = 1.f;
    float _pole0 = -1.f;
    float _kappa = 10.f;
    float _gamma = 40.f;
    float _alpha = 1.f;
    float _fov_h = 40.f / 180.f * 3.1415f;
    float _alpha_fov = 7.5f;

    float saturate(float x);
    float saturateDerivative(float x);
    float kappaFunction(float h, float alpha);
};
