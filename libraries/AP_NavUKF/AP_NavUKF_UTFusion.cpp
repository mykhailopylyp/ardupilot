#include <AP_HAL/AP_HAL.h>

#include "AP_NavUKF.h"
#include "AP_NavUKF_core.h"

#include <AP_DAL/AP_DAL.h>

/*
  Shared unscented-transform helpers for covariance prediction and
  scalar measurement updates. Attitude sigma points are multiplicative
  (rotation-vector on the quaternion manifold). Residuals are mapped
  back into R^4 at the mean quaternion so P stays in the same space
  as the predict step.
*/

bool NavUKF_core::drawSigmaPoints(const ftype *mean)
{
    ukf_n = uint8_t(stateIndexLim + 1);
    const ftype ukf_alpha = constrain_ftype(frontend->_ukf_alpha, 1.0e-3f, 1.0f);
    const ftype ukf_beta  = constrain_ftype(frontend->_ukf_beta, 0.0f, 10.0f);
    const ftype ukf_kappa = constrain_ftype(frontend->_ukf_kappa, -24.0f, 24.0f);
    const ftype n_f = (ftype)ukf_n;
    const ftype lambda = ukf_alpha * ukf_alpha * (n_f + ukf_kappa) - n_f;
    const ftype n_lambda = n_f + lambda;
    if (n_lambda <= 0.0f) {
        return false;
    }
    const ftype gamma = sqrtF(n_lambda);
    ukf_Wm0 = lambda / n_lambda;
    ukf_Wc0 = ukf_Wm0 + (1.0f - ukf_alpha * ukf_alpha + ukf_beta);
    ukf_Wi  = 0.5f / n_lambda;
    ukf_n_sigma = uint8_t(2 * ukf_n + 1);

    auto &L = KHP;
    ftype (*chol_scratch)[24] = &sigma_prop[24];
    for (uint8_t i = 0; i < ukf_n; i++) {
        for (uint8_t j = 0; j < ukf_n; j++) {
            L[i][j] = P[i][j];
        }
    }
    bool chol_ok = false;
    for (uint8_t attempt = 0; attempt < 5 && !chol_ok; attempt++) {
        for (uint8_t i = 0; i < ukf_n; i++) {
            for (uint8_t j = 0; j < ukf_n; j++) {
                chol_scratch[i][j] = L[i][j];
            }
            if (attempt > 0) {
                chol_scratch[i][i] += 1.0e-6f * (ftype)(1u << (attempt - 1));
            }
        }
        chol_ok = choleskyLower(chol_scratch, ukf_n);
        if (!chol_ok) {
            for (uint8_t i = 0; i < ukf_n; i++) {
                for (uint8_t j = 0; j < ukf_n; j++) {
                    L[i][j] = P[i][j];
                }
            }
        }
    }
    if (!chol_ok) {
        return false;
    }
    for (uint8_t i = 0; i < ukf_n; i++) {
        for (uint8_t j = 0; j < ukf_n; j++) {
            L[i][j] = chol_scratch[i][j];
        }
    }

    for (uint8_t j = 0; j < ukf_n; j++) {
        sigma_prop[0][j] = mean[j];
    }

    const QuaternionF qnom(mean[0], mean[1], mean[2], mean[3]);
    for (uint8_t i = 0; i < ukf_n; i++) {
        ftype dq[4];
        for (uint8_t j = 0; j < 4; j++) {
            const ftype L_ji = (j >= i) ? L[j][i] : 0.0f;
            dq[j] = gamma * L_ji;
        }
        const ftype q_dot_dq = qnom[0]*dq[0] + qnom[1]*dq[1] + qnom[2]*dq[2] + qnom[3]*dq[3];
        dq[0] -= q_dot_dq * qnom[0];
        dq[1] -= q_dot_dq * qnom[1];
        dq[2] -= q_dot_dq * qnom[2];
        dq[3] -= q_dot_dq * qnom[3];
        Vector3F dtheta(
            2.0f * (qnom[0]*dq[1] - qnom[1]*dq[0] - qnom[2]*dq[3] + qnom[3]*dq[2]),
            2.0f * (qnom[0]*dq[2] + qnom[1]*dq[3] - qnom[2]*dq[0] - qnom[3]*dq[1]),
            2.0f * (qnom[0]*dq[3] - qnom[1]*dq[2] + qnom[2]*dq[1] - qnom[3]*dq[0])
        );
        QuaternionF dq_pos, dq_neg;
        dq_pos.from_axis_angle(dtheta);
        dq_neg.from_axis_angle(-dtheta);
        QuaternionF q_pos = qnom * dq_pos;
        QuaternionF q_neg = qnom * dq_neg;
        q_pos.normalize();
        q_neg.normalize();

        sigma_prop[i + 1][0] = q_pos[0];
        sigma_prop[i + 1][1] = q_pos[1];
        sigma_prop[i + 1][2] = q_pos[2];
        sigma_prop[i + 1][3] = q_pos[3];
        sigma_prop[i + 1 + ukf_n][0] = q_neg[0];
        sigma_prop[i + 1 + ukf_n][1] = q_neg[1];
        sigma_prop[i + 1 + ukf_n][2] = q_neg[2];
        sigma_prop[i + 1 + ukf_n][3] = q_neg[3];

        for (uint8_t j = 4; j < ukf_n; j++) {
            const ftype L_ji = (j >= i) ? L[j][i] : 0.0f;
            const ftype step = gamma * L_ji;
            sigma_prop[i + 1][j] = mean[j] + step;
            sigma_prop[i + 1 + ukf_n][j] = mean[j] - step;
        }
    }

    const ftype qref[4] = { mean[0], mean[1], mean[2], mean[3] };
    for (uint8_t s = 1; s < ukf_n_sigma; s++) {
        if ((sigma_prop[s][0] * qref[0] + sigma_prop[s][1] * qref[1] +
             sigma_prop[s][2] * qref[2] + sigma_prop[s][3] * qref[3]) < 0) {
            sigma_prop[s][0] = -sigma_prop[s][0];
            sigma_prop[s][1] = -sigma_prop[s][1];
            sigma_prop[s][2] = -sigma_prop[s][2];
            sigma_prop[s][3] = -sigma_prop[s][3];
        }
    }
    return true;
}

static void ukf_tnb_from_state(const ftype x[24], Matrix3F &Tnb)
{
    QuaternionF quat(x[0], x[1], x[2], x[3]);
    quat.normalize();
    quat.inverse().rotation_matrix(Tnb);
}

static Vector3F ukf_mag_pred(const ftype x[24])
{
    const ftype q0 = x[0];
    const ftype q1 = x[1];
    const ftype q2 = x[2];
    const ftype q3 = x[3];
    const ftype magN = x[16];
    const ftype magE = x[17];
    const ftype magD = x[18];
    const Matrix3F DCM {
        q0*q0 + q1*q1 - q2*q2 - q3*q3,
        2.0f*(q1*q2 + q0*q3),
        2.0f*(q1*q3 - q0*q2),
        2.0f*(q1*q2 - q0*q3),
        q0*q0 - q1*q1 + q2*q2 - q3*q3,
        2.0f*(q2*q3 + q0*q1),
        2.0f*(q1*q3 + q0*q2),
        2.0f*(q2*q3 - q0*q1),
        q0*q0 - q1*q1 - q2*q2 + q3*q3
    };
    return Vector3F(
               DCM[0][0]*magN + DCM[0][1]*magE + DCM[0][2]*magD + x[19],
               DCM[1][0]*magN + DCM[1][1]*magE + DCM[1][2]*magD + x[20],
               DCM[2][0]*magN + DCM[2][1]*magE + DCM[2][2]*magD + x[21]
           );
}

ftype NavUKF_core::predictObservation(const ftype x[24], UKFObs obs) const
{
    switch (obs) {
    case UKFObs::State:
        return x[ut_obs.state_index];

    case UKFObs::MagX:
        return ukf_mag_pred(x).x;
    case UKFObs::MagY:
        return ukf_mag_pred(x).y;
    case UKFObs::MagZ:
        return ukf_mag_pred(x).z;

    case UKFObs::Yaw321: {
        QuaternionF quat(x[0], x[1], x[2], x[3]);
        quat.normalize();
        ftype roll, pitch, yaw;
        quat.to_euler(roll, pitch, yaw);
        return yaw;
    }
    case UKFObs::Yaw312: {
        QuaternionF quat(x[0], x[1], x[2], x[3]);
        quat.normalize();
        return quat.to_vector312().z;
    }

    case UKFObs::Declination:
        return atan2F(x[17], x[16]);

    case UKFObs::TAS:
        return norm(x[4] - x[22], x[5] - x[23], x[6]);

    case UKFObs::Beta: {
        Matrix3F Tnb;
        ukf_tnb_from_state(x, Tnb);
        const Vector3F rel_wind_ned(x[4] - x[22], x[5] - x[23], x[6]);
        const Vector3F rel_body = Tnb * rel_wind_ned;
        if (fabsF(rel_body.x) < 1.0e-3f) {
            return 0.0f;
        }
        return constrain_ftype(rel_body.y / rel_body.x, -0.5f, 0.5f);
    }

#if UKF_FEATURE_DRAG_FUSION
    case UKFObs::DragX:
    case UKFObs::DragY: {
        Matrix3F Tnb;
        ukf_tnb_from_state(x, Tnb);
        const Vector3F rel_wind_ned(x[4] - x[22], x[5] - x[23], x[6]);
        const Vector3F rel_wind_body = Tnb * rel_wind_ned;
        const uint8_t axis = (obs == UKFObs::DragX) ? 0 : 1;
        const ftype dragForceSign = is_positive(rel_wind_body[axis]) ? -1.0f : 1.0f;
        const ftype bcoef = (axis == 0) ? ut_obs.drag_bcoef_x : ut_obs.drag_bcoef_y;
        const bool using_bcoef = (axis == 0) ? ut_obs.using_bcoef_x : ut_obs.using_bcoef_y;
        ftype predAccel = 0.0f;
        if (ut_obs.using_mcoef && using_bcoef) {
            predAccel = (0.5f / bcoef) * ut_obs.drag_rho * sq(rel_wind_body[axis]) * dragForceSign
                        - rel_wind_body[axis] * ut_obs.drag_mcoef * ut_obs.drag_density_ratio;
        } else if (ut_obs.using_mcoef) {
            predAccel = -rel_wind_body[axis] * ut_obs.drag_mcoef * ut_obs.drag_density_ratio;
        } else if (using_bcoef) {
            predAccel = (0.5f / bcoef) * ut_obs.drag_rho * sq(rel_wind_body[axis]) * dragForceSign;
        }
        return predAccel;
    }
#endif

#if UKF_FEATURE_OPTFLOW_FUSION
    case UKFObs::FlowX:
    case UKFObs::FlowY: {
        Matrix3F Tnb;
        ukf_tnb_from_state(x, Tnb);
        const Vector3F vel(x[4], x[5], x[6]);
        Vector3F relVelSensor = (Tnb * vel) + (ut_obs.body_rate % ut_obs.pos_offset_body);
        const ftype range = MAX(ut_obs.range, 0.1f);
        if (obs == UKFObs::FlowX) {
            return relVelSensor.y / range;
        }
        return -relVelSensor.x / range;
    }
#endif

#if UKF_FEATURE_BODY_ODOM
    case UKFObs::BodyVelX:
    case UKFObs::BodyVelY:
    case UKFObs::BodyVelZ: {
        Matrix3F Tnb;
        ukf_tnb_from_state(x, Tnb);
        const Vector3F vel(x[4], x[5], x[6]);
        Vector3F bodyVel = Tnb * vel;
        if (imuDataDelayed.delAngDT > 0.001f) {
            bodyVel += (imuDataDelayed.delAng * (1.0f / imuDataDelayed.delAngDT)) % ut_obs.pos_offset_body;
        }
        if (obs == UKFObs::BodyVelX) {
            return bodyVel.x;
        }
        if (obs == UKFObs::BodyVelY) {
            return bodyVel.y;
        }
        return bodyVel.z;
    }
#endif

#if UKF_FEATURE_BEACON_FUSION
    case UKFObs::RngBcn: {
        const Vector3F delta(x[7] - ut_obs.bcn_pos.x,
                             x[8] - ut_obs.bcn_pos.y,
                             x[9] - ut_obs.bcn_pos.z);
        return delta.length();
    }
#endif

    default:
        return 0.0f;
    }
}

static void ukf_attitude_state_residual(const ftype *mean, const ftype *x_s, uint8_t n, ftype dx[24])
{
    QuaternionF q_mean(mean[0], mean[1], mean[2], mean[3]);
    QuaternionF q_s(x_s[0], x_s[1], x_s[2], x_s[3]);
    if ((q_s[0]*q_mean[0] + q_s[1]*q_mean[1] + q_s[2]*q_mean[2] + q_s[3]*q_mean[3]) < 0) {
        q_s[0] = -q_s[0];
        q_s[1] = -q_s[1];
        q_s[2] = -q_s[2];
        q_s[3] = -q_s[3];
    }
    QuaternionF qerr = q_mean.inverse() * q_s;
    if (qerr[0] < 0) {
        qerr[0] = -qerr[0];
        qerr[1] = -qerr[1];
        qerr[2] = -qerr[2];
        qerr[3] = -qerr[3];
    }
    Vector3F dtheta;
    qerr.to_axis_angle(dtheta);
    const QuaternionF vq(0.0f, 0.5f*dtheta.x, 0.5f*dtheta.y, 0.5f*dtheta.z);
    const QuaternionF dq = q_mean * vq;
    dx[0] = dq[0];
    dx[1] = dq[1];
    dx[2] = dq[2];
    dx[3] = dq[3];
    for (uint8_t j = 4; j < n; j++) {
        dx[j] = x_s[j] - mean[j];
    }
}

bool NavUKF_core::ukfComputeUpdate(ftype z_meas, ftype R, UKFObs obs, uint32_t kalman_mask,
                                   ftype &innov, ftype &varInnov, bool wrap_angle)
{
    ftype mean[24];
    for (uint8_t i = 0; i <= stateIndexLim; i++) {
        mean[i] = statesArray[i];
    }
    if (!drawSigmaPoints(mean)) {
        forceCovariancePSD(P, sigma_prop, uint8_t(stateIndexLim + 1));
        innov = 0.0f;
        varInnov = R;
        return true;
    }

    ftype z[49];
    ftype zhat = 0.0f;
    for (uint8_t s = 0; s < ukf_n_sigma; s++) {
        const ftype W = (s == 0) ? ukf_Wm0 : ukf_Wi;
        ftype zi = predictObservation(sigma_prop[s], obs);
        if (wrap_angle) {
            zi = wrap_PI(zi - z_meas);
        }
        z[s] = zi;
        zhat += W * zi;
    }

    ftype Pzz = R;
    ftype Pxz[24] = {};
    for (uint8_t s = 0; s < ukf_n_sigma; s++) {
        const ftype W = (s == 0) ? ukf_Wc0 : ukf_Wi;
        const ftype dz = z[s] - zhat;
        Pzz += W * dz * dz;
        ftype dx[24] = {};
        ukf_attitude_state_residual(mean, sigma_prop[s], ukf_n, dx);
        for (uint8_t j = 0; j < ukf_n; j++) {
            Pxz[j] += W * dx[j] * dz;
        }
    }

    if (wrap_angle) {
        innov = wrap_PI(zhat);
    } else {
        innov = zhat - z_meas;
    }
    varInnov = Pzz;

    if (Pzz < R) {
        forceCovariancePSD(P, sigma_prop, uint8_t(stateIndexLim + 1));
        return true;
    }

    const ftype Pzz_inv = 1.0f / Pzz;
    for (uint8_t i = 0; i < 24; i++) {
        ftype Ki = 0.0f;
        if ((i <= stateIndexLim) && (kalman_mask & (1u << i))) {
            Ki = Pxz[i] * Pzz_inv;
        }
        Kfusion[i] = Ki;
    }
    return false;
}

bool NavUKF_core::ukfApplyUpdate(ftype innov, ftype varInnov, bool force)
{
    for (uint8_t i = 0; i <= stateIndexLim; i++) {
        for (uint8_t j = 0; j <= stateIndexLim; j++) {
            KHP[i][j] = Kfusion[i] * varInnov * Kfusion[j];
        }
    }
    return FinishFusion(innov, force);
}

bool NavUKF_core::FuseScalarUT(ftype z_meas, ftype R, UKFObs obs, uint32_t kalman_mask,
                               ftype &innov, ftype &varInnov, bool force, bool wrap_angle)
{
    if (ukfComputeUpdate(z_meas, R, obs, kalman_mask, innov, varInnov, wrap_angle)) {
        return true;
    }
    return ukfApplyUpdate(innov, varInnov, force);
}
