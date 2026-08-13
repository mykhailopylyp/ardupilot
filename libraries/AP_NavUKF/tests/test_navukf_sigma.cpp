#include <AP_gtest.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX

// Standalone unscented-transform helpers mirrored from NavUKF covariance predict.
static void ukf_weights(uint8_t n, float alpha, float beta, float kappa,
                        float &Wm0, float &Wc0, float &Wi, float &gamma)
{
    const float n_f = (float)n;
    const float lambda = alpha * alpha * (n_f + kappa) - n_f;
    const float n_lambda = n_f + lambda;
    EXPECT_GT(n_lambda, 0.0f);
    gamma = sqrtf(n_lambda);
    Wm0 = lambda / n_lambda;
    Wc0 = Wm0 + (1.0f - alpha * alpha + beta);
    Wi = 0.5f / n_lambda;
}

TEST(NavUKFSigma, WeightsSumToOne)
{
    float Wm0, Wc0, Wi, gamma;
    const uint8_t n = 6;
    ukf_weights(n, 0.1f, 2.0f, 0.0f, Wm0, Wc0, Wi, gamma);
    const float sum_m = Wm0 + 2.0f * n * Wi;
    EXPECT_NEAR(sum_m, 1.0f, 1.0e-5f);
    EXPECT_GT(gamma, 0.0f);
    EXPECT_TRUE(isfinite(Wc0));
}

TEST(NavUKFSigma, LinearIdentityUT)
{
    // For f(x)=x, UT mean must equal prior mean and cov recover within tolerance.
    const uint8_t n = 3;
    float alpha = 0.001f, beta = 2.0f, kappa = 0.0f;
    float Wm0, Wc0, Wi, gamma;
    ukf_weights(n, alpha, beta, kappa, Wm0, Wc0, Wi, gamma);

    float x[3] = {1.0f, -2.0f, 0.5f};
    float P[3][3] = {
        {0.04f, 0, 0},
        {0, 0.09f, 0},
        {0, 0, 0.01f}
    };
    // Cholesky L
    float L[3][3] = {};
    for (uint8_t i = 0; i < n; i++) {
        for (uint8_t j = 0; j <= i; j++) {
            float sum = P[i][j];
            for (uint8_t k = 0; k < j; k++) {
                sum -= L[i][k] * L[j][k];
            }
            if (i == j) {
                L[i][j] = sqrtf(sum);
            } else {
                L[i][j] = sum / L[j][j];
            }
        }
    }
    float sig[7][3];
    for (uint8_t j = 0; j < n; j++) {
        sig[0][j] = x[j];
    }
    for (uint8_t i = 0; i < n; i++) {
        for (uint8_t j = 0; j < n; j++) {
            const float Lji = (j >= i) ? L[j][i] : 0.0f;
            sig[i + 1][j] = x[j] + gamma * Lji;
            sig[i + 1 + n][j] = x[j] - gamma * Lji;
        }
    }
    float mean[3] = {};
    for (uint8_t s = 0; s < 2 * n + 1; s++) {
        const float W = (s == 0) ? Wm0 : Wi;
        for (uint8_t j = 0; j < n; j++) {
            mean[j] += W * sig[s][j];
        }
    }
    EXPECT_NEAR(mean[0], x[0], 1.0e-2f);
    EXPECT_NEAR(mean[1], x[1], 1.0e-2f);
    EXPECT_NEAR(mean[2], x[2], 1.0e-2f);

    float Pout[3][3] = {};
    for (uint8_t s = 0; s < 2 * n + 1; s++) {
        const float W = (s == 0) ? Wc0 : Wi;
        float dx[3];
        for (uint8_t j = 0; j < n; j++) {
            dx[j] = sig[s][j] - mean[j];
        }
        for (uint8_t i = 0; i < n; i++) {
            for (uint8_t j = i; j < n; j++) {
                Pout[i][j] += W * dx[i] * dx[j];
                Pout[j][i] = Pout[i][j];
            }
        }
    }
    EXPECT_NEAR(Pout[0][0], P[0][0], 1.0e-3f);
    EXPECT_NEAR(Pout[1][1], P[1][1], 1.0e-3f);
    EXPECT_NEAR(Pout[2][2], P[2][2], 1.0e-3f);
}

TEST(NavUKFSigma, QuaternionSignAlign)
{
    // Opposite-hemisphere quaternions should be flipped before averaging
    Quaternion q1(1, 0, 0, 0);
    Quaternion q2(-1, 0, 0, 0);
    float dot = q1[0]*q2[0] + q1[1]*q2[1] + q1[2]*q2[2] + q1[3]*q2[3];
    EXPECT_LT(dot, 0);
    if (dot < 0) {
        q2 = Quaternion(-q2[0], -q2[1], -q2[2], -q2[3]);
    }
    EXPECT_NEAR(q1[0], q2[0], 1.0e-6f);
}

#endif

AP_GTEST_MAIN()
