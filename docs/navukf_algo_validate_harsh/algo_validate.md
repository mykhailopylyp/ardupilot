# NavUKF algorithm test / validation vs EKF3

Same profiles and UKF defaults as NavUKFAlgoValidate, plus SITL IMU/gyro/magnetometer noise, axis scale error, mag soft-iron, altitude-dependent mag anomaly, motor mag interference, and IMU lever-arm. Medium GPS. Score = mean(UKF_primary/EKF_primary) over att/pos/vel.

## SITL sensor params

| Param | Value |
|------:|------:|
| ARMING_MAGTHRESH | 400 |
| SIM_ACC1_RND | 2.5 |
| SIM_ACC1_SCAL_X | 1.03 |
| SIM_ACC1_SCAL_Y | 0.97 |
| SIM_ACC1_SCAL_Z | 1.02 |
| SIM_GYR1_RND | 8 |
| SIM_GYR1_SCALE_X | 2.5 |
| SIM_GYR1_SCALE_Y | -1.5 |
| SIM_GYR1_SCALE_Z | 3 |
| SIM_IMU_POS_X | 0.12 |
| SIM_IMU_POS_Y | 0.06 |
| SIM_IMU_POS_Z | -0.04 |
| SIM_MAG1_DIA_X | 1.06 |
| SIM_MAG1_DIA_Y | 0.95 |
| SIM_MAG1_DIA_Z | 1.04 |
| SIM_MAG1_ODI_X | 0.04 |
| SIM_MAG1_ODI_Y | -0.03 |
| SIM_MAG1_ODI_Z | 0.035 |
| SIM_MAG1_SCALING | 1.04 |
| SIM_MAG_ALY_HGT | 180 |
| SIM_MAG_ALY_X | 80 |
| SIM_MAG_ALY_Y | 40 |
| SIM_MAG_ALY_Z | -50 |
| SIM_MAG_MOT_X | 6 |
| SIM_MAG_MOT_Y | -4 |
| SIM_MAG_MOT_Z | 3 |
| SIM_MAG_RND | 20 |
| SIM_VIB_FREQ_X | 22 |
| SIM_VIB_FREQ_Y | 28 |
| SIM_VIB_FREQ_Z | 18 |

## Test set

| Profile | EKF att | UKF att | EKF pos | UKF pos | EKF vel | UKF vel | score | UKF better |
|---------|--------:|--------:|--------:|--------:|--------:|--------:|------:|:----------:|
| tune_bank | 6.2311 | 5.6068 | 3.6948 | 4.0235 | 0.6522 | 0.5938 | 0.9664 | True |
| tune_vert | 2.2042 | 3.8651 | 3.2414 | 2.4219 | 0.6631 | 0.8355 | 1.2535 | False |
| tune_acro | 3.8419 | 3.7318 | 2.7978 | 2.3057 | 0.6686 | 0.6124 | 0.9038 | True |

Test mean UKF/EKF score: **1.0413**

## Validation (held-out)

| Profile | EKF att | UKF att | EKF pos | UKF pos | EKF vel | UKF vel | score | UKF better |
|---------|--------:|--------:|--------:|--------:|--------:|--------:|------:|:----------:|
| val_mixed | 3.4508 | 8.9814 | 2.5226 | 2.5277 | 0.8220 | 1.1585 | 1.6714 | False |
| val_endurance | 11.3503 | 12.7943 | 4.0757 | 3.5527 | 1.0475 | 1.0623 | 1.0043 | False |

Validation mean UKF/EKF score: **1.3379** (UKF better overall: False)
