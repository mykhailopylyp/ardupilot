# NavUKF algorithm test / validation vs EKF3

Restored Euclidean UT mean and additive IMU Q after tangent-mean and cubature failed on the test set (cubature tune_bank att ratio ~16). Scaled UT ALPHA=0.35 BETA=2 KAPPA=0 SIGMA=0. Medium GPS. Score = mean(UKF_primary/EKF_primary) over att/pos/vel.

Cubature (UKF_SIGMA=1) was screened out on tune_bank (attitude RMS ratio ~16 vs EKF3).

## Test set

| Profile | EKF att | UKF att | EKF pos | UKF pos | EKF vel | UKF vel | score | UKF better |
|---------|--------:|--------:|--------:|--------:|--------:|--------:|------:|:----------:|
| tune_bank | 1.1516 | 1.1224 | 3.8772 | 3.5956 | 0.3909 | 0.4255 | 0.9968 | True |
| tune_vert | 1.1559 | 1.1140 | 2.8246 | 2.8635 | 0.4923 | 0.5073 | 1.0027 | False |
| tune_acro | 1.3305 | 1.4515 | 2.7322 | 2.9816 | 0.4895 | 0.5169 | 1.0794 | False |

Test mean UKF/EKF score: **1.0263**

## Validation (held-out)

| Profile | EKF att | UKF att | EKF pos | UKF pos | EKF vel | UKF vel | score | UKF better |
|---------|--------:|--------:|--------:|--------:|--------:|--------:|------:|:----------:|
| val_mixed | 1.2169 | 1.4456 | 2.4949 | 2.8167 | 0.5696 | 0.6003 | 1.1236 | False |
| val_endurance | 1.2148 | 1.2062 | 4.0290 | 3.9934 | 0.5872 | 0.5981 | 1.0009 | False |

Validation mean UKF/EKF score: **1.0623** (UKF better overall: False)
