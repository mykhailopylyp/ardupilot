# NavUKF vs EKF3 SITL RMS (vs SIM truth)

Both EK3 and UKF enabled with IMU_MASK=1 (single core each on IMU0). RMS vs SIM attitude and SIM2 position/velocity; EKF/UKF position origin offset removed using first 10 armed samples in window. Flight profile=circle. UKF_ALPHA=0.35 UKF_BETA=2 UKF_KAPPA=0. Window starts 15s after CIRCLE entry (post climb-out).

## Primary: EKF3 (`AHRS_EKF_TYPE=3`)

| Estimator | Samples | RMS attitude (deg) | RMS roll | RMS pitch | RMS yaw | RMS position (m) | RMS PN | RMS PE | RMS PD | RMS vel (m/s) |
|-----------|---------|-------------------:|---------:|----------:|--------:|-----------------:|-------:|-------:|-------:|--------------:|
| XKF1 | 1992 | 0.4976 | 0.0938 | 0.1868 | 0.4516 | 0.9804 | 0.4117 | 0.8850 | 0.0916 | 0.1587 |
| UKF1 | 1992 | 0.4671 | 0.1277 | 0.2164 | 0.3938 | 0.9934 | 0.4123 | 0.8996 | 0.0866 | 0.1591 |

Log: `/home/bobr/Github/mpylyp/ardupilot/docs/navukf_rms_results/run_ekf3_primary.bin`

## Primary: UKF (`AHRS_EKF_TYPE=4`)

| Estimator | Samples | RMS attitude (deg) | RMS roll | RMS pitch | RMS yaw | RMS position (m) | RMS PN | RMS PE | RMS PD | RMS vel (m/s) |
|-----------|---------|-------------------:|---------:|----------:|--------:|-----------------:|-------:|-------:|-------:|--------------:|
| XKF1 | 1998 | 0.5260 | 0.1020 | 0.2361 | 0.4588 | 0.8975 | 0.4168 | 0.7896 | 0.0908 | 0.1660 |
| UKF1 | 1998 | 0.4790 | 0.1339 | 0.2474 | 0.3877 | 0.9011 | 0.4168 | 0.7946 | 0.0825 | 0.1682 |

Log: `/home/bobr/Github/mpylyp/ardupilot/docs/navukf_rms_results/run_ukf_primary.bin`

