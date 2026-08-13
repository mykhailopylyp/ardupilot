# NavUKF (UT) vs EKF3 SITL RMS - medium GPS

Both EK3 and UKF enabled with IMU_MASK=1 (single core each on IMU0). RMS vs SIM attitude and SIM2 position/velocity; EKF/UKF position origin offset removed using first 10 armed samples in window. Window starts 15s after CIRCLE entry (post climb-out). UKF_USE_UT=1 UKF_ALPHA=0.5 UKF_BETA=2 UKF_KAPPA=0. Medium GPS: SIM_GPS1_FIXTYPE=3, ACC=2.5m, HNSE=2.0m, NOISE=2.5m, NUMSATS=8, LAG_MS=200; EK3/UKF POSNE_M_NSE=2.0.

## Primary: EKF3 (`AHRS_EKF_TYPE=3`)

| Estimator | Samples | RMS attitude (deg) | RMS roll | RMS pitch | RMS yaw | RMS position (m) | RMS PN | RMS PE | RMS PD | RMS vel (m/s) |
|-----------|---------|-------------------:|---------:|----------:|--------:|-----------------:|-------:|-------:|-------:|--------------:|
| XKF1 | 2015 | 1.7368 | 0.4215 | 0.4487 | 1.6240 | 2.5047 | 1.3889 | 2.0757 | 0.1896 | 0.3750 |
| UKF1 | 2015 | 1.7249 | 0.4768 | 0.4805 | 1.5865 | 2.4915 | 1.3705 | 2.0725 | 0.1836 | 0.3784 |

Log: `/home/bobr/Github/mpylyp/ardupilot/docs/navukf_rms_results_ut_medium_gps/run_ekf3_primary.bin`

## Primary: UKF (`AHRS_EKF_TYPE=4`)

| Estimator | Samples | RMS attitude (deg) | RMS roll | RMS pitch | RMS yaw | RMS position (m) | RMS PN | RMS PE | RMS PD | RMS vel (m/s) |
|-----------|---------|-------------------:|---------:|----------:|--------:|-----------------:|-------:|-------:|-------:|--------------:|
| XKF1 | 1910 | 1.5988 | 0.4710 | 0.4657 | 1.4551 | 2.2084 | 1.1841 | 1.8521 | 0.2110 | 0.3800 |
| UKF1 | 1910 | 1.6316 | 0.5679 | 0.5159 | 1.4399 | 2.2081 | 1.1776 | 1.8580 | 0.1914 | 0.3824 |

Log: `/home/bobr/Github/mpylyp/ardupilot/docs/navukf_rms_results_ut_medium_gps/run_ukf_primary.bin`

