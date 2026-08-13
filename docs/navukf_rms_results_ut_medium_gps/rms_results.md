# NavUKF (UT) vs EKF3 SITL RMS - medium GPS

Both EK3 and UKF enabled with IMU_MASK=1 (single core each on IMU0). RMS vs SIM attitude and SIM2 position/velocity; EKF/UKF position origin offset removed using first 10 armed samples in window. Flight profile=circle. UKF_ALPHA=0.35 UKF_BETA=2 UKF_KAPPA=0. Window starts 15s after CIRCLE entry (post climb-out). Medium GPS: SIM_GPS1_FIXTYPE=3, ACC=2.5m, HNSE=2.0m, NOISE=2.5m, NUMSATS=8, LAG_MS=200; EK3/UKF POSNE_M_NSE=2.0.

## Primary: EKF3 (`AHRS_EKF_TYPE=3`)

| Estimator | Samples | RMS attitude (deg) | RMS roll | RMS pitch | RMS yaw | RMS position (m) | RMS PN | RMS PE | RMS PD | RMS vel (m/s) |
|-----------|---------|-------------------:|---------:|----------:|--------:|-----------------:|-------:|-------:|-------:|--------------:|
| XKF1 | 2015 | 1.7781 | 0.4189 | 0.4386 | 1.6715 | 2.8007 | 1.4386 | 2.3959 | 0.1841 | 0.3792 |
| UKF1 | 2015 | 1.8405 | 0.4738 | 0.5380 | 1.6952 | 2.7880 | 1.4286 | 2.3878 | 0.1746 | 0.3929 |

Log: `/home/bobr/Github/mpylyp/ardupilot/docs/navukf_rms_results_ut_medium_gps/run_ekf3_primary.bin`

## Primary: UKF (`AHRS_EKF_TYPE=4`)

| Estimator | Samples | RMS attitude (deg) | RMS roll | RMS pitch | RMS yaw | RMS position (m) | RMS PN | RMS PE | RMS PD | RMS vel (m/s) |
|-----------|---------|-------------------:|---------:|----------:|--------:|-----------------:|-------:|-------:|-------:|--------------:|
| XKF1 | 2025 | 1.7044 | 0.4203 | 0.4439 | 1.5910 | 2.4349 | 1.3179 | 2.0385 | 0.1908 | 0.3750 |
| UKF1 | 2025 | 1.7726 | 0.4776 | 0.5382 | 1.6200 | 2.4320 | 1.3116 | 2.0408 | 0.1723 | 0.3887 |

Log: `/home/bobr/Github/mpylyp/ardupilot/docs/navukf_rms_results_ut_medium_gps/run_ukf_primary.bin`

