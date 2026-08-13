# NavUKF vs EKF3 SITL RMS - complex long route

Both EK3 and UKF enabled with IMU_MASK=1 (single core each on IMU0). RMS vs SIM attitude and SIM2 position/velocity; EKF/UKF position origin offset removed using first 10 armed samples in window. Flight profile=complex. UKF_ALPHA=0.35 UKF_BETA=2 UKF_KAPPA=0. Complex route: FBWA left circuit, climb/descend, CIRCLE, LOITER, ACRO rolls/loops, FBWB circuit, second circuit, CIRCLE; then RTL. Medium GPS: SIM_GPS1_FIXTYPE=3, ACC=2.5m, HNSE=2.0m, NOISE=2.5m, NUMSATS=8, LAG_MS=200; EK3/UKF POSNE_M_NSE=2.0.

## Primary: EKF3 (`AHRS_EKF_TYPE=3`)

| Estimator | Samples | RMS attitude (deg) | RMS roll | RMS pitch | RMS yaw | RMS position (m) | RMS PN | RMS PE | RMS PD | RMS vel (m/s) |
|-----------|---------|-------------------:|---------:|----------:|--------:|-----------------:|-------:|-------:|-------:|--------------:|
| XKF1 | 11233 | 2.2102 | 0.9373 | 0.4972 | 1.9389 | 3.1993 | 2.5270 | 1.9342 | 0.3303 | 0.6940 |
| UKF1 | 11233 | 2.7002 | 1.4705 | 0.5976 | 2.1845 | 3.2216 | 2.5613 | 1.9155 | 0.3864 | 0.7283 |

Log: `/home/bobr/Github/mpylyp/ardupilot/docs/navukf_rms_results_ut_complex/run_ekf3_primary.bin`

## Primary: UKF (`AHRS_EKF_TYPE=4`)

| Estimator | Samples | RMS attitude (deg) | RMS roll | RMS pitch | RMS yaw | RMS position (m) | RMS PN | RMS PE | RMS PD | RMS vel (m/s) |
|-----------|---------|-------------------:|---------:|----------:|--------:|-----------------:|-------:|-------:|-------:|--------------:|
| XKF1 | 11018 | 2.1434 | 0.9150 | 0.5221 | 1.8666 | 3.5768 | 2.1220 | 2.8549 | 0.3751 | 0.7101 |
| UKF1 | 11018 | 2.4550 | 1.2778 | 0.6284 | 1.9999 | 3.6787 | 2.2396 | 2.8965 | 0.3570 | 0.7471 |

Log: `/home/bobr/Github/mpylyp/ardupilot/docs/navukf_rms_results_ut_complex/run_ukf_primary.bin`

