# NavUKF (UT) vs EKF3 SITL RMS - complex long route

Both EK3 and UKF enabled with IMU_MASK=1 (single core each on IMU0). RMS vs SIM attitude and SIM2 position/velocity; EKF/UKF position origin offset removed using first 10 armed samples in window. Flight profile=complex. UKF_USE_UT=1 UKF_ALPHA=0.5 UKF_BETA=2 UKF_KAPPA=0. Complex route: FBWA left circuit, climb/descend, CIRCLE, LOITER, ACRO rolls/loops, FBWB circuit, second circuit, CIRCLE; then RTL. Medium GPS: SIM_GPS1_FIXTYPE=3, ACC=2.5m, HNSE=2.0m, NOISE=2.5m, NUMSATS=8, LAG_MS=200; EK3/UKF POSNE_M_NSE=2.0.

## Primary: EKF3 (`AHRS_EKF_TYPE=3`)

| Estimator | Samples | RMS attitude (deg) | RMS roll | RMS pitch | RMS yaw | RMS position (m) | RMS PN | RMS PE | RMS PD | RMS vel (m/s) |
|-----------|---------|-------------------:|---------:|----------:|--------:|-----------------:|-------:|-------:|-------:|--------------:|
| XKF1 | 11399 | 3.8247 | 3.4051 | 0.4876 | 1.6720 | 3.6626 | 2.4279 | 2.7254 | 0.3039 | 0.6646 |
| UKF1 | 11399 | 3.8439 | 3.4226 | 0.5941 | 1.6457 | 3.6921 | 2.4520 | 2.7313 | 0.3991 | 0.7048 |

Log: `/home/bobr/Github/mpylyp/ardupilot/docs/navukf_rms_results_ut_complex/run_ekf3_primary.bin`

## Primary: UKF (`AHRS_EKF_TYPE=4`)

| Estimator | Samples | RMS attitude (deg) | RMS roll | RMS pitch | RMS yaw | RMS position (m) | RMS PN | RMS PE | RMS PD | RMS vel (m/s) |
|-----------|---------|-------------------:|---------:|----------:|--------:|-----------------:|-------:|-------:|-------:|--------------:|
| XKF1 | 11270 | 1.8735 | 0.5296 | 0.4960 | 1.7273 | 3.6682 | 2.3820 | 2.7716 | 0.3164 | 0.6741 |
| UKF1 | 11270 | 1.9207 | 0.6619 | 0.5975 | 1.7012 | 3.7036 | 2.4094 | 2.7812 | 0.4199 | 0.7136 |

Log: `/home/bobr/Github/mpylyp/ardupilot/docs/navukf_rms_results_ut_complex/run_ukf_primary.bin`

