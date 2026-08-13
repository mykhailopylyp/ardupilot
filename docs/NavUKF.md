# NavUKF User Guide

Unscented Kalman Filter navigation backend for ArduPilot (`libraries/AP_NavUKF`), structured like EKF3. On SITL it is available by default (`HAL_NAVUKF_AVAILABLE`). Flight boards keep it disabled unless enabled via custom build / `HAL_NAVUKF_AVAILABLE`.

**AI-assisted contribution.**

## What it is

- Same 24-state INS model and delayed-horizon fusion layout as EKF3.
- **Covariance prediction** and **all measurement updates** use the unscented transform (sigma points via `UKF_ALPHA` / `UKF_BETA` / `UKF_KAPPA`). There is no Jacobian measurement or predict path.
- Selected with `AHRS_EKF_TYPE=4`.

## Parameters

Prefix: `UKF_`

| Parameter | Meaning |
|-----------|---------|
| `UKF_ENABLE` | Run UKF maths (1=on). Still need `AHRS_EKF_TYPE=4` to fly on UKF. |
| `UKF_IMU_MASK` | Bitmask of IMUs → UKF cores (use `1` for a single core on IMU0). |
| `UKF_PRIMARY` | Preferred core index while disarmed. |
| `UKF_ALPHA` | Unscented transform α (default `0.35`). |
| `UKF_BETA` | Unscented transform β (default `2`). |
| `UKF_KAPPA` | Unscented transform κ (default `0`). |
| Other `UKF_*` | Ported from EKF3 noise/gate/source parameters (`GYRO_P_NSE`, `ACC_P_NSE`, GPS/baro/mag gates, `SRC*`, …). |

AHRS:

| Parameter | Value |
|-----------|-------|
| `AHRS_EKF_TYPE` | `4` = NavUKF (`3` remains EKF3 default) |

## Configure SITL to run UKF

```sh
./waf configure --board sitl
./waf plane

Tools/autotest/sim_vehicle.py -v ArduPlane -f plane --console --map \
  -P AHRS_EKF_TYPE=4 \
  -P UKF_ENABLE=1 \
  -P UKF_IMU_MASK=1
```

Or after boot (then reboot SITL / soft reboot):

```text
AHRS_EKF_TYPE 4
UKF_ENABLE 1
UKF_IMU_MASK 1
UKF_ALPHA 0.35
UKF_BETA 2
UKF_KAPPA 0
```

Expect status text similar to: `AHRS: UKF active`.

Logs (when logging enabled): `UKF1`–`UKF4`, `UKFQ`, `UKFS`.

## How to run tests

### Unit tests (GTest)

```sh
./waf configure --board sitl
./waf --targets tests/test_navukf_sigma
./build/sitl/tests/test_navukf_sigma
```

Covers UT weight normalisation, linear identity recoverability, and quaternion sign alignment.

### Autotest smoke (Plane)

```sh
./waf configure --board sitl
./waf plane
Tools/autotest/autotest.py build.Plane test.Plane.NavUKFSmoke
```

Or force AHRS type globally for a suite:

```sh
Tools/autotest/autotest.py build.Plane test.Plane.NavUKFSmoke --force-ahrs-type=4
```

`NavUKFSmoke` sets `AHRS_EKF_TYPE=4`, waits for `AHRS: UKF active`, takes off, short FBWA segment, RTL.

### Build-only check

```sh
./waf configure --board sitl && ./waf plane
```

## Notes / limits

- Default **off** on non-SITL boards (flash/CPU).
- Covariance prediction and measurement fusion both use the unscented transform (no Jacobian algebra).
- UT sigma points use each point’s own start-of-step attitude and raw IMU deltas with bias scaled by `del*DT/dtEkfAvg` (matching strapdown).
- Prefer `UKF_ALPHA` around `0.35` (tuned vs EKF3 on complex SITL flights; not `0.001`): tiny alpha makes central UT weights ~`-1/α²` and ill-conditioned `P`. Attitude sigma points are multiplicative (rotation-vector on the quaternion manifold).
- Scaled UT can leave `P` briefly ill-conditioned (`Pzz < R` or `FinishFusion` refusing an update). NavUKF repairs `P` and skips that sample instead of `BAD_*MAG` / `CovarianceInit`, so AHRS can keep UKF primary (no DCM flicker).
- Dual-estimator RMS (medium GPS): `test.Plane.NavUKFEKF3RMS_UT` → `docs/navukf_rms_results_ut_medium_gps/`.
- Sigma param sweep (UKF primary only): `test.Plane.NavUKFSigmaSweep` → `docs/navukf_sigma_sweep/`.
- Tune/validate campaign: `test.PlaneTests1a.NavUKFTuneValidate` → `docs/navukf_tune_validate/`.
- Optional EKF3-style log messages (beacon/timing/GSF detail) are trimmed to keep `LogMessages` under ID limits.
- Prefer comparing UKF vs EKF3 on the same SITL mission before any flight use.
- Do not set UKF as default for production vehicles without maintainer review.
