# NavUKF Implementation Plan

Plan for an Unscented Kalman Filter navigation estimator that follows ArduPilot’s existing EKF architecture (primarily `AP_NavEKF3`), replacing Jacobian-based covariance propagation and measurement updates with sigma-point transforms.

This document is a design plan only. No filter code is implied to exist yet. There is currently **no UKF** in the ArduPilot tree.

---

## 1. Goals

- Provide a navigation filter with the **same external contracts** as EKF3 so AHRS, arming, failsafes, and vehicle code can consume it without special cases.
- Keep the **same 24-state INS model** and delayed-horizon fusion architecture as EKF3.
- Replace EKF linearisation with an **Unscented Transform** for:
  - process covariance prediction (IMU strapdown), and
  - measurement updates (GPS, baro, mag, airspeed, optflow, etc.).
- Make the feature **compile-time optional** and safe to leave disabled on constrained boards.
- Prefer **clone-and-adapt** of `AP_NavEKF3` over inventing a new AHRS/filter topology.
- Treat **testing as a verification gate per phase** (unit, SITL UKF core, truth/EKF3 compare, perf), runnable by humans or AI agents via exit-coded commands and JSON artifacts.

## 2. Non-goals (initial delivery)

- Replacing EKF3 as the default (`AHRS_EKF_TYPE` stays 3).
- Matching EKF3 feature parity on day one (beacons, drag, SRTM, etc. can land in later phases).
- Changing `AP_DAL`, Replay format, or MAVLink GCS protocols beyond additive logging.
- Modifying submodules or inventing new sensor drivers.
- Claiming flight-safety readiness without SITL comparison and review by maintainers.

---

## 3. Why mirror EKF3

EKF3 is the production navigation filter. Its value for this work is the surrounding machinery, not the Jacobian algebra:

| Concern | EKF3 mechanism | UKF should reuse |
|--------|----------------|------------------|
| Frontend / cores | `NavEKF3` + `NavEKF3_core` | `NavUKF` + `NavUKF_core` |
| Multi-IMU lanes | `EK3_IMU_MASK`, `errorScore`, `switchLane` | Same pattern (`UKF_*`) |
| Delayed fusion | `EKF_Buffer` IMU/obs rings | Same buffers |
| Sensor I/O | `AP_DAL` + Measurements | Same |
| Source selection | `AP_NavEKF_Source` (`EK3_SRC*`) | Share or parallel `UKF_SRC*` |
| AHRS backend | `AP_AHRS_NavEKF3` | `AP_AHRS_NavUKF` |
| Status contract | `nav_filter_status`, faults, variances | Identical structs |
| Output timing | Fusion horizon + `calcOutputStates()` | Same |

What changes is the **math core**: `CovariancePrediction`, Jacobian/`H_*` formation, and `FinishFusion` / gain application.

---

## 4. Recommended filter formulation

### 4.1 State vector (same as EKF3)

Mirror `NavEKF3_core::state_elements` (24 floats):

| Index | State |
|------:|-------|
| 0–3 | Quaternion (NED → body) |
| 4–6 | Velocity NED (m/s) |
| 7–9 | Position NED (m) |
| 10–12 | Gyro delta-angle bias (rad) |
| 13–15 | Accel delta-velocity bias (m/s) |
| 16–18 | Earth mag field (Gauss) |
| 19–21 | Body mag field (Gauss) |
| 22–23 | Wind NE (m/s) |

Retain `stateIndexLim` so inactive wind/mag/bias blocks can shrink work the same way EKF3 does.

### 4.2 Error-state UKF (preferred)

EKF3 does **not** treat the full quaternion as a Euclidean KF state for covariance purposes. Attitude error is handled carefully; strapdown integrates the quaternion, then fusion corrects it. Generated covariance code assumes that formulation (`libraries/AP_NavEKF3/derivation/`).

**Recommendation:** implement an **error-state / multiplicative UKF**:

- Nominal state: full quaternion + remaining Euclidean states (same storage layout as EKF3).
- Uncertainty / sigma points: **3-parameter attitude error** (rotation vector) + Euclidean errors for the other states → **23-dimensional** error vector when fully active (or less when `stateIndexLim` shrinks).
- After each predict/update, apply attitude error to the nominal quaternion (multiplicative reset) and zero the attitude-error mean.

**Alternative (not preferred for first port):** full-state UKF with 24 states and ad-hoc quaternion normalisation of every sigma point. Simpler to sketch, weaker numerically, and less aligned with EKF3.

### 4.3 Unscented transform (classic)

For error dimension \(n\) (typically ≤ 23):

- \(2n + 1\) sigma points.
- Parameters (tunable via `UKF_*` params, with sensible defaults):
  - \(\alpha\) (spread), \(\beta\) (prior; often 2 for Gaussian), \(\kappa\) (secondary scaling).
  - \(\lambda = \alpha^2 (n + \kappa) - n\).
- Weights \(W_m\), \(W_c\) for mean and covariance.
- Process noise \(Q\) and measurement noise \(R\) reused conceptually from EKF3’s process/measurement noise parameters where possible.

Predict:

1. Form sigma points from \(\hat{x}\), \(P\) (Cholesky / square-root of \(P\)).
2. Propagate each through strapdown (same IMU delta-angle / delta-velocity path as `UpdateStrapdownEquationsNED`).
3. Recover mean and \(P = \sum W_c (x_i - \hat{x})(\cdot)^T + Q\).

Update (per scalar or vector measurement, matching EKF3’s sequential fusion style where practical):

1. Propagate sigma points through measurement function \(h(x)\).
2. Compute \(\hat{z}\), \(P_{zz}\), \(P_{xz}\).
3. \(K = P_{xz} P_{zz}^{-1}\), innovate, update \(P\).
4. Apply the same innovation gating / consistency checks EKF3 uses (`testRatio`, gates).
5. `ConstrainStates` / `ConstrainVariances` after updates.

Square-root UKF (QR / Cholesky factorisation of \(P\)) is a later optimisation if numerical health or MCU cost demands it.

---

## 5. Proposed library and file layout

New library: `libraries/AP_NavUKF/`

Mirror EKF3’s split; names intentionally parallel:

```text
libraries/AP_NavUKF/
  AP_NavUKF.h / .cpp              # frontend: params, multi-core, public API
  AP_NavUKF_core.h / .cpp         # per-IMU core: state, P, UpdateFilter
  AP_NavUKF_Sigma.cpp             # sigma-point generation, UT predict/update helpers
  AP_NavUKF_Control.cpp           # aiding modes, wind/mag learning, filter status
  AP_NavUKF_Measurements.cpp      # sensor read into delayed buffers (clone)
  AP_NavUKF_PosVelFusion.cpp      # GPS / baro / range / body-odom (UKF update)
  AP_NavUKF_MagFusion.cpp         # mag / yaw
  AP_NavUKF_AirDataFusion.cpp     # TAS / sideslip / drag (phased)
  AP_NavUKF_OptFlowFusion.cpp     # optical flow (phased)
  AP_NavUKF_Outputs.cpp           # getters for AHRS
  AP_NavUKF_VehicleStatus.cpp     # GPS quality / flight detection helpers
  AP_NavUKF_Logging.cpp           # UKF* / XUKF* log messages
  AP_NavUKF_feature.h             # compile-time feature gates
  LogStructure.h                  # log packet layouts
  derivation/                     # optional sympy / notebooks for UT validation
```

Shared code to **depend on**, not copy blindly:

- `libraries/AP_NavEKF/EKF_Buffer.*`
- `libraries/AP_NavEKF/AP_Nav_Common.h` (`nav_filter_status`, `MAX_EKF_CORES`)
- `libraries/AP_NavEKF/AP_NavEKF_Source.*` (initially share source sets; decide later if UKF needs its own)
- `libraries/AP_NavEKF/EKFGSF_yaw.*` (optional; keep GSF as companion yaw estimator)
- `libraries/AP_DAL/` for sensor abstraction and Replay

AHRS glue:

- `libraries/AP_AHRS/AP_AHRS_NavUKF.h/.cpp` — clone of `AP_AHRS_NavEKF3`
- Extend `AP_AHRS::EKFType` with e.g. `UKF = 4` (value TBD; must not collide with `SIM=10`, `EXTERNAL=11`)
- Gates in `AP_AHRS_config.h`: `HAL_NAVUKF_AVAILABLE`, `AP_AHRS_NAVUKF_ENABLED`

Build / options:

- Add library to Waf common vehicle libraries (or only when enabled).
- `Tools/scripts/build_options.py`: Feature `UKF` (default off).
- `#if HAL_NAVUKF_AVAILABLE` around all code paths (same style as EKF3).

Vehicle parameters (each vehicle `Parameters.cpp`):

```cpp
GOBJECTN(ahrs.ekf_ukf.UKF, NavUKF, "UKF_", NavUKF),
```

(Exact member name to match AHRS embedding style used for `ahrs.ekf3`.)

---

## 6. What to copy vs replace

### Copy / adapt with minimal semantic change

- Frontend lifecycle: `InitialiseFilter`, core allocation, lane switch, primary selection.
- `UpdateFilter` control flow (order of Select/Fuse stages).
- Measurements + ring buffers + fusion-horizon timing.
- `calcOutputStates()` complementary filter to present time.
- Outputs / status / pre-arm checks API surface used by `AP_AHRS_NavEKF3`.
- Constraint helpers, mag table limits, reset bookkeeping (yaw/pos/vert reset counters).
- Logging *structure* (new message names; same fields where possible).

### Replace

| EKF3 piece | UKF replacement |
|------------|-----------------|
| `CovariancePrediction()` sympy algebra | Sigma-point process UT + \(Q\) |
| Hand/`H_*` Jacobians in Fuse* | Measurement UT / \(h(x)\) on sigma points |
| `Kfusion` / `KHP` / `FinishFusion` | UKF Kalman gain from \(P_{xz}\), \(P_{zz}\); symmetric \(P\) update |
| Attitude error linearisation assumptions | Multiplicative attitude error sigma points |

### Keep for comparison / fallback

- Do not remove EKF3.
- Allow both to compile; select via `AHRS_EKF_TYPE`.
- Optional: dual-run in SITL for innovation/covariance comparison (dev builds only).

---

## 7. UpdateFilter loop (target)

Preserve EKF3’s stage order (`NavEKF3_core::UpdateFilter`):

1. `controlFilterModes()`
2. `readIMUData(predict)`
3. If `runUpdates`:
   - Strapdown nominal integration (`UpdateStrapdownEquationsNED` or UKF-aware equivalent)
   - **UKF covariance / mean predict** via sigma points
   - GSF yaw predict (if retained)
   - `SelectMagFusion` → UKF mag/yaw update
   - `SelectVelPosFusion` → UKF vel/pos/height update
   - GSF correction / yaw reset hooks
   - Optional: beacon / optflow / body-odom / TAS / beta-drag (phased)
   - `updateFilterStatus()`
4. `calcOutputStates()`

Innovation gating, sensor selection (`AP_NavEKF_Source`), and aiding modes (`AID_ABSOLUTE` / `NONE` / `RELATIVE`) stay policy-compatible with EKF3 so vehicle behaviour does not fork.

---

## 8. Parameters (initial set)

Prefix: `UKF_` (≤16 char full names).

Minimum viable:

| Param | Role |
|-------|------|
| `UKF_ENABLE` | Run UKF maths (flight use still needs `AHRS_EKF_TYPE`) |
| `UKF_IMU_MASK` | Multi-core IMU mask |
| `UKF_PRIMARY` | Forced primary when disarmed |
| `UKF_ERR_THRESH` | Lane switch score threshold |
| `UKF_ALPHA` / `UKF_BETA` / `UKF_KAPPA` | Unscented transform tuning |
| Noise / gate mirrors | Port critical `EK3_*` noise and gate params (`GYRO_P_NSE`, `ACC_P_NSE`, GPS/baro/mag noises, gates) |

Later: affinity, GSF masks, flow, drag, options bitmask — only as features are ported.

Register with full `@Param` / `@DisplayName` / `@Description` / `@User` annotations. **Do not** renumber existing EKF3 indices.

---

## 9. Logging

Add parallel messages (names TBD with logging maintainers), e.g.:

- `UKF1` attitude/vel/pos
- `UKF2` biases/wind/mag
- `UKF3` innovations
- `UKF4` variances/status
- `UKFQ` quaternion
- Optional: sigma-point health (min eigenvalue of \(P\), Cholesky failures)

Replay: prefer DAL-compatible I/O so existing Replay workflows can exercise UKF once an AHRS type is selected.

---

## 10. Performance and embedded constraints

UKF is heavier than EKF3’s specialised predict:

- Per IMU predict: \(2n+1\) strapdown propagations (n ≈ 15–23).
- Per measurement: another \(2n+1\) measurement-model evaluations (can be cheaper than full strapdown).
- Memory: sigma-point matrix \((2n+1) \times n\), plus temporaries; avoid large stack — use core-scoped buffers / shared scratch like `NavEKF_core_common`.

Mitigations:

1. Default **off** in `build_options.py`.
2. Phase 1: SITL / high-resource boards only.
3. Respect `stateIndexLim` (inhibit wind/mag early).
4. Sequential scalar updates (as EKF3 often does) to limit \(P_{zz}\) size.
5. Consider square-root form and/or reduced-state UKF if flash/CPU blow up.
6. Profile on a representative ChibiOS board before claiming hardware readiness.

Binary size and CPU must be treated as first-class acceptance criteria, not afterthoughts.

---

## 11. Phased implementation (with verification gates)

Every phase has **implementation work** and a **verification gate**. A phase is not done until its gate commands exit 0 and artifacts are recorded. Gates are written so a human or AI agent can run them without judgment calls.

Subsystem-correct commits (`AP_NavUKF:`, `AP_AHRS:`, `autotest:`, `Tools:`, …). Disclose AI assistance in PRs. Do not fabricate test results.

### Phase 0 — Scaffolding

**Implement**

- Create `AP_NavUKF` library skeleton + feature gate + Waf wiring.
- AHRS backend stub + `AHRS_EKF_TYPE` value (proposal: `4`).
- Vehicle `GOBJECTN` registration behind `#if`.
- Empty/core bootstrap that initialises state like EKF3 but is not required to be flight-capable yet.
- Skeleton `libraries/AP_NavUKF/tests/` that builds (can be a trivial `EXPECT_TRUE(true)` until maths land).

**Verify (gate 0)**

```sh
./waf configure --board sitl
./waf plane                    # default build still healthy with UKF gated off or stubbed
./waf --targets tests/test_navukf_sigma   # or whatever the unit-test target is named
```

Pass criteria: plane links; UKF unit-test target builds and passes; firmware still defaults to EKF3.

### Phase 1 — Core UKF maths (SITL, single core)

**Implement**

- Error-state sigma-point helpers + unit tests (see §12.1).
- Strapdown + UT predict; GPS vel/pos + baro height UKF updates.
- Constraints, status flags, basic `UKF*` / `XUKF*` logging.
- Enable **one UKF core** on SITL (`UKF_ENABLE=1`, `UKF_IMU_MASK=1`, `AHRS_EKF_TYPE=4`).
- Autotest smoke: Plane takeoff + short FBWA/AUTO segment on UKF (see §12.2).

**Verify (gate 1)**

```sh
./waf configure --board sitl
./waf --targets tests/test_navukf_sigma
./waf plane
Tools/autotest/autotest.py build.Plane test.Plane.NavUKFSmoke
# optional truth compare once logging exists:
# Tools/autotest/autotest.py test.Plane.NavUKFSIMCompare
```

Pass criteria:

- All UKF unit tests green.
- Smoke test arms, flies, lands/disarms without EKF failsafe / NaN / lane collapse.
- If SIM compare enabled: attitude/vel/pos vs `SIM`/`SIM2` within §12.3 tolerances for primary UKF core (`C==0`).

### Phase 2 — Mag / yaw / airspeed

**Implement**

- Mag 3-axis and Euler yaw paths; optional GSF reuse.
- Airspeed + sideslip for Plane.
- Extend unit tests for mag/yaw measurement models; Plane autotest that exercises TAS fusion.

**Verify (gate 2)**

```sh
./waf --targets tests/test_navukf_sigma
Tools/autotest/autotest.py build.Plane test.Plane.NavUKFMagYaw test.Plane.NavUKFAirspeed
```

Pass criteria: no mag anomaly loops; yaw innovation ratios stay gated; airspeed fusion activates when TAS available; SIM attitude/yaw errors within tolerances.

### Phase 3 — Multi-core and robustness

**Implement**

- Second UKF core (`UKF_IMU_MASK` with ≥2 IMUs in SITL), lane switch, affinity, inactive bias learning.
- Vibration / bad-IMU mitigations ported from EKF3 control logic.
- Pre-arm check parity with EKF3 where applicable.

**Verify (gate 3)**

```sh
Tools/autotest/autotest.py build.Plane test.Plane.NavUKFMultiCore
# force lane switch / primary selection scenarios as defined in the test
```

Pass criteria: both cores initialise; forced lane switch updates AHRS without crash; `errorScore` / primary index behave sanely; reset counters reported to AHRS.

### Phase 4 — Optional aiding

**Implement**

- Optical flow, body odom, beacons, drag behind `AP_NavUKF_feature.h`.
- Feature-specific autotests only for enabled features.

**Verify (gate 4)**

```sh
# only for features compiled in:
Tools/autotest/autotest.py test.Copter.NavUKFOptFlow   # example
```

Pass criteria: feature-specific smoke green; default SITL build with features off still green.

### Phase 5 — Hardening, EKF3 comparison, performance

**Implement**

- Full autotest registration for Plane + Copter UKF suites.
- Offline EKF3↔UKF Replay/log compare tooling (§12.4).
- CPU/flash/perf harness (§12.5).
- Maintainer-facing parameter/wiki notes.

**Verify (gate 5)**

```sh
./waf --targets tests/test_navukf_sigma
Tools/autotest/autotest.py build.Plane test.Plane.NavUKF
Tools/autotest/autotest.py build.Copter test.Copter.NavUKF
# comparison + perf scripts (artifact-producing; see §12.4–12.5)
Tools/scripts/navukf_compare_ekf3.py --log <path> --out /tmp/navukf_compare.json
Tools/scripts/navukf_perf_report.py --board sitl --out /tmp/navukf_perf.json
```

Pass criteria: suites green; compare JSON within budgets; perf JSON within CPU/size budgets or explicitly documented as SITL-only.

---

## 12. Testing strategy

Testing is a **first-class deliverable**, not a late add-on. Prefer deterministic commands, numeric thresholds, and machine-readable artifacts so AI agents can run, parse, and stop on failure without human interpretation.

### 12.0 Principles (agent-friendly)

| Rule | Why |
|------|-----|
| Every check is a shell command with exit code 0/≠0 | Agents gate on process status |
| Thresholds live in code or JSON, not prose | Avoid “looks OK” reviews |
| Write artifacts under `/tmp/navukf_*` or `build/sitl/navukf_*` | Agents can `Read` JSON/logs |
| One primary UKF core first (`IMU_MASK=1`) | Smaller flaky surface |
| Never claim a test ran unless the command was executed | AGENTS.md / CoC |
| Keep EKF3 default; UKF tests set `AHRS_EKF_TYPE` explicitly | No silent behaviour change |
| Prefer extending existing helpers (`assert_ekfs_match_sim_state`, Replay) | Less bespoke harness drift |

Canonical agent loop per change:

1. Build affected targets.
2. Run unit tests.
3. Run the smallest autotest that covers the change.
4. If maths/perf touched: run compare and/or perf script; attach JSON summary to the PR notes.
5. Stop and fix on first red; do not skip gates.

### 12.1 Unit tests (GTest)

Location: `libraries/AP_NavUKF/tests/` (pattern: `libraries/AP_NavEKF/tests/test_ring_buffer.cpp`, `#include <AP_gtest.h>`).

Build/run:

```sh
./waf configure --board sitl
./waf --targets tests/test_navukf_sigma
# binary typically under build/sitl/tests/
```

**Required cases (Phase 1+)**

| Test | Assert |
|------|--------|
| Sigma weights sum | \(\sum W_m = 1\), \(\sum W_c\) consistent for chosen \(\alpha,\beta,\kappa\) |
| Cholesky / sqrtP | Fails cleanly on non-PSD input; succeeds on SPD fixture |
| Linear UT identity | For \(f(x)=Ax\), UT mean/cov matches analytic within tight eps |
| Additive process noise | After predict with \(Q\), diagonal grows by expected amount on fixture |
| Multiplicative attitude reset | Apply error rotation → quat updates → error state zeros; quat stays unit |
| \(P\) symmetry / PSD | After predict and after update, \(P \approx P^T\), diagonals ≥ floors |
| Sequential scalar update | Known linear measurement recovers state within eps |
| Gating | Innovation beyond gate does not update (or updates per policy under test) |
| `stateIndexLim` shrink | With wind/mag inhibited, sigma dimension and `P` blocks match |

**Later cases (Phase 2+)**

- Mag / yaw \(h(x)\) fixtures (body mag from earth field + attitude).
- Airspeed / sideslip measurement models.
- Constraint helpers do not NaN states.

Unit tests must be **HAL-light** (SITL/Linux), deterministic, and free of timing sleeps. No network. No SITL vehicle process.

### 12.2 SITL with a UKF core

**Minimum runtime config (single core)**

```text
AHRS_EKF_TYPE = 4          # UKF (final value TBD; keep consistent in tests)
UKF_ENABLE    = 1
UKF_IMU_MASK  = 1          # one core on IMU0
EK3_ENABLE    = 1          # keep EKF3 present for dual-log / fallback experiments
```

Manual bring-up (agent or human):

```sh
./waf configure --board sitl && ./waf plane
Tools/autotest/sim_vehicle.py -v ArduPlane -f plane --console --map \
  -P AHRS_EKF_TYPE=4 -P UKF_ENABLE=1 -P UKF_IMU_MASK=1
```

**Autotest naming (proposed)**

| Test | Vehicle | Intent |
|------|---------|--------|
| `NavUKFSmoke` | Plane, Copter | Arm, short mission / loiter, disarm; UKF healthy |
| `NavUKFSIMCompare` | Plane / Rover-style path | Extend `assert_ekfs_match_sim_state` to UKF log msgs |
| `NavUKFMagYaw` | Plane | Yaw stable under mag; no reset storm |
| `NavUKFAirspeed` | Plane | TAS fusion path exercised |
| `NavUKFMultiCore` | Plane SITL dual-IMU | Two cores + lane switch |
| `NavUKFReplay` | Plane | Replay bit: log → Replay tool → estimates finite / matched |

Wire via `Tools/autotest/arduplane.py` (and copter) like existing `Replay` / `SIMCompare`. Reuse `force_ahrs_type` plumbing in `vehicle_test_suite.py` so CI can run:

```sh
Tools/autotest/autotest.py build.Plane test.Plane.NavUKFSmoke --force-ahrs-type=4
```

(Extend `--force-ahrs-type` handling to set `UKF_ENABLE` / mask when type is UKF.)

**Smoke pass/fail (machine-checkable)**

- `AHRS: UKF active` (or equivalent) status text seen once.
- No `EKF`/`UKF` failsafe triggering landing/RTL unexpectedly.
- Primary core `C==0` log stream present for UKF estimate messages.
- Vehicle completes scripted path within timeout.
- Exit code 0 from autotest.

### 12.3 Accuracy vs simulator truth

Extend the existing helper used by Rover/Blimp `SIMCompare`:

- `VehicleTestSuite.assert_ekfs_match_sim_state()` already compares `NKF1`/`XKF1` to `SIM`/`SIM2` with sustained-violation logic (`vehicle_test_suite.py`).
- Add UKF message type(s) (e.g. `UKF1` / `XUKF1`) to `ekf_message_types`.
- Prefer **per-filter tolerance dict** already supported by that helper.

**Initial tolerance budgets (tunable; store in test)**

| Quantity | UKF vs SIM (armed, after settle) | Notes |
|----------|----------------------------------|-------|
| Roll/Pitch | ≤ 5 deg sustained | Match EKF3 helper defaults initially |
| Yaw | ≤ 10 deg sustained | Mag/GSF dependent |
| Velocity | ≤ 1.5–2.0 m/s | Loosen for Plane in wind |
| Pos NE | ≤ 5 m | After GPS aiding |
| Pos D | ≤ 3 m | Baro |

Settle time: ignore first N seconds after arm (`ignore_before_time_s`). Fail only on **sustained** violations (`max_violation_duration_s`), same as EKF3 compares.

Artifact: autotest progress log is enough; optional CSV dump of errors under `/tmp/navukf_sim_err.csv` for agent debugging.

### 12.4 EKF3 ↔ UKF comparison (same inputs)

Goal: on identical IMU/GPS/baro streams, UKF should not be wildly worse than EKF3.

**Method A — Dual enable, single flight (SITL)**

1. `EK3_ENABLE=1`, `UKF_ENABLE=1`, `AHRS_EKF_TYPE=4` (UKF flies the plane; EKF3 still updates if architecture allows passive run).
2. If passive EKF3-while-UKF-primary is **not** supported initially, use Method B.

**Method B — Replay (preferred for maths)**

1. Fly/generate a log with full sensor logging (`LOG_DISARMED` / Replay-friendly settings as in Plane `Replay` tests).
2. Run `build/sitl/tool/Replay` configured for EKF3 → log A.
3. Run Replay configured for UKF → log B.
4. Diff primary-core attitude/vel/pos and innovation ratios.

**Agent script contract (proposed)**

```sh
Tools/scripts/navukf_compare_ekf3.py --log path.bin --out /tmp/navukf_compare.json
```

JSON schema (illustrative):

```json
{
  "ok": true,
  "samples": 12000,
  "rms": {"roll_deg": 0.4, "pitch_deg": 0.5, "yaw_deg": 1.2, "pos_ne_m": 0.8, "vel_mps": 0.3},
  "budgets": {"roll_deg": 2.0, "pitch_deg": 2.0, "yaw_deg": 5.0, "pos_ne_m": 3.0, "vel_mps": 1.0},
  "ok_fields": ["roll_deg", "pitch_deg", "yaw_deg", "pos_ne_m", "vel_mps"]
}
```

Exit `0` iff `ok == true`. Agents parse `ok` / failed fields only.

### 12.5 Performance comparison

Measure cost relative to EKF3; do not guess.

| Metric | How | Budget (initial) |
|--------|-----|------------------|
| Flash / binary size | `./waf plane` build summary Text/Data for `arduplane` with UKF on vs off | Document delta; flag if SITL grows > agreed threshold (e.g. +15% Text) |
| CPU (SITL proxy) | Loop timing counters or `PM`/`PERF` logs over identical mission | UKF frame time ≤ ~3× EKF3 on same machine for single core, or documented |
| CPU (HW, later) | ChibiOS thread/loop load on representative H7 | Must leave control loops with margin; else keep UKF SITL-only |
| Memory | Sigma buffers + `P`; static accounting in code review + optional watermark | No large stack arrays; prefer core-scoped/static scratch |

**Agent script contract (proposed)**

```sh
Tools/scripts/navukf_perf_report.py --board sitl --out /tmp/navukf_perf.json
```

Produce on/off (or EKF3 vs UKF) size and a SITL timing summary. Exit ≠0 if budgets exceeded unless `--allow-overbudget` (explicit, for characterisation runs).

Characterisation runs are allowed to be over budget **only** if the PR/plan states “SITL-only; not for F4/F7”.

### 12.6 Build-matrix checks (CI-oriented)

| Config | Expect |
|--------|--------|
| SITL, UKF disabled (default) | Identical to today; Plane/Copter autotest subset green |
| SITL, UKF enabled | Unit tests + `NavUKFSmoke` green |
| Custom build server option `UKF` off | No UKF symbols required |
| Board without `HAL_NAVUKF_AVAILABLE` | Compiles |

Agent compile smoke:

```sh
./waf configure --board sitl && ./waf plane
./waf configure --board sitl --enable-UKF   # exact flag TBD via build_options
./waf plane && ./waf --targets tests/test_navukf_sigma
```

### 12.7 Verification checklist template (paste into PR)

```text
Verification
- [ ] Unit: ./waf --targets tests/test_navukf_sigma  (PASS/FAIL + log path)
- [ ] Build: ./waf plane (UKF config noted)
- [ ] SITL: autotest NavUKFSmoke (vehicle, commit SHA)
- [ ] Truth: NavUKFSIMCompare or assert_ekfs_match_sim_state (tolerances)
- [ ] EKF3 compare JSON: /tmp/navukf_compare.json (ok=true)
- [ ] Perf JSON: /tmp/navukf_perf.json (budgets or SITL-only waiver)
- [ ] AI-assisted: yes/no; tests listed above were actually executed
```

### 12.8 What not to do

- Do not replace or weaken EKF3 tests to make UKF look better.
- Do not run only `sim_vehicle` manually and call it CI coverage.
- Do not accept NaNs, non-finite quaternions, or silent `P` non-PSD without failing a test.
- Do not enable UKF by default in autotest global defaults.

---

## 13. Open design decisions (need maintainer input before merge)

1. **`AHRS_EKF_TYPE` value** for UKF (proposal: `4`).
2. **Error-state vs full-state** sigma points — this plan recommends error-state.
3. **Share `EK3_SRC*`** vs independent `UKF_SRC*` source parameters.
4. **Log message names** (`UKF1` vs `XUKF1`) and whether dual-logging EKF3+UKF is allowed in production builds.
5. Whether UKF may call into shared fusion helpers with EKF3 or must remain a fully separate tree to avoid coupling.
6. Acceptance bar for enabling on F4/F7 vs H7/SITL-only initially.
7. **Perf budgets**: concrete Text%/CPU× limits for merge vs “SITL-only” waiver policy.
8. Whether Replay should gain a first-class UKF backend in the same tool binary as EKF3.

---

## 14. Risks

- **CPU/flash**: largest practical risk; may force SITL-only or reduced-state forever on small MCUs. Mitigate with §12.5 gates.
- **Attitude mathematics**: incorrect quaternion/error-state handling will diverge silently; unit tests + Replay compare are mandatory.
- **Flaky SITL compares**: use sustained-violation logic already in `assert_ekfs_match_sim_state`, not instantaneous thresholds.
- **Parity drift**: cloning EKF3 then diverging control logic makes bugfixes hard; document intentional differences.
- **Safety**: navigation filters are flight-critical; UKF must not become default without extensive evidence and human maintainer ownership.
- **Duplication cost**: large code volume; prefer shared buffers/status/DAL, isolate only the estimator maths when possible.
- **Agent false confidence**: long `sim_vehicle` sessions without autotest assertions are insufficient; require exit-coded gates.

---

## 15. Suggested first concrete code step

After this plan is agreed:

1. Add `HAL_NAVUKF_AVAILABLE` (default 0) and empty `NavUKF` / `NavUKF_core` with EKF3-compatible public getters returning safe defaults.
2. Wire `AP_AHRS_NavUKF` and an `AHRS_EKF_TYPE` option behind the gate.
3. Add `tests/test_navukf_sigma` skeleton + Waf target (**gate 0**).
4. Port bootstrap init + strapdown only; still use a trivial \(P\) predict; keep unit tests compiling.
5. Introduce sigma-point predict; expand unit tests; compare \(P\) growth vs EKF3 on Replay when possible.
6. Add GPS/baro UKF updates; land `NavUKFSmoke` + optional `NavUKFSIMCompare` (**gate 1**).

---

## 16. Reference map (current tree)

| Path | Role |
|------|------|
| `libraries/AP_NavEKF3/` | Template implementation |
| `libraries/AP_NavEKF3/AP_NavEKF3_core.cpp` | `UpdateFilter`, `CovariancePrediction`, `FinishFusion` |
| `libraries/AP_NavEKF3/derivation/` | EKF equation generation (for behavioural comparison) |
| `libraries/AP_AHRS/AP_AHRS_NavEKF3.*` | Backend shim to clone |
| `libraries/AP_AHRS/AP_AHRS_config.h` | Availability macros |
| `libraries/AP_NavEKF/` | Shared buffers, status, sources, GSF |
| `libraries/AP_NavEKF/tests/` | GTest pattern to copy |
| `Tools/autotest/vehicle_test_suite.py` | `assert_ekfs_match_sim_state`, Replay helpers, `force_ahrs_type` |
| `Tools/autotest/rover.py` (`SIMCompare`) | Truth-compare mission pattern |
| `Tools/autotest/arduplane.py` (`Replay`) | Replay correctness pattern |
| `Tools/scripts/build_options.py` | Custom build server feature flags |

---

## 17. Summary

NavUKF should be an **architecture-level clone of EKF3** with an **error-state Unscented Transform** substituted for Jacobian covariance prediction and measurement linearisation. Ship it behind compile-time and parameter gates.

**Verification is part of the plan:** each phase has an exit-coded gate (unit tests → SITL UKF core smoke → SIM truth compare → EKF3 Replay compare → perf/size budgets). Harnesses should emit JSON/logs an AI agent can parse. Do not merge maths changes without green gates, and do not make UKF the default AHRS without maintainer-owned evidence.
