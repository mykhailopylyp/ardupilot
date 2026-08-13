# NavUKF UT tune / validation

Tune UKF_ALPHA/BETA/KAPPA on held-in complex profiles; score = mean(UKF_primary/EKF_primary) over att/pos/vel. Validate on held-out profiles with winning params. Medium GPS; UKF_USE_UT=1.

## Winning params (from tune set)

| Param | Value |
|------:|------:|
| UKF_ALPHA | 0.35 |
| UKF_BETA | 2 |
| UKF_KAPPA | 0 |
| Tune mean UKF/EKF score | 0.9433 |

## Tune combo ranking

| ALPHA | BETA | KAPPA | mean score | failed |
|------:|-----:|------:|-----------:|:------:|
| 0.35 | 2 | 0 | 0.9433 | False |
| 0.45 | 2 | 0 | 0.9439 | False |
| 0.55 | 2 | 0 | 0.9512 | False |
| 0.4 | 2 | 3 | 0.9550 | False |
| 0.5 | 0 | 0 | 0.9577 | False |
| 0.5 | 2 | 0 | 0.9640 | False |

## Validation (held-out)

| Profile | EKF att | UKF att | EKF pos | UKF pos | EKF vel | UKF vel | score | UKF better |
|---------|--------:|--------:|--------:|--------:|--------:|--------:|------:|:----------:|
| val_mixed | 10.8823 | 3.8489 | 2.9610 | 2.6678 | 1.0944 | 1.1561 | 0.7704 | True |
| val_endurance | 1.2165 | 1.2515 | 3.9506 | 4.0020 | 0.5915 | 0.6016 | 1.0196 | False |

Validation mean UKF/EKF score: **0.8950** (UKF better overall: True)
