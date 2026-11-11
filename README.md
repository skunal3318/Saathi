# SAATHI – preliminary foot-mounted INS results

Offline test of the SAATHI navigation method (IEEE MYOSA 7.0 proposal) with a phone IMU,
before the MYOSA kit arrives.

## Setup
- iPhone 14 strapped flat on the right instep, recorded with the Sensor Logger app at 100 Hz
  (accelerometer, gyroscope, barometer).
- Two stair runs, `data/c1` and `data/c2`: stand still ~20 s, climb ground → 1st floor
  (8 steps, landing, 8 steps), pause at the top, walk back down to the start mark.
- Floor height: 2.64 m **nominal** (16 risers × 6.5 in); still to be re-measured with a tape.
- `data/demo` is a short desk recording used only to check the logging setup; the analysis
  does not use it.
- The phone's `device id` field has been removed from every `Metadata.csv`.

## Method (`code/saathi_zupt.py`)
- 9-state error-state EKF (position, velocity, attitude) with zero-velocity updates (ZUPT).
  The planned firmware uses a 15-state filter (adds gyro and accelerometer bias); here the
  gyro bias is taken from the start-up still period.
- SHOE/GLRT stance detector with the OpenShoe default parameters (γ = 3×10⁴), not tuned to
  these runs.
- Navigation starts from the first still period of ≥ 5 s (the phone was still being strapped
  on at the start of each recording).
- The arrival point is the last foot-flat period lasting ≥ 0.8 s; the recordings end while the
  phone is being unstrapped, and those later samples are excluded.

## Run
```
pip install -r requirements.txt
python code/analyse.py code data results
```

## Results (`results/metrics.json`)

| Run | Walk | Return to start (3-D) | Floor height, foot INS | Floor height, barometer (on foot) |
|---|---|---|---|---|
| C1 | 16.0 m | 0.73 m (4.6%) | 3.05 m | 2.77 m |
| C2 | 16.2 m | 0.24 m (1.5%) | 3.25 m | 2.84 m |

Sensitivity of the INS floor height to the stance-detector threshold γ:

| γ | C1 | C2 |
|---|---|---|
| 1×10⁴ | 3.06 m | 3.13 m |
| 3×10⁴ (default) | 3.05 m | 3.25 m |
| 1×10⁵ | 2.70 m | 2.81 m |

## Limitations
- Two runs only: evidence, not statistics. More runs will be recorded; γ will be tuned on some
  and results reported on the rest.
- INS height depends strongly on detector tuning, which is why SAATHI uses a barometer to
  confirm floor changes.
- The barometer here was on the foot. In SAATHI it is on the wrist (arm movement), which is
  not tested yet, and the iPhone barometer is quieter than the BMP180 in the MYOSA kit.
- A phone IMU is better than the MPU6050 in the MYOSA kit, so these numbers validate the
  method, not the final hardware accuracy.

## Licence
MIT – see `LICENSE`.
