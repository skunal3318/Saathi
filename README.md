# SAATHI – preliminary foot-mounted INS results

Offline test of the SAATHI navigation method (MYOSA 7.0 proposal) using a phone IMU
before the MYOSA kit arrives.

**Setup:** iPhone 14 strapped flat on the right instep, Sensor Logger app, 100 Hz
(accelerometer, gyroscope, barometer). Two stair runs (`data/c1`, `data/c2`): stand still
~20 s, climb ground → 1st floor (8 steps, landing, 8 steps; riser 6.5 in, so 2.64 m),
pause, walk back down to the start mark. `data/demo` is a desk test of the recording setup.

**Method (`code/saathi_zupt.py`):** 9-state error-state EKF with zero-velocity updates
(ZUPT), SHOE/GLRT stance detector with the OpenShoe default parameters (not tuned to
these runs), roll/pitch alignment from the still period, yaw = 0 at start.

**Run:**
```
pip install numpy pandas matplotlib
python code/analyse.py code data results
```

**Results (`results/metrics.json`):**

| Run | INS floor height | Barometer | Measured | 3-D closing error |
|---|---|---|---|---|
| C1 | 3.05 m | 2.77 m | 2.64 m | 1.34 m |
| C2 | 3.25 m | 2.84 m | 2.64 m | 0.25 m |

The INS over-reads height by 15–23%; the barometer is closer, which supports using it to
confirm floor changes. A phone IMU is better than the MPU6050 in the MYOSA kit, so these
numbers validate the method, not the final hardware accuracy.
