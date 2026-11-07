"""SAATHI - foot-mounted ZUPT-aided inertial navigation (offline, phone IMU).

Error-state EKF in the style of OpenShoe (Skog, Nilsson, Handel):
  nominal state: position p (n), velocity v (n), attitude R (body->nav)
  error state:   dp, dv, dtheta  (9 states)
  measurement:   v = 0 during detected stance phases (ZUPT)
Stance detection: SHOE / GLRT detector over a short window.
Nav frame: x, y horizontal (x = initial walking direction is NOT assumed;
yaw = 0 at start), z up.  Gravity = [0, 0, -g].

Input: a Sensor Logger (iOS) export directory.
  AccelerometerUncalibrated.csv  raw accelerometer in g (iOS sign: reads -1 g on
                                 the up axis at rest, i.e. minus specific force)
  Gyroscope.csv                  bias-calibrated rate in rad/s
  Barometer.csv                  pressure (hPa) and iOS relativeAltitude (m)
"""
import sys, json
import numpy as np
import pandas as pd

G = 9.80665


# ---------------------------------------------------------------- data
def load_run(path):
    acc = pd.read_csv(f"{path}/AccelerometerUncalibrated.csv")
    gyr = pd.read_csv(f"{path}/Gyroscope.csv")
    baro = pd.read_csv(f"{path}/Barometer.csv")
    t = gyr["seconds_elapsed"].to_numpy()
    w = gyr[["x", "y", "z"]].to_numpy()
    ta = acc["seconds_elapsed"].to_numpy()
    f = np.column_stack([np.interp(t, ta, -G * acc[c].to_numpy()) for c in "xyz"])
    return t, f, w, baro


# ---------------------------------------------------------------- helpers
def skew(v):
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])


def exp_so3(phi):
    th = np.linalg.norm(phi)
    if th < 1e-12:
        return np.eye(3) + skew(phi)
    k = skew(phi / th)
    return np.eye(3) + np.sin(th) * k + (1 - np.cos(th)) * k @ k


def init_attitude(f_mean):
    """Roll/pitch from mean specific force at rest, yaw = 0."""
    ax, ay, az = f_mean
    roll = np.arctan2(ay, az)
    pitch = np.arctan2(-ax, np.hypot(ay, az))
    cr, sr, cp, sp = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    return Ry @ Rx  # body -> nav


def glrt_detector(f, w, win=5, sig_a=0.01, sig_w=0.1 * np.pi / 180, gamma=3e4):
    """SHOE stance detector. Returns boolean array (True = foot stationary)."""
    n = len(f)
    T = np.full(n, np.inf)
    h = win // 2
    for k in range(h, n - h):
        fs, ws = f[k - h:k + h + 1], w[k - h:k + h + 1]
        fm = fs.mean(0)
        u = fm / np.linalg.norm(fm)
        T[k] = (np.sum((fs - G * u) ** 2) / sig_a**2 + np.sum(ws**2) / sig_w**2) / win
    return T < gamma, T


# ---------------------------------------------------------------- filter
def run_ins(t, f, w, zupt, init_still=3.0,
            sig_acc=0.5, sig_gyro=0.5 * np.pi / 180, sig_vel=0.01):
    n = len(t)
    k0 = np.searchsorted(t, t[0] + init_still)
    R = init_attitude(f[:k0].mean(0))
    gbias = w[:k0].mean(0)  # residual gyro bias from the initial still period
    p = np.zeros(3); v = np.zeros(3)
    P = np.diag([1e-6] * 3 + [1e-6] * 3 + [(0.1 * np.pi / 180) ** 2] * 3)
    H = np.zeros((3, 9)); H[:, 3:6] = np.eye(3)
    Rm = np.eye(3) * sig_vel**2
    g_n = np.array([0, 0, -G])
    pos = np.zeros((n, 3)); vel = np.zeros((n, 3))
    for k in range(1, n):
        dt = t[k] - t[k - 1]
        wk = w[k] - gbias
        R = R @ exp_so3(wk * dt)
        a_n = R @ f[k] + g_n
        v = v + a_n * dt
        p = p + v * dt
        F = np.eye(9)
        F[0:3, 3:6] = np.eye(3) * dt
        F[3:6, 6:9] = -skew(R @ f[k]) * dt
        Gm = np.zeros((9, 6)); Gm[3:6, 0:3] = R * dt; Gm[6:9, 3:6] = -R * dt
        Q = np.diag([sig_acc**2] * 3 + [sig_gyro**2] * 3)
        P = F @ P @ F.T + Gm @ Q @ Gm.T
        if zupt[k]:
            S = H @ P @ H.T + Rm
            K = P @ H.T @ np.linalg.inv(S)
            dx = K @ (-v)
            p = p + dx[0:3]
            v = v + dx[3:6]
            R = exp_so3(dx[6:9]) @ R
            P = (np.eye(9) - K @ H) @ P
            P = (P + P.T) / 2
        pos[k] = p; vel[k] = v
    return pos, vel


def find_start(t, w, f, min_still=5.0, thr_w=0.15, thr_f=0.3):
    """End of the first still stretch >= min_still s (the 20 s stand after mounting).
    Returns (k_align_start, k_nav_start)."""
    dt = np.median(np.diff(t)); n1 = int(1.0 / dt)
    wn = np.linalg.norm(w, axis=1); fn = np.linalg.norm(f, axis=1)
    wmax = pd.Series(wn).rolling(n1, center=True, min_periods=1).max().to_numpy()
    fstd = pd.Series(fn).rolling(n1, center=True, min_periods=1).std().fillna(0).to_numpy()
    still = (wmax < thr_w) & (fstd < thr_f)
    run = 0
    for k in range(len(t)):
        run = run + 1 if still[k] else 0
        if run * dt >= min_still and (k + 1 == len(t) or not still[k + 1]):
            return max(0, k - int(3.0 / dt)), k
    raise RuntimeError("no still period found")


def count_steps(zupt):
    """Number of stance->swing transitions (one per stride of the shod foot)."""
    z = zupt.astype(int)
    return int(np.sum((z[1:] == 0) & (z[:-1] == 1)))


if __name__ == "__main__":
    run = sys.argv[1]
    t, f, w, baro = load_run(run)
    k_a, k_s = find_start(t, w, f)
    t, f, w = t[k_a:], f[k_a:], w[k_a:]
    zupt, T = glrt_detector(f, w)
    pos, vel = run_ins(t, f, w, zupt)
    out = {
        "duration_s": float(t[-1] - t[0]),
        "strides": count_steps(zupt),
        "end_xyz_m": pos[-1].round(3).tolist(),
        "max_height_m": float(pos[:, 2].max()),
        "path_length_m": float(np.sum(np.linalg.norm(np.diff(pos, axis=0), axis=1))),
    }
    print(json.dumps(out, indent=1))
