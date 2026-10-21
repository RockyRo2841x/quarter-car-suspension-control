"""Independent equation-level comparison; this does not execute Simulink.

Parameters and controller settings are read from the original SLX archives.
Both systems receive identical road signals and start at zero equilibrium.
"""

import argparse
import csv
import hashlib
import json
from pathlib import Path
import re
import zipfile
import xml.etree.ElementTree as ET

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import solve_ivp

ROOT = Path(__file__).resolve().parents[1]


def read_model(path):
    with zipfile.ZipFile(path) as archive:
        if archive.testzip() is not None:
            raise ValueError(f"Invalid archive: {path}")
        scripts = []
        for name in archive.namelist():
            if name.startswith("simulink/stateflow/") and name.endswith(".xml"):
                tree = ET.fromstring(archive.read(name))
                scripts.extend(p.text for p in tree.iter("P") if p.get("Name") == "script")
        script = next(s for s in scripts if "xdd" in s and "zdd" in s)
        parameters = {
            key: float(re.search(rf"\b{key}\s*=\s*([\d.]+)\s*;", script).group(1))
            for key in ("ms", "mu", "k", "b", "kt")
        }
        tree = ET.fromstring(archive.read("simulink/systems/system_root.xml"))
        pid = next((b for b in tree.iter("Block") if b.get("Name") == "PID Controller"), None)
        settings = {} if pid is None else {p.get("Name"): p.text for p in pid.findall("./InstanceData/P")}
    return parameters, settings


def force(state, gains):
    if gains is None:
        return np.zeros_like(state[0])
    kp, ki, kd, n = gains
    error = -state[0]
    return kp * error + ki * state[4] + kd * n * (error - state[5])


def derivative(t, state, p, gains, road):
    x, xd, z, zd = state[:4]
    fa = force(state, gains)
    coupling = p["b"] * (xd - zd) + p["k"] * (x - z)
    result = [xd, (-coupling + fa) / p["ms"], zd,
              (coupling - p["kt"] * (z - road(t)) - fa) / p["mu"]]
    if gains is not None:
        error = -x
        result.extend([error, gains[3] * (error - state[5])])
    return result


def simulate(p, gains, road, times, max_step):
    count = 4 if gains is None else 6
    result = solve_ivp(
        derivative, (times[0], times[-1]), np.zeros(count), t_eval=times,
        args=(p, gains, road), rtol=1e-8, atol=1e-10, max_step=max_step,
    )
    if not result.success or not np.isfinite(result.y).all():
        raise RuntimeError(result.message)
    acceleration = np.asarray([
        derivative(t, state, p, gains, road)[1]
        for t, state in zip(times, result.y.T)
    ])
    return result.y, acceleration, force(result.y, gains)


def rms(values):
    return float(np.sqrt(np.mean(np.square(values))))


def metrics(states, acceleration, fa, road_values):
    return {
        "body_displacement_rms_m": rms(states[0]),
        "body_displacement_peak_abs_m": float(np.max(np.abs(states[0]))),
        "body_acceleration_rms_m_per_s2": rms(acceleration),
        "body_acceleration_peak_abs_m_per_s2": float(np.max(np.abs(acceleration))),
        "suspension_travel_peak_abs_m": float(np.max(np.abs(states[0] - states[2]))),
        "tire_deflection_peak_abs_m": float(np.max(np.abs(states[2] - road_values))),
        "actuator_force_rms_N": rms(fa),
        "actuator_force_peak_abs_N": float(np.max(np.abs(fa))),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "results")
    args = parser.parse_args()
    passive_path = ROOT / "models/quarter_car_passive.slx"
    active_path = ROOT / "models/quarter_car_active.slx"
    p, _ = read_model(passive_path)
    active_p, settings = read_model(active_path)
    if p != active_p:
        raise ValueError("Original vehicle parameters differ")
    expected = {"Form": "Parallel", "TimeDomain": "Continuous-time", "UseFilter": "on", "LimitOutput": "off"}
    if any(settings.get(k) != v for k, v in expected.items()):
        raise ValueError("Controller configuration is not supported by this reproduction")
    gains = tuple(float(settings[k]) for k in ("P", "I", "D", "N"))
    times = np.linspace(0, 10, 5001)
    roads = {"step_50mm_at_1s": lambda t: 0.05 if t >= 1 else 0.0,
             "sine_30mm_1rad_per_s": lambda t: 0.03 * np.sin(t)}

    # Independent invariants: zero equilibrium and the no-actuation limit.
    assert np.allclose(derivative(0, np.zeros(6), p, gains, lambda t: 0), 0)
    probe = np.array([0.01, -0.2, -0.03, 0.4, 0.0, 0.0])
    assert np.allclose(derivative(0, probe, p, (0, 0, 0, gains[3]), lambda t: 0)[:4],
                       derivative(0, probe[:4], p, None, lambda t: 0))

    args.output.mkdir(parents=True, exist_ok=True)
    summary = {"method": "Independent SciPy integration of equations and PID settings extracted from SLX; not Simulink execution",
               "parameters": p, "pid": dict(zip(("Kp", "Ki", "Kd", "N"), gains)),
               "duration_s": 10, "sample_interval_s": 0.002, "metrics_window": "full 0–10 seconds",
               "source_sha256": {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in (passive_path, active_path)},
               "scenarios": {}, "max_displacement_difference_on_step_refinement_m": 0.0}
    fig, axes = plt.subplots(2, 2, figsize=(12, 7.5), sharex="col", constrained_layout=True)
    for col, (name, road) in enumerate(roads.items()):
        road_values = np.array([road(t) for t in times])
        series = {"time_s": times, "road_m": road_values}
        summary["scenarios"][name] = {}
        for label, controller in (("passive", None), ("active", gains)):
            states, accel, fa = simulate(p, controller, road, times, 0.005)
            refined, _, _ = simulate(p, controller, road, times, 0.0025)
            difference = float(np.max(np.abs(states[[0, 2]] - refined[[0, 2]])))
            if difference > 1e-6:
                raise AssertionError(f"Displacement convergence check failed: {difference}")
            summary["max_displacement_difference_on_step_refinement_m"] = max(
                summary["max_displacement_difference_on_step_refinement_m"], difference)
            summary["scenarios"][name][label] = metrics(states, accel, fa, road_values)
            axes[0, col].plot(times, states[0] * 1000, label=label.capitalize(), linewidth=1.6)
            axes[1, col].plot(times, accel, label=label.capitalize(), linewidth=1.3)
            for suffix, data in (("body_m", states[0]), ("wheel_m", states[2]),
                                 ("body_acceleration_m_per_s2", accel), ("force_N", fa)):
                series[f"{label}_{suffix}"] = data
        axes[0, col].plot(times, road_values * 1000, ":", label="Road", color="0.45")
        axes[0, col].set_title("50 mm step at 1 s" if col == 0 else "30 mm sine, 1 rad/s")
        axes[1, col].set_xlabel("Time (s)")
        with (args.output / f"{name}.csv").open("w", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(series)
            writer.writerows(zip(*series.values()))
    for ax in axes.flat:
        ax.grid(True, alpha=0.25)
        ax.legend(loc="best", fontsize=8)
    axes[0, 0].set_ylabel("Body displacement (mm)")
    axes[1, 0].set_ylabel("Body acceleration (m/s²)")
    fig.suptitle("Quarter-car suspension: identical inputs and zero initial states\nIndependent equation-level reproduction • unlimited ideal actuator", fontsize=12)
    fig.savefig(args.output / "comparison.png", dpi=160)
    plt.close(fig)
    (args.output / "metrics.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
