"""Builds the pot figures from data/pot_cal.csv.
Usage (from the pot_servo folder):  python3 scripts/make_figures.py
Outputs: figures/pot_transfer_function.png, figures/pot_error_loo.png
Fit uses the linear region 80-300 deg; error is estimated by leave-one-out validation."""
import numpy as np, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

d = np.loadtxt("data/pot_cal.csv", delimiter=",", skiprows=1)
a, adc, sd = d[:, 0], d[:, 1], d[:, 2]
m = (a >= 80) & (a <= 300)
k, c = np.polyfit(a[m], adc[m], 1)
print(f"Transfer function: ADC = {k:.3f}*angle {c:+.1f}  ->  angle = {1/k:.5f}*ADC + {-c/k:.3f}")

plt.figure(figsize=(7.5, 5))
plt.errorbar(a[m], adc[m], yerr=sd[m], fmt="o", capsize=3, label="measured, linear region (used in fit)")
plt.errorbar(a[~m], adc[~m], yerr=sd[~m], fmt="s", color="gray", capsize=3, label="outside linear region (end dead zones)")
xs = np.linspace(60, 320, 50)
plt.plot(xs, k * xs + c, "-", label=f"fit: ADC = {k:.3f}·θ {c:+.1f}")
plt.xlabel("Shaft angle θ (deg)"); plt.ylabel("ADC counts (0–1023)")
plt.title("Bourns 3386 potentiometer – transfer function (3.3 V supply)")
plt.grid(alpha=.4); plt.legend(); plt.tight_layout()
plt.savefig("figures/pot_transfer_function.png", dpi=150)

aa, dd = a[m], adc[m]
err = []
for i in range(len(aa)):
    keep = np.ones(len(aa), bool); keep[i] = False
    s, cc = np.polyfit(aa[keep], dd[keep], 1)
    err.append((dd[i] - cc) / s - aa[i])
err = np.array(err)
print(" true  error");  [print(f"{t:5.0f}  {e:+5.1f}") for t, e in zip(aa, err)]
print(f"Leave-one-out: max |error| = {abs(err).max():.1f} deg, RMS = {np.sqrt((err**2).mean()):.1f} deg")

plt.figure(figsize=(7.5, 3.5))
plt.axhline(0, color="k", lw=.8)
plt.bar(aa, err, width=8)
plt.xlabel("True angle (deg)"); plt.ylabel("Error (deg)")
plt.title(f"Pot conversion error (leave-one-out): max {abs(err).max():.1f}°, RMS {np.sqrt((err**2).mean()):.1f}°")
plt.grid(alpha=.4, axis="y"); plt.tight_layout()
plt.savefig("figures/pot_error_loo.png", dpi=150)
