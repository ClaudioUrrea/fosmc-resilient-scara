"""Graphical abstract (revised version R1) for 'Fractional-Order Sliding-Mode Control of a Rail-Mounted Redundant
SCARA Under Actuator Degradation and Elbow Locking' (Fractal Fract.). All numbers are taken from Tables 5 and 8, Section 3.6 and
Section 3.7 of the revised manuscript (v1.1.1). Usage: python GA_Fractal_and_Fractional.py"""
import numpy as np, matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Rectangle, Circle, Polygon, Arc, FancyArrowPatch
from matplotlib import font_manager
import os, glob
# ------------------------------------------------------------------ results shown (manuscript v1.1.1)
RMSE = {'S1': {'ISMC-J': 0.031, 'FOSMC-J': 0.016},        # mm, 20 evaluation runs, jointly tuned pair (Table 5)
        'S2': {'ISMC-J': 0.079, 'FOSMC-J': 0.026}}
GAIN = {'S1': 49, 'S2': 67}                               # % RMSE reduction of FOSMC-J vs ISMC-J (p = 1.9e-6)
POST = {'gain': '−8 to −10%', 'fail': '0–2 of 100'}              # post-lock RMSE reduction FOSMC-J vs ISMC-J and failures, gain schedule at reconfiguration (Section 3.6)
DELAY = {'ISMC-A': 7, 'FOSMC': 5}                         # tolerable input delay in ms, equal gains, 20 runs (Figure 12)
TOTAL_RUNS = '≈ 5600'
# ------------------------------------------------------------------ fonts (Arial on Windows/macOS, Liberation Sans on Linux)
CANDIDATES = [
    ('Arial', [r'C:\Windows\Fonts\arial.ttf', r'C:\Windows\Fonts\arialbd.ttf',
               r'C:\Windows\Fonts\ariali.ttf', r'C:\Windows\Fonts\arialbi.ttf',
               '/Library/Fonts/Arial.ttf', '/System/Library/Fonts/Supplemental/Arial.ttf']),
    ('Liberation Sans', glob.glob('/usr/share/fonts/truetype/liberation/LiberationSans-*.ttf')),
]
FONT = 'DejaVu Sans'
for name, files in CANDIDATES:
    found = [f for f in files if os.path.isfile(f)]
    if found:
        for f in found: font_manager.fontManager.addfont(f)
        FONT = name; break
print('Font used:', FONT)
matplotlib.rcParams.update({'font.family': FONT, 'mathtext.fontset': 'custom', 'mathtext.rm': FONT,
                            'mathtext.it': FONT + ':italic', 'mathtext.bf': FONT + ':bold', 'axes.unicode_minus': True})
RED, BLUE, GREEN, GREY, DARK = '#d62728', '#1f77b4', '#2ca02c', '#f4f5f7', '#1d2b3a'
W, H = 11.0, 5.6
fig = plt.figure(figsize=(W, H), dpi=250)
ax = fig.add_axes([0, 0, 1, 1]); ax.set_xlim(0, W); ax.set_ylim(0, H); ax.axis('off')

def panel(x, y, w, h, title, color):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle='round,pad=0.02,rounding_size=0.12', fc=GREY, ec='#c9ced6', lw=1.2))
    ax.add_patch(FancyBboxPatch((x, y + h - 0.46), w, 0.46, boxstyle='round,pad=0.02,rounding_size=0.12', fc=color, ec='none'))
    ax.add_patch(Rectangle((x - 0.02, y + h - 0.46), w + 0.04, 0.2, fc=color, ec='none'))
    ax.text(x + w / 2, y + h - 0.23, title, ha='center', va='center', color='white', fontsize=13.5, fontweight='bold')

ax.text(W / 2, 5.36, 'Fractional-order sliding-mode control of a redundant rail-mounted SCARA under actuator faults',
        ha='center', va='center', fontsize=14.6, fontweight='bold', color=DARK)
PY, PH = 0.62, 4.42
panel(0.18, PY, 3.40, PH, 'Redundant robot under faults', '#44546a')
panel(3.80, PY, 3.40, PH, 'Fractional control + analysis', RED)
panel(7.42, PY, 3.40, PH, 'Fair benchmark: key results', '#2f6f4f')
for x0 in (3.60, 7.22):
    ax.add_patch(FancyArrowPatch((x0 - 0.02, PY + PH / 2), (x0 + 0.22, PY + PH / 2), arrowstyle='simple,head_width=10,head_length=10', color='#8a94a6', lw=0))

# ---------------- panel A: robot drawing
ox, oy = 0.45, 1.55
t = np.linspace(0, 2 * np.pi, 400); cx, cy = 1.88, 3.92
ax.plot(cx + 1.30 * np.sin(t), cy + 0.24 * np.sin(2 * t), '--', color=GREEN, lw=1.8)
ax.text(cx, cy + 0.43, 'Dispensing contour', ha='center', fontsize=10.5, color=GREEN)
ax.add_patch(FancyBboxPatch((ox, oy), 2.85, 0.2, boxstyle='round,pad=0.01,rounding_size=0.05', fc='#b8bec8', ec='#6b7280', lw=1))
ax.add_patch(Rectangle((ox - 0.06, oy - 0.02), 0.08, 0.3, fc=RED, ec='none')); ax.add_patch(Rectangle((ox + 2.83, oy - 0.02), 0.08, 0.3, fc=RED, ec='none'))
cxr = 1.35; ax.add_patch(FancyBboxPatch((cxr - 0.3, oy + 0.2), 0.6, 0.2, boxstyle='round,pad=0.01,rounding_size=0.04', fc='#f39c12', ec='#b9770e', lw=1))
ax.annotate('', xy=(cxr + 0.75, oy - 0.25), xytext=(cxr - 0.1, oy - 0.25), arrowprops=dict(arrowstyle='->', lw=1.4, color=DARK))
ax.text(cxr + 0.32, oy - 0.5, 'Rail $d$', ha='center', fontsize=10.5, color=DARK)
J1 = np.array([cxr, oy + 0.40]); L1, L2 = 1.20, 0.95
t0 = -0.305; Tt = np.array([cx + 1.30 * np.sin(t0), cy + 0.24 * np.sin(2 * t0)])
dv = Tt - J1; D = np.hypot(*dv); c2 = (D ** 2 - L1 ** 2 - L2 ** 2) / (2 * L1 * L2); th2 = np.arccos(np.clip(c2, -1, 1))
th1 = np.arctan2(dv[1], dv[0]) - np.arctan2(L2 * np.sin(th2), L1 + L2 * np.cos(th2))
E = J1 + L1 * np.array([np.cos(th1), np.sin(th1)]); T = E + L2 * np.array([np.cos(th1 + th2), np.sin(th1 + th2)])
ax.plot(*zip(J1, E), color='#3b6fb6', lw=11, solid_capstyle='round'); ax.plot(*zip(E, T), color='#6fb1e6', lw=9, solid_capstyle='round')
for P in (J1, E): ax.add_patch(Circle(P, 0.09, fc='white', ec=DARK, lw=1.6, zorder=5))
ax.add_patch(Circle(T, 0.07, fc=DARK, ec='white', lw=1.2, zorder=6))
ax.text(T[0] - 0.12, T[1] - 0.26, 'Tool', ha='right', fontsize=9.5, color=DARK)
lx, ly = J1[0] - 0.62, J1[1] + 0.35
ax.add_patch(Polygon([(lx, ly + 0.34), (lx + 0.18, ly + 0.34), (lx + 0.09, ly + 0.14), (lx + 0.22, ly + 0.14), (lx - 0.02, ly - 0.2), (lx + 0.06, ly + 0.06), (lx - 0.06, ly + 0.06)], fc='#f1c40f', ec='#b7950b', lw=1))
ax.text(lx + 0.08, ly - 0.42, '50% LOE\non $\\theta_1$', ha='center', fontsize=10, color=DARK)
px, py = E[0] + 0.2, E[1] - 0.02
ax.add_patch(FancyBboxPatch((px, py), 0.3, 0.24, boxstyle='round,pad=0.01,rounding_size=0.03', fc=RED, ec='#8e1b1b', lw=1, zorder=6))
ax.add_patch(Arc((px + 0.15, py + 0.24), 0.2, 0.26, theta1=0, theta2=180, lw=2.2, color='#8e1b1b', zorder=6))
ax.text(px + 0.15, py - 0.28, 'Elbow lock', ha='center', va='center', fontsize=10, color=DARK)
ax.text(1.88, 0.83, 'P–R–R chain: one redundant DOF\nfor a planar task (650-mm-reach SCARA)', ha='center', va='center', fontsize=9.5, color='#374151')

# ---------------- panel B
bx = 3.80 + 1.70
ax.text(bx, 4.43, r'$s=\dot{e}+\Lambda_1 e+\Lambda_2\, I^{\lambda}e,\ \ \lambda\in(0,1]$', ha='center', va='center', fontsize=15, color=DARK)
ax.text(bx, 4.06, r'$\lambda=1$: integer ISMC  ·  $\lambda<1$: FOSMC', ha='center', va='center', fontsize=10, color='#374151')
items = [('Truncated-memory analysis', 'implemented GL sum = exact truncated\nkernel; stable for every memory length'),
         ('Ultimate boundedness under LOE', 'scope stated; corollary for the reduced\nchain after reconfiguration'),
         ('Fault-aware redundancy', 'post-lock feasibility condition\n$\\rightarrow$ null-space rule for $\\theta_2^{\\star}$'),
         ('Momentum-observer FDI', 'all joints monitored; lock isolated in\n≈ 29 ms; no missed detection')]
y = 3.62
for i, (h, s) in enumerate(items):
    ax.add_patch(FancyBboxPatch((3.98, y - 0.62), 3.04, 0.6, boxstyle='round,pad=0.01,rounding_size=0.07', fc='white', ec='#e3b3b3', lw=1))
    ax.add_patch(Circle((4.2, y - 0.32), 0.13, fc=RED, ec='none')); ax.text(4.2, y - 0.32, str(i + 1), ha='center', va='center', color='white', fontsize=10, fontweight='bold')
    ax.text(4.42, y - 0.17, h, ha='left', va='center', fontsize=10.5, fontweight='bold', color=DARK)
    ax.text(4.42, y - 0.44, s, ha='left', va='center', fontsize=8.6, color='#374151', linespacing=1.0)
    y -= 0.71

# ---------------- panel C
ax2 = fig.add_axes([0.725, 0.46, 0.225, 0.30])
sc = ['S1', 'S2']; w = 0.34; xs = np.arange(2)
vi = [RMSE[s]['ISMC-J'] for s in sc]; vf = [RMSE[s]['FOSMC-J'] for s in sc]
ax2.bar(xs - w / 2, vi, w, color=BLUE, label='ISMC-J'); ax2.bar(xs + w / 2, vf, w, color=RED, label='FOSMC-J')
for i in range(2):
    ax2.text(xs[i] - w / 2, vi[i] + 0.002, f'{vi[i]:.3f}', ha='center', va='bottom', fontsize=8.6, color=DARK)
    ax2.text(xs[i] + w / 2, vf[i] + 0.002, f'{vf[i]:.3f}', ha='center', va='bottom', fontsize=8.6, color=DARK)
    ax2.text(xs[i] + 0.02, 0.094 if i else 0.050, f'−{GAIN[sc[i]]}%', ha='center', fontsize=11.5, fontweight='bold', color=RED)
ax2.set_xticks(xs); ax2.set_xticklabels(['No fault', '50% LOE on $\\theta_1$'], fontsize=8.8)
ax2.set_ylim(0, 0.115); ax2.set_yticks([0, 0.04, 0.08]); ax2.tick_params(axis='y', labelsize=8.5)
ax2.set_ylabel('RMSE (mm)', fontsize=9.5); ax2.legend(fontsize=8.2, frameon=False, loc='upper left', bbox_to_anchor=(0.0, 1.02))
ax2.set_title('Tuned pair, equal activity budget', fontsize=10.2, fontweight='bold', color=DARK, pad=4)
for s_ in ('top', 'right'): ax2.spines[s_].set_visible(False)
ax2.set_facecolor(GREY)

def tile(x, y, big, small, col):
    ax.add_patch(FancyBboxPatch((x, y), 1.58, 1.0, boxstyle='round,pad=0.01,rounding_size=0.08', fc='white', ec=col, lw=1.4))
    ax.text(x + 0.79, y + 0.78, big, ha='center', va='center', fontsize=12.5, fontweight='bold', color=col)
    ax.text(x + 0.79, y + 0.36, small, ha='center', va='center', fontsize=7.8, color='#374151', linespacing=1.15)
tile(7.54, 0.72, f"{POST['gain']}", f"post-lock RMSE with a gain\nschedule at reconfiguration\n({POST['fail']} failures)", RED)
tile(9.20, 0.72, f"{DELAY['FOSMC']} vs {DELAY['ISMC-A']} ms", 'tolerable input delay,\nthe cost of the order\n(FOSMC vs ISMC-A)', '#2f6f4f')

ax.add_patch(FancyBboxPatch((0.18, 0.1), 10.64, 0.38, boxstyle='round,pad=0.01,rounding_size=0.08', fc=DARK, ec='none'))
ax.text(W / 2, 0.29, f'{TOTAL_RUNS} simulation runs  ·  Python ≡ MATLAB (≤ 10$^{{−15}}$)  ·  RoboDK digital twin (0.001 mm)  ·  Open code and data (MIT)',
        ha='center', va='center', color='white', fontsize=10.2)
fig.savefig('Graphical_Abstract.png', dpi=250, facecolor='white')
from PIL import Image                      # MDPI: RGB, no transparency
im = Image.open('Graphical_Abstract.png').convert('RGB')
im.save('Graphical_Abstract.png', dpi=(250, 250))
im.save('Graphical_Abstract.tiff', compression='tiff_lzw', dpi=(250, 250))
print('ok')
