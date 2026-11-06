"""Render the user-supplied actuator comparison as a static English PNG."""
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

ROWS = [
    ('AKE40-6', 140, 1.8, 5.5),
    ('AKE50-8', 190, 2.5, 7.5),
    ('AKE60-8', 260, 5, 12.5),
    ('GIM3505-8 / GDZ34', 97, .67, 1.79),
    ('GIM4305-10 / GDZ34', 150, 1.05, 3.82),
    ('DM-J3507-2EC (24 V)', 150, .8, 3),
]
BLUE, ORANGE = '#2369ad', '#cf681e'
plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 11})
fig, ax = plt.subplots(figsize=(14, 9), layout='constrained')
right = ax.twinx()
ax.scatter([r[1] for r in ROWS], [r[2] for r in ROWS], s=64, c=BLUE,
           edgecolors='white', linewidths=.6, zorder=4)
right.scatter([r[1] for r in ROWS], [r[3] for r in ROWS], s=65, c=ORANGE,
              marker='D', edgecolors='white', linewidths=.6, zorder=4)
ax.set(xlim=(75, 285), ylim=(0, 14), xlabel='Mass (g)',
       ylabel='Rated / continuous torque (N·m)')
right.set(ylim=(0, 14), ylabel='Peak / maximum torque (N·m)')
ax.yaxis.label.set_color(BLUE)
right.yaxis.label.set_color(ORANGE)
ax.tick_params(axis='y', colors=BLUE)
right.tick_params(axis='y', colors=ORANGE)
ax.set_xticks(range(80, 281, 20))
ax.set_yticks(range(0, 15, 2))
right.set_yticks(range(0, 15, 2))
ax.grid(color='#dfe3e8', linewidth=.7)
ax.set_axisbelow(True)
fig.suptitle('Actuator Mass vs. Torque', fontsize=20, fontweight='bold')
ax.set_title('Identical torque scales on both axes', fontsize=11, color='#555555', pad=14)
for i in range(3):
    name, mass, rated, peak = ROWS[i]
    right.annotate(name, (mass, peak), xytext=(-10, 12) if i == 3 else (9, 10),
                   textcoords='offset points', ha='right' if i == 3 else 'left',
                   fontsize=10, color=ORANGE)
    # Place the two labels nearest the dense cluster away from its callouts.
    offset = (-18, 23) if i == 0 else (10, 9)
    ax.annotate(name, (mass, rated), xytext=offset, textcoords='offset points',
                ha='right' if i in (0, 3) else 'left', fontsize=10, color=BLUE)
# Direct component labels for both torque series.
peak_offsets = [(10, 12), (12, 10), (12, -12)]
rated_positions = [(80, .25), (163, 1.35), (175, .45)]
for i in range(3, len(ROWS)):
    name, mass, rated, peak = ROWS[i]
    right.annotate(name, (mass, peak), xytext=peak_offsets[i - 3],
                   textcoords='offset points', fontsize=10, color=ORANGE)
    ax.annotate(name, (mass, rated), xytext=rated_positions[i - 3],
                textcoords='data', fontsize=10, color=BLUE)
ax.legend(handles=[
    Line2D([], [], color=BLUE, marker='o', linestyle='none',
           label='Rated / continuous (left axis)', markersize=8),
    Line2D([], [], color=ORANGE, marker='D', linestyle='none',
           label='Peak / maximum (right axis)', markersize=8),
], loc='upper center', bbox_to_anchor=(.65, .99), frameon=False)
out = Path(__file__).resolve().parents[2] / 'docs' / 'actuator_mass_torque.png'
fig.savefig(out, dpi=200, facecolor='white')
plt.close(fig)
print(out)
