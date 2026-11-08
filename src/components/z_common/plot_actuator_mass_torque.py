"""Plot numeric Markdown tables; missing torque stays missing, never zero."""
from pathlib import Path
import argparse
import math
import re


def read_rows(path):
    lines = path.read_text().splitlines()
    header = ['厂商/型号', '重量 g', '额定/持续扭矩 N·m', '峰值/最大扭矩 N·m']
    rows = []
    active = False
    for line in lines:
        if not line.startswith('|'):
            if active:
                break
            continue
        cells = [c.strip() for c in line.strip().strip('|').split('|')]
        if not active:
            if cells[:4] == header:
                active = True
            continue
        if all(re.fullmatch(r':?-+:?', c) for c in cells):
            continue
        if len(cells) < 4:
            raise ValueError(f'{path}: malformed row: {line}')
        values = []
        for cell in cells[1:4]:
            value = None if cell == 'N/A' else float(cell)
            if value is not None and (not math.isfinite(value) or value <= 0):
                raise ValueError(f'{path}: invalid positive value {cell}')
            values.append(value)
        if values[0] is None or all(v is None for v in values[1:]):
            raise ValueError(f'{path}: mass and at least one torque required')
        if any(r[0] == cells[0] for r in rows):
            raise ValueError(f'{path}: duplicate model {cells[0]}')
        rows.append((cells[0], *values))
    if not active or not rows:
        raise ValueError(f'{path}: required numeric table missing or empty')
    return rows


def render(root, output, source_commit):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D

    rows = read_rows(root / 'vendor/components/joint/lists.md')
    motors = read_rows(root / 'vendor/components/motor/lists.md')
    rows += motors
    blue, orange = '#2369ad', '#cf681e'
    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 11})
    fig, ax = plt.subplots(figsize=(14, 9), layout='constrained')
    right = ax.twinx()
    rated = [r for r in rows if r[2] is not None]
    peak = [r for r in rows if r[3] is not None]
    ax.scatter([r[1] for r in rated], [r[2] for r in rated], s=64, c=blue,
               edgecolors='white', linewidths=.6, zorder=4)
    right.scatter([r[1] for r in peak], [r[3] for r in peak], s=65, c=orange,
                  marker='D', edgecolors='white', linewidths=.6, zorder=4)
    lower = min(75, min(r[1] for r in rows) - 20)
    upper = max(285, max(r[1] for r in rows) + 25)
    ceiling = max(14, math.ceil((max(r[c] for r in rows for c in (2, 3)
                                    if r[c] is not None) + 1) / 2) * 2)
    ax.set(xlim=(lower, upper), ylim=(0, ceiling), xlabel='Mass (g)',
           ylabel='Rated / continuous torque (N·m)')
    right.set(ylim=(0, ceiling), ylabel='Peak / maximum torque (N·m)')
    ax.yaxis.label.set_color(blue)
    right.yaxis.label.set_color(orange)
    ax.tick_params(axis='y', colors=blue)
    right.tick_params(axis='y', colors=orange)
    ax.set_xticks(range(math.ceil(lower / 20) * 20, math.floor(upper / 20) * 20 + 1, 20))
    ax.set_yticks(range(0, ceiling + 1, 2))
    right.set_yticks(range(0, ceiling + 1, 2))
    ax.grid(color='#dfe3e8', linewidth=.7)
    ax.set_axisbelow(True)
    fig.suptitle('Actuator Mass vs. Torque', fontsize=20, fontweight='bold')
    ax.set_title('Identical torque scales on both axes', fontsize=11, color='#555555', pad=14)

    legend = ax.legend(handles=[
        Line2D([], [], color=blue, marker='o', linestyle='none',
               label='Rated / continuous (left axis)', markersize=8),
        Line2D([], [], color=orange, marker='D', linestyle='none',
               label='Peak / maximum (right axis)', markersize=8),
    ], loc='upper center', bbox_to_anchor=(.65, .99), frameon=False)
    # Place full Markdown names using rendered bounding boxes, without model-specific rules.
    from matplotlib.transforms import Bbox
    fig.set_dpi(200)
    fig.canvas.draw()
    fig.set_layout_engine(None)
    renderer = fig.canvas.get_renderer()
    bounds = ax.get_window_extent(renderer)
    occupied = [legend.get_window_extent(renderer).padded(5)]
    for row in rows:
        for column in (2, 3):
            if row[column] is not None:
                x, y = ax.transData.transform((row[1], row[column]))
                occupied.append(Bbox.from_bounds(x - 8, y - 8, 16, 16))
    motor_names = {r[0] for r in motors}
    for name, mass, continuous, maximum in rows:
        for torque, axis, color in [(maximum, right, orange), (continuous, ax, blue)]:
            if torque is None:
                continue
            label = name
            if name in motor_names:
                label += '\n(motor shaft' + ('; rated N/A)' if continuous is None else ')')
            annotation = axis.annotate(label, (mass, torque), xytext=(0, 0),
                                       textcoords='offset points', fontsize=9, color=color,
                                       annotation_clip=False)
            annotation.set_in_layout(False)
            placed = False
            for radius in range(12, 301, 12):
                for dx, dy in [(radius, radius), (-radius, radius),
                               (radius, -radius), (-radius, -radius),
                               (radius, 0), (-radius, 0), (0, radius), (0, -radius)]:
                    annotation.set_position((dx, dy))
                    annotation.set_ha('right' if dx < 0 else 'left')
                    annotation.set_va('top' if dy < 0 else 'bottom')
                    box = annotation.get_window_extent(renderer).padded(5)
                    if (bounds.contains(box.x0, box.y0) and bounds.contains(box.x1, box.y1)
                            and not any(box.overlaps(other) for other in occupied)):
                        occupied.append(box)
                        placed = True
                        break
                if placed:
                    break
            if not placed:
                plt.close(fig)
                raise ValueError(f'Cannot place label without overlap: {name}; enlarge figure')
            if radius > 12:
                connector = axis.annotate('', (mass, torque), xytext=(dx, dy),
                                          textcoords='offset points',
                                          arrowprops={'arrowstyle': '-', 'color': color, 'lw': .6})
                connector.set_in_layout(False)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=200, facecolor='white',
                metadata={'Software': 'chihuahua actuator plot', 'SourceCommit': source_commit})
    plt.close(fig)
    return fig


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[3])
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--source-commit', required=True)
    args = parser.parse_args()
    render(args.root, args.output, args.source_commit)
