"""Build local CAD review artifacts; pass an explicit visualization renderer."""
import argparse
from pathlib import Path
import subprocess
import sys
import zipfile

ROOT = Path(__file__).resolve().parents[3]
SOURCE = Path(__file__).resolve().parent
OUT = ROOT / 'artifacts/ri_tendon_module'
VENDOR = ROOT / 'vendor/components/motor'


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--renderer', type=Path, required=True,
                        help='Path to the visualization render.py script')
    args = parser.parse_args()
    if not args.renderer.is_file():
        raise FileNotFoundError(args.renderer)
    markup = (SOURCE / 'viewer.template.html').read_text().replace(
        '%%DATA%%', (OUT / 'geometry.b64').read_text())
    if len(markup.encode()) >= 1_000_000:
        raise RuntimeError('Inline CAD exceeds 1 MB; no artifact published')
    if '%%DATA%%' in markup or '<html' in markup or '<body' in markup:
        raise RuntimeError('Invalid visualization fragment')
    required = ['ri30_motor_module.step', 'ri20_motor_module.step',
                'wood_leg_joint.step', 'complete_single_axis.step',
                'abduction_cradle.step', 'components.json', 'motor_validation.json']
    for name in required:
        if not (OUT / name).is_file():
            raise FileNotFoundError(OUT / name)
    out = OUT / 'ri-tendon-assembly.html'
    out.write_text(markup)
    subprocess.run([sys.executable, str(args.renderer.resolve()), str(out),
                    str(OUT / 'ri-assembly-preview.html'), '--force'], check=True)
    archive = OUT / 'ri_tendon_cad_review.zip'
    with zipfile.ZipFile(archive, 'w', zipfile.ZIP_DEFLATED) as z:
        for p in [ROOT / 'docs/ri_tendon_module.md', ROOT / 'src/__init__.py',
                  ROOT / 'src/components/__init__.py', *SOURCE.glob('*.py'),
                  SOURCE / 'viewer.template.html']:
            z.write(p, str(p.relative_to(ROOT)))
        for name in required:
            p = OUT / name
            z.write(p, str(p.relative_to(ROOT)))
        for p in VENDOR.rglob('*'):
            if p.is_file():
                z.write(p, str(p.relative_to(ROOT)))
    print(f'Fragment: {out}\nCAD archive: {archive}')


if __name__ == '__main__':
    main()
