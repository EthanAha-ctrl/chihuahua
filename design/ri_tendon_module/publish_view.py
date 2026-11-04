from pathlib import Path
import json, subprocess, sys, zipfile

root=Path(__file__).resolve().parent
viz=Path('/home/admin/.codex/visualizations/2026/11/03/01a1a1e1-2d1c-7bb3-a292-e0f1a25e5ca7')
out=viz/'ri-tendon-assembly.html'
markup=(root/'viewer.template.html').read_text().replace('%%DATA%%',(root/'output/geometry.b64').read_text())
if len(markup.encode())>=1_000_000:
    raise RuntimeError(f'Inline CAD exceeds 1 MB limit: {len(markup.encode())} bytes; no artifact published')
if '%%DATA%%' in markup or '<html' in markup or '<body' in markup:
    raise RuntimeError('Invalid visualization fragment')
out.write_text(markup)
subprocess.run([sys.executable,'/home/admin/.codex/plugins/cache/openai-bundled/visualize/1.0.66/skills/visualize/scripts/render.py',str(out),str(viz/'ri-assembly-preview.html'),'--force'],check=True)
archive=root/'output/ri_tendon_cad_review.zip'
with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED) as z:
    for name in ['README.md','build_cad.py','joint_geometry.py','supplemental_geometry.py']:
        z.write(root/name,name)
    for name in ['ri30_motor_module.step','ri20_motor_module.step','wood_leg_joint.step','complete_single_axis.step','abduction_cradle.step','components.json','motor_validation.json']:
        p=root/'output'/name
        if name=='motor_validation.json' and not p.exists():
            print('Motor validation still pending; archive will need refresh.',flush=True)
            continue
        z.write(p,'output/'+name)
    for p in (root/'vendor').rglob('*'):
        if p.is_file():z.write(p,str(p.relative_to(root)))
print('Fragment bytes:',out.stat().st_size,'CAD archive:',archive.stat().st_size,flush=True)
