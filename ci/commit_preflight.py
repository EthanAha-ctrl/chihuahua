"""Regenerate and stage plot from the Git index, with Git-object provenance."""
from pathlib import Path
import argparse
import json
import os
import subprocess
import sys
import tempfile

SCRIPT = 'src/components/z_common/plot_actuator_mass_torque.py'
INPUTS = ['vendor/components/joint/lists.md', 'vendor/components/motor/lists.md',
          SCRIPT, 'ci/commit_preflight.py', 'pyproject.toml', 'uv.lock']
PNG = 'docs/actuator_mass_torque.png'
STATE = 'docs/actuator_mass_torque.provenance.json'


def git(root, *args, data=None):
    return subprocess.run(['git', *args], cwd=root, input=data,
                          stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=True).stdout


def indexed(root, path):
    return git(root, 'show', ':' + path)


def run(root, source_base_commit='HEAD'):
    head = git(root, 'rev-parse', '--verify', source_base_commit + '^{commit}').decode().strip()
    blobs = {p: git(root, 'rev-parse', ':' + p).decode().strip() for p in INPUTS}
    tracked = git(root, 'ls-files', '--', STATE).strip()
    state = json.loads(indexed(root, STATE)) if tracked else None
    if state and state['input_blobs'] == blobs:
        actual = git(root, 'rev-parse', ':' + PNG).decode().strip()
        if actual != state['output_blob']:
            raise RuntimeError('staged PNG differs from provenance; restore it or regenerate explicitly')
        git(root, 'cat-file', '-e', state['source_base_commit'] + '^{commit}')
        print('Plot preflight: unchanged Git input blobs; skipped.')
        return False
    # Never overwrite an unstaged edit to generated artifacts.
    for path in [PNG, STATE]:
        if git(root, 'diff', '--name-only', '--', path).strip():
            raise RuntimeError(f'unstaged edits in generated artifact: {path}')
        if (root / path).exists() and not git(root, 'ls-files', '--', path).strip():
            raise RuntimeError(f'untracked output would be overwritten: {path}')
    with tempfile.TemporaryDirectory(prefix='actuator-plot-') as directory:
        snapshot = Path(directory)
        for path in INPUTS:
            target = snapshot / path
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(indexed(root, path))
        # Execute the staged renderer against staged Markdown, never working-tree data.
        (snapshot / PNG).parent.mkdir(parents=True, exist_ok=True)
        cache = root / '.tools/matplotlib-cache'
        cache.mkdir(parents=True, exist_ok=True)
        env = dict(os.environ, MPLCONFIGDIR=str(cache))
        subprocess.run([sys.executable, str(snapshot / SCRIPT), '--root', str(snapshot),
                        '--output', str(snapshot / PNG), '--source-commit', head],
                       check=True, env=env)
        payload = (snapshot / PNG).read_bytes()
        state = {'source_base_commit': head, 'input_blobs': blobs,
                 'output_blob': git(root, 'hash-object', '--stdin', data=payload).decode().strip()}
        (root / PNG).parent.mkdir(parents=True, exist_ok=True)
        (root / PNG).write_bytes(payload)
        (root / STATE).write_text(json.dumps(state, indent=2, sort_keys=True) + '\n')
    git(root, 'add', '--', PNG, STATE)
    print('Plot preflight: generated and staged PNG + Git provenance.')
    return True


if __name__ == '__main__':
    try:
        parser = argparse.ArgumentParser()
        parser.add_argument('--source-base-commit', default='HEAD')
        args = parser.parse_args()
        run(Path(git(Path.cwd(), 'rev-parse', '--show-toplevel').decode().strip()), args.source_base_commit)
    except Exception as error:
        print(f'FATAL: plot preflight failed: {error}', file=sys.stderr)
        raise
