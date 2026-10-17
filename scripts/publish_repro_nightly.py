"""Publish tested artifacts as a permanent build and a rolling prerelease."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess

REPOSITORY = 'thereprocase/OrcaSlicer-nest3d'
TAG = 'repro-nightly'


def ensure_archive_tag(tag, sha):
    """Verify the actual tag commit before creating or resuming a release."""
    ref = 'refs/tags/' + tag
    result = subprocess.run(['git', 'ls-remote', '--tags', 'origin', ref, ref + '^{}'],
                            capture_output=True, text=True, check=True)
    refs = dict(line.split()[::-1] for line in result.stdout.splitlines())
    actual = refs.get(ref + '^{}', refs.get(ref))
    if actual is not None:
        if actual != sha:
            raise ValueError('Existing archive tag targets a different commit: ' + tag)
        return
    subprocess.run(['git', 'push', 'origin', f'{sha}:{ref}'], check=True)


def publish_build_archive(sha, run_id, assets, notes):
    """Retry unfinished drafts; leave an already published build untouched."""
    if not re.fullmatch('[a-f0-9]{40}', sha) or not re.fullmatch('[0-9]+', run_id):
        raise ValueError('Archive requires a full SHA and numeric build run ID')
    tag = f'repro-build-{run_id}-{sha[:12]}'
    ensure_archive_tag(tag, sha)
    view = subprocess.run(
        ['gh', 'release', 'view', tag, '--json', 'isDraft'],
        capture_output=True, text=True)
    if view.returncode == 0:
        release = json.loads(view.stdout)
        if not release['isDraft']:
            return tag
    else:
        subprocess.run(['gh', 'release', 'create', tag, '--verify-tag',
                        '--draft', '--prerelease', '--latest=false',
                        '--title', f'Repro build {run_id} · {sha[:12]}',
                        '--notes-file', str(notes)], check=True)
    subprocess.run(['gh', 'release', 'upload', tag, *map(str, assets), '--clobber'], check=True)
    subprocess.run(['gh', 'release', 'edit', tag, '--draft=false', '--prerelease',
                    '--latest=false'], check=True)
    return tag


def prepare_assets(artifacts, output):
    output.mkdir(exist_ok=False)
    assets = []
    for directory in sorted(artifacts.iterdir()):
        if not directory.is_dir() or not directory.name.startswith('OrcaSlicer'):
            continue
        if '_Mac_bundle_' in directory.name:
            continue
        files = sorted(path for path in directory.rglob('*') if path.is_file())
        if not files:
            continue
        if any(path.is_symlink() for path in directory.rglob('*')):
            raise ValueError(f'Unexpected artifact symlink: {directory.name}')
        name = re.sub(r'_V[^_]+', '_nightly', directory.name)
        if len(files) == 1:
            suffix = files[0].suffix
            target = output / (name if name.endswith(suffix) else name + suffix)
            if target.exists():
                raise ValueError(f'Duplicate release asset: {target.name}')
            shutil.copyfile(files[0], target)
        else:
            target = output / (name + '.zip')
            if target.exists():
                raise ValueError(f'Duplicate release asset: {target.name}')
            shutil.make_archive(str(target.with_suffix('')), 'zip', directory)
        assets.append(target)
    required = {'.AppImage', '.dmg', '.exe', '.flatpak'}
    missing = required - {path.suffix for path in assets}
    if missing:
        raise ValueError(f'Missing platform packages: {sorted(missing)}')
    checksums = output / 'SHA256SUMS'
    with checksums.open('w') as stream:
        for path in assets:
            with path.open('rb') as package:
                digest = hashlib.file_digest(package, 'sha256').hexdigest()
            stream.write(f'{digest}  {path.name}\n')
    return [*assets, checksums]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('artifacts', type=Path)
    parser.add_argument('--sha', required=True)
    parser.add_argument('--run-id', default=os.environ.get('GITHUB_RUN_ID'),
                        help='Original Build all run ID (not a separate publisher run)')
    parser.add_argument('--output', type=Path, default=Path('nightly-release'))
    parser.add_argument('--dry-run', action='store_true')
    parser.add_argument('--archive-only', action='store_true',
                        help='Retain a verified older build without moving the rolling nightly')
    args = parser.parse_args()
    if not re.fullmatch('[a-f0-9]{40}', args.sha):
        parser.error('--sha must be a full commit SHA')
    assets = prepare_assets(args.artifacts, args.output)
    print('\n'.join(str(path) for path in assets), flush=True)
    if args.dry_run:
        return
    if os.environ.get('GH_REPO') != REPOSITORY or os.environ.get('GITHUB_REF') != 'refs/heads/repro-dev':
        parser.error('Publishing requires this fork and refs/heads/repro-dev')
    if not args.run_id or not re.fullmatch('[0-9]+', args.run_id):
        parser.error('Publishing requires --run-id or GITHUB_RUN_ID')
    notes = args.output / 'release-notes.md'
    notes.write_text(
        f'Built from `repro-dev` commit `{args.sha}` after the multi-platform build and test gates.\n\n'
        f'[CI build and tests](https://github.com/{REPOSITORY}/actions/runs/{args.run_id}) · '
        f'[Permanent build download](https://github.com/{REPOSITORY}/releases/tag/'
        f'repro-build-{args.run_id}-{args.sha[:12]})\n\n'
        'Includes the clean Nest3D arranger, CLI automation/bridge options and GUI/CLI retraction protection. '
        'Applications support both graphical and command-line use. Windows portable downloads include their resources.\n\n'
        'Development builds: physical printing and hardware compatibility remain profile-specific. '
        'Check SHA256SUMS after downloading.\n')
    publish_build_archive(args.sha, args.run_id, assets, notes)
    if args.archive_only:
        return
    view = subprocess.run(['gh', 'release', 'view', TAG], stdout=subprocess.DEVNULL,
                          stderr=subprocess.DEVNULL)
    if view.returncode:
        subprocess.run(['gh', 'release', 'create', TAG, '--target', args.sha, '--prerelease',
                        '--title', 'Repro development nightly', '--notes-file', str(notes)], check=True)
    else:
        subprocess.run(['gh', 'release', 'edit', TAG, '--prerelease',
                        '--title', 'Repro development nightly', '--notes-file', str(notes)], check=True)
    subprocess.run(['gh', 'release', 'upload', TAG, *map(str, assets), '--clobber'], check=True)
    subprocess.run(['git', 'tag', '--force', TAG, args.sha], check=True)
    subprocess.run(['git', 'push', 'origin', f'refs/tags/{TAG}', '--force'], check=True)


if __name__ == '__main__':
    main()
