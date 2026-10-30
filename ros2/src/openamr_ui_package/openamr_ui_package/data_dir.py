"""Where the UI keeps maps and routes (openamrobot-ui#37).

Routes and the current map/route selection live in the same data directory
as recordings (``~/.openamr_ui``). Maps live in their own directory, set by
the OPENAMROBOT_MAPS_DIR environment variable (default: ``maps`` inside the
data directory), so it can be a volume that the robot platform also mounts
and its map_server can read. Neither is ever under the installed package
share directory or the source tree, so a rebuild or reinstall of the
package cannot overwrite or delete them.

Seeding runs once per data directory: on the first start the maps and routes
shipped with the package are copied in and a ``.seeded`` marker is written;
later starts copy nothing, so an example the user deleted stays deleted.

When OPENAMROBOT_MAPS_DIR is set to a folder other than the data directory's
own ``maps`` folder and that old folder has maps, its maps are copied, once,
into OPENAMROBOT_MAPS_DIR (marker ``.maps_migrated``). The old folder is never
changed and stays as a backup.

Every file created here is created atomically and never overwrites an
existing file: a copy is written to a temporary file in its target folder and
published with ``os.link``, which fails if the name exists; the selection
file and the markers are created with ``O_CREAT | O_EXCL``.

Imports nothing from ROS, so it is tested with plain ``python3 -m pytest``.
"""

import contextlib
import logging
import os
import shutil
import tempfile

import yaml

DATA_SUBDIRS = ('maps', 'paths')
MAPS_DIR_ENV = 'OPENAMROBOT_MAPS_DIR'
CURRENT_FILE = 'current_map_route.yaml'
SEEDED_MARKER = '.seeded'
MIGRATED_MARKER = '.maps_migrated'
# Package documentation files, not map or route data.
SKIP_FILES = {'README.md'}

LOG = logging.getLogger(__name__)


def default_data_dir():
    """Return the data directory; flask_app.py keeps recordings under it."""
    return os.path.join(os.path.expanduser('~'), '.openamr_ui')


def maps_dir(data_dir, environ=os.environ):
    """Return OPENAMROBOT_MAPS_DIR if set, else the maps folder of data_dir."""
    return environ.get(MAPS_DIR_ENV) or os.path.join(data_dir, 'maps')


def _relocate(path, share_dir, targets):
    """Point a path under the package share maps/paths at its new folder."""
    for sub in DATA_SUBDIRS:
        old = os.path.join(share_dir, sub) + os.sep
        if path.startswith(old):
            return os.path.join(targets[sub], path[len(old):])
    return path


def _create(target, write):
    """Create target unless it exists; return True if this call created it.

    ``write(path)`` fills a temporary file in the target folder, which is
    then published with os.link, so the final name never holds a partial file
    and an existing file (a user's, or one another node just created) is
    never replaced. The temporary file is always removed.
    """
    directory = os.path.dirname(target)
    os.makedirs(directory, exist_ok=True)
    fd, tmp = tempfile.mkstemp(dir=directory, prefix='.' + os.path.basename(target) + '.',
                               suffix='.tmp')
    os.close(fd)
    try:
        write(tmp)
        with open(tmp, 'rb') as f:
            os.fsync(f.fileno())
        os.link(tmp, target)
        return True
    except FileExistsError:
        return False
    finally:
        with contextlib.suppress(OSError):
            os.unlink(tmp)


def _create_copy(source, target):
    """Copy source to target unless target exists; return True if created."""
    return _create(target, lambda tmp: shutil.copyfile(source, tmp))


def _create_text(target, text):
    """Write text to target unless target exists; return True if created."""
    def write(tmp):
        with open(tmp, 'w', encoding='utf-8') as f:
            f.write(text)
    return _create(target, write)


def _create_exclusive(path, text):
    """Create path with text unless it exists (O_CREAT | O_EXCL); return True if created."""
    try:
        fd = os.open(path, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o644)
    except FileExistsError:
        return False
    with os.fdopen(fd, 'w', encoding='utf-8') as f:
        f.write(text)
        f.flush()
        os.fsync(f.fileno())
    return True


def _replace_text(path, text):
    """Replace path with text atomically (temporary file, fsync, os.replace)."""
    fd, tmp = tempfile.mkstemp(dir=os.path.dirname(path),
                               prefix='.' + os.path.basename(path) + '.', suffix='.tmp')
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as f:
            f.write(text)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, path)
    finally:
        with contextlib.suppress(OSError):
            os.unlink(tmp)


def _inside(path, folder):
    return os.path.abspath(path).startswith(os.path.abspath(folder) + os.sep)


def _resolves_inside(path, folder):
    """True when path, with every symlink resolved, lies inside folder."""
    real_path, real_folder = os.path.realpath(path), os.path.realpath(folder)
    return real_path != real_folder and os.path.commonpath([real_path, real_folder]) == real_folder


def _maps_in(folder):
    """Yield the base path (no .yaml) of every map yaml under folder."""
    for dirpath, _dirs, files in os.walk(folder):
        for name in sorted(files):
            if name.endswith('.yaml') and not name.endswith('_ros.yaml'):
                yield os.path.join(dirpath, name[:-len('.yaml')])


def migrate_maps(old_dir, new_dir, current_file):
    """Copy every map from old_dir into new_dir once; never move, delete or overwrite.

    A map is its yaml, the image it names and, if present, <name>_ros.yaml.
    A map whose name already exists in new_dir is skipped and logged. A map
    whose image resolves outside old_dir (an absolute path elsewhere, a ..
    path or a symlink out) is skipped and logged too: nothing of it is
    copied, so the old folder and new_dir both stay as they were. An
    absolute image path inside old_dir is rewritten to new_dir in the copy
    only. The selection is repointed when it names a map this call copied.
    Returns the map paths (relative to old_dir, without .yaml) that were copied.
    """
    marker = os.path.join(new_dir, MIGRATED_MARKER)
    if os.path.abspath(old_dir) == os.path.abspath(new_dir) or os.path.exists(marker):
        return []
    maps = list(_maps_in(old_dir)) if os.path.isdir(old_dir) else []
    if not maps:
        return []
    os.makedirs(new_dir, exist_ok=True)

    copied = []
    for base in maps:
        rel = os.path.relpath(base, old_dir)
        target = os.path.join(new_dir, rel)
        with open(f'{base}.yaml', encoding='utf-8') as f:
            text = f.read()
        info = yaml.safe_load(text) or {}
        image = info.get('image') or f'{os.path.basename(base)}.png'
        image_path = image if os.path.isabs(image) else os.path.join(os.path.dirname(base), image)
        if not _resolves_inside(image_path, old_dir):
            LOG.warning('Map %s not migrated: its image %s is outside %s, so it was not '
                        'copied and the map stays only in %s.', rel, image, old_dir, old_dir)
            continue
        files = [(image_path, os.path.join(new_dir, os.path.relpath(image_path, old_dir)))]
        if os.path.exists(f'{base}_ros.yaml'):
            files.append((f'{base}_ros.yaml', f'{target}_ros.yaml'))
        clash = [t for t in [f'{target}.yaml'] + [t for _s, t in files] if os.path.lexists(t)]
        if clash:
            LOG.warning('Map %s not migrated: %s already exists in %s; both copies kept.',
                        rel, os.path.relpath(clash[0], new_dir), new_dir)
            continue
        for source, dest in files:
            if os.path.exists(source):
                _create_copy(source, dest)
        if os.path.isabs(image) and _inside(image, old_dir):
            info['image'] = os.path.join(new_dir, os.path.relpath(image, old_dir))
            text = yaml.safe_dump(info, sort_keys=False)
        # The yaml goes last, so a map yaml in new_dir always has its image.
        if _create_text(f'{target}.yaml', text):
            copied.append(rel)

    _repoint_selection(current_file, old_dir, new_dir, copied)
    _create_exclusive(marker, f'Maps from {old_dir} were copied here once; that folder '
                              'was left unchanged as a backup.\n')
    return copied


def _repoint_selection(current_file, old_dir, new_dir, copied):
    """Point the selection at the copy of its map, if this migration made one."""
    if not os.path.exists(current_file):
        return
    with open(current_file, encoding='utf-8') as f:
        selection = yaml.safe_load(f) or {}
    map_file = selection.get('map_file') or ''
    if not _inside(map_file, old_dir):
        return
    rel = os.path.relpath(map_file, old_dir)
    if not rel.endswith('.yaml') or rel[:-len('.yaml')] not in copied:
        LOG.warning('Selected map %s left as is: it was not copied to %s (the name is '
                    'already in use there, or it is not a map).', map_file, new_dir)
        return
    selection['map_file'] = os.path.join(new_dir, rel)
    _replace_text(current_file, yaml.safe_dump(selection))


def seed(share_dir, data_dir, environ=os.environ):
    """Seed data_dir once from the shipped maps and routes, never overwriting.

    Then migrate maps from data_dir's own maps folder into OPENAMROBOT_MAPS_DIR
    once, when that is set to another folder. Returns the maps folder, routes
    folder and current selection file to use.
    """
    targets = {'maps': maps_dir(data_dir, environ), 'paths': os.path.join(data_dir, 'paths')}
    for folder in targets.values():
        os.makedirs(folder, exist_ok=True)
    marker = os.path.join(data_dir, SEEDED_MARKER)

    if not os.path.exists(marker):
        for sub in DATA_SUBDIRS:
            source_root = os.path.join(share_dir, sub)
            for dirpath, _dirs, files in os.walk(source_root):
                rel = os.path.relpath(dirpath, source_root)
                for name in files:
                    if name in SKIP_FILES and rel == os.curdir:
                        continue
                    target = os.path.normpath(os.path.join(targets[sub], rel, name))
                    _create_copy(os.path.join(dirpath, name), target)

    current = os.path.join(data_dir, CURRENT_FILE)
    if not os.path.exists(current):
        selection = {'map_file': '', 'route_file': ''}
        shipped = os.path.join(share_dir, 'param', CURRENT_FILE)
        if os.path.exists(shipped):
            with open(shipped, encoding='utf-8') as f:
                selection.update(yaml.safe_load(f) or {})
        for key in ('map_file', 'route_file'):
            selection[key] = _relocate(selection.get(key) or '', share_dir, targets)
        _create_exclusive(current, yaml.safe_dump(selection))

    _create_exclusive(marker, 'Shipped maps and routes were copied here once; '
                              'delete this file to copy them again.\n')
    migrate_maps(os.path.join(data_dir, 'maps'), targets['maps'], current)
    return {'maps': targets['maps'], 'paths': targets['paths'], 'current': current}
