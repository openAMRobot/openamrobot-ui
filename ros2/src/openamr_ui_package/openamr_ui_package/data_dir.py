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

Every seeded file, the selection file and the marker are created atomically
and never overwrite an existing file: a seeded file is written to a
temporary file in its target folder and published with ``os.link``, which
fails if the name exists; the selection file and the marker are created
with ``O_CREAT | O_EXCL``.

Imports nothing from ROS, so it is tested with plain ``python3 -m pytest``.
"""

import contextlib
import os
import shutil
import tempfile

import yaml

DATA_SUBDIRS = ('maps', 'paths')
MAPS_DIR_ENV = 'OPENAMROBOT_MAPS_DIR'
CURRENT_FILE = 'current_map_route.yaml'
SEEDED_MARKER = '.seeded'
# Package documentation files, not map or route data.
SKIP_FILES = {'README.md'}


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


def _create_copy(source, target):
    """Copy source to target unless target exists; return True if created.

    The copy goes to a temporary file in the target folder and is published
    with os.link, so the final name never holds a partial file and an
    existing file (a user's, or one another seeder just created) is never
    replaced.
    """
    directory = os.path.dirname(target)
    os.makedirs(directory, exist_ok=True)
    fd, tmp = tempfile.mkstemp(dir=directory, prefix='.' + os.path.basename(target) + '.',
                               suffix='.tmp')
    os.close(fd)
    try:
        shutil.copyfile(source, tmp)
        with open(tmp, 'rb') as f:
            os.fsync(f.fileno())
        os.link(tmp, target)
        return True
    except FileExistsError:
        return False
    finally:
        with contextlib.suppress(OSError):
            os.unlink(tmp)


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


def seed(share_dir, data_dir, environ=os.environ):
    """Seed data_dir once from the shipped maps and routes, never overwriting.

    Maps go to maps_dir(); routes and the selection file to data_dir.
    Returns the maps folder, routes folder and current selection file to use.
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
    return {'maps': targets['maps'], 'paths': targets['paths'], 'current': current}
