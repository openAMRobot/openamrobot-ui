"""Where the UI keeps maps and routes (openamrobot-ui#37).

Routes and the current map/route selection live in the same data directory
as recordings (``~/.openamr_ui``). Maps live in their own directory, set by
the OPENAMROBOT_MAPS_DIR environment variable (default: ``maps`` inside the
data directory), so it can be a volume that the robot platform also mounts
and its map_server can read. Neither is ever under the installed package
share directory or the source tree, so a rebuild or reinstall of the
package cannot overwrite or delete them. On first start the maps and routes
shipped with the package are copied in; a file that already exists in the
data directory is never overwritten.

Imports nothing from ROS, so it is tested with plain ``python3 -m pytest``.
"""

import os
import shutil

import yaml

DATA_SUBDIRS = ('maps', 'paths')
MAPS_DIR_ENV = 'OPENAMROBOT_MAPS_DIR'
CURRENT_FILE = 'current_map_route.yaml'
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


def seed(share_dir, data_dir, environ=os.environ):
    """Copy shipped maps and routes into their folders without overwriting.

    Maps go to maps_dir(); routes and the selection file to data_dir.
    Returns the maps folder, routes folder and current selection file to use.
    """
    targets = {'maps': maps_dir(data_dir, environ), 'paths': os.path.join(data_dir, 'paths')}
    for sub in DATA_SUBDIRS:
        source_root = os.path.join(share_dir, sub)
        target_root = targets[sub]
        os.makedirs(target_root, exist_ok=True)
        for dirpath, _dirs, files in os.walk(source_root):
            rel = os.path.relpath(dirpath, source_root)
            for name in files:
                if name in SKIP_FILES and rel == os.curdir:
                    continue
                target = os.path.normpath(os.path.join(target_root, rel, name))
                if os.path.lexists(target):
                    continue
                os.makedirs(os.path.dirname(target), exist_ok=True)
                shutil.copy2(os.path.join(dirpath, name), target)

    current = os.path.join(data_dir, CURRENT_FILE)
    os.makedirs(data_dir, exist_ok=True)
    if not os.path.exists(current):
        selection = {'map_file': '', 'route_file': ''}
        shipped = os.path.join(share_dir, 'param', CURRENT_FILE)
        if os.path.exists(shipped):
            with open(shipped, encoding='utf-8') as f:
                selection.update(yaml.safe_load(f) or {})
        for key in ('map_file', 'route_file'):
            selection[key] = _relocate(selection.get(key) or '', share_dir, targets)
        with open(current, 'w', encoding='utf-8') as f:
            yaml.safe_dump(selection, f)

    return {'maps': targets['maps'], 'paths': targets['paths'], 'current': current}
