"""Where the UI keeps maps and routes (openamrobot-ui#37).

Maps, routes and the current map/route selection live in the same data
directory as recordings (``~/.openamr_ui``), never under the installed
package share directory or the source tree, so a rebuild or reinstall of the
package cannot overwrite or delete them. On first start the maps and routes
shipped with the package are copied in; a file that already exists in the
data directory is never overwritten.

Imports nothing from ROS, so it is tested with plain ``python3 -m pytest``.
"""

import os
import shutil

import yaml

DATA_SUBDIRS = ('maps', 'paths')
CURRENT_FILE = 'current_map_route.yaml'
# Package documentation files, not map or route data.
SKIP_FILES = {'README.md'}


def default_data_dir():
    """Return the data directory; flask_app.py keeps recordings under it."""
    return os.path.join(os.path.expanduser('~'), '.openamr_ui')


def _relocate(path, share_dir, data_dir):
    """Point a path under the package share maps/paths into the data directory."""
    for sub in DATA_SUBDIRS:
        old = os.path.join(share_dir, sub) + os.sep
        if path.startswith(old):
            return os.path.join(data_dir, sub, path[len(old):])
    return path


def seed(share_dir, data_dir):
    """Copy shipped maps and routes into data_dir without overwriting.

    Returns the maps folder, routes folder and current selection file to use.
    """
    for sub in DATA_SUBDIRS:
        source_root = os.path.join(share_dir, sub)
        target_root = os.path.join(data_dir, sub)
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
    if not os.path.exists(current):
        selection = {'map_file': '', 'route_file': ''}
        shipped = os.path.join(share_dir, 'param', CURRENT_FILE)
        if os.path.exists(shipped):
            with open(shipped, encoding='utf-8') as f:
                selection.update(yaml.safe_load(f) or {})
        for key in ('map_file', 'route_file'):
            selection[key] = _relocate(selection.get(key) or '', share_dir, data_dir)
        with open(current, 'w', encoding='utf-8') as f:
            yaml.safe_dump(selection, f)

    return {
        'maps': os.path.join(data_dir, 'maps'),
        'paths': os.path.join(data_dir, 'paths'),
        'current': current,
    }
