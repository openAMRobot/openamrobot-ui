"""Rename and delete the files of one saved map, inside its own folder only.

A map is ``<name>.yaml`` plus the image its ``image:`` field names. Maps saved
by the platform map saver (SaveMap) have only those two files; older maps
also have a ``<name>_ros.yaml`` map_server parameter file, which is handled
when it exists and skipped when it does not.

The ``image:`` field comes from a file on disk, so it is not trusted. These
functions resolve it (absolute paths, ``..`` and symlinks included) and refuse
the whole operation, before changing anything, when the image resolves
outside the folder that holds the map's yaml. An image that another map in
the same folder also names is kept, never renamed or deleted.

Pure module: no ROS imports, so its tests run with
``python3 -m pytest test/test_map_files.py``.
"""

import os

import yaml


class MapFileError(Exception):
    """The operation was refused; nothing on disk was changed."""


def _read_yaml(path):
    with open(path, 'r') as file:
        return yaml.safe_load(file) or {}


def _image_field(map_base):
    field = _read_yaml(f'{map_base}.yaml').get('image')
    return field if isinstance(field, str) and field else f'{os.path.basename(map_base)}.png'


def _image_path(map_base, field):
    folder = os.path.dirname(map_base)
    return field if os.path.isabs(field) else os.path.join(folder, field)


def _inside(path, folder):
    """True when path, with every symlink resolved, lies inside folder."""
    real_path = os.path.realpath(path)
    real_folder = os.path.realpath(folder)
    return os.path.commonpath([real_path, real_folder]) == real_folder and real_path != real_folder


def owned_image(map_base):
    """Return (image path, image field) for <map_base>.yaml.

    Raises MapFileError when the image resolves outside the map's folder.
    """
    field = _image_field(map_base)
    path = _image_path(map_base, field)
    if not _inside(path, os.path.dirname(map_base)):
        raise MapFileError(
            f'the map image "{field}" is outside the map folder, so it was left '
            'unchanged and nothing was renamed or deleted')
    return path, field


def _shared_with(map_base, image_path):
    """Names of other maps in the same folder whose image is image_path."""
    folder = os.path.dirname(map_base)
    target = os.path.realpath(image_path)
    names = []
    for entry in sorted(os.listdir(folder)):
        if not entry.endswith('.yaml') or entry.endswith('_ros.yaml'):
            continue
        other = os.path.join(folder, entry[:-len('.yaml')])
        if other == map_base:
            continue
        try:
            other_path = _image_path(other, _image_field(other))
        except (OSError, yaml.YAMLError):
            continue
        if os.path.realpath(other_path) == target:
            names.append(entry[:-len('.yaml')])
    return names


def delete_map(map_base):
    """Delete <map_base>.yaml, its _ros.yaml and its image. Returns notes."""
    image, field = owned_image(map_base)
    shared = _shared_with(map_base, image)
    os.remove(f'{map_base}.yaml')
    if os.path.exists(f'{map_base}_ros.yaml'):
        os.remove(f'{map_base}_ros.yaml')
    if shared:
        return [f'The image "{field}" was kept because map "{shared[0]}" also uses it.']
    if os.path.lexists(image):
        os.remove(image)
    return []


def rename_map(old_base, new_base):
    """Rename <old_base> to <new_base> in the same folder. Returns notes."""
    if os.path.exists(f'{new_base}.yaml'):
        raise MapFileError(f'a map named "{os.path.basename(new_base)}" already exists')
    old_image, field = owned_image(old_base)
    shared = _shared_with(old_base, old_image)
    notes = []

    map_yaml = _read_yaml(f'{old_base}.yaml')
    if shared:
        new_image = old_image
        notes.append(f'The image "{field}" kept its name because map "{shared[0]}" also uses it.')
    else:
        new_image = f'{new_base}{os.path.splitext(old_image)[1] or ".png"}'
        map_yaml['image'] = new_image if os.path.isabs(field) else os.path.basename(new_image)
    with open(f'{old_base}.yaml', 'w') as file:
        yaml.dump(map_yaml, file)

    old_ros, new_ros = f'{old_base}_ros.yaml', f'{new_base}_ros.yaml'
    if os.path.exists(old_ros):
        ros_yaml = _read_yaml(old_ros)
        params = ros_yaml.get('map_server', {}).get('ros__parameters')
        (params if isinstance(params, dict) else ros_yaml)['yaml_filename'] = (
            f'{os.path.basename(new_base)}.yaml')
        with open(old_ros, 'w') as file:
            yaml.dump(ros_yaml, file)
        os.rename(old_ros, new_ros)

    os.rename(f'{old_base}.yaml', f'{new_base}.yaml')
    if new_image != old_image and os.path.lexists(old_image):
        os.rename(old_image, new_image)
    return notes
