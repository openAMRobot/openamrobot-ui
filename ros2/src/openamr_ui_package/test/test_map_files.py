"""Rename and delete touch only image files inside the map's own folder.

Pure test: imports nothing from ROS, so it runs with
``python3 -m pytest test/test_map_files.py``.
"""

import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from openamr_ui_package import map_files  # noqa: E402


def _write(path, text):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w') as file:
        file.write(text)


def _snapshot(root):
    """Every file and link under root, with its content or link target."""
    result = {}
    for folder, _dirs, files in os.walk(root):
        for name in files:
            path = os.path.join(folder, name)
            if os.path.islink(path):
                result[path] = ('link', os.readlink(path))
            else:
                with open(path, 'rb') as file:
                    result[path] = ('file', file.read())
    return result


@pytest.fixture
def tree(tmp_path):
    maps = tmp_path / 'maps'
    group = maps / 'G'
    outside = tmp_path / 'outside'
    sibling = maps / 'Other'
    _write(str(outside / 'shared.png'), 'outside image')
    _write(str(sibling / 'other.png'), 'sibling image')
    group.mkdir(parents=True)
    return {'group': str(group), 'outside': str(outside), 'sibling': str(sibling)}


def _map(tree, name, image_field, ros=False):
    base = os.path.join(tree['group'], name)
    _write(f'{base}.yaml', f'image: {image_field}\nresolution: 0.05\n')
    if ros:
        _write(f'{base}_ros.yaml', f'yaml_filename: {name}.yaml\n')
    return base


def _outside_cases(tree):
    """(case name, image field, setup) for every way out of the folder."""
    link = os.path.join(tree['group'], 'A.png')

    def symlink():
        os.symlink(os.path.join(tree['outside'], 'shared.png'), link)

    return [
        ('absolute', os.path.join(tree['outside'], 'shared.png'), lambda: None),
        ('dotdot', '../Other/other.png', lambda: None),
        ('symlink', 'A.png', symlink),
    ]


@pytest.mark.parametrize('case', ['absolute', 'dotdot', 'symlink'])
def test_delete_refuses_an_image_outside_the_folder(tree, case):
    field, setup = {c[0]: c[1:] for c in _outside_cases(tree)}[case]
    setup()
    base = _map(tree, 'A', field, ros=True)
    outside_before = _snapshot(tree['outside'])
    sibling_before = _snapshot(tree['sibling'])
    group_before = _snapshot(tree['group'])

    with pytest.raises(map_files.MapFileError, match='outside the map folder'):
        map_files.delete_map(base)

    assert _snapshot(tree['outside']) == outside_before
    assert _snapshot(tree['sibling']) == sibling_before
    assert _snapshot(tree['group']) == group_before


@pytest.mark.parametrize('case', ['absolute', 'dotdot', 'symlink'])
def test_rename_refuses_an_image_outside_the_folder(tree, case):
    field, setup = {c[0]: c[1:] for c in _outside_cases(tree)}[case]
    setup()
    base = _map(tree, 'A', field, ros=True)
    outside_before = _snapshot(tree['outside'])
    sibling_before = _snapshot(tree['sibling'])
    group_before = _snapshot(tree['group'])

    with pytest.raises(map_files.MapFileError, match='outside the map folder'):
        map_files.rename_map(base, os.path.join(tree['group'], 'B'))

    assert _snapshot(tree['outside']) == outside_before
    assert _snapshot(tree['sibling']) == sibling_before
    assert _snapshot(tree['group']) == group_before


def test_delete_keeps_an_image_another_map_uses(tree):
    _write(os.path.join(tree['group'], 'shared.png'), 'image')
    base = _map(tree, 'A', 'shared.png')
    _map(tree, 'C', 'shared.png')

    notes = map_files.delete_map(base)

    assert not os.path.exists(f'{base}.yaml')
    assert os.path.exists(os.path.join(tree['group'], 'shared.png'))
    assert notes and 'map "C" also uses it' in notes[0]


def test_rename_keeps_an_image_another_map_uses(tree):
    _write(os.path.join(tree['group'], 'shared.png'), 'image')
    base = _map(tree, 'A', 'shared.png')
    _map(tree, 'C', 'shared.png')
    new = os.path.join(tree['group'], 'B')

    notes = map_files.rename_map(base, new)

    assert os.path.exists(os.path.join(tree['group'], 'shared.png'))
    assert not os.path.exists(os.path.join(tree['group'], 'B.png'))
    with open(f'{new}.yaml') as file:
        assert 'image: shared.png' in file.read()
    assert notes and 'map "C" also uses it' in notes[0]


def test_delete_removes_an_image_inside_the_folder(tree):
    _write(os.path.join(tree['group'], 'A.png'), 'image')
    base = _map(tree, 'A', 'A.png', ros=True)
    outside_before = _snapshot(tree['outside'])

    assert map_files.delete_map(base) == []

    assert os.listdir(tree['group']) == []
    assert _snapshot(tree['outside']) == outside_before


def test_rename_moves_an_image_inside_the_folder(tree):
    _write(os.path.join(tree['group'], 'A.pgm'), 'image')
    base = _map(tree, 'A', os.path.join(tree['group'], 'A.pgm'), ros=True)
    new = os.path.join(tree['group'], 'B')

    assert map_files.rename_map(base, new) == []

    assert sorted(os.listdir(tree['group'])) == ['B.pgm', 'B.yaml', 'B_ros.yaml']
    with open(f'{new}.yaml') as file:
        assert f'image: {new}.pgm' in file.read()
    with open(f'{new}_ros.yaml') as file:
        assert 'yaml_filename: B.yaml' in file.read()


def test_rename_onto_an_existing_map_changes_nothing(tree):
    _write(os.path.join(tree['group'], 'A.png'), 'image')
    base = _map(tree, 'A', 'A.png')
    _map(tree, 'B', 'B.png')
    before = _snapshot(tree['group'])

    with pytest.raises(map_files.MapFileError, match='already exists'):
        map_files.rename_map(base, os.path.join(tree['group'], 'B'))

    assert _snapshot(tree['group']) == before
