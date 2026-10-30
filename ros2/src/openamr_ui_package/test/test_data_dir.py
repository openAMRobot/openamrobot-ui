"""Maps and routes persist in the data directory across reinstalls (#37).

Pure test: imports data_dir and PyYAML only, so it runs with
``python3 -m pytest test/test_data_dir.py`` without ROS.
"""

import os
import shutil
import sys

import pytest
import yaml

PACKAGE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(PACKAGE_DIR)))
sys.path.insert(0, PACKAGE_DIR)

from openamr_ui_package import data_dir  # noqa: E402, I100


@pytest.fixture(autouse=True)
def _no_maps_dir_override(monkeypatch):
    """Hide any OPENAMROBOT_MAPS_DIR set by the caller from tests of the default."""
    monkeypatch.delenv('OPENAMROBOT_MAPS_DIR', raising=False)


def _install(share):
    """Simulate a package install: copy the shipped maps, paths and param."""
    if os.path.exists(share):
        shutil.rmtree(share)
    for sub in ('maps', 'paths', 'param'):
        shutil.copytree(os.path.join(PACKAGE_DIR, sub), os.path.join(share, sub))


def _tree(root):
    found = {}
    for dirpath, _dirs, files in os.walk(root):
        for name in files:
            path = os.path.join(dirpath, name)
            with open(path, 'rb') as f:
                found[os.path.relpath(path, root)] = f.read()
    return found


def test_clean_install_copies_shipped_maps_and_routes(tmp_path):
    share, data = str(tmp_path / 'share'), str(tmp_path / 'data')
    _install(share)
    locations = data_dir.seed(share, data)
    assert os.path.isfile(os.path.join(locations['maps'], 'Welcome', 'Start.yaml'))
    assert os.path.isfile(os.path.join(locations['paths'], 'Welcome', 'Start', 'Building.csv'))
    assert not os.path.exists(os.path.join(locations['maps'], 'README.md'))
    with open(locations['current']) as f:
        assert yaml.safe_load(f) == {'map_file': '', 'route_file': ''}


def test_save_writes_only_under_the_data_dir_and_survives_reinstall(tmp_path):
    share, data = str(tmp_path / 'share'), str(tmp_path / 'data')
    _install(share)
    shipped = _tree(share)
    locations = data_dir.seed(share, data)

    # Save: a new route, an edited shipped route, and a new map.
    new_route = os.path.join(locations['paths'], 'Welcome', 'Start', 'Mine.csv')
    with open(new_route, 'w') as f:
        f.write('1,2,0,0,0,0,1,0,0,0, 1\n')
    edited = os.path.join(locations['paths'], 'Welcome', 'Start', 'Building.csv')
    with open(edited, 'w') as f:
        f.write('edited\n')
    os.makedirs(os.path.join(locations['maps'], 'Lab'))
    with open(os.path.join(locations['maps'], 'Lab', 'Floor.yaml'), 'w') as f:
        f.write('image: Floor.png\n')
    with open(locations['current'], 'w') as f:
        yaml.safe_dump({'map_file': 'x.yaml', 'route_file': new_route}, f)
    assert _tree(share) == shipped

    # Reinstall: the share directory is replaced, with a changed shipped file.
    _install(share)
    with open(os.path.join(share, 'paths', 'Welcome', 'Start', 'Building.csv'), 'w') as f:
        f.write('new release\n')
    data_dir.seed(share, data)

    with open(new_route) as f:
        assert f.read() == '1,2,0,0,0,0,1,0,0,0, 1\n'
    with open(edited) as f:
        assert f.read() == 'edited\n'
    assert os.path.isfile(os.path.join(locations['maps'], 'Lab', 'Floor.yaml'))
    with open(locations['current']) as f:
        assert yaml.safe_load(f)['route_file'] == new_route


def test_selection_pointing_into_the_share_is_moved_to_the_data_dir(tmp_path):
    share, data = str(tmp_path / 'share'), str(tmp_path / 'data')
    _install(share)
    with open(os.path.join(share, 'param', 'current_map_route.yaml'), 'w') as f:
        yaml.safe_dump({'map_file': os.path.join(share, 'maps', 'Welcome', 'Start.yaml'),
                        'route_file': os.path.join(share, 'paths', 'Welcome', 'Start',
                                                   'Building.csv')}, f)
    with open(data_dir.seed(share, data)['current']) as f:
        selection = yaml.safe_load(f)
    assert selection['map_file'] == os.path.join(data, 'maps', 'Welcome', 'Start.yaml')
    assert selection['route_file'].startswith(os.path.join(data, 'paths'))


def test_data_dir_is_the_recordings_parent():
    assert data_dir.default_data_dir() == os.path.join(os.path.expanduser('~'), '.openamr_ui')


def test_compose_keeps_the_data_dir_on_a_named_volume():
    with open(os.path.join(REPO_ROOT, 'docker-compose.yml')) as f:
        compose = yaml.safe_load(f)
    volumes = compose['services']['openamr-ui']['volumes']
    mounts = dict(v.split(':', 1) for v in volumes)
    assert mounts.get('openamr_ui_data') == '/root/.openamr_ui'
    assert 'openamr_ui_data' in compose['volumes']


def test_maps_dir_defaults_to_the_data_dir_maps_folder():
    assert data_dir.maps_dir('/d', environ={}) == os.path.join('/d', 'maps')
    empty = {'OPENAMROBOT_MAPS_DIR': ''}
    assert data_dir.maps_dir('/d', environ=empty) == os.path.join('/d', 'maps')


def test_maps_are_seeded_and_saved_under_openamrobot_maps_dir(tmp_path):
    share, data, maps = str(tmp_path / 'share'), str(tmp_path / 'data'), str(tmp_path / 'maps')
    _install(share)
    locations = data_dir.seed(share, data, environ={'OPENAMROBOT_MAPS_DIR': maps})
    assert locations['maps'] == maps
    assert os.path.isfile(os.path.join(maps, 'Welcome', 'Start.yaml'))
    # Routes and the selection stay in the UI data directory.
    assert locations['paths'] == os.path.join(data, 'paths')
    assert os.path.isfile(os.path.join(data, 'paths', 'Welcome', 'Start', 'Building.csv'))
    assert locations['current'] == os.path.join(data, 'current_map_route.yaml')
    assert not os.path.exists(os.path.join(data, 'maps'))


def test_selection_into_the_share_moves_to_openamrobot_maps_dir(tmp_path):
    share, data, maps = str(tmp_path / 'share'), str(tmp_path / 'data'), str(tmp_path / 'maps')
    _install(share)
    with open(os.path.join(share, 'param', 'current_map_route.yaml'), 'w') as f:
        yaml.safe_dump({'map_file': os.path.join(share, 'maps', 'Welcome', 'Start.yaml'),
                        'route_file': ''}, f)
    current = data_dir.seed(share, data, environ={'OPENAMROBOT_MAPS_DIR': maps})['current']
    with open(current) as f:
        assert yaml.safe_load(f)['map_file'] == os.path.join(maps, 'Welcome', 'Start.yaml')


def test_compose_puts_maps_on_the_shared_openamrobot_maps_volume():
    with open(os.path.join(REPO_ROOT, 'docker-compose.yml')) as f:
        compose = yaml.safe_load(f)
    service = compose['services']['openamr-ui']
    mounts = dict(v.split(':', 1) for v in service['volumes'])
    assert mounts['openamrobot_maps'] == '/data/maps'
    assert service['environment']['OPENAMROBOT_MAPS_DIR'] == '/data/maps'
    # A fixed volume name, so another compose project can mount the same one.
    assert compose['volumes']['openamrobot_maps'] == {'name': 'openamrobot_maps'}
