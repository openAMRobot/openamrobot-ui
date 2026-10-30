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


def _shipped_files(share):
    found = {}
    for sub in ('maps', 'paths'):
        root = os.path.join(share, sub)
        for dirpath, _dirs, files in os.walk(root):
            for name in files:
                rel = os.path.relpath(os.path.join(dirpath, name), root)
                if rel != 'README.md':
                    with open(os.path.join(dirpath, name), 'rb') as f:
                        found[os.path.join(sub, rel)] = f.read()
    return found


def test_second_start_does_not_restore_a_deleted_example(tmp_path):
    share, data = str(tmp_path / 'share'), str(tmp_path / 'data')
    _install(share)
    locations = data_dir.seed(share, data)
    assert os.path.isfile(os.path.join(data, data_dir.SEEDED_MARKER))
    example = os.path.join(locations['paths'], 'Group1', 'Map1', 'Route1.csv')
    os.remove(example)
    data_dir.seed(share, data)
    assert not os.path.exists(example)


def test_a_user_file_that_appears_during_seeding_is_never_overwritten(tmp_path, monkeypatch):
    share, data = str(tmp_path / 'share'), str(tmp_path / 'data')
    _install(share)
    target = os.path.join(data, 'paths', 'Welcome', 'Start', 'Building.csv')
    real_copyfile = shutil.copyfile

    def user_saves_first(src, dst):
        # The user saves a route of the same name while seeding copies it.
        if os.path.basename(src) == 'Building.csv' and not os.path.exists(target):
            with open(target, 'w') as f:
                f.write('user route\n')
        return real_copyfile(src, dst)
    monkeypatch.setattr(shutil, 'copyfile', user_saves_first)
    data_dir.seed(share, data)
    with open(target) as f:
        assert f.read() == 'user route\n'


def test_concurrent_first_starts_leave_every_file_intact(tmp_path, monkeypatch):
    import threading

    share, data = str(tmp_path / 'share'), str(tmp_path / 'data')
    _install(share)
    real_copyfile = shutil.copyfile
    a_paused, b_done = threading.Event(), threading.Event()

    def copy_then_crash_in_a(src, dst):
        # Seeder A writes half of its first file, waits until seeder B has
        # finished a whole seed, then dies, as a process killed mid-copy.
        if threading.current_thread().name == 'A' and not a_paused.is_set():
            with open(src, 'rb') as f:
                data_bytes = f.read()
            with open(dst, 'wb') as f:
                f.write(data_bytes[:len(data_bytes) // 2])
            a_paused.set()
            assert b_done.wait(timeout=5)
            raise OSError('seeder A was killed')
        return real_copyfile(src, dst)
    monkeypatch.setattr(shutil, 'copyfile', copy_then_crash_in_a)
    errors = []

    def seeder_a():
        try:
            data_dir.seed(share, data)
        except OSError as error:
            errors.append(str(error))

    def seeder_b():
        assert a_paused.wait(timeout=5)
        data_dir.seed(share, data)
        b_done.set()
    threads = [threading.Thread(target=seeder_a, name='A'),
               threading.Thread(target=seeder_b, name='B')]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()

    assert errors == ['seeder A was killed']
    seeded = {}
    for rel in _shipped_files(share):
        with open(os.path.join(data, rel), 'rb') as f:
            seeded[rel] = f.read()
    assert seeded == _shipped_files(share)
    with open(os.path.join(data, 'current_map_route.yaml')) as f:
        assert yaml.safe_load(f) == {'map_file': '', 'route_file': ''}
    leftovers = [name for _d, _s, files in os.walk(data) for name in files
                 if name.endswith('.tmp')]
    assert leftovers == []
    assert os.path.isfile(os.path.join(data, data_dir.SEEDED_MARKER))


# Upgrade from the default maps folder to OPENAMROBOT_MAPS_DIR (#53).

def _upgrade_fixture(tmp_path):
    """Return (share, data, new maps dir, old maps dir, snapshot of the old folder).

    An installation seeded with the default paths, with a user map Lab/Floor
    (legacy _ros.yaml too) that is selected, a map Lab/Abs whose image path is
    absolute, and a map Lab/Clash whose name already exists in the new folder.
    """
    share, data = str(tmp_path / 'share'), str(tmp_path / 'data')
    new = str(tmp_path / 'volume' / 'maps')
    _install(share)
    data_dir.seed(share, data, environ={})
    old = os.path.join(data, 'maps')
    lab = os.path.join(old, 'Lab')
    os.makedirs(lab)
    for name, image in (('Floor', 'Floor.png'), ('Abs', os.path.join(lab, 'Abs.png')),
                        ('Clash', 'Clash.png')):
        with open(os.path.join(lab, f'{name}.yaml'), 'w') as f:
            f.write(f'image: {image}\nresolution: 0.05\n')
        with open(os.path.join(lab, f'{name}.png'), 'wb') as f:
            f.write(f'{name} old'.encode())
    with open(os.path.join(lab, 'Floor_ros.yaml'), 'w') as f:
        f.write('map_server:\n  ros__parameters:\n    yaml_filename: Floor.yaml\n')
    os.makedirs(os.path.join(new, 'Lab'))
    with open(os.path.join(new, 'Lab', 'Clash.yaml'), 'w') as f:
        f.write('image: Clash.png\nresolution: 0.1\n')
    with open(os.path.join(data, 'current_map_route.yaml'), 'w') as f:
        yaml.safe_dump({'map_file': os.path.join(lab, 'Floor.yaml'), 'route_file': ''}, f)
    return share, data, new, old, _tree(old)


def _start(share, data, new):
    return data_dir.seed(share, data, environ={'OPENAMROBOT_MAPS_DIR': new})


def _record_warnings(monkeypatch):
    """Collect data_dir's warnings directly, independent of logging setup."""
    messages = []
    monkeypatch.setattr(data_dir.LOG, 'warning', lambda msg, *a: messages.append(msg % a))
    return messages


def test_upgrade_copies_user_maps_and_repoints_the_selection(tmp_path):
    share, data, new, old, before = _upgrade_fixture(tmp_path)
    _start(share, data, new)
    for name in ('Floor.yaml', 'Floor.png', 'Floor_ros.yaml'):
        with open(os.path.join(old, 'Lab', name), 'rb') as a, \
                open(os.path.join(new, 'Lab', name), 'rb') as b:
            assert a.read() == b.read(), name
    assert os.path.isfile(os.path.join(new, 'Welcome', 'Start.yaml'))
    with open(os.path.join(data, 'current_map_route.yaml')) as f:
        assert yaml.safe_load(f)['map_file'] == os.path.join(new, 'Lab', 'Floor.yaml')
    assert _tree(old) == before  # The old folder is a backup, never changed.
    assert os.path.isfile(os.path.join(new, data_dir.MIGRATED_MARKER))


def test_absolute_image_path_is_rewritten_in_the_copy_only(tmp_path):
    share, data, new, old, before = _upgrade_fixture(tmp_path)
    _start(share, data, new)
    with open(os.path.join(new, 'Lab', 'Abs.yaml')) as f:
        assert yaml.safe_load(f)['image'] == os.path.join(new, 'Lab', 'Abs.png')
    with open(os.path.join(old, 'Lab', 'Abs.yaml')) as f:
        assert yaml.safe_load(f)['image'] == os.path.join(old, 'Lab', 'Abs.png')
    assert os.path.isfile(os.path.join(new, 'Lab', 'Abs.png'))


def test_colliding_map_is_skipped_logged_and_both_copies_kept(tmp_path, monkeypatch):
    share, data, new, old, before = _upgrade_fixture(tmp_path)
    warnings = _record_warnings(monkeypatch)
    _start(share, data, new)
    with open(os.path.join(new, 'Lab', 'Clash.yaml')) as f:
        assert f.read() == 'image: Clash.png\nresolution: 0.1\n'
    assert not os.path.exists(os.path.join(new, 'Lab', 'Clash.png'))
    assert _tree(old) == before
    assert any('Lab/Clash not migrated' in m for m in warnings), warnings
    assert os.path.isfile(os.path.join(new, 'Lab', 'Floor.yaml'))


def test_selection_of_a_skipped_map_is_left_unchanged_and_logged(tmp_path, monkeypatch):
    share, data, new, old, before = _upgrade_fixture(tmp_path)
    warnings = _record_warnings(monkeypatch)
    selected = os.path.join(old, 'Lab', 'Clash.yaml')
    with open(os.path.join(data, 'current_map_route.yaml'), 'w') as f:
        yaml.safe_dump({'map_file': selected, 'route_file': ''}, f)
    _start(share, data, new)
    with open(os.path.join(data, 'current_map_route.yaml')) as f:
        assert yaml.safe_load(f)['map_file'] == selected
    assert any('Selected map' in m and 'left as is' in m for m in warnings), warnings


def test_second_start_does_not_migrate_again(tmp_path):
    share, data, new, old, before = _upgrade_fixture(tmp_path)
    _start(share, data, new)
    floor = os.path.join(new, 'Lab', 'Floor.yaml')
    assert os.path.isfile(floor)
    os.remove(floor)  # The user deletes the migrated copy.
    _start(share, data, new)
    assert not os.path.exists(floor)
    with open(os.path.join(data, 'current_map_route.yaml')) as f:
        assert yaml.safe_load(f)['map_file'] == floor


def test_concurrent_first_starts_migrate_safely(tmp_path, monkeypatch):
    import threading

    share, data, new, old, before = _upgrade_fixture(tmp_path)
    real_copyfile = shutil.copyfile
    a_paused, b_done = threading.Event(), threading.Event()

    def copy_then_crash_in_a(src, dst):
        # Node A writes half of the Floor image, waits until node B has run a
        # whole first start, then dies, as a process killed mid-copy.
        if (threading.current_thread().name == 'A' and src.endswith('Floor.png')
                and not a_paused.is_set()):
            with open(src, 'rb') as f:
                content = f.read()
            with open(dst, 'wb') as f:
                f.write(content[:len(content) // 2])
            a_paused.set()
            assert b_done.wait(timeout=5)
            raise OSError('node A was killed')
        return real_copyfile(src, dst)
    monkeypatch.setattr(shutil, 'copyfile', copy_then_crash_in_a)
    errors = []

    def node_a():
        try:
            _start(share, data, new)
        except OSError as error:
            errors.append(str(error))

    def node_b():
        assert a_paused.wait(timeout=5)
        _start(share, data, new)
        b_done.set()
    threads = [threading.Thread(target=node_a, name='A'),
               threading.Thread(target=node_b, name='B')]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()

    assert errors == ['node A was killed']
    for name in ('Floor.yaml', 'Floor.png', 'Floor_ros.yaml'):
        with open(os.path.join(old, 'Lab', name), 'rb') as a, \
                open(os.path.join(new, 'Lab', name), 'rb') as b:
            assert a.read() == b.read(), name
    leftovers = [n for _d, _s, files in os.walk(str(tmp_path)) for n in files
                 if n.endswith('.tmp')]
    assert leftovers == []
    with open(os.path.join(data, 'current_map_route.yaml')) as f:
        assert yaml.safe_load(f)['map_file'] == os.path.join(new, 'Lab', 'Floor.yaml')
    assert _tree(old) == before
