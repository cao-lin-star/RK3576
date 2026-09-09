import hashlib
import json

from rk3576_footbath_mobile.map_store import MapStore


def fixture(tmp_path):
    path=tmp_path/'a.yaml'
    path.write_text('image: a.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n')
    image=tmp_path/'a.pgm'; image.write_bytes(b'P5\n1 1\n255\n\xff')
    data=dict(version=1,frame_id='map',pose=dict(x=1.,y=2.,yaw=.3),
              geometry=dict(resolution=.05,origin=[0,0,0]),
              image_sha256=hashlib.sha256(image.read_bytes()).hexdigest())
    (tmp_path/'a.home.json').write_text(json.dumps(data))
    return MapStore([tmp_path]),path


def test_home_survives_rename_and_trash(tmp_path):
    store,path=fixture(tmp_path)
    home=store.home(path)
    assert home==dict(x=1.,y=2.,yaw=.3)
    renamed=store.rename(path,'b')
    assert store.home(renamed)==home
    destination=store.trash(renamed)
    from pathlib import Path
    assert (Path(destination)/'b.home.json').is_file()


def test_changed_image_rejects_stale_home(tmp_path):
    store,path=fixture(tmp_path)
    (tmp_path/'a.pgm').write_bytes(b'changed')
    assert store.home(path) is None


def test_changed_geometry_rejects_stale_home(tmp_path):
    store,path=fixture(tmp_path)
    path.write_text('image: a.pgm\nresolution: 0.1\norigin: [0, 0, 0]\n')
    assert store.home(path) is None


def test_old_map_and_invalid_pose_do_not_guess_origin(tmp_path):
    store,path=fixture(tmp_path)
    (tmp_path/'a.home.json').unlink()
    assert store.home(path) is None
    (tmp_path/'a.home.json').write_text('{bad json')
    assert store.home(path) is None


def test_fallback_only_before_user_initialization():
    from unittest.mock import Mock
    from rk3576_footbath_mobile.gateway import Gateway
    g=Mock(); g.mode='navigation'; g.saved_map_home=dict(x=1.,y=2.,yaw=.3)
    g.return_transition.pending=None; g.count_subscribers.return_value=1
    g.healthy.return_value=True
    for state in ('waiting','confirmed','failed'):
        g.initial_state=state
        Gateway._try_saved_initial(g)
    g._initial.assert_not_called()
    g.initial_state='not_set'
    Gateway._try_saved_initial(g)
    g._initial.assert_called_once_with(g.saved_map_home,source='saved_home')
    g.nav.send_goal_async.assert_not_called()
