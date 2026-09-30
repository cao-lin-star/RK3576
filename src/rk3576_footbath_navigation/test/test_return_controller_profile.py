"""No-motion checks of precise staging in both saved-map navigation entry paths."""
import copy
import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml
from launch import LaunchContext


SOURCE = Path(__file__).resolve().parents[2]
PARAMS = SOURCE / 'rk3576_footbath_navigation/config/nav2_params.yaml'
LAUNCH = SOURCE / 'rk3576_footbath_navigation/launch/navigation.launch.py'
spec = importlib.util.spec_from_file_location('precise_return_launch', LAUNCH)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def generated_profile(context):
    action = module.navigation_stack(context)[0]
    params = dict(action.launch_arguments)['params_file']
    # IncludeLaunchDescription normalizes substitutions on some launch versions.
    if isinstance(params, (list, tuple)):
        assert len(params) == 1
        params = params[0]
    assert isinstance(params, module.NavigationReturnParams)
    return Path(params.perform(context))


def remove_generated(path, data):
    for key in ('default_nav_to_pose_bt_xml', 'default_nav_through_poses_bt_xml'):
        Path(data['bt_navigator']['ros__parameters'][key]).unlink()
    path.unlink()


@pytest.mark.parametrize('return_session', ['false', 'true'])
def test_both_navigation_entry_paths_have_separate_precise_plugins(return_session):
    context = LaunchContext()
    context.launch_configurations.update(params_file=str(PARAMS), dock_return=return_session)
    before = PARAMS.read_bytes()
    base = yaml.safe_load(before)
    path = generated_profile(context)
    configured = yaml.safe_load(path.read_text())
    try:
        control = configured['controller_server']['ros__parameters']
        original = base['controller_server']['ros__parameters']
        assert control['FollowPath'] == original['FollowPath']
        assert control['general_goal_checker'] == original['general_goal_checker']
        assert control['FollowPath']['xy_goal_tolerance'] == .15
        assert control['general_goal_checker']['xy_goal_tolerance'] == .15
        assert control['general_goal_checker']['yaw_goal_tolerance'] == .2
        expected_return = copy.deepcopy(original['FollowPath'])
        expected_return['xy_goal_tolerance'] = .03
        assert control['ReturnPath'] == expected_return
        assert control['return_goal_checker']['xy_goal_tolerance'] == .03
        assert control['return_goal_checker']['yaw_goal_tolerance'] == 3.142
        assert control['controller_plugins'] == ['FollowPath', 'ExplorePath', 'ReturnPath']
        assert control['goal_checker_plugins'] == ['general_goal_checker', 'exploration_goal_checker', 'return_goal_checker']
        # The observed 12.87 cm false-success position must no longer satisfy
        # precise staging; its 3 cm criterion leaves the 4 cm dock gate intact.
        observed_error = .128738
        assert observed_error > control['return_goal_checker']['xy_goal_tolerance']
        assert observed_error < control['general_goal_checker']['xy_goal_tolerance']
        assert control['return_goal_checker']['xy_goal_tolerance'] < .04
        navigator = configured['bt_navigator']['ros__parameters']
        for key in ('default_nav_to_pose_bt_xml', 'default_nav_through_poses_bt_xml'):
            followers = list(ET.parse(navigator[key]).iter('FollowPath'))
            assert followers
            assert all(p.get('controller_id') == 'FollowPath' for p in followers)
            assert all(p.get('goal_checker_id') == 'general_goal_checker' for p in followers)
        # No footprint, inflation, speed, obstacle or localization relaxation.
        for name, value in base.items():
            if name not in ('controller_server', 'bt_navigator'):
                assert configured[name] == value
        assert PARAMS.read_bytes() == before
    finally:
        remove_generated(path, configured)


def test_return_tree_is_explicit_and_keeps_bounded_recovery():
    directory = SOURCE / 'rk3576_footbath_exploration'
    tree = ET.parse(directory / 'behavior_trees/home_return_precise.xml')
    paths = list(tree.iter('FollowPath'))
    assert len(paths) == 1
    assert paths[0].get('controller_id') == 'ReturnPath'
    assert paths[0].get('goal_checker_id') == 'return_goal_checker'
    assert not list(tree.iter('Spin'))
    assert tree.find('.//RecoveryNode').get('number_of_retries') == '1'
    supervisor = (directory / 'rk3576_footbath_exploration/supervisor.py').read_text()
    assert "'behavior_trees', home_behavior_tree_name(self.home.dock.enabled)" in supervisor
    # The mapping stack still has one ordinary plugin and hands off to saved
    # navigation before sending the return BT (covered by HomeReturn tests).
    explore = ET.parse(directory / 'behavior_trees/exploration_limited_recovery.xml')
    assert all(p.get('controller_id') == 'ExplorePath' for p in explore.iter('FollowPath'))
    config=yaml.safe_load(PARAMS.read_text())['controller_server']['ros__parameters']
    assert 'RotateToGoal' not in config['ExplorePath']['critics']
    assert 'GoalAlign' not in config['ExplorePath']['critics']
    assert config['exploration_goal_checker']['yaw_goal_tolerance']>3.14159
    assert 'RotateToGoal' in config['FollowPath']['critics']
    assert all(p.get('goal_checker_id') == 'exploration_goal_checker' for p in explore.iter('FollowPath'))


def test_custom_default_tree_policy_is_preserved(tmp_path):
    custom = tmp_path / 'custom.xml'
    custom.write_text('<root main_tree_to_execute="MainTree"><BehaviorTree ID="MainTree">'
                      '<Sequence><Wait wait_duration="7.0"/>'
                      '<FollowPath path="{path}" controller_id="FollowPath"/>'
                      '</Sequence></BehaviorTree></root>')
    data = yaml.safe_load(PARAMS.read_text())
    data['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml'] = str(custom)
    input_path = tmp_path / 'params.yaml'
    input_path.write_text(yaml.safe_dump(data))
    context = LaunchContext()
    context.launch_configurations.update(params_file=str(input_path), dock_return='false')
    output = generated_profile(context)
    result = yaml.safe_load(output.read_text())
    try:
        tree = ET.parse(result['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml'])
        assert tree.find('.//Wait').get('wait_duration') == '7.0'
        assert tree.find('.//FollowPath').get('goal_checker_id') == 'general_goal_checker'
        assert custom.read_text().count('goal_checker_id') == 0
    finally:
        remove_generated(output, result)


@pytest.mark.parametrize('controller,checker', [('', ''), ('FollowPath', 'general_goal_checker')])
def test_empty_ids_are_made_explicit_and_explicit_ids_preserved(tmp_path, controller, checker):
    source = tmp_path / 'tree.xml'
    source.write_text(f'<root><FollowPath controller_id="{controller}" '
                      f'goal_checker_id="{checker}"/></root>')
    output = Path(module.explicit_navigation_tree(
        source, ['FollowPath', 'ExplorePath', 'ReturnPath'], ['general_goal_checker', 'exploration_goal_checker', 'return_goal_checker']))
    try:
        follower = ET.parse(output).find('.//FollowPath')
        assert follower.get('controller_id') == 'FollowPath'
        assert follower.get('goal_checker_id') == 'general_goal_checker'
    finally:
        output.unlink()


@pytest.mark.parametrize('selection', ['missing_checker', '{unproven_dynamic_checker}'])
def test_invalid_custom_selection_fails_closed(tmp_path, selection):
    source = tmp_path / 'tree.xml'
    source.write_text(f'<root><FollowPath goal_checker_id="{selection}"/></root>')
    with pytest.raises(ValueError, match='unsupported goal_checker_id'):
        module.explicit_navigation_tree(source, ['FollowPath'], ['general_goal_checker'])


def test_reserved_return_profile_cannot_be_silently_overwritten(tmp_path):
    data = yaml.safe_load(PARAMS.read_text())
    data['controller_server']['ros__parameters']['ReturnPath'] = {'xy_goal_tolerance': .5}
    source = tmp_path / 'bad.yaml'
    source.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match='reserved'):
        module.NavigationReturnParams(source_file=str(source), param_rewrites={}).perform(LaunchContext())


def test_failed_second_custom_tree_cleans_generated_files(tmp_path, monkeypatch):
    data = yaml.safe_load(PARAMS.read_text())
    data['bt_navigator']['ros__parameters']['default_nav_through_poses_bt_xml'] = str(tmp_path/'missing.xml')
    source = tmp_path / 'params.yaml'
    source.write_text(yaml.safe_dump(data))
    monkeypatch.setattr(module.tempfile, 'tempdir', str(tmp_path))
    with pytest.raises(FileNotFoundError):
        module.NavigationReturnParams(source_file=str(source), param_rewrites={}).perform(LaunchContext())
    assert list(tmp_path.iterdir()) == [source]
