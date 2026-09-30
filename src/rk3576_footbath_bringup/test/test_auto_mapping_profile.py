from pathlib import Path
import xml.etree.ElementTree as ET
import yaml

ROOT = Path(__file__).resolve().parents[2]


def test_stable_mapping_controller_and_no_glass_costmap():
    launch = (ROOT/'rk3576_footbath_bringup/launch/auto_mapping.launch.py').read_text()
    assert 'RotationShimController' not in launch
    assert 'exploration_nav_params' not in launch
    params = yaml.safe_load((ROOT/'rk3576_footbath_navigation/config/nav2_params.yaml').read_text())
    control = params['controller_server']['ros__parameters']
    assert control['FollowPath']['plugin'] == 'dwb_core::DWBLocalPlanner'
    assert control['FollowPath']['max_vel_x'] == .20
    assert control['FollowPath']['sim_time'] == 1.5
    for name in ('local_costmap', 'global_costmap'):
        plugins = params[name][name]['ros__parameters']['plugins']
        assert 'side_ultrasonic_layer' not in plugins
        assert 'glass_range_layer' not in plugins
        assert 'hazard_layer' in plugins
    tree = ET.parse(ROOT/'rk3576_footbath_exploration/behavior_trees/exploration_limited_recovery.xml')
    assert tree.find('.//RecoveryNode').get('number_of_retries') == '1'
    assert tree.find('.//FollowPath').get('goal_checker_id') == 'exploration_goal_checker'

    assert tree.find('.//RateController') is None
    assert tree.find('.//PipelineSequence') is None
    assert tree.find('.//BackUp') is None
    assert tree.find('.//Spin') is None
