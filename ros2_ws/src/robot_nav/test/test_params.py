"""config/nav2_params.yaml の書きまちがいを見つけるテスト(Nav2 がなくても動く)."""

import os

import yaml

CONFIG = os.path.join(os.path.dirname(__file__), '..', 'config', 'nav2_params.yaml')


def load():
    with open(CONFIG, encoding='utf-8') as f:
        return yaml.safe_load(f)


def params(doc, *names):
    for name in names:
        doc = doc[name]
    return doc['ros__parameters']


def test_plugin_names_match_behavior_tree():
    """標準のビヘイビアツリーは GridBased と FollowPath という名前で部品を呼ぶ."""
    doc = load()
    assert params(doc, 'planner_server')['planner_plugins'] == ['GridBased']
    controller = params(doc, 'controller_server')
    # FollowPath は標準のビヘイビアツリーの名前。FollowPathTurning はロボットコンソールが選ぶ
    # (swerve_drive の console_state.py の CONTROLLER_KEEP / CONTROLLER_TURNING と同じ名前)
    assert controller['controller_plugins'] == ['FollowPath', 'FollowPathTurning']
    for key in ('progress_checker_plugins', 'goal_checker_plugins', 'controller_plugins'):
        for name in controller[key]:
            assert 'plugin' in controller[name], name


def test_turning_controller_matches_the_normal_one():
    """2 つの走り方は、速さの上限などがそろっていて、向きの気にし方だけが違う."""
    controller = params(load(), 'controller_server')
    keep, turning = controller['FollowPath'], controller['FollowPathTurning']
    for key in ('motion_model', 'vx_max', 'vx_min', 'vy_max', 'wz_max', 'ax_max', 'ay_max',
                'az_max'):
        assert keep[key] == turning[key], key
    assert keep['GoalAngleCritic']['threshold_to_consider'] < 1.0
    assert turning['GoalAngleCritic']['threshold_to_consider'] >= 10.0
    assert 'TwirlingCritic' not in turning['critics']     # 走りながら回るのをじゃましない


def test_robot_radius_is_the_same_everywhere():
    doc = load()
    local = params(doc, 'local_costmap', 'local_costmap')
    world = params(doc, 'global_costmap', 'global_costmap')
    assert local['robot_radius'] == world['robot_radius']
    for costmap in (local, world):
        assert costmap['inflation_layer']['inflation_radius'] > costmap['robot_radius']
        for name in costmap['plugins']:
            assert 'plugin' in costmap[name], name


def test_holonomic_and_field_maps():
    doc = load()
    controller = params(doc, 'controller_server')
    for name in controller['controller_plugins']:
        assert controller[name]['motion_model'] == 'Omni'
    amcl = params(doc, 'amcl')
    assert amcl['robot_model_type'] == 'nav2_amcl::OmniMotionModel'
    assert amcl['map_topic'] == '/lidar_map'        # LiDAR に映る物だけの地図
    for name in ('local_costmap', 'global_costmap'):
        assert params(doc, name, name)['static_layer']['map_topic'] == '/map'
    behaviors = params(doc, 'behavior_server')
    # assisted_teleop は cmd_vel_teleop を購読するので、手で操縦する指令と混ざらないように使わない
    assert 'assisted_teleop' not in behaviors['behavior_plugins']
    for name in behaviors['behavior_plugins']:
        assert 'plugin' in behaviors[name], name
