from types import SimpleNamespace as NS
from unittest.mock import Mock
from geometry_msgs.msg import Twist
from rk3576_footbath_safety.velocity_limiter import AutoCmdVelLimiter


def limiter():
    node=object.__new__(AutoCmdVelLimiter)
    node._recovery_active=True
    node._forward_command=Mock()
    node._publish_zero=Mock()
    return node


def test_nav_commands_cannot_override_reverse():
    n=limiter(); m=Twist(); m.linear.x=.2
    n._on_command(m)
    n._forward_command.assert_not_called()


def test_recovery_only_allows_bounded_straight_reverse():
    n=limiter()
    for v,w in [(.01,0.),(-.04,0.),(-.03,.01),(float('nan'),0.)]:
        m=Twist(); m.linear.x=v; m.angular.z=w
        n._on_recovery(m)
    n._forward_command.assert_not_called()
    m=Twist(); m.linear.x=-.03
    n._on_recovery(m)
    n._forward_command.assert_called_once()


def test_repeated_mode_heartbeat_does_not_pulse_zero():
    n=limiter(); n._on_recovery_active(NS(data=True))
    n._publish_zero.assert_not_called()
    n._on_recovery_active(NS(data=False))
    n._publish_zero.assert_called_once()
    assert n._last_command is None
