from model import PubsubTopic
import ipc
from motors import AcornMotorInterface
import math


def test_tick(fixture_path, mocker):
    ps = ipc.MemPubSub()
    mocker.patch('ipc.ZMQPub', lambda _: ps)
    mocker.patch('ipc.LeakyZMQSub', lambda: ps)
    sub = ps.sub(None, PubsubTopic.MOTORS_TO_REMOTE_CONTROL)

    motor = AcornMotorInterface(simulated_hardware=True)
    motor.tick()
    assert sub() is not None

    ps.pub(PubsubTopic.REMOTE_CONTROL_TO_MOTORS, {corner: (math.pi, 2) for corner in ["front_left", "front_right", "rear_left", "rear_right"]})
    motor.tick()
    assert sub() is not None, "should have sent the commands to motor"
    for d in motor.odrives:
        assert d.position == 180
        if "rear" in d.name:
            assert d.velocity == -2000
        else:
            assert d.velocity == 2000
