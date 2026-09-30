"""Offline pseudo-MCU integration. Never opens a real serial device."""
import binascii
import os
import pty
import struct
import subprocess
import time
import pytest
import rclpy
from geometry_msgs.msg import Twist


@pytest.mark.parametrize('enabled',[0,1])
def test_confirmation_gates_commands(enabled):
    binary=os.environ.get('FOOTBATH_TEST_BRIDGE')
    if not binary: pytest.skip('set FOOTBATH_TEST_BRIDGE to a built bridge executable')
    assert os.environ.get('ROS_LOCALHOST_ONLY')=='1'
    assert os.environ.get('ROS_DOMAIN_ID')=='179'
    master,slave=pty.openpty()
    os.set_blocking(master,False)
    rclpy.init()
    node=rclpy.create_node('side_config_pty_test')
    publisher=node.create_publisher(Twist,'/test_side_config_cmd',10)
    env=dict(os.environ,FOOTBATH_SIDE_ULTRASONIC_ENABLED=str(enabled))
    proc=subprocess.Popen([binary,'--ros-args','-p','serial.port:='+os.ttyname(slave),
        '-p','topics.cmd_vel:=/test_side_config_cmd'],env=env,
        stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
    buffer=bytearray()
    def send(kind,payload):
        body=struct.pack('<BBHH',1,kind,0,len(payload))+payload
        os.write(master,b'\xaa\x55'+body+struct.pack('<H',binascii.crc_hqx(body,0xffff)))
    def run(seconds,ack=None):
        frames=[];end=time.monotonic()+seconds
        while time.monotonic()<end:
            send(3,struct.pack('<IH',1000,0))
            if ack is not None: send(10,ack)
            msg=Twist();msg.linear.x=.05;publisher.publish(msg)
            rclpy.spin_once(node,timeout_sec=.03)
            try: buffer.extend(os.read(master,65536))
            except BlockingIOError: pass
            while len(buffer)>=10:
                if buffer[:2]!=b'\xaa\x55': del buffer[0];continue
                length=struct.unpack_from('<H',buffer,6)[0]
                if len(buffer)<length+10: break
                frames.append((buffer[3],bytes(buffer[8:8+length])))
                del buffer[:length+10]
        return frames
    def moving(frames):
        return any(kind==1 and any(struct.unpack('<ff',payload)) for kind,payload in frames)
    try:
        frames=run(1.5)
        assert not moving(frames)
        assert (9,bytes([1,enabled])) in frames
        assert not moving(run(.5,b'\x02')) # Malformed state cannot acknowledge.
        assert not moving(run(.5,bytes([1-enabled])))
        assert moving(run(.8,bytes([enabled])))
        run(.9) # Let feedback expire, discard the allowed initial 600 ms.
        frames=run(.4)
        assert not moving(frames)
        assert any(kind==4 for kind,_ in frames)
        assert moving(run(.8,bytes([enabled])))
    finally:
        proc.terminate()
        try: proc.wait(timeout=3)
        except subprocess.TimeoutExpired: proc.kill();proc.wait()
        node.destroy_node();rclpy.shutdown()
        os.close(master);os.close(slave)
