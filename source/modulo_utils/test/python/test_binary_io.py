import time

import clproto
import state_representation as sr
from modulo_interfaces.msg import EncodedState
from modulo_utils.binary_io import BinaryReader, BinaryRecorder


def test_binary_io():
    current_time = time.time()
    random_state = sr.CartesianState().Random("test")
    recorder = BinaryRecorder(f"/tmp/{current_time}")
    recorder.open()
    msg = EncodedState()
    msg.data = clproto.encode(random_state, clproto.MessageType.CARTESIAN_STATE_MESSAGE)
    recorder.write(msg)
    recorder.close()

    reader = BinaryReader(f"/tmp/{current_time}")
    data = reader.read()
    assert data
    assert len(data) == 1
    assert data[0]["state"].get_name() == random_state.get_name()
