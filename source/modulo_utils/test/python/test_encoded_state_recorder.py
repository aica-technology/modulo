import os
import shutil
import time

import clproto
import state_representation as sr
from modulo_interfaces.msg import EncodedState
from modulo_utils.encoded_state_recorder import (EncodedStateRecorder,
                                                 read_encoded_state_recording,
                                                 read_recording_directory)

directory = "/tmp/recording"


def test_encoded_state_recorder():
    if os.path.exists(directory) and os.path.isdir(directory):
        shutil.rmtree(directory)
    current_time = f"{time.time()}"
    random_state = sr.CartesianState().Random("test")
    msg = EncodedState()
    msg.data = clproto.encode(random_state, clproto.MessageType.CARTESIAN_STATE_MESSAGE)
    with EncodedStateRecorder(os.path.join(directory, current_time)) as rec:
        rec.write(msg)

    data = read_encoded_state_recording(os.path.join(directory, current_time))
    assert data
    assert len(data) == 1
    assert data[0]["state"].get_name() == random_state.get_name()

    full_data = read_recording_directory(directory)
    assert len(full_data) == 1
    assert full_data[f"{current_time}"]
