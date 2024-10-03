import os
import struct
import time
from datetime import datetime
from io import BufferedWriter
from typing import List

import clproto
from modulo_interfaces.msg import EncodedState


class BinaryRecorder():
    def __init__(self, filepath: str):
        """
        Construct the BinaryRecorder. By calling first open() and then write(msg) successively, the BinaryRecorder will
        write the provided message as binary data to a file under the desired path. 

        :param filepath: The full path of the recording file
        """
        self._filepath = filepath
        if not self._filepath.endswith(".bin"):
            self._filepath += ".bin"
        os.makedirs(os.path.dirname(self._filepath), exist_ok=True)
        self._file: BufferedWriter

    def open(self):
        """
        Open the file for writing.
        """
        self._file = open(self._filepath, 'wb')

    def write(self, msg: EncodedState):
        """
        Write an EncodedState to the file.
        """
        self._file.write(clproto.pack_fields([struct.pack('d', time.time()), bytes(msg.data)]))

    def close(self):
        """
        Close the file after writing.
        """
        self._file.close()


def read_binary_file(filepath: str):
    """
    Decode the binary data of a file created by a BinaryRecorder and return its content.

    :param filepath: The full path of the recorded file
    """
    if not filepath.endswith(".bin"):
        filepath += ".bin"
    data = []
    block_size = 4
    with open(filepath, 'rb') as f:
        field_bytes = f.read(block_size)
        while field_bytes:
            package = field_bytes
            # read the size of the timestamp
            timestamp_size_bytes = f.read(block_size)
            package += timestamp_size_bytes
            timestamp_size = int.from_bytes(timestamp_size_bytes, byteorder='little', signed=False)
            # read the size of the state
            state_size_bytes = f.read(block_size)
            package += state_size_bytes
            state_size = int.from_bytes(state_size_bytes, byteorder='little', signed=False)
            # read the whole package
            package += f.read(timestamp_size)
            package += f.read(state_size)
            # decode the package with clproto
            fields = clproto.unpack_fields(package)
            timestamp = datetime.fromtimestamp(struct.unpack('d', fields[0])[0])
            state = clproto.decode(fields[1])
            data.append({'timestamp': timestamp, 'state': state})
            # read again the first 4 bytes
            field_bytes = f.read(block_size)
    return data


def read_directory(directory, filenames: List[str] = None) -> dict:
    """
    Read a directory of recorded files.

    :param directory: The path to recording directory
    :param filenames: If provided, only read the given files in the recording directory
    :return: The content of the files as a dict with {name: data}
    """
    if not os.path.isdir(directory):
        return {}
    data = {}
    if filenames is None:
        filenames = next(os.walk(directory))[2]
    for file in filenames:
        if file.endswith(".bin"):
            file = file[:-4]
        reader = BinaryReader(os.path.join(directory, file))
        data[file] = reader.read()
    return data


def read_directories(directory: str, recording_directories: List[str] = None, filenames: List[str] = None) -> dict:
    """
    Read a directory tree of recorded files.

    :param directory: The path to the directry containing one or several recording directories
    :param recording_directories: If provided, only read the files in the given recording directories
    :param filenames: If provided, only read the given files in each recording directory
    :return: The dataset of recorded states per recording and filename as a dict of dict {recording: {name: data}}
    """
    if not os.path.isdir(directory):
        return {}
    data = {}
    if recording_directories is None:
        recording_directories = next(os.walk(directory))[1]
    for recording_dir in recording_directories:
        recording_data = read_directory(os.path.join(directory, recording_dir), filenames)
        if not recording_data:
            continue
        data[recording_dir] = recording_data
    return data
