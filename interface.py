from pathlib import Path
from shutil import move
from test_generator import TestGenerator
import os
from utils.xml_to_bng_files import convert_test


def move_files(dbe, dbc):
    """Moves specified xml files to the done folder.
    :param dbc: Full path to the dbc file.
    :param dbe: Full path to the dbe file.
    :return: Void.
    """
    destination_path = os.path.dirname(os.path.realpath(__file__)) + "\\done"
    if not os.path.exists(destination_path):
        os.mkdir(destination_path)
    filename_dbc = os.path.basename(os.path.normpath(dbc))
    filename_dbe = os.path.basename(os.path.normpath(dbe))
    move(dbc, os.path.join(destination_path, filename_dbc))
    move(dbe, os.path.join(destination_path, filename_dbe))


if __name__ == '__main__':
    gen = TestGenerator("easy")         # Parameter is optional, you can also call gen.set_difficulty(str)
    gen.set_files_name("test_case")     # This method call is optional

    while True:
        for paths in gen.getTest():
            dbe = paths[0]
            dbc = paths[1]
            convert_test(dbc, dbe)
            # All your code goes here
            # Be sure to call onTestFinished(sid, vid)
