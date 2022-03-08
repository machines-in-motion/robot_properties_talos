"""gepetto_gui_loader

Load the robot in the gepetto-gui.

License: BSD 3-Clause License
Copyright (C) 2018-2019, New York University , Max Planck Gesellschaft
Copyright note valid unless otherwise stated in individual files.
All rights reserved.
"""

import time
from config import TalosArmConfig
from py_gepetto_gui_helper.gepetto_gui_scene import GepettoGuiScene
from py_gepetto_gui_helper.robot_visual import RobotVisual
from py_gepetto_gui_helper.frame import Frame

def create_scene():
    """
    Just create a scene for the talos_arm to be in
    """
    return GepettoGuiScene("talos_arm_scene", "talos_arm_window")


def load_talos_arm_in_gepetto_gui(gepetto_scene, robot_name):
    """
    Load the talos_arm meshes in the scene
    """
    config = TalosArmConfig()
    return RobotVisual(gepetto_scene, robot_name, config.urdf_path,
                       config.meshes_path)

def display_talos_arm_in_gepetto_gui(launch_gepetto_gui_exec=False):
    """
    Uses the function above to load the urdf model of talos_arm in gepetto gui
    and load it in the initial configuration
    """

    if launch_gepetto_gui_exec:
        # create a new window
        gepetto_gui_process = GepettoGuiScene.open_gepetto_gui()

    # create a scene in it
    gui_scene = create_scene()
    # load the robot
    talos_arm_visual = load_talos_arm_in_gepetto_gui(gui_scene, "talos_arm")
    # place the robot in initial configuration
    config = TalosArmConfig()
    talos_arm_visual.display(config.q0)
    # place the world frame
    world_frame = Frame(gui_scene)

    if launch_gepetto_gui_exec:
        # close the window after little while
        time.sleep(5)
        GepettoGuiScene.close_gepetto_gui(gepetto_gui_process)

    return gui_scene, talos_arm_visual, world_frame

if __name__ == "__main__":
    display_talos_arm_in_gepetto_gui()
    

