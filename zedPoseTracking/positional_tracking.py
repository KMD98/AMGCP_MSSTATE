########################################################################
#
# Copyright (c) 2021, STEREOLABS.
#
# All rights reserved.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
########################################################################

"""
    This sample shows how to track the position of the ZED camera 
    and displays it in a OpenGL window.
"""

import sys
import ogl_viewer.tracking_viewer as gl
import pyzed.sl as sl
from math import degrees


if __name__ == "__main__":

    init_params = sl.InitParameters(camera_resolution=sl.RESOLUTION.VGA,
                                 coordinate_units=sl.UNIT.METER,
                                 coordinate_system=sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP)
                                 
    # If applicable, use the SVO given as parameter
    # Otherwise use ZED live stream
    if len(sys.argv) == 2:
        filepath = sys.argv[1]
        print("Using SVO file: {0}".format(filepath))
        init_params.set_from_svo_file(filepath)

    zed = sl.Camera()
    status = zed.open(init_params)
    if status != sl.ERROR_CODE.SUCCESS:
        print(repr(status))
        exit()

    #create class object.
    initial_position = sl.Transform()
    # Set the initial positon of the Camera Frame at 0.5m above the World Frame
    initial_translation = sl.Translation()
    initial_orientation = sl.Orientation()
    initial_translation.init_vector(0,0,0.5)
    initial_orientation.init_vector(0,0,0,1) #feed gps rtk heading before start
    initial_position.set_translation(initial_translation) #set the object at 0.5m above ground
    initial_position.set_orientation(initial_orientation) #set the object at x degrees from y axis

    tracking_params = sl.PositionalTrackingParameters(initial_position) #begin tracking with the inital position
    zed.enable_positional_tracking(tracking_params)

    runtime = sl.RuntimeParameters()
    camera_pose = sl.Pose()

    camera_info = zed.get_camera_information()
    translation_object = sl.Translation()

    # Create OpenGL viewer
    viewer = gl.GLViewer()
    viewer.init(camera_info.camera_model)

    py_translation = sl.Translation()
    pose_data = sl.Transform()

    text_translation = ""
    text_rotation = ""

    while viewer.is_available():
        if zed.grab(runtime) == sl.ERROR_CODE.SUCCESS:
            tracking_state = zed.get_position(camera_pose, sl.REFERENCE_FRAME.WORLD)
            if tracking_state == sl.POSITIONAL_TRACKING_STATE.OK:
                rotation = camera_pose.get_rotation_vector() #euler angles in rad in form pitch roll yaw
                translation = camera_pose.get_translation(py_translation) #current position vector in form x,y,z right handed z up
                text_rotation = str((round(degrees(round(rotation[0], 2)),2), round(degrees(round(rotation[1], 2)),2), round(degrees(round(rotation[2], 2)),2)))
                text_translation = str((round(translation.get()[0], 2), round(translation.get()[1], 2), round(translation.get()[2], 2)))
                pose_data = camera_pose.pose_data(sl.Transform())
            viewer.updateData(pose_data, text_translation, text_rotation, tracking_state)

    viewer.exit()
    zed.close()

