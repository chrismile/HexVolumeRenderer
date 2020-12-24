import math
import g

#local
def init_scene():
    g.set_duration(0.0)
    #g.set_renderer("Face-Based Volume")
    #g.set_mesh("0003 - Deformation Large", "bumpy_torus_R620_50M.zip/bumpy_torus_R620_50M", "vtk")
    #g.set_transfer_function("BumpyTorus24M_MultiVar2.xml")

def move_camera_circle():
    total_time = 40.0
    radius = 0.65

    camera_positions = []
    camera_yaw_pitch = []

    n = 60
    for i in range(n+1):
        t = float(i) / float(n)
        angle = t * 2.0 * math.pi + math.pi / 2.0
        camera_positions.append((radius * math.cos(angle), -0.024, radius * math.sin(angle)))
        camera_yaw_pitch.append((math.pi + angle, 0.0))

    # Move lens to start position.
    g.set_duration(0)
    g.set_camera_position(camera_positions[0])
    g.set_camera_yaw_pitch_rad(camera_yaw_pitch[0])

    for i in range(1, len(camera_positions)):
        g.set_duration(total_time / float(n))
        g.set_camera_position(camera_positions[i])
        g.set_camera_yaw_pitch_rad(camera_yaw_pitch[i])


def replay():
    init_scene()
    move_camera_circle()
