import math
import g

#local
def init_scene():
    g.set_duration(0.0)
    #g.set_renderer("Face-Based Volume")
    #g.set_mesh("2011 - All-Hex Mesh Generation via Volumetric PolyCube Deformation", "anc101_a1", "mesh")
    #g.set_transfer_function("Standard_anc_vid.xml")

def move_camera_circle():
    total_time = 30.0
    radius = 0.8

    camera_positions = []
    camera_yaw_pitch = []

    n = 60
    for i in range(n+1):
        t = float(i) / float(n)
        angle = t * 2.0 * math.pi + math.pi / 2.0
        camera_positions.append((radius * math.cos(angle), 0.0, radius * math.sin(angle)))
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
