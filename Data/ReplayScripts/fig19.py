import math
import g

#local
def init_scene():
    position_middle = (0.098, -0.074, 0.083)

    g.set_duration(0.0)
    g.set_camera_position(0.09029, -0.0507826, 0.187197)
    g.set_camera_yaw_pitch_rad(-1.62982, -0.165676)
    g.set_renderer("ClearView (Unified)")
    g.set_mesh("2016 - All-Hex Meshing Using Closed-Form Induced Polycube", "grayloc-hex", "vtk")
    g.set_transfer_function("Standard_grayloc.xml")
    g.set_duration(0.0)
    g.set_rendering_algorithm_settings({
        "line_width": 0.0016,
        "lod_value_context": 0.0,
        "lod_value_focus": 0.15,
        "use_screen_space_lens": False,
        "screen_space_lens_radius": 0.0,
        "screen_space_lens_position": (10.5, 10.5),
        "focus_outline_width": 3.2,
        "clip_focus_outline": False,
        "focus_outline_color": (1.0, 0.0, 0.0),
        "important_lines": 0.802,
        "important_cells": 0.562,
        "object_space_lens_radius": 0.068,
    })
    g.set_duration(0.0001)
    g.set_duration(0.0)
    g.set_rendering_algorithm_settings({
        "object_space_lens_position": position_middle
    })
    g.set_duration(1.0)

def replay():
    init_scene()
