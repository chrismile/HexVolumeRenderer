import math
import g

#local
def init_scene():
    g.set_duration(0.0)
    g.set_renderer("ClearView (Unified)")
    g.set_mesh(
        "2020 - LoopyCuts - Practical Feature-Preserving Block Decomposition for Strongly Hex-Dominant Meshing",
        "motor_tail", "mesh")
    g.set_transfer_function("motor_tail.xml")
    g.set_camera_checkpoint("Overview")
    g.set_rendering_algorithm_settings({
        "line_width": 0.0022,
        "modulate_line_thickness_by_depth": True,
        "lod_value_context": 0.002,
        "lod_value_focus": 0.251,
        "use_screen_space_lens": True,
        "screen_space_lens_radius": 438.0,
        "screen_space_lens_position": (0.005, -0.45),
        "focus_outline_width": 14.0,
        "clip_focus_outline": False,
        "focus_outline_color": (1.0, 0.0, 0.0),
        "important_lines": 0.554,
        "important_cells": 0.300,
        "use_num_cells_or_volume": 0.522,
        "lod_merge_factor": 0.0,
    })
    g.set_duration(1.0)

def replay():
    init_scene()
