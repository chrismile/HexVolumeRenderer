import math
import g

#local
def init_scene():
    g.set_duration(0.0)
    g.set_renderer("ClearView (Unified)")
    g.set_mesh("2012 - All-Hex Meshing using Singularity-Restricted Field", "sculpture/sculpture-B", "vtk")
    g.set_transfer_function("Standard.xml")
    g.set_camera_checkpoint("Video")
    g.set_rendering_algorithm_settings({
        "line_width": 0.0040,
        "lod_value_context": 0.3,
        "lod_value_focus": 0.3,
        "use_screen_space_lens": True,
        "screen_space_lens_radius": 0.0,
        "screen_space_lens_position": (10.5, 10.5),
        "focus_outline_width": 9.0,
        "clip_focus_outline": False,
        "focus_outline_color": (1.0, 0.0, 0.0),
        "important_lines": 0.644,
        "important_cells": 0.446,
    })
    g.set_duration(2)

def length_vec2(v):
    return math.sqrt(v[0] * v[0] + v[1] * v[1])
def distance_vec2(v0, v1):
    return length_vec2((v0[0] - v1[0], v0[1] - v1[1]))

def move_lens_overview():
    total_time = 16.0
    lens_positions = [
        (1.28472, -0.401389),
        (1.03472, -0.579167),
        (0.793056, -0.619444),
        (0.293056, -0.416667),
        (0.334722, 0.102778),
        (0.440278, 0.590278),
        (0.509722, 0.172222),
        (1.17778, 0.155556),
        (1.26944, -0.405556)
    ]

    # Move lens to start position.
    g.set_duration(0)
    g.set_rendering_algorithm_settings({ "screen_space_lens_position": lens_positions[0] })
    g.set_duration(2)
    g.set_rendering_algorithm_settings({ "screen_space_lens_radius": 360.0 })
    g.set_duration(2)

    total_length = 0.0
    for i in range(1, len(lens_positions)):
        total_length += distance_vec2(lens_positions[i-1], lens_positions[i])

    for i in range(1, len(lens_positions)):
        curr_distance = distance_vec2(lens_positions[i-1], lens_positions[i])
        g.set_duration(curr_distance * total_time / total_length)
        g.set_rendering_algorithm_settings({ "screen_space_lens_position": lens_positions[i] })

    # Hide lens.
    #g.set_duration(2)
    #g.set_duration(1)
    #g.set_rendering_algorithm_settings({ "screen_space_lens_radius": 0.0 })
    #g.set_duration(0)
    #g.set_rendering_algorithm_settings({ "screen_space_lens_position": (10.5, 10.5) })
    #g.set_duration(2)

def replay():
    init_scene()
    move_lens_overview()
