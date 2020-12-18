import math
import g

#local
def init_scene():
    g.set_use_camera_flight(True)
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

def length_vec2(v):
    return math.sqrt(v[0] * v[0] + v[1] * v[1])
def distance_vec2(v0, v1):
    return length_vec2((v0[0] - v1[0], v0[1] - v1[1]))

def move_lens_overview():
    lens_positions = [
        (0.9, -0.615278),
        (0.9, -0.615278),
        (0.9, -0.615278),
        (1.03333, -0.615278),
        (1.00694, -0.615278),
        (1.03333, -0.615278),
        (1.06528, -0.598611),
        (1.08194, -0.586111),
        (1.08889, -0.577778),
        (1.12222, -0.565278),
        (1.18056, -0.568056),
        (1.2, -0.572222),
        (1.24028, -0.583333),
        (1.29444, -0.605556),
        (1.31944, -0.615278),
        (1.31944, -0.613889),
        (1.31944, -0.593056),
        (1.31111, -0.573611),
        (1.29028, -0.431944),
        (1.30, -0.204167),
        (1.31, -0.00277776),
        (1.32, 0.215278),
        (1.315, 0.372222),
        (1.31, 0.466667),
        (1.295, 0.47),
        (1.295, 0.47),
        (1.295, 0.47),
        (1.2475, 0.47),
        (1.22417, 0.448611),
        (1.18889, 0.436111),
        (1.17361, 0.423611),
        (1.17083, 0.419444),
        (1.16389, 0.409722),
        (1.06528, 0.351389),
        (0.872222, 0.273611),
        (0.701389, 0.195833),
        (0.650556, 0.138889),
        (0.709722, 0.0833334),
        (0.818611, 0.0347222),
        (0.98917, 0.00833333),
        (1.17778, 0.00833333),
        (1.32917, 0.00972223),
        (1.40833, 0.00555551),
        (1.49556, 0),
        (1.56972, 0),
        (1.58417, 0.00416672),
        (1.60278, 0.0111111),
        (1.60417, 0.0111111),
        (1.60417, 0.0111111),
        (1.60278, 0.0125),
        (1.6, 0.0138888),
        (1.5875, 0.0138888),
        (1.49306, -0.0388889),
        (1.39444, -0.0847222),
        (1.33056, -0.115278),
        (1.15278, -0.195833),
        (0.961111, -0.2875),
        (0.844444, -0.351389),
        (0.766667, -0.4),
        (0.736111, -0.433333),
        (0.740278, -0.443056),
        (0.763056, -0.472222),
        (0.813056, -0.474389),
        (0.843056, -0.476389),
    ]

    # Move lens to start position.
    g.set_duration(0)
    g.set_rendering_algorithm_settings({ "screen_space_lens_position": lens_positions[0] })
    g.set_rendering_algorithm_settings({ "screen_space_lens_radius": 360.0 })

    for i in range(1, len(lens_positions)):
        g.set_duration(0.5)
        g.set_rendering_algorithm_settings({ "screen_space_lens_position": lens_positions[i] })

def replay():
    init_scene()
    move_lens_overview()
