import math
import g

#local
def init_scene():
    g.set_duration(0.0)
    g.set_renderer("ClearView (Unified)")
    g.set_mesh("2011 - All-Hex Mesh Generation via Volumetric PolyCube Deformation", "anc101_a1", "mesh")
    g.set_transfer_function("Standard_anc_vid.xml")
    g.set_camera_checkpoint("Frontal")
    g.set_rendering_algorithm_settings({
        "line_width": 0.0016,
        "lod_value_context": 0.128,
        "lod_value_focus": 0.15,
        "use_screen_space_lens": True,
        "screen_space_lens_radius": 0.0,
        "screen_space_lens_position": (10.5, 10.5),
        "focus_outline_width": 9.0,
        "clip_focus_outline": False,
        "focus_outline_color": (1.0, 0.0, 0.0),
        "important_lines": 0.571,
        "important_cells": 0.522,
    })
    g.set_duration(2)

def length_vec2(v):
    return math.sqrt(v[0] * v[0] + v[1] * v[1])
def distance_vec2(v0, v1):
    return length_vec2((v0[0] - v1[0], v0[1] - v1[1]))

def move_lens_overview():
    total_time = 30.0
    lens_positions = [
        (1.2423566666666666, -0.458333),
        (-0.3131943333333333, -0.458333),
        (-1.2020833333333334, -0.458333),
        (-0.6854166333333335, 0.186111),
        (-0.6451393333333333, 0.283333),
        (-0.10208333333333333, 0.283333),
        (0.21874966666666656, 0.283333),
        (0.35624666666666654, 0.152778),
        (0.5131966666666667, 0.1),
        (1.2159766666666667, -0.470833)
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

        if i == 1:
            g.set_duration(2)
        if i == 2:
            g.set_duration(0.5)
        if i == 3:
            g.set_duration(2)
        if i == 7:
            g.set_duration(2)

    g.set_duration(4)


def replay():
    init_scene()
    move_lens_overview()
