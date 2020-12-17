import math
import g

#local
def init_scene():
    g.set_duration(0.0)
    g.set_renderer("ClearView (Unified)")
    g.set_mesh("2015 - Practical Hex-Mesh Optimization via Edge-Cone Rectification", "dragon_out", "mesh")
    g.set_transfer_function("Standard_dragon.xml")
    g.set_camera_checkpoint("Overview")
    g.set_rendering_algorithm_settings({
        "line_width": 0.0018,
        "lod_value_context": 0.126,
        "lod_value_focus": 0.192,
        "use_screen_space_lens": True,
        "screen_space_lens_radius": 0.0,
        "screen_space_lens_position": (10.5, 10.5),
        "focus_outline_width": 9.0,
        "clip_focus_outline": False,
        "focus_outline_color": (1.0, 0.0, 0.0),
        "important_lines": 0.457,
        "important_cells": 0.440,
    })
    g.set_duration(2)

def length_vec2(v):
    return math.sqrt(v[0] * v[0] + v[1] * v[1])
def distance_vec2(v0, v1):
    return length_vec2((v0[0] - v1[0], v0[1] - v1[1]))

def move_lens_overview():
    total_time = 42.0
    lens_positions = [
        (-0.288889, 0.505556),
        (-0.208333, 0.508333),
        (-0.1375, 0.513889),
        (-0.0305555, 0.519444),
        (0.0625, 0.533333),
        (0.172222, 0.529167),
        (0.236111, 0.538889),
        (0.286111, 0.543056),
        (0.3625, 0.540278),
        (0.411111, 0.531944),
        (0.426389, 0.534722),
        (0.440278, 0.488889),
        (0.448611, 0.433333),
        (0.447222, 0.388889),
        (0.431944, 0.343056),
        (0.4125, 0.302778),
        (0.388889, 0.244444),
        (0.35, 0.177778),
        (0.319444, 0.0986111),
        (0.270833, 0.0430555),
        (0.222222, -0.0375),
        (0.169444, -0.120833),
        (0.158333, -0.183333),
        (0.130556, -0.305556),
        (0.144444, -0.368056),
        (0.169444, -0.438889),
        (0.175, -0.5),
        (0.186111, -0.548611),
        (0.227778, -0.548611),
        (0.273611, -0.559722),
        (0.327778, -0.561111),
        (0.384722, -0.572222),
        (0.458333, -0.575),
        (0.529167, -0.565278),
        (0.577778, -0.468056),
        (0.651389, -0.398611),
        (0.698611, -0.301389),
        (0.751389, -0.173611),
        (0.7875, -0.0555556),
        (0.876389, 0.0597222),
        (0.941667, 0.168056),
        (1.00139, 0.241667),
        (1.10972, 0.268056),
        (1.22222, 0.276389),
        (1.28611, 0.263889),
        (1.35278, 0.231944),
        (1.39028, 0.198611),
        (1.39861, 0.165278),
        (1.39583, 0.118056),
        (1.37917, 0.0513889),
        (1.34444, -0.0319445),
        (1.29444, -0.108333),
        (1.25278, -0.195833),
        (1.24028, -0.258333),
        (1.23611, -0.316667),
        (1.2375, -0.375),
        (1.2375, -0.445833),
        (1.21806, -0.501389),
        (1.21806, -0.569444),
        (1.24306, -0.619444),
        (1.30139, -0.633333),
        (1.39722, -0.625),
        (1.49722, -0.613889),
        (1.62361, -0.588889),
        (1.70556, -0.472222),
        (1.71806, -0.305556),
        (1.72778, -0.1375),
        (1.72778, -0.0125),
        (1.73056, 0.0902778)
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

    g.set_duration(4)

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
