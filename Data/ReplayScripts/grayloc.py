import math
import g

#local
def init_scene():
    g.set_duration(0.0)
    g.set_renderer("ClearView (Unified)")
    g.set_mesh("2016 - All-Hex Meshing Using Closed-Form Induced Polycube", "grayloc-hex", "vtk")
    g.set_transfer_function("Standard_grayloc.xml")
    g.set_camera_checkpoint("Overview")
    g.set_rendering_algorithm_settings({
        "line_width": 0.0016,
        "lod_value_context": 0.0,
        "lod_value_focus": 0.15,
        "use_screen_space_lens": True,
        "screen_space_lens_radius": 0.0,
        "screen_space_lens_position": (10.5, 10.5),
        "focus_outline_width": 11.5,
        "clip_focus_outline": False,
        "focus_outline_color": (1.0, 0.0, 0.0),
        "important_lines": 0.562,
        "important_cells": 0.562,
    })
    g.set_duration(2)

def length_vec2(v):
    return math.sqrt(v[0] * v[0] + v[1] * v[1])
def distance_vec2(v0, v1):
    return length_vec2((v0[0] - v1[0], v0[1] - v1[1]))

def move_lens_overview():
    total_time = 10.0
    lens_positions = [
        (0.5076366666666666, -0.401389),
        (0.25763666666666657, -0.579167),
        (0.01597266666666681, -0.619444),
        (-0.48402733333333336, -0.416667),
        (-0.4423613333333334, 0.102778),
        (-0.3368053333333333, 0.590278),
        (-0.2673613333333334, 0.172222),
        (0.40069666666666703, 0.155556),
        (0.4923566666666665, -0.405556)
    ]

    # Move lens to start position.
    g.set_duration(0)
    g.set_rendering_algorithm_settings({ "screen_space_lens_position": lens_positions[0] })
    g.set_duration(1.5)
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
    g.set_duration(2)
    g.set_duration(1)
    g.set_rendering_algorithm_settings({ "screen_space_lens_radius": 0.0 })
    g.set_duration(0)
    g.set_rendering_algorithm_settings({ "screen_space_lens_position": (10.5, 10.5) })
    g.set_duration(2)

def move_camera():
    g.set_duration(3)
    g.set_camera_checkpoint("CloseUp")
    g.set_duration(2)

def change_important_lines_close():
    # Make lens grow.
    g.set_duration(0)
    g.set_rendering_algorithm_settings({ "screen_space_lens_position": (0.167916, 0.0), "screen_space_lens_radius": 0.0 })
    g.set_duration(1)
    g.set_rendering_algorithm_settings({ "screen_space_lens_radius": 560.0 })
    g.set_duration(1)

    # Increase the number of shown lines and cells.
    g.set_duration(3)
    g.set_rendering_algorithm_settings({
        "important_lines": 0.783,
    })
    g.set_duration(1)
    g.set_duration(3)
    g.set_rendering_algorithm_settings({
        "important_cells": 0.783,
    })
    g.set_duration(1)
    g.set_duration(2)
    g.set_rendering_algorithm_settings({
        "important_cells": 0.562,
    })
    g.set_duration(1)
    g.set_duration(1)
    g.set_rendering_algorithm_settings({ "screen_space_lens_radius": 0.0 })
    g.set_duration(0)
    g.set_rendering_algorithm_settings({ "screen_space_lens_position": (10.5, 10.5) })
    g.set_duration(1)

def object_space_lens_close():
    position_near = (0.092, -0.056, 0.161)
    position_far = (0.100, -0.101, -0.073)
    position_middle = (0.093, -0.074, 0.064)

    g.set_duration(0.0)
    g.set_rendering_algorithm_settings({
        "use_screen_space_lens": False,
        "object_space_lens_radius": 0.0,
        "object_space_lens_position": position_middle,
        "focus_outline_width": 3.2
    })

    g.set_duration(2)
    g.set_rendering_algorithm_settings({ "object_space_lens_radius": 0.075 })
    g.set_duration(2)

    g.set_duration(2)
    g.set_rendering_algorithm_settings({ "object_space_lens_position": position_near })
    g.set_duration(2)
    g.set_rendering_algorithm_settings({ "object_space_lens_position": position_far })
    g.set_duration(2)
    g.set_rendering_algorithm_settings({ "object_space_lens_position": position_middle })
    g.set_duration(4)

def replay():
    init_scene()
    move_lens_overview()
    move_camera()
    change_important_lines_close()
    object_space_lens_close()
