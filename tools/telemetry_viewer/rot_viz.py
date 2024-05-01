"""
Adapted from the 3D example provided in the DearPyGUI docs.

https://dearpygui.readthedocs.io/en/latest/documentation/drawing-api.html#d-operations
"""

import math

import dearpygui.dearpygui as dpg

SIZE = 5
VERTICES = [
        [-SIZE, -SIZE, -SIZE],  # 0 near side
        [ SIZE, -SIZE, -SIZE],  # 1
        [-SIZE,  SIZE, -SIZE],  # 2
        [ SIZE,  SIZE, -SIZE],  # 3
        [-SIZE, -SIZE,  SIZE],  # 4 far side
        [ SIZE, -SIZE,  SIZE],  # 5
        [-SIZE,  SIZE,  SIZE],  # 6
        [ SIZE,  SIZE,  SIZE],  # 7
        [-SIZE, -SIZE, -SIZE],  # 8 left side
        [-SIZE,  SIZE, -SIZE],  # 9
        [-SIZE, -SIZE,  SIZE],  # 10
        [-SIZE,  SIZE,  SIZE],  # 11
        [ SIZE, -SIZE, -SIZE],  # 12 right side
        [ SIZE,  SIZE, -SIZE],  # 13
        [ SIZE, -SIZE,  SIZE],  # 14
        [ SIZE,  SIZE,  SIZE],  # 15
        [-SIZE, -SIZE, -SIZE],  # 16 bottom side
        [ SIZE, -SIZE, -SIZE],  # 17
        [-SIZE, -SIZE,  SIZE],  # 18
        [ SIZE, -SIZE,  SIZE],  # 19
        [-SIZE,  SIZE, -SIZE],  # 20 top side
        [ SIZE,  SIZE, -SIZE],  # 21
        [-SIZE,  SIZE,  SIZE],  # 22
        [ SIZE,  SIZE,  SIZE],  # 23
    ]

RED = [255, 0, 0, 255]
GREEN = [0, 255, 0, 255]
BLUE = [0, 0, 255, 255]
YELLOW = [255, 255, 0, 255]
CYAN = [0, 255, 255, 255]
PURPLE = [255, 0, 255, 255]


class RotationVisualizer:
    def __init__(self):
        with dpg.draw_layer(depth_clipping=True, perspective_divide=True, cull_mode=dpg.mvCullMode_Back) as draw_layer:
            self.draw_layer = draw_layer

            with dpg.draw_node() as cube:
                self.cube = cube

                dpg.draw_triangle(VERTICES[1],  VERTICES[2],  VERTICES[0], color=[0,0,0.0],  fill=GREEN)
                dpg.draw_triangle(VERTICES[1],  VERTICES[3],  VERTICES[2], color=[0,0,0.0],  fill=GREEN)
                dpg.draw_triangle(VERTICES[7],  VERTICES[5],  VERTICES[4], color=[0,0,0.0],  fill=RED)
                dpg.draw_triangle(VERTICES[6],  VERTICES[7],  VERTICES[4], color=[0,0,0.0],  fill=RED)
                dpg.draw_triangle(VERTICES[9],  VERTICES[10], VERTICES[8], color=[0,0,0.0],  fill=BLUE)
                dpg.draw_triangle(VERTICES[9],  VERTICES[11], VERTICES[10], color=[0,0,0.0], fill=BLUE)
                dpg.draw_triangle(VERTICES[15], VERTICES[13], VERTICES[12], color=[0,0,0.0], fill=YELLOW)
                dpg.draw_triangle(VERTICES[14], VERTICES[15], VERTICES[12], color=[0,0,0.0], fill=YELLOW)
                dpg.draw_triangle(VERTICES[18], VERTICES[17], VERTICES[16], color=[0,0,0.0], fill=PURPLE)
                dpg.draw_triangle(VERTICES[19], VERTICES[17], VERTICES[18], color=[0,0,0.0], fill=PURPLE)
                dpg.draw_triangle(VERTICES[21], VERTICES[23], VERTICES[20], color=[0,0,0.0], fill=CYAN)
                dpg.draw_triangle(VERTICES[23], VERTICES[22], VERTICES[20], color=[0,0,0.0], fill=CYAN)

            dpg.set_clip_space(draw_layer, 0, 0, 500, 500, -1.0, 1.0)

        self.view = dpg.create_fps_matrix([0, 0, 50], 0.0, 0.0)
        self.proj = dpg.create_perspective_matrix(45.0 * math.pi / 180.0, 1.0, 0.1, 100)
        self.transform([1.0, 0.0, 0.0], 0)

    def transform(self, axis, rotation):
        model = dpg.create_rotation_matrix(rotation, axis)
        dpg.apply_transform(self.cube, self.proj * self.view * model)

    def update(self, parent_width, parent_height):
        extent = min(parent_width, parent_height)
        dpg.set_clip_space(self.draw_layer, 0, 0, extent, extent, -1.0, 1.0)
