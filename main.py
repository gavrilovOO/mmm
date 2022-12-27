import utils.utils as util
import numpy as np

x_0 = 5
y_0 = -10
r = 1
n_points = 10

h_parts = 1
time = 1
h = 0.0001

a = np.array([
    [0, 0, 0],  # a0
    [0, 0, 0],  # a1
    [0, 1 / 2, 0],  #a2
])
b = np.array([0, 0, 1])  #нижняя строка
c = np.array([0, 0, 1 / 2])  #левый столбец

body = util.create_material_body(x_0, y_0, r, n_points)
mov = util.move_material_body(time, h, body, a, b, c)

util.plot_trajectory(body, mov)

vf = util.move_through_space(1, 0.1)
util.plot_velocity_fields(vf)
