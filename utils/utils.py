import models.models as model
import matplotlib.pyplot as plotlib
import numpy as np


def f_x(t, x):
    return - np.exp(t) * x


def f_y(t, y):
    return np.exp(t) * y


def create_material_body(x_0, y_0, r, n_points):
    t = 0
    m = 0
    material_points = []

    theta = np.linspace(0, np.pi / 2, n_points)

    def get_coords(r):
        return x_0 + r * np.cos(theta), y_0 + r * np.sin(theta)

    x_r, y_r = get_coords(r)

    def append_mpoints(x, y):
        material_points.append(model.MaterialPoint(m, x, y, f_x(t, x), f_y(t, y), x, y, t))

    for i in range(len(x_r)):
        append_mpoints(x_r[i], y_r[i])

    material_body = model.MaterialBody(material_points)
    return material_body


def runge_method(x_0, h, n, func, a, b, c):
    x_t = [x_0]
    t = 0
    for i in range(n):
        x_n = x_t[i]
        k1 = func(t, x_n)
        k2 = func(t + c[2] * h, x_n + a[2, 1] * h * k1)
        x_t.append(x_n + h * (k1 * b[1] + k2 * b[2]))
        t += h
    return x_t


def move_material_body(time, h, mb, a, b, c):
    point_trajectories = []
    for i in range(len(mb.material_points)):
        x_0 = mb.material_points[i].x_0
        y_0 = mb.material_points[i].y_0

        n = int(time / h) + 1
        x_t = runge_method(x_0, h, n, f_x, a, b, c)
        y_t = runge_method(y_0, h, n, f_y, a, b, c)

        point_trajectories.append(model.PointTrajectory(mb.material_points[i], x_t, y_t))
    body_trajectory = model.BodyTrajectory(point_trajectories, mb)
    return body_trajectory


def plot_trajectory(mb, tr):
    for i in range(len(mb.material_points)):
        plotlib.plot(mb.material_points[i].coord_x, mb.material_points[i].coord_y, 'r.')
    for i in range(len(mb.material_points)):
        plotlib.plot(tr.point_trajectories[i].x, tr.point_trajectories[i].y, 'b', linewidth=0.5)
    for i in range(len(mb.material_points)):
        time = len(tr.point_trajectories[i].x) - 1
        plotlib.plot(tr.point_trajectories[i].x[time], tr.point_trajectories[i].y[time], 'g.')
    plotlib.axis('equal')
    plotlib.grid()
    #plotlib.show()
    plotlib.savefig('assets/plot_trajectory.svg', format='svg', dpi=1200)


def move_through_space(time, h):
    t = h
    m = 0
    a = np.linspace(-3, 3, 7)
    x_s, y_s = np.meshgrid(a, a)
    velocity_fields = []
    for n in range(int(time / h)):
        space_points = []
        for i in range(7):
            for j in range(7):
                x = x_s[i, j]
                y = y_s[i, j]
                space_points.append(model.SpacePoint(m, x, y, f_x(t, x), f_y(t, y), t))
                m += 1
        velocity_fields.append(model.SpaceGrid(space_points))
        t += h
    return velocity_fields


def plot_velocity_fields(vf):
    h = vf[0].space_points[0].t
    t = h
    for n in range(len(vf)):
        plotlib.figure(n)
        plotlib.suptitle('t = ' + str(t))
        m = 0
        coord_x = []
        coord_y = []
        v_x = []
        v_y = []
        for i in range(7):
            for j in range(7):
                coord_x.append(vf[n].space_points[m].coord_x)
                coord_y.append(vf[n].space_points[m].coord_y)
                v_x.append(vf[n].space_points[m].velocity_x)
                v_y.append(vf[n].space_points[m].velocity_y)
                m += 1
        plotlib.subplot(1, 2, 1)
        plotlib.quiver(coord_x, coord_y, v_x, v_y)
        for p in range(1, 3):
            for q in range(1, 3):
                x = np.linspace(0.1, 5.0, 100)
                d = f_y(t, 1) / f_x(t, 1)
                c = q * (p ** d)
                y = c * (-x ** d)
                plotlib.subplot(1, 2, 2)
                plotlib.axis([-1, 5, -5, 1])
                plotlib.plot(x, y)
        t += h
        #plotlib.show()
        plotlib.savefig('assets/velocity_fields' + str(n) + '.svg', format='svg', dpi=1200)
