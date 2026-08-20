import numpy as np
import matplotlib.pyplot as plt


def R(deg):
    t = np.radians(deg)
    return np.array([
        [np.cos(t), -np.sin(t), 0],
        [np.sin(t), np.cos(t), 0],
    ]
    )


vz = np.array([0, 0, 1])

v30 = R(30) @ vz
v60_twice = R(60) @ v30
v60_once = R(60) @vz

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 5))

def arrow(ax, v, color='k'):
    ax.annotate('', xy=v, xytext=v+0.1*v, arrowprops=dict(arrowstyle='->', color=color))
    ax.text(v[0], v[1], v[2], f'({v[0]:.1f}, {v[1]:.1f}, {v[2]:.1f})')


def base(ax, v, color='k'):
    r = np.linalg.norm(v)
    ax.quiver(0, 0, 0, *v, color=color, scale=r)
    ax.text(0, 0, 0, f'({v[0]:.1f}, {v[1]:.1f}, {v[2]:.1f})')
    ax.plot(0, 0, 0, marker='o', markersize=10, color=color)



arrow(ax1, vz)
arrow(ax1, v30)
arrow(ax1, v60_twice)
arrow(ax1, v60_once)
