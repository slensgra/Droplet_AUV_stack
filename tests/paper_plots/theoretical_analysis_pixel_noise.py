from matplotlib import pyplot as plt
import numpy as np

fig, ax = plt.subplots(1, 1, figsize=(8, 6))

focal_length = 0.5
fiducial_line = 2.0
focal_point_x = 0.6
x_ofs = 0.3

# p0 below line
def intersection_with_horizontal_line(line_y, p0, p1):
    vec = np.array(p1) - np.array(p0)

    y_int_val = line_y / vec[1]
    x_int = y_int_val * vec[0]

    return np.array(p0) + np.array([x_int, y_int_val * vec[1]])

plt.tick_params(left=False,
                bottom=False,
                labelleft=False,
                labelbottom=False)

ax.axhline(fiducial_line, color='k', linestyle='-')
ax.axhline(focal_length, linestyle='--', color='k')
ax.axhline(0.0, color='k', linestyle='-')
ax.spines[['right', 'top', 'left', 'bottom']].set_visible(False)

ax.axvline(0.0, color='k', linestyle='--')
ax.set_xlim(-0.1, 0.9)

ax.scatter([0.0, x_ofs], [fiducial_line, fiducial_line], c='r', s=90.0, marker='o')
ax.scatter([focal_point_x], [0.0], c='r', s=90.0, marker='o')
x1 = intersection_with_horizontal_line(focal_length, [focal_point_x, 0.0], [0.0, fiducial_line])
x2 = intersection_with_horizontal_line(focal_length, [focal_point_x, 0.0], [x_ofs, fiducial_line])
ax.scatter([x1[0]], [x1[1]], c='r', s=90.0, marker='o')
ax.scatter([x2[0]], [x2[1]], c='r', s=90.0, marker='o')

ax.plot([0.0, focal_point_x], [fiducial_line, 0.0], c='r', linestyle='--')
ax.plot([x_ofs, focal_point_x], [fiducial_line, 0.0], c='r', linestyle='--')

# red arrow
plt.arrow(focal_point_x + 0.1, 0.01, 0.0, 1.98, head_width=0.02, head_length=0.05, linewidth=2, color='r', length_includes_head=True)
plt.arrow(0.01, -0.04, focal_point_x - 0.02, 0.0, head_width=0.05, head_length=0.02, linewidth=2, color='r', length_includes_head=True)
plt.arrow(focal_point_x + 0.2, 0.01, 0.0, focal_length - 0.01, head_width=0.02, head_length=0.05, linewidth=2, color='r', length_includes_head=True)
ax.text(focal_point_x + 0.11, 1.0, '$t_z$', fontsize=20)
ax.text((focal_point_x - 0.05)/2.0, -0.17, '$t_x$', fontsize=20)
ax.text(focal_point_x + 0.21, (focal_length / 2.0) - 0.05, '$f$', fontsize=20)
ax.text(focal_point_x, -0.16, '$f_p$', fontsize=20)

ax.text(0.0 - 0.04, fiducial_line + 0.1, '$(0,0)$', fontsize=20)
ax.text(x_ofs - 0.04, fiducial_line + 0.1, '$(x,0)$', fontsize=20)

ax.text(x1[0] - 0.01, x1[1] + 0.07, '$x_1$', fontsize=20)
ax.text(x2[0] - 0.005, x2[1] + 0.07, '$x_2$', fontsize=20)

fig.savefig('./theoretical_analysis_pixel_noise.pdf')
plt.show()