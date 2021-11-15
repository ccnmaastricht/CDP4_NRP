import numpy as np


def ind2rad(ind, res=48, fov=1.13):
    center_ind = (res - 1) / 2

    displacement_px = ind - center_ind
    ratio_px = displacement_px / (res - 1)

    # normalized to range [-1, 1]
    displacement_rad = ratio_px * 2

    return displacement_rad


def get_global_salmap(res=320, fov=1.13, eye_limit=0.942477796):

    fov_one_side = eye_limit + fov / 2
    px_per_radiant = res / fov

    px_global = np.around(px_per_radiant * fov_one_side * 2)
    global_salmap = np.zeros((int(px_global), int(px_global)))

    return global_salmap

