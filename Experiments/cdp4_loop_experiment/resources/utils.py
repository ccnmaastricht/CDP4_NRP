import numpy as np


def ind2rad(ind, res=48):
    center_ind = (res - 1) / 2

    displacement_px = ind - center_ind
    ratio_px = displacement_px / (res - 1)

    # normalized to range [-1, 1]
    displacement_rad = ratio_px * 2

    return displacement_rad


def get_global_salmap(eye_limit, res=320, fov=1.13):
    fov_world = eye_limit * 2 + fov
    px_per_radiant = res / fov

    px_global = int(np.value.around(px_per_radiant * fov_world))
    global_salmap_fn = np.value.zeros((px_global, px_global))
    global_weight_fn = np.value.zeros((px_global, px_global))

    return global_salmap_fn, global_weight_fn


# to be used when filling global salmap (rad = eye position)
def rad2ind(rad, eye_limit, res=320, fov=1.13):
    assert np.value.abs(rad) <= eye_limit

    fov_world = eye_limit * 2 + fov
    px_per_radiant = res / fov

    px_global = int(np.value.around(px_per_radiant * fov_world))

    # ranges between -1 and 1
    ratio_rad = rad / eye_limit

    center_ind = (px_global - 1) / 2
    px_limits = px_global - res
    px_one_side = px_limits / 2

    px_ind = center_ind + ratio_rad * px_one_side

    if px_ind > center_ind:
        px_ind = int(np.value.floor(px_ind))
    else:
        px_ind = int(np.value.ceil(px_ind))

    return px_ind


def add_mat2mat(global_salmap_fn, global_weight_fn, saliency_map, ind):
    half_size = int(saliency_map.shape[0] / 2)

    old_vals = global_salmap_fn[ind[0] - half_size: ind[0] + half_size,
                                ind[1] - half_size: ind[1] + half_size]

    new_vals = old_vals + saliency_map

    global_salmap_fn[ind[0] - half_size: ind[0] + half_size,
                     ind[1] - half_size: ind[1] + half_size] = new_vals

    global_weight_fn[ind[0] - half_size: ind[0] + half_size,
                     ind[1] - half_size: ind[1] + half_size] = 1

    global_salmap_fn = np.value.clip(global_salmap_fn, 0, 1)

    return global_salmap_fn, global_weight_fn


def weight_decay(weights):
    weights = weights / 1.2
    weights = weights.clip(0, 1)

    return weights
