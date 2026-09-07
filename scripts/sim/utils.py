import numpy as np
import cv2
from math import *


def Draw(img_map, scale, bot_pos, sensor_data, bot_param, goal=None, clusters=None, path=None):
    img = img_map.copy()
    img = cv2.resize(
        img,
        (round(scale * img.shape[1]), round(scale * img.shape[0])),
        interpolation=cv2.INTER_LINEAR,
    )
    img = Map2Image(img)
    plist = EndPoint(bot_pos, bot_param, sensor_data)
    for pts in plist:
        cv2.line(
            img,
            (int(scale * bot_pos[0]), int(scale * bot_pos[1])),
            (int(scale * pts[0]), int(scale * pts[1])),
            (255, 0, 0),
            1,
        )
    if goal is not None:
        cv2.circle(
            img,
            (int(scale * goal[0]), int(scale * goal[1])),
            max(2, int(4 * scale)),
            (0, 255, 0),
            2,
        )
    if clusters is not None:
        for cluster in clusters:
            for pt in cluster:
                cv2.circle(
                    img,
                    (int(scale * pt[0]), int(scale * pt[1])),
                    max(1, int(2 * scale)),
                    (255, 200, 0),
                    -1,
                )
    if path is not None and len(path) >= 2:
        for i in range(1, len(path)):
            cv2.line(
                img,
                (int(scale * path[i - 1][0]), int(scale * path[i - 1][1])),
                (int(scale * path[i][0]), int(scale * path[i][1])),
                (0, 255, 255),
                2,
            )

    cv2.circle(
        img,
        (int(scale * bot_pos[0]), int(scale * bot_pos[1])),
        int(3 * scale),
        (0, 0, 255),
        -1,
    )
    return img


def DrawParticle(img, plist, scale=1.0):
    for p in plist:
        cv2.circle(img, (int(scale * p.pos[0]), int(scale * p.pos[1])), int(2), (0, 200, 0), -1)
    return img


def SensorMapping(m, bot_pos, bot_param, sensor_data):
    inter = (bot_param[2] - bot_param[1]) / (bot_param[0] - 1)
    for i in range(bot_param[0]):
        if sensor_data[i] > bot_param[3] - 1 or sensor_data[i] < 1:
            continue
        theta = bot_pos[2] + bot_param[1] + i * inter
        m.GridMapLine(
            int(bot_pos[0]),
            int(bot_pos[0] + sensor_data[i] * np.cos(np.deg2rad(theta))),
            int(bot_pos[1]),
            int(bot_pos[1] + sensor_data[i] * np.sin(np.deg2rad(theta))),
        )


def AdaptiveGetMap(gmap, padding=20):
    mimg = gmap.GetMapProb(
        gmap.boundary[0] - padding,
        gmap.boundary[1] + padding,
        gmap.boundary[2] - padding,
        gmap.boundary[3] + padding,
    )
    mimg = (255 * mimg).astype(np.uint8)
    mimg = cv2.cvtColor(mimg, cv2.COLOR_GRAY2RGB)
    return mimg


def action_to_control(action, bot_param):
    v_max, w_max = bot_param[4], bot_param[5]
    v, w = 0, 0
    if action == 1:
        v = v_max
    elif action == 2:
        v = -v_max
    elif action == 3:
        w = -w_max
    elif action == 4:
        w = w_max
    return v, w


def EndPoint(pos, bot_param, sensor_data):
    pts_list = []
    inter = (bot_param[2] - bot_param[1]) / (bot_param[0] - 1)
    for i in range(bot_param[0]):
        theta = pos[2] + bot_param[1] + i * inter
        pts_list.append(
            [
                pos[0] + sensor_data[i] * np.cos(np.deg2rad(theta)),
                pos[1] + sensor_data[i] * np.sin(np.deg2rad(theta)),
            ]
        )
    return pts_list


def gaussian(x, mu, sig):
    return 1.0 / (sqrt(2.0 * pi) * sig) * np.exp(-np.power((x - mu) / sig, 2.0) / 2)


def Bresenham(x0, x1, y0, y1):
    rec = []
    "Bresenham's line algorithm"
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    x, y = x0, y0
    sx = -1 if x0 > x1 else 1
    sy = -1 if y0 > y1 else 1
    if dx > dy:
        err = dx / 2.0
        while x != x1:
            rec.append((x, y))
            err -= dy
            if err < 0:
                y += sy
                err += dx
            x += sx
    else:
        err = dy / 2.0
        while y != y1:
            rec.append((x, y))
            err -= dx
            if err < 0:
                x += sx
                err += dy
            y += sy
    return rec


def Image2Map(fname):
    im = cv2.imread(fname)
    if im is None:
        raise FileNotFoundError(f"Cannot read map image: {fname}")
    m = np.asarray(im)
    if m.ndim == 3:
        m = cv2.cvtColor(m, cv2.COLOR_BGR2GRAY)
    m = m.astype(float) / 255.0
    return m


def Map2Image(m):
    img = (255 * m).astype(np.uint8)
    img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
    return img


def Rotation2Deg(R):
    cos = R[0, 0]
    sin = R[1, 0]
    theta = np.rad2deg(np.arccos(np.abs(cos)))

    if cos > 0 and sin > 0:
        return theta
    elif cos < 0 and sin > 0:
        return 180 - theta
    elif cos < 0 and sin < 0:
        return 180 + theta
    elif cos > 0 and sin < 0:
        return 360 - theta
    elif cos == 0 and sin > 0:
        return 90.0
    elif cos == 0 and sin < 0:
        return 270.0
    elif cos > 0 and sin == 0:
        return 0.0
    elif cos < 0 and sin == 0:
        return 180.0


def gmap_bounds(gmap, padding: int = 20):
    if gmap.boundary[0] > gmap.boundary[1]:
        return None
    x0 = gmap.boundary[0] - padding
    x1 = gmap.boundary[1] + padding + 1
    y0 = gmap.boundary[2] - padding
    y1 = gmap.boundary[3] + padding + 1
    return x0, x1, y0, y1


def gmap_occ_grid(gmap, padding: int = 20):
    """Occupancy probabilities; array indices (row, col) = (y, x) grid."""
    bounds = gmap_bounds(gmap, padding)
    if bounds is None:
        return None, None
    x0, x1, y0, y1 = bounds
    return gmap.GetMapProb(x0, x1, y0, y1), (x0, y0)


def pose_to_grid_ij(pose, gmap, origin):
    gx = int(round(float(pose[0]) / gmap.gsize))
    gy = int(round(float(pose[1]) / gmap.gsize))
    x0, y0 = origin
    return (gy - y0, gx - x0)


def grid_ij_to_world(ij, origin, gsize: float = 1.0):
    if ij is None:
        return None
    x0, y0 = origin
    i, j = ij
    return (float(x0 + j) * gsize, float(y0 + i) * gsize)


def clusters_ij_to_world(clusters, origin, gsize: float = 1.0):
    if not clusters:
        return None
    return [[grid_ij_to_world(pt, origin, gsize) for pt in cluster] for cluster in clusters]
