import numpy as np
from GridMap import *
import random
import math
import utils
import copy
import threading


class Particle:
    def __init__(self, pos, bot_param, gmap):
        self.pos = pos
        self.bot_param = bot_param
        self.gmap = gmap

    def Sampling(self, control, dt=1, sig=[0.4, 0.4, 0.4]):
        v, w = control
        th = np.deg2rad(self.pos[2])
        self.pos[0] -= v * np.sin(th) * dt
        self.pos[1] += v * np.cos(th) * dt
        self.pos[2] = (self.pos[2] + w * dt) % 360

        self.pos[0] += random.gauss(0, sig[0])
        self.pos[1] += random.gauss(0, sig[1])
        self.pos[2] += random.gauss(0, sig[2])

    def NearestDistance(self, x, y, wsize, th):
        min_dist = 9999
        min_x = None
        min_y = None
        gsize = self.gmap.gsize
        xx = int(round(x / gsize))
        yy = int(round(y / gsize))
        for i in range(xx - wsize, xx + wsize):
            for j in range(yy - wsize, yy + wsize):
                if self.gmap.GetGridProb((i, j)) < th:
                    dist = (i - xx) * (i - xx) + (j - yy) * (j - yy)
                    if dist < min_dist:
                        min_dist = dist
                        min_x = i
                        min_y = j

        return math.sqrt(float(min_dist) * gsize)

    def LikelihoodField(self, sensor_data):
        p_hit = 0.9
        p_rand = 0.1
        sig_hit = 3.0
        log_q = 0.0
        plist = utils.EndPoint(self.pos, self.bot_param, sensor_data)
        for i in range(len(plist)):
            if sensor_data[i] > self.bot_param[3] - 1 or sensor_data[i] < 1:
                continue
            dist = self.NearestDistance(plist[i][0], plist[i][1], 4, 0.2)
            p = p_hit * utils.gaussian(0, dist, sig_hit) + p_rand / self.bot_param[3]
            log_q += math.log(max(p, 1e-300))
        return math.exp(log_q)

    def Mapping(self, sensor_data):
        inter = (self.bot_param[2] - self.bot_param[1]) / (self.bot_param[0] - 1)
        for i in range(self.bot_param[0]):
            if sensor_data[i] > self.bot_param[3] - 1 or sensor_data[i] < 1:
                continue
            theta = self.pos[2] + self.bot_param[1] + i * inter
            self.gmap.GridMapLine(
                int(self.pos[0]),
                int(self.pos[0] + sensor_data[i] * np.cos(np.deg2rad(theta))),
                int(self.pos[1]),
                int(self.pos[1] + sensor_data[i] * np.sin(np.deg2rad(theta))),
            )


class ParticleFilter:
    def __init__(self, pos, bot_param, gmap, size):
        self.size = size
        self.particle_list = []
        self.weights = np.ones((size), dtype=float) / size
        p = Particle(pos.copy(), bot_param, copy.deepcopy(gmap))
        for i in range(size):
            self.particle_list.append(copy.deepcopy(p))

    def ParticleMapping(plist, sensor_data):
        threads = []
        for p in plist:
            threads.append(threading.Thread(target=p.Mapping, args=(sensor_data,)))

        for t in threads:
            t.start()

        for t in threads:
            t.join()

    def _normalize_weights(self, field):
        field = np.asarray(field, dtype=float)
        field = np.nan_to_num(field, nan=0.0, posinf=0.0, neginf=0.0)
        total = field.sum()
        if total <= 0.0:
            return np.ones(self.size, dtype=float) / self.size
        return field / total

    def Resampling(self, sensor_data):
        map_rec = np.zeros((self.size))
        weights = self._normalize_weights(self.weights)
        re_id = np.random.choice(self.size, self.size, p=weights)
        new_particle_list = []
        for i in range(self.size):
            if map_rec[re_id[i]] == 0:
                self.particle_list[re_id[i]].Mapping(sensor_data)
                map_rec[re_id[i]] = 1
            new_particle_list.append(copy.deepcopy(self.particle_list[re_id[i]]))
        self.particle_list = new_particle_list
        self.weights = np.ones((self.size), dtype=float) / float(self.size)

    def Feed(self, control, sensor_data):
        field = np.zeros((self.size), dtype=float)
        for i in range(self.size):
            self.particle_list[i].Sampling(control)
            field[i] = self.particle_list[i].LikelihoodField(sensor_data)
            # self.particle_list[i].Mapping(sensor_data)

        self.weights = self._normalize_weights(field)

    def best_index(self) -> int:
        return int(np.argmax(self.weights))

    def best_particle(self) -> Particle:
        return self.particle_list[self.best_index()]

    def update(self, control, sensor_data):
        """
        One SLAM tick: Feed → Resampling (maps on resampled particles).
        """
        self.Feed(control, sensor_data)
        self.Resampling(sensor_data)
        best = self.best_particle()
        return {"pose": best.pos, "gmap": best.gmap}
