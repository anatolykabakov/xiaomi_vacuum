import copy
import argparse
from pathlib import Path

import cv2
import numpy as np
from GridMap import GridMap
from ParticleFilter import ParticleFilter
from SingleBotLaser2D import SingleBotLaser2Dgrid
from frontier import FrontierExplorer
from planner import plan_path
from utils import (
    AdaptiveGetMap,
    Draw,
    DrawParticle,
    SensorMapping,
    action_to_control,
)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    default_map = Path(__file__).resolve().parent / "map.png"
    parser.add_argument("--map", type=str, default=str(default_map))
    args = parser.parse_args()

    cv2.namedWindow("view", cv2.WINDOW_AUTOSIZE)
    cv2.namedWindow("map", cv2.WINDOW_AUTOSIZE)

    bot_param = [240, -30.0, 210.0, 150.0, 6.0, 6.0]
    bot_pos = np.array([150.0, 100.0, 0.0])
    env = SingleBotLaser2Dgrid(bot_pos, bot_param, args.map)

    map_param = [0.4, -0.4, 5.0, -5.0]
    m = GridMap(map_param, gsize=1.0)
    sensor_data = env.Sensor()
    SensorMapping(m, env.bot_pos, env.bot_param, sensor_data)

    img = Draw(env.img_map, 1, env.bot_pos, sensor_data, env.bot_param)
    mimg = AdaptiveGetMap(m)
    cv2.imshow("view", img)
    cv2.imshow("map", mimg)

    pf = ParticleFilter(bot_pos.copy(), bot_param, copy.deepcopy(m), 10)
    explorer = FrontierExplorer()
    path = []

    while True:
        action = -1
        k = cv2.waitKey(1)
        if k == ord("w"):
            action = 1
        elif k == ord("s"):
            action = 2
        elif k == ord("a"):
            action = 3
        elif k == ord("d"):
            action = 4

        if action > 0:
            control = action_to_control(action, bot_param)

            env.Move(control)
            scan = env.Sensor()
            SensorMapping(m, env.bot_pos, bot_param, scan)
            pf_state = pf.update(control, scan)
            goal, clusters = explorer.update(pf_state["pose"], pf_state["gmap"])
            print("Pose: {}, Goal: {}".format(pf_state["pose"], goal))
            path = plan_path(pf_state["gmap"], pf_state["pose"], goal)
            print(path)

            img = Draw(
                env.img_map,
                1,
                env.bot_pos,
                scan,
                bot_param,
                goal,
                clusters,
                path,
            )
            mimg = AdaptiveGetMap(m)
            imgp0 = AdaptiveGetMap(pf_state["gmap"])

            img = DrawParticle(img, pf.particle_list)
            cv2.imshow("view", img)
            cv2.imshow("map", mimg)
            cv2.imshow("particle_map", imgp0)

    cv2.destroyAllWindows()
