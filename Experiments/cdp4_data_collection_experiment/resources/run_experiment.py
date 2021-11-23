import os
import yaml
import time
import random

import numpy as np

from spawn import *
from pynrp.virtual_coach import VirtualCoach
from cdp4_data_collection import CDP4DataCollection


if __name__ == '__main__':
    vc = VirtualCoach(environment='http://frontend:9000', storage_username='nrpuser',
                      storage_password='password')
    sim = vc.launch_experiment('cdp4_data_collection_experiment_0')
    sim.start()
    time.sleep(10)
    cdp4_api = CDP4DataCollection()

    with open('../cdp4_loop.py', 'r') as file:
        transfer_function = file.read()

    rooms = ['bed_room', 'kitchen', 'living_room', 'office']
    layouts = 3

    for sequence in range(20):
        print("##############")
        print("iteration # {}".format(sequence + 1))
        print("##############")

        # Spawn a random room layout
        room = np.random.choice(rooms)
        tf = transfer_function % (room, str(sequence))
        layout = np.random.randint(0, layouts)
        print("spawning room {}, layout {}".format(room, layout))
        spawned_models = spawn_room(room, layout, cdp4_api.spawn_model_srv, cdp4_api.set_model_state_pub)

        time.sleep(2)
        print("adding transfer function ...")
        sim.add_transfer_function(tf)
        print("Done")
        print("Collecting Data ...")
        time.sleep(1000)

        sim.delete_transfer_function('cdp4_loop')
        time.sleep(1)
        delete_room(spawned_models, cdp4_api)
        time.sleep(5)

    print("Stopping experiment ...")
    sim.stop()
