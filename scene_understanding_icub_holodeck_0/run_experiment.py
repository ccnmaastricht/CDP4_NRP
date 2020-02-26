import numpy as np
import rospy
from hbp_nrp_virtual_coach.virtual_coach import VirtualCoach 
from cdp4_data_collection import CDP4DataCollection as data_collection 

objects = list(['aeroplane_toy','cube_a'])

vc = VirtualCoach(environment='local', storage_username='nrpuser')
sim = vc.launch_experiment('scene_understanding_icub_holodeck_0')
sim.start()

dc = data_collection()

for i in range(100):
    pose = dc.generate_random_pose()
    idx = np.random.randint(2)
    dc.add_object(objects[idx], pose)
    rospy.sleep(0.8)
    dc.delete_object(objects[idx])
    print(i)
sim.stop()
