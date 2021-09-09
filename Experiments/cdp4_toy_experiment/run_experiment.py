import os
import numpy as np
import rospy
from PIL import Image
from geometry_msgs.msg import Pose
from hbp_nrp_virtual_coach.virtual_coach import VirtualCoach 
from cdp4_data_collection import CDP4DataCollection as data_collection 


objects = list(['cube_a', 'coffee_machine','aeroplane_toy'])

# create directory to save training data if doesn't exist
if not os.path.exists('training_data'):
    os.mkdir('training_data')
    os.mkdir('training_data/images/')

vc = VirtualCoach(environment='local', storage_username='nrpuser', storage_password='password')
sim = vc.launch_experiment('cdp4_toy_experiment_0')
sim.start()

dc = data_collection()
path = 'training_data'


orig_pose = Pose()
orig_pose.position.z = 10
for i in range(3):
    dc.add_object(objects[i], orig_pose)



for i in range(100):
    pose = dc.generate_random_pose()
    idx = np.random.randint(3)
    dc.set_object_pose(objects[idx], pose, True)
    dc.move_eyes(pose.position)
    rospy.sleep(0.2)
    image = dc.capture_image().astype(np.uint8)
    image = Image.fromarray(image)
    file_name = '%s/images/snapshot_%.3d.jpg' % (path, i) 
    image.save(file_name)
    dc.set_object_pose(objects[idx], orig_pose, False)
    rospy.sleep(0.1)
sim.stop()

file_name = '%s/labels.txt' % path
with open(file_name, 'w') as f:
    for item in dc.spawned_objects:
        f.write('%s\n' % item)

