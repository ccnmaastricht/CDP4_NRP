import rospy
from spiking_saccade_generator.srv import MoveEyes

service = rospy.ServiceProxy('/move_eyes', MoveEyes)

last_horizontal = 0
last_vertical = 0

displacement_horizontal = 0.5
displacement_vertical = 0.5

previous_count = [0, 0, 0, 0]

for i in range(20):
    sg_output = service.call(0., 1000., displacement_horizontal, displacement_vertical,
                             last_horizontal, last_vertical, previous_count)

    last_horizontal = sg_output.horizontal
    last_vertical = sg_output.vertical
    previous_count = sg_output.previous_count_new
