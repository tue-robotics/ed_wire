#! /usr/bin/env python

import rospy

from wire_msgs.msg import WorldState, ObjectState
from problib.msg import

rospy.init_node("ed_wire_test")
pub = rospy.Publisher('/amigo/wire/world_state', WorldState, queue_size=1)

object = ObjectState()
object.ID = 123

xr = range(0, 30, 3)
for x in xr:
    x = x/10.

    # object.properties.append()
    # msg = WorldState()


