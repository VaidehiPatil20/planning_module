#!/usr/bin/env python3
import rospy
from planning_module.srv import PlanRequest

def test_planning_client():
    rospy.init_node('test_planning_client')

    rospy.wait_for_service('/plan_request')
    try:
        plan_request_service = rospy.ServiceProxy('/plan_request', PlanRequest)
        
        mpl_command = "move(renishaw, BA9BC999-063D-44B9-8798-9A37F18680F3, \"x: 0.3554363, y: 0.07570673, z: 1.28765107, R: 0.3834635, P: -0.03105832, Y: 0.34727256\")"
        
        response = plan_request_service(mplcommand=mpl_command)
        rospy.loginfo(f"Received response: {response.valid_path}")

      
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

if __name__ == '__main__':
    try:
        test_planning_client()
    except rospy.ROSInterruptException:
        pass
