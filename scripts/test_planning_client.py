#!/usr/bin/env python3
import rospy
from planning_module.srv import PlanRequest

def test_planning_client():
    rospy.init_node('test_planning_client')

    rospy.wait_for_service('/plan_request')
    try:
        plan_request_service = rospy.ServiceProxy('/plan_request', PlanRequest)
        
        mpl_command = "move(renishaw, BA9BC999-063D-44B9-8798-9A37F18680F3, \"x: 0.11, y: 0.006, z: 0.76, R: 0.0, P: 0.0, Y: 0.0\")"
        
        response = plan_request_service(mplcommand=mpl_command)
        rospy.loginfo(f"Received response: {response.valid_path}")

      
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

if __name__ == '__main__':
    try:
        test_planning_client()
    except rospy.ROSInterruptException:
        pass
