#!/usr/bin/env python3
import rospy
import json
from planning_module.srv import PlanRequest

def test_client():
    rospy.init_node('test_planning_client')
    rospy.wait_for_service('/plan_request')
    try:
        plan_request = rospy.ServiceProxy('/plan_request', PlanRequest)
        command = {
            "uuid": "5BFBE7E3-32AC-4148-A23A-20CE801E91E1",
            "directive": "move",
            "parameters": {
                "object": "RENISHAW_TOOL",
                "reference": "NANO_SURFACE",
                "coordinates": {
                    "x": 0.5553,
                    "y": -0.05120,
                    "z": 1.0561,
                    "R": 3.1416,
                    "P": 0,
                    "Y": 0
                },
                "options": {
                    "AVOID_COLLISION_DISABLED": 1,
                    "MAX_VEL": 0.25,
                    "ASYNC": 0,
                    "last_motion": 0
                }
            },
            "metadata": {
                "line_number": 13,
                "statement": "move(RENISHAW_TOOL, NANO_SURFACE, {x:0.55532 y: -0.05120, z: 1.0561, R:3.1416, P:0, Y:0}, {AVOID_COLLISION_DISABLED:1, MAX_VEL: 0.25, ASYNC:0,  last_motion:0})",
                "type": "command",
                "comment": None,
                "id": 5,
                "parsed_at": 1717612965.473957,
                "dispatched_at": None,
                "fulfilled_at": None
            },
            "valid": True
        }
        command_str = json.dumps(command)
        response = plan_request(command_str)
        rospy.loginfo(f"Received response: {response}")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

if __name__ == '__main__':
    try:
        test_client()
    except rospy.ROSInterruptException:
        pass
