'''
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
                "object": "irb1300_1150_link_6",
                "reference": "irb1300_1150_base_link",
                "coordinates": {
                    "x": 0.5553,
                    "y": -0.05120,
                    "z": 1.0561,
                    "R": 1.1416,
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

'''

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
                "object": "irb1300_1150_link_6",
                "reference": "ARM_MODULE_1_base_link",
                "coordinates": {
                    "x": 1.175,
                    "y": 0.2625,
                    "z": 1.650,
                    "R": 0,
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
'''
#!/usr/bin/env python3
import rospy
import json
from planning_module.srv import PlanRequest

def test_client():
    rospy.init_node('test_planning_client')
    rospy.wait_for_service('/plan_request')
    try:
        plan_request = rospy.ServiceProxy('/plan_request', PlanRequest)
    
        joint_command = {
                "uuid": "EC167DB4-F4F5-49CC-80D3-29919871F5A6",
                "directive": "move",
                "parameters": {
                "object": "ARM_ID",
                "reference": "JOINT_SPACE",
                "coordinates": {
                    "j1": 0,
                    "j2": 0,
                    "j3": 0,
                    "j4": 0,
                    "j5": 1.57,
                    "j6": 0
                },
                "options": {
                    "TOL": 0.01
                }
                },
                "metadata": {
                "line_number": 55,
                "statement": "move(ARM_ID, JOINT_SPACE, {j1:0, j2: 0, j3: 0, j4:0, j5: 1.57, j6:0}, {TOL: 0.01})",
                "type": "command",
                "comment": None,
                "id": 27,
                "parsed_at": 1718838191.957207,
                "dispatched_at": None,
                "fulfilled_at": None
                },
                "valid": True

  }

     
        command_str = json.dumps(joint_command)
        response = plan_request(command_str)
        rospy.loginfo(f"Received joint response: {response}")

    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

if __name__ == '__main__':
    try:
        test_client()
    except rospy.ROSInterruptException:
        pass
'''