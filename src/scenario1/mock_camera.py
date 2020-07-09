#!/usr/bin/env python2
import rospy
import gazebo_msgs.msg
import std_msgs.msg
import std_srvs.srv


def calcDistance(agentPose, victimPose):
    return ((agentPose.position.x - victimPose.position.x)**2
            + (agentPose.position.y - victimPose.position.y)**2)**0.5


class MockCamera:
    def __init__(self):
        self.victim_pub = rospy.Publisher(
            'mock_camera/victim_detection',
            std_msgs.msg.Int32,
            queue_size=5,
            latch=True
        )

        self.camera_is_on = False
        self.camera_switch_service = rospy.Service(
            'mock_camera/switch',
            std_srvs.srv.SetBool,
            self.camera_switch_callback)

        self.victims_detected = []
        self.gazebo_model_sub = rospy.Subscriber(
            '/gazebo/model_states',
            gazebo_msgs.msg.ModelStates,
            self.gazebo_model_callback
        )

    def gazebo_model_callback(self, msg):
        victims_numer = len(self.victims_detected)

        if self.camera_is_on:
            try:
                node_namespace = rospy.get_namespace()
                self.agent_name = rospy.get_param(
                                node_namespace + 'jason/agent_name')
                agent_index = msg.name.index(self.agent_name)
                agent_pose = msg.pose[agent_index]

                self.dist = rospy.get_param(
                                node_namespace
                                + 'mock_camera/detection_distance')

                victim_index = [index for index, model in enumerate(msg.name)
                                if model.startswith('victim')
                                and model not in self.victims_detected]

                for index in victim_index:
                    if calcDistance(agent_pose, msg.pose[index]) < self.dist:
                        self.victims_detected.append(msg.name[index])
                        victim_msg = std_msgs.msg.Int32()
                        victim_msg.data = victims_numer + 1
                        self.victim_pub.publish(victim_msg)
            except KeyError:
                pass
            except ValueError:
                pass

    def camera_switch_callback(self, req):
        self.camera_is_on = req.data
        return std_srvs.srv.SetBoolResponse(True, "Is On? " + str(req.data))


if __name__ == '__main__':
    rospy.init_node('mock_camera')
    mock_camera = MockCamera()
    rospy.spin()
