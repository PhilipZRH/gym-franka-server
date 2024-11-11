#! /usr/bin/env python3

import asyncio
import json
import rospy
import time
import websockets
import numpy as np

from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import PoseStamped
from franka_gripper.msg import GraspActionGoal, MoveActionGoal
from franka_msgs.msg import ErrorRecoveryActionGoal, FrankaState
from actionlib_msgs.msg import GoalID


class WebSocketInterface:
    def __init__(self):
        self.robot_state_subscriber = rospy.Subscriber(
            "/franka_state_controller/franka_states",
            FrankaState,
            self.update_robot_state,
        )

        self.pose_publisher = rospy.Publisher(
            "/cartesian_impedance_controller/equilibrium_pose",
            PoseStamped,
            queue_size=1,
        )
        self.gripper_publisher = rospy.Publisher(
            "/franka_gripper/move/goal", MoveActionGoal, queue_size=1
        )
        self.grasp_publisher = rospy.Publisher(
            "/franka_gripper/grasp/goal", GraspActionGoal, queue_size=1
        )
        self.grasp_cancel_publisher = rospy.Publisher(
            "/franka_gripper/grasp/cancel", GoalID, queue_size=1
        )
        self.error_recovery_publisher = rospy.Publisher(
            "/franka_control/error_recovery/goal", ErrorRecoveryActionGoal, queue_size=1
        )

        self.init_position = np.array([0.3, 0, 0.5])
        self.init_rotation = R.from_quat([-1, 0, 0, 0])

        self.position = None
        self.rotation = None
        self.start_input_position = None
        self.start_input_rotation = None
        self.robot_state = None

        self.gripper_goal = 0.04
        self.active_grasp_id = None
        self.previous_gripper_action_time = None

        if rospy.get_param("/websocket_interface/client") == "iOS":
            self.position_gain = 2
            self.rotation_gain = 2
            self.coordinate_change = np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]])
        else:
            self.position_gain = 1
            self.rotation_gain = 1
            self.coordinate_change = np.eye(3)

        self.active_connection = None
        self.loop = asyncio.get_event_loop()
        rospy.loginfo("[Initialized] WebSocket interface ready to start")

    async def start_server(self):
        await websockets.serve(
            self.handle_connection,
            rospy.get_param("/websocket_interface/address"),
            int(rospy.get_param("/websocket_interface/port")),
        )
        rospy.loginfo("[Started] WebSocket server is running")

    async def handle_connection(self, websocket, path):
        if self.active_connection is not None:
            await websocket.close(1008, "Another connection is already active")
            return

        self.active_connection = websocket
        rospy.loginfo("[Connected] WebSocket connection established")
        try:
            async for message in websocket:
                await self.process_command(message)
        except Exception as e:
            rospy.loginfo(f"[WebSocket Interface] Connection closed: {e}")
        finally:
            self.active_connection = None
            rospy.signal_shutdown(
                "[WebSocket Interface] Connection closed. Shutting down."
            )

    async def process_command(self, message):
        data = json.loads(message)

        message_type = data["type"]
        if message_type == "Message":
            rospy.loginfo(f"[WebSocket Message Received] {data['content']}")
        elif message_type == "Start":
            pose = data["pose"]
            self.start_input_position = np.array(
                [pose["position"]["x"], pose["position"]["y"], pose["position"]["z"]]
            )
            self.start_input_rotation = R.from_quat(
                [
                    pose["rotation"]["x"],
                    pose["rotation"]["y"],
                    pose["rotation"]["z"],
                    pose["rotation"]["w"],
                ]
            )
        elif message_type == "Track":
            pose = data["pose"]
            input_position = np.array(
                [pose["position"]["x"], pose["position"]["y"], pose["position"]["z"]]
            )
            input_rotation = R.from_quat(
                [
                    pose["rotation"]["x"],
                    pose["rotation"]["y"],
                    pose["rotation"]["z"],
                    pose["rotation"]["w"],
                ]
            )

            delta_position = (
                (input_position - self.start_input_position)
                @ self.coordinate_change.T
                * self.position_gain
            )

            delta_rotation = input_rotation * self.start_input_rotation.inv()
            delta_rotation = R.from_rotvec(
                delta_rotation.as_rotvec() * self.rotation_gain
            )
            delta_rotation = R.from_matrix(
                self.coordinate_change
                @ delta_rotation.as_matrix()
                @ self.coordinate_change.T
            )

            self.position = self.init_position + delta_position
            self.rotation = delta_rotation * self.init_rotation
            self.pose_publisher.publish(self.create_pose_command())
        elif message_type == "Gripper":
            gripper_action = data["action"]
            if gripper_action == "open":
                self.move_gripper(1)
            elif gripper_action == "close":
                self.move_gripper(-1)
        elif message_type == "Mode":
            await self.wait_for_robot_state()
            await self.active_connection.send(
                json.dumps({"mode": self.robot_state.robot_mode})
            )
        elif message_type == "State":
            await self.wait_for_robot_state()
            await self.active_connection.send(
                json.dumps(
                    {
                        "q": self.robot_state.q,
                        "dq": self.robot_state.dq,
                    }
                )
            )
        elif message_type == "Recovery":
            self.publish_error_recovery()

    async def wait_for_robot_state(self):
        while self.robot_state is None:
            await asyncio.sleep(0.001)

    def move_gripper(self, action):
        if action > 0 and self.gripper_goal != 0.04:
            if self.previous_gripper_action_time is not None:
                new_gripper_action_time = time.time()
                if new_gripper_action_time - self.previous_gripper_action_time < 1.5:
                    return
            self.gripper_goal = 0.04
            self.publish_gripper_command()
        elif action < 0 and self.gripper_goal != 0.005:
            if self.previous_gripper_action_time is not None:
                new_gripper_action_time = time.time()
                if new_gripper_action_time - self.previous_gripper_action_time < 1.5:
                    return
            self.gripper_goal = 0.005
            self.publish_gripper_command()
        else:
            return

        self.previous_gripper_action_time = time.time()

    def create_pose_command(self):
        pose_command = PoseStamped()
        pose_command.pose.position.x = self.position[0]
        pose_command.pose.position.y = self.position[1]
        pose_command.pose.position.z = self.position[2]
        quat = self.rotation.as_quat()
        pose_command.pose.orientation.x = quat[0]
        pose_command.pose.orientation.y = quat[1]
        pose_command.pose.orientation.z = quat[2]
        pose_command.pose.orientation.w = quat[3]
        return pose_command

    def publish_error_recovery(self):
        recovery_command = ErrorRecoveryActionGoal()
        self.error_recovery_publisher.publish(recovery_command)

    def publish_gripper_command(self):
        if self.gripper_goal > 0.035:
            if self.active_grasp_id is not None:
                self.grasp_cancel_publisher.publish(self.active_grasp_id)
                self.active_grasp_id = None
            move_command = MoveActionGoal()
            move_command.goal.width = 0.08
            move_command.goal.speed = 0.15
            self.gripper_publisher.publish(move_command)

        else:
            grasp_command = GraspActionGoal()
            grasp_command.goal.width = self.gripper_goal
            grasp_command.goal.force = 0.5
            grasp_command.goal.speed = 0.2
            grasp_command.goal.epsilon.inner = 0.05
            grasp_command.goal.epsilon.outer = 0.05
            self.active_grasp_id = grasp_command.goal_id
            self.grasp_publisher.publish(grasp_command)

    def update_robot_state(self, msg: FrankaState):
        self.robot_state = msg


async def main():
    rospy.init_node("websocket_interface")
    rospy.loginfo("[Init] WebSocket Interface")
    websocket_interface = WebSocketInterface()
    await websocket_interface.start_server()

    while not rospy.is_shutdown():
        await asyncio.sleep(1)


if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    try:
        loop.run_until_complete(main())
    except KeyboardInterrupt:
        pass
    finally:
        loop.close()
