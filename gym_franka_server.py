import asyncio
import json
import os
import subprocess
import websockets


SERVER_IP = os.environ.get("SERVER_IP")
SERVER_PORT = 8888
FCI_IP = os.environ.get("FCI_IP")
ROS_INTERFACE_IP = "localhost"
ROS_INTERFACE_PORT = 6666

DT = 0.5


class GymFrankaServer:
    def __init__(self):
        self.translation = None
        self.rotation = None
        self.ros_websocket = None
        self.move_history = []
        self.websocket = None
        self.websocket_server = None

    async def start_server(self):
        self.websocket_server = await websockets.serve(
            self.handle_connection,
            SERVER_IP,
            SERVER_PORT,
            ping_interval=None,
        )
        print(
            f"[Gym Franka Server] WebSocket server started on {SERVER_IP}:{SERVER_PORT}"
        )
        await self.websocket_server.wait_closed()

    async def handle_connection(self, websocket, path):
        self.websocket = websocket
        print("[Gym Franka Server] Connected!")
        try:
            async for message in websocket:
                if not await self.process_request(message):
                    break
        except Exception as e:
            print(f"[Gym Franka Server] Connection closed: {e}")
            self.websocket_server.close()
        finally:
            self.websocket = None

    async def process_request(self, message):
        try:
            data = json.loads(message)
            command = data["command"]
            timestamp = data["timestamp"]
        except json.JSONDecodeError as e:
            print(f"Failed to parse JSON: {e}")
            return True

        if command == "reset":
            print("[Gym Franka Server] Reset.")
            await self.reset()
            echo_message = json.dumps({"timestamp": timestamp})
            await self.websocket.send(echo_message)
        elif command == "move":
            reflex = False
            pose = data["pose"]
            gripper_action = data["gripper_action"]
            wait = data["wait"]

            print(
                f"[Gym Franka Server] Step: Position: {pose['position']}, Rotation: {pose['rotation']}, Gripper: {gripper_action}"
            )
            if await self.get_robot_mode() == 4:
                reflex = True
            if not await self.is_ready():
                await self.wait_for_ready()
            if not reflex:
                await self.step(pose=pose, gripper_action=gripper_action)
            if wait:
                await asyncio.sleep(DT)

            while not await self.is_ready():
                reflex = True
                print("[Gym Franka Server] Attempting Reflex rollback...")
                await self.wait_for_ready()
                self.move_history.pop()
                if len(self.move_history) > 0:
                    await self.send_ros_command(self.move_history[-1])
                    await asyncio.sleep(DT)
            status = "Reflex" if reflex else "Success"
            echo_message = json.dumps({"status": status, "timestamp": timestamp})
            await self.websocket.send(echo_message)
        elif command == "grasp":
            await self.send_ros_command({"type": "Gripper", "action": "close"})
        elif command == "release":
            await self.send_ros_command({"type": "Gripper", "action": "open"})
        elif command == "Close":
            print("[Gym Franka Server] Shutdown.")
            return False
        elif command == "state":
            state = await self.get_robot_state()
            echo_message = json.dumps({"state": state, "timestamp": timestamp})
            await self.websocket.send(echo_message)

        return True

    async def send_ros_command(self, command):
        if self.ros_websocket:
            await self.ros_websocket.send(json.dumps(command))

    async def get_robot_mode(self):
        if self.ros_websocket:
            await self.send_ros_command({"type": "Mode"})
            response = await self.ros_websocket.recv()
            data = json.loads(response)
            return data["mode"]
        return 0

    async def get_robot_state(self):
        if self.ros_websocket:
            await self.send_ros_command({"type": "State"})
            response = await self.ros_websocket.recv()
            data = json.loads(response)
            return data
        return None

    async def is_ready(self):
        robot_mode = await self.get_robot_mode()
        return robot_mode == 2

    async def wait_for_ready(self):
        message_printed = False
        while True:
            robot_mode = await self.get_robot_mode()
            if robot_mode != 2:
                if not message_printed:
                    print(
                        "[Gym Franka Server] Waiting for robot... Robot Status:",
                        end=" ",
                        flush=True,
                    )
                    message_printed = True
                print(robot_mode, end=" ", flush=True)
                await self.send_ros_command({"type": "Recovery"})
                await asyncio.sleep(0.1)
            else:
                if message_printed:
                    print(flush=True)
                return

    async def establish_ros_connection(self):
        command = (
            f"gnome-terminal --window --wait -x roslaunch gym_franka_server ci_socket.launch "
            f"robot_ip:={FCI_IP} address:={ROS_INTERFACE_IP} port:={ROS_INTERFACE_PORT} client:=gym"
        )
        self.ros_process = subprocess.Popen(
            command,
            shell=True,
            executable="/bin/bash",
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        for attempt in range(10):
            try:
                self.ros_websocket = await websockets.connect(
                    f"ws://{ROS_INTERFACE_IP}:{ROS_INTERFACE_PORT}"
                )
                print(
                    f"[Gym Franka Server] Connected to ROS WebSocket on attempt {attempt + 1}"
                )
                return True
            except Exception as e:
                print(
                    f"[Gym Franka Server] Failed to connect to ROS WebSocket (attempt {attempt + 1}): {e}"
                )
                self.ros_websocket = None
                if attempt < 9:  # Don't sleep after the last attempt
                    await asyncio.sleep(1)

        print(
            "[Gym Franka Server] Failed to connect to ROS WebSocket after 10 attempts"
        )
        return False

    async def home(self):
        if self.ros_websocket is not None:
            await self.wait_for_ready()
            await self.ros_websocket.close()
            self.ros_websocket = None
            self.ros_process.wait()
            self.ros_process = None
        command = f"gnome-terminal --window --wait -x roslaunch gym_franka_server home.launch robot_ip:={FCI_IP}"
        reset_process = subprocess.Popen(
            command,
            shell=True,
            executable="/bin/bash",
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        reset_process.wait()

    async def hard_reset(self):
        print("[Gym Franka Server] Fatal error! Hard reset...")
        if self.ros_websocket is not None:
            await self.ros_websocket.close()
            self.ros_websocket = None
        if self.ros_process is not None:
            self.ros_process.kill()
            self.ros_process = None
        await self.establish_ros_connection()
        await self.wait_for_ready()
        await self.reset()

    async def reset(self):
        await self.home()
        await self.establish_ros_connection()
        if not await self.is_ready():
            await self.wait_for_ready()
        self.move_history = []
        self.pose = {
            "position": {"x": 0.3, "y": 0, "z": 0.5},
            "rotation": {"x": 0, "y": -1, "z": 0, "w": 0},
        }
        await self.send_ros_command(
            {
                "type": "Start",
                "pose": self.pose,
            }
        )

    async def step(self, pose, gripper_action=None):
        self.pose = pose
        move_command = {
            "type": "Track",
            "pose": pose,
        }
        self.move_history.append(move_command)
        await self.send_ros_command(move_command)
        if gripper_action is not None and gripper_action != 0:
            if gripper_action < 0:
                action = "close"
            else:
                action = "open"
            await self.send_ros_command({"type": "Gripper", "action": action})


if __name__ == "__main__":
    server = GymFrankaServer()
    asyncio.get_event_loop().run_until_complete(server.start_server())
