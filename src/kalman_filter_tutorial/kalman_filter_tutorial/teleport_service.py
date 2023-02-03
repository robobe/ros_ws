import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.srv._set_entity_state import SetEntityState_Response
from std_srvs.srv import Trigger
from std_srvs.srv._trigger import Trigger_Response
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from threading import Event

SRV_RESET_MODEL_STATE = "/reset_model_pose"
SRV_SET_ENTITY = "/gazebo/set_entity_state"
MODEL_NAME = "burger"

class MyNode(Node):
    def __init__(self):
        node_name="teleport_service"
        super().__init__(node_name)
        self.callback_group = ReentrantCallbackGroup()
        self.declare_parameter('pos_x', value=0)
        self.__server = self.create_service(Trigger,
            SRV_RESET_MODEL_STATE,
            self.__trigger_handler,
            callback_group=self.callback_group)
        self.set_entity_state_client = self.create_client(SetEntityState, 
            SRV_SET_ENTITY,
            callback_group=self.callback_group)
        self.get_logger().info(f"init {node_name}")

    def __trigger_handler(self, request, response: Trigger_Response):
        self.get_logger().info("start handle reset pose handler")
        request = SetEntityState.Request()
        request.state.reference_frame = "world"
        request.state.name = MODEL_NAME
        request.state.pose.position.x = 2.0
        request.state.pose.position.y = 2.0
        request.state.pose.position.z = 0.0
        request.state.pose.orientation.x = 0.0
        request.state.pose.orientation.y = 0.0
        request.state.pose.orientation.z = 0.0
        request.state.pose.orientation.w = 1.0

        event=Event()
        def done_callback(future):
            self.get_logger().info("state result")
            nonlocal event
            event.set()

        try:
            future = self.set_entity_state_client.call_async(request)
            future.add_done_callback(done_callback)
            self.get_logger().info("Send entity state request")
        except:
            self.get_logger().error("-=-------------------")
        event.wait()

        set_response: SetEntityState_Response
        set_response = future.result()
        response.success = set_response.success
        return response

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    # https://gist.github.com/driftregion/14f6da05a71a57ef0804b68e17b06de5

    # https://ubuntu.com/blog/creating-a-ros-2-cli-command-and-verb

#     ros2 service call /demo/set_entity_state gazebo_msgs/srv/SetEntityState \
# "state: {name: cube::link, pose: \
# {position:{x: 0.0, y: 0.0, z: 2.5},
# orientation:{x: 0.7071, y: 0.0, z: 0.7071, w: 0.0}}, \
# reference_frame: world}"