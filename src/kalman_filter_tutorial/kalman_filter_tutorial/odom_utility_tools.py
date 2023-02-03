import rclpy
from rclpy.node import Node
from rclpy.logging import LoggingSeverity
from std_msgs.msg import Float64
from gazebo_msgs.srv import GetModelList, GetEntityState
from gazebo_msgs.srv._get_model_list import GetModelList_Response
from gazebo_msgs.srv._get_entity_state import GetEntityState_Response
from rclpy.qos import qos_profile_system_default
import numpy as np

TOPIC_GROUND_TRUTH_X = "/ground_truth_x"
TOPIC_NOISY_ODOM_X = "/noisy_odom_x"
SERVICE_ENTITY_STATE = "/gazebo/get_entity_state"
SERVICE_MODEL_LIST = "/get_model_list"
MODEL_NAME = "burger"
PERIOD = 1 / 2


class MyNode(Node):
    def __init__(self):
        node_name = "kf_auxiliary_topics"
        # self.get_logger().set_level(LoggingSeverity.INFO)
        super().__init__(node_name)
        self.__alpha = 4
        self.__noisy_odom = 0
        self.__last_ground_truth = 0

        self.truth_pub = self.create_publisher(
            Float64, TOPIC_GROUND_TRUTH_X, qos_profile=qos_profile_system_default
        )
        self.noisy_odom_pub = self.create_publisher(
            Float64, TOPIC_NOISY_ODOM_X, qos_profile=qos_profile_system_default
        )
        self.model_list_srv = self.create_client(GetModelList, SERVICE_MODEL_LIST)
        self.entity_state_srv = self.create_client(GetEntityState, SERVICE_ENTITY_STATE)
        self.model_list_srv.wait_for_service()
        self.entity_state_srv.wait_for_service()
        request = GetModelList.Request()
        future = self.model_list_srv.call_async(request)
        future.add_done_callback(self.__model_list_handler)
        self.get_logger().info(f"init {node_name}")

    def __model_list_handler(self, future):
        response: GetModelList_Response
        response = future.result()
        if not response.success:
            raise SystemExit

        for item in response.model_names:
            if item == MODEL_NAME:
                self.get_logger().info(f"--found {MODEL_NAME}")
                self.timer = self.create_timer(PERIOD, self.__timer_handler)

    def __timer_handler(self):
        state_request = GetEntityState.Request()
        state_request.name = MODEL_NAME
        state_request.reference_frame = "world"
        future = self.entity_state_srv.call_async(state_request)
        future.add_done_callback(self.__entity_state_handler)

    def __entity_state_handler(self, future):
        response: GetEntityState_Response
        response = future.result()
        ground_truth_x = response.state.pose.position.x

        msg = Float64()
        msg.data = ground_truth_x
        self.truth_pub.publish(msg)

        ground_truth_delta = ground_truth_x = self.__last_ground_truth

        sd_trans = self.__alpha * (ground_truth_delta)
        self.__noisy_odom += np.random.normal(ground_truth_delta, sd_trans * sd_trans)
        msg = Float64()
        msg.data = self.__noisy_odom
        self.noisy_odom_pub.publish(msg)

        self.__last_ground_truth = ground_truth_x


def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
