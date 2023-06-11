import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, QoSDurabilityPolicy



class ExtendMapNode(Node):

    def __init__(self):
        super().__init__('map_edit')
        qos_profile = QoSProfile(depth=10, durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(OccupancyGrid, '/map', qos_profile=qos_profile)
        self.subscription = self.create_subscription(OccupancyGrid, '/st_map', self.listener_callback, 10)

    def listener_callback(self, msg):
        padding = 10.0 #meters
        # Determine number of cells to add based on resolution and desired extension in meters
        cells_to_add = int(padding / msg.info.resolution) 
        print(cells_to_add)
        # create a new OccupancyGrid with extra cells on each side
        new_msg = OccupancyGrid()
        new_msg.header = msg.header
        new_msg.info.resolution = msg.info.resolution
        new_msg.info.width = msg.info.width + 2 * cells_to_add  # Add cells to each side
        new_msg.info.height = msg.info.height + 2 * cells_to_add  # Add cells to each side
        new_msg.info.origin.position.x = msg.info.origin.position.x - padding
        new_msg.info.origin.position.y = msg.info.origin.position.y - padding
        new_msg.info.origin.position.z = 0.0
        new_msg.data = [-1] * (new_msg.info.width * new_msg.info.height)  # -1 represents unknown in the OccupancyGrid
        # copy the old data into the new OccupancyGrid
        for i in range(msg.info.height):
            # print('here', msg.info.width * msg.info.height)
            # print('here', new_msg.info.width * new_msg.info.height)
            for j in range(msg.info.width):
                old_index = i * msg.info.width + j
                new_index = (i + cells_to_add) * new_msg.info.width + (j + cells_to_add)
                new_msg.data[new_index] = msg.data[old_index]

        # publish the new OccupancyGrid
        self.publisher.publish(new_msg)

def main(args=None):
    rclpy.init(args=args)

    node = ExtendMapNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

