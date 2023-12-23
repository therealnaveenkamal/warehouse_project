import rclpy
from std_srvs.srv import Empty
import std_msgs
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
import time
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from time import sleep
from geometry_msgs.msg import Polygon, Point32

from nav_msgs.msg import Odometry
import math
import sys



def update_shape(node, side):
    publisher_global = node.create_publisher(Polygon, '/global_costmap/footprint', 10)
    publisher_local = node.create_publisher(Polygon, '/local_costmap/footprint', 10)
    footprint = Polygon()
    side = 0.5
    footprint.points = [
        Point32(x= side/2, y= side/2),
        Point32(x= side/2, y=-side/2),
        Point32(x=-side/2, y=-side/2),
        Point32(x=-side/2, y= side/2)
    ]
    print('Publishing Normal Polygon')
    publisher_global.publish(footprint)
    publisher_local.publish(footprint)
    sleep(1)
        
def elevator_push_up(node, use_sim_time):
    publisher_up = None
    msg = None
    if(use_sim_time):
        publisher_up = node.create_publisher(std_msgs.msg.Empty, '/elevator_up', 10)
        msg = std_msgs.msg.Empty()
    else:
        publisher_up = node.create_publisher(std_msgs.msg.String, '/elevator_up', 10)
        msg = std_msgs.msg.String()
        msg.data = ''

    print('Publishing elevator up')
    publisher_up.publish(msg)
    sleep(1)
    publisher_up.publish(msg)
    sleep(5)
    publisher_up.publish(msg)
    sleep(5)

def elevator_push_down(node, use_sim_time):
    publisher_down = None
    msg = None
    if(use_sim_time):
        publisher_down = node.create_publisher(std_msgs.msg.Empty, '/elevator_down', 10)
        msg = std_msgs.msg.Empty()
    else:
        publisher_down = node.create_publisher(std_msgs.msg.String, '/elevator_down', 10)
        msg = std_msgs.msg.String()
        msg.data = ''

    print('Publishing elevator down')
    publisher_down.publish(msg)
    sleep(1)
    publisher_down.publish(msg)
    sleep(5)
    publisher_down.publish(msg)
    sleep(5)
    
def reinitialize_global_localization(node):
    client = node.create_client(Empty, '/reinitialize_global_localization')
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Service /reinitialize_global_localization not available, waiting...')
    request = Empty.Request()
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
    if future.result() is not None:
        node.get_logger().info('Global localization reinitialized successfully!')
    else:
        node.get_logger().error('Failed to reinitialize global localization.')

global_amcl_pose = None
def amcl_pose_callback(msg):
    global global_amcl_pose
    global_amcl_pose = msg.pose.pose

def spin_for_duration(node, duration, angular_speed, topic):
    global global_amcl_pose
    publisher = node.create_publisher(Twist, topic, 10)
    subscriber = node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', amcl_pose_callback, 10)
    twist_msg = Twist()
    twist_msg.angular.z = angular_speed
    start_time = time.time()
    while time.time() - start_time < duration:
        node.get_logger().info('Time: %f, Duration: %f' % (time.time() - start_time, duration))
        publisher.publish(twist_msg)
        rclpy.spin_once(node, timeout_sec=0.1)
    twist_msg.angular.z = 0.0
    publisher.publish(twist_msg)

def move_below_cart(node, duration, speed, topic):
    global global_amcl_pose
    publisher = node.create_publisher(Twist, topic, 10)
    subscriber = node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', amcl_pose_callback, 10)
    twist_msg = Twist()
    twist_msg.linear.x = speed
    start_time = time.time()
    while time.time() - start_time < duration:
        #node.get_logger().info('Time: %f, Duration: %f' % (time.time() - start_time, duration))
        publisher.publish(twist_msg)
        rclpy.spin_once(node, timeout_sec=0.1)
    
    twist_msg.linear.x = 0.0
    publisher.publish(twist_msg)

def move_to_goal(node, iPose, fPose, navigator):
    node.get_logger().info('STARTING NAVIGATOR')
    
    iPose.header.stamp = navigator.get_clock().now().to_msg()
    navigator.setInitialPose(iPose)

    navigator.waitUntilNav2Active()
    node.get_logger().info('Initialize Pose Successful')
    
    fPose.header.stamp = navigator.get_clock().now().to_msg()
    navigator.goToPose(fPose)

    i = 0
    while not navigator.isTaskComplete():
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print(
                'Estimated time of arrival: '
                + '{0:.0f}'.format(
                    Duration.from_msg(feedback.estimated_time_remaining).nanoseconds
                    / 1e9
                )
                + ' seconds.'
            )

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

def path_to_shipping(node, iPose, poses, navigator):
    node.get_logger().info('Final AMCL Pose: %s' % global_amcl_pose)
    node.get_logger().info('STARTING NAVIGATOR')
    
    navigator.setInitialPose(iPose)

    navigator.waitUntilNav2Active()
    node.get_logger().info('Initialize Pose Successful')
    
    navigator.followWaypoints(poses)

    i = 0
    while not navigator.isTaskComplete():
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print(
                'Executing current waypoint: '
                + str(feedback.current_waypoint + 1)
                + '/'
                + str(len(poses))
            )


def main():
    global global_amcl_pose
    rclpy.init()
    use_sim_time = True
    if len(sys.argv) > 1:
        for arg in sys.argv[1:]:
            key, value = arg.split('=')
            if key.lower() == 'use_sim_time':
                use_sim_time = value.lower() == 'true'
                break

    node = rclpy.create_node('navigation_node')

    global_amcl_pose = None
    reinitialize_global_localization(node)
    spin_duration = 3
    angular_speed_left = 1.25
    angular_speed_right = -1.25
    topic = '/robot/cmd_vel'
    if(use_sim_time):
        topic = '/robot/cmd_vel'
    else:
        topic = '/cmd_vel'

    spin_for_duration(node, spin_duration, -angular_speed_left, topic)
    spin_for_duration(node, spin_duration+3, -angular_speed_right, topic)
    spin_for_duration(node, spin_duration+3, -angular_speed_left, topic)
    spin_for_duration(node, spin_duration+3, -angular_speed_right, topic)

    navigator = BasicNavigator()
    init_position = None

    if global_amcl_pose is not None and use_sim_time==True:
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'robot_odom'
        initial_pose.pose = global_amcl_pose
        initial_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.pose.position.x = 5.8
        goal_pose.pose.position.y = -0.2
        goal_pose.pose.position.z = global_amcl_pose.position.z
        goal_pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=-0.72, w=0.68)
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        move_to_goal(node, initial_pose, goal_pose, navigator)

        move_below_cart(node, 11, 0.2, topic)
        node.get_logger().info('RB1 placement under cart is successful')
        elevator_push_up(node, use_sim_time)
        node.get_logger().info('Cart Lift-Off Phase Successful, Updating Shape')
        update_shape(node, 0.8)
        node.get_logger().info('Shape Updated, Moving Back')
        move_below_cart(node, 11, -0.20, topic)
        spin_for_duration(node, 4, -0.75, topic)
        node.get_logger().info('Computing New Goal')

        new_init_pose = PoseStamped()
        new_init_pose.header.frame_id = 'map'
        new_init_pose.pose = global_amcl_pose
        new_init_pose.header.stamp = navigator.get_clock().now().to_msg()
        destposes = []
        dest_pose1 = PoseStamped()
        dest_pose1.header.frame_id = 'map'
        dest_pose1.header.stamp = navigator.get_clock().now().to_msg()
        dest_pose1.pose.position.x = 4.0
        dest_pose1.pose.position.y = 0.0
        dest_pose1.pose.position.z = 0.0
        dest_pose1.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.999935, w=0.0114049)
        destposes.append(dest_pose1)
        dest_pose5 = PoseStamped()
        dest_pose5.header.frame_id = 'map'
        dest_pose5.header.stamp = navigator.get_clock().now().to_msg()
        dest_pose5.pose.position.x = 1.5
        dest_pose5.pose.position.y = 0.0
        dest_pose5.pose.position.z = 0.0
        dest_pose5.pose.orientation = Quaternion(x=0.0, y=0.0, z=-0.999935, w=0.0114049)
        destposes.append(dest_pose5)
        path_to_shipping(node, new_init_pose, destposes, navigator)

        angular_speed_left = 1.25
        angular_speed_right = -1.25
        spin_for_duration(node, 10, -angular_speed_left, topic)
        spin_for_duration(node, 10, -angular_speed_right, topic)

        turn_init_pose = PoseStamped()
        turn_init_pose.header.frame_id = 'map'
        turn_init_pose.pose = global_amcl_pose
        turn_init_pose.header.stamp = navigator.get_clock().now().to_msg()

        destposes2 = []
        dest_pose5 = PoseStamped()
        dest_pose5.header.frame_id = 'map'
        dest_pose5.header.stamp = navigator.get_clock().now().to_msg()
        dest_pose5.pose.position.x = 0.6
        dest_pose5.pose.position.y = -1.0
        dest_pose5.pose.position.z = 0.0
        dest_pose5.pose.orientation = Quaternion(x=0.0, y=0.0, z=-0.999935, w=0.0114049)
        destposes2.append(dest_pose5)

        ship_dest = PoseStamped()
        ship_dest.header.frame_id = 'map'
        ship_dest.header.stamp = navigator.get_clock().now().to_msg()
        ship_dest.pose.position.x = 0.6
        ship_dest.pose.position.y = -2.5
        ship_dest.pose.position.z = 0.0
        ship_dest.pose.orientation = Quaternion(x=0.0, y=0.0, z=-0.71, w=0.69)
        destposes2.append(ship_dest)

        path_to_shipping(node, turn_init_pose, destposes2, navigator)

        node.get_logger().info('Shipping Task Success')

        elevator_push_down(node, use_sim_time)
        move_below_cart(node, 7, -0.2, topic)
        update_shape(node, 0.5)

        ship_init_pose = PoseStamped()
        ship_init_pose.header.frame_id = 'map'
        ship_init_pose.pose = global_amcl_pose
        ship_init_pose.header.stamp = navigator.get_clock().now().to_msg()
        final_dest = PoseStamped()
        final_dest.header.frame_id = 'map'
        final_dest.header.stamp = navigator.get_clock().now().to_msg()
        final_dest.pose.position.x = 0.05
        final_dest.pose.position.y = 0.05
        final_dest.pose.position.z = 0.0
        final_dest.pose.orientation = Quaternion(x=0.0, y=0.0, z=-0.7, w=0.7)

        move_to_goal(node, ship_init_pose, final_dest, navigator)
        node.get_logger().info('Final Task Success')


    elif global_amcl_pose is not None and use_sim_time==False:
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'robot_odom'
        initial_pose.pose = global_amcl_pose
        initial_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.pose.position.x = 4.0
        goal_pose.pose.position.y = -1.0
        goal_pose.pose.position.z = global_amcl_pose.position.z
        goal_pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=-0.7, w=0.7)
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        move_to_goal(node, initial_pose, goal_pose, navigator)

        move_below_cart(node, 9, 0.15, topic) #TO CHECK
        node.get_logger().info('RB1 placement under cart is successful')
        elevator_push_up(node, use_sim_time)
        node.get_logger().info('Cart Lift-Off Phase Successful, Updating Shape')
        update_shape(node, 0.8)
        node.get_logger().info('Shape Updated, Moving Back')
        move_below_cart(node, 6, -0.20, topic)
        spin_for_duration(node, 2, -0.75, topic)
        node.get_logger().info('Computing New Goal')

        new_init_pose = PoseStamped()
        new_init_pose.header.frame_id = 'map'
        new_init_pose.pose = global_amcl_pose
        new_init_pose.header.stamp = navigator.get_clock().now().to_msg()
        destposes = []
        dest_pose1 = PoseStamped()
        dest_pose1.header.frame_id = 'map'
        dest_pose1.header.stamp = navigator.get_clock().now().to_msg()
        dest_pose1.pose.position.x = 2.46
        dest_pose1.pose.position.y = -0.68
        dest_pose1.pose.position.z = 0.0
        dest_pose1.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.998, w=0.054)
        destposes.append(dest_pose1)

        dest_pose5 = PoseStamped()
        dest_pose5.header.frame_id = 'map'
        dest_pose5.header.stamp = navigator.get_clock().now().to_msg()
        dest_pose5.pose.position.x = 1.5
        dest_pose5.pose.position.y = -0.68
        dest_pose5.pose.position.z = 0.0
        dest_pose5.pose.orientation = Quaternion(x=0.0, y=0.0, z=-0.998, w=0.054)
        destposes.append(dest_pose5)

        path_to_shipping(node, new_init_pose, destposes, navigator)
        
        angular_speed_left = 1.0
        angular_speed_right = -1.0
        spin_for_duration(node, 10, -angular_speed_left, topic)
        spin_for_duration(node, 10, -angular_speed_right, topic)


        destposes2 = []
        turn_init_pose = PoseStamped()
        turn_init_pose.header.frame_id = 'map'
        turn_init_pose.pose = global_amcl_pose
        turn_init_pose.header.stamp = navigator.get_clock().now().to_msg()
        dest_pose_new = PoseStamped()
        dest_pose_new.header.frame_id = 'map'
        dest_pose_new.header.stamp = navigator.get_clock().now().to_msg()
        dest_pose_new.pose.position.x = -0.18
        dest_pose_new.pose.position.y = -1.0
        dest_pose_new.pose.position.z = 0.0
        dest_pose_new.pose.orientation = Quaternion(x=0.0, y=0.0, z=-0.998, w=0.054)
        destposes2.append(dest_pose_new)

        ship_dest = PoseStamped()
        ship_dest.header.frame_id = 'map'
        ship_dest.header.stamp = navigator.get_clock().now().to_msg()
        ship_dest.pose.position.x = -0.18
        ship_dest.pose.position.y = -2.52
        ship_dest.pose.position.z = 0.0
        ship_dest.pose.orientation = Quaternion(x=0.0, y=0.0, z=-0.76, w=0.64)
        destposes2.append(ship_dest)
        
        path_to_shipping(node, turn_init_pose, destposes2, navigator)


        node.get_logger().info('Shipping Task Success')

        elevator_push_down(node, use_sim_time)
        move_below_cart(node, 7, -0.2, topic)
        update_shape(node, 0.5)

        ship_init_pose = PoseStamped()
        ship_init_pose.header.frame_id = 'map'
        ship_init_pose.pose = global_amcl_pose
        ship_init_pose.header.stamp = navigator.get_clock().now().to_msg()
        final_dest = PoseStamped()
        final_dest.header.frame_id = 'map'
        final_dest.header.stamp = navigator.get_clock().now().to_msg()
        final_dest.pose.position.x = 0.0
        final_dest.pose.position.y = 0.0
        final_dest.pose.position.z = 0.0
        final_dest.pose.orientation = Quaternion(x=0.0, y=0.0, z=-0.7, w=0.7)

        move_to_goal(node, ship_init_pose, final_dest, navigator)
        node.get_logger().info('Final Task Success')

    else:
        node.get_logger().warning('No AMCL Pose received. Cannot set the goal pose.')
    




if __name__ == '__main__':
    main()