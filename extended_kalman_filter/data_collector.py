import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

class Ndt():
    def __init__(self):
    
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odomcallBack)
        self.imu_sub = rospy.Subscriber("/imu", Imu, self.imucallBack)
        self.odom_pub = rospy.Publisher("/odom_pub", Odometry, queue_size=10)
        self.imu_pub = rospy.Publisher("/imu_pub", Imu, queue_size=10)
        
    def odomcallBack(self, data):

        self.odom_pub.publish(data)
    
        self.odomOrtX = data.pose.pose.orientation.x
        self.odomOrtY = data.pose.pose.orientation.y
        self.odomOrtZ = data.pose.pose.orientation.z
        self.odomOrtW = data.pose.pose.orientation.w

        self.odomPosX = data.pose.pose.position.x
        self.odomPosY = data.pose.pose.position.y
        self.odomPosZ = data.pose.pose.position.z

        self.odomPoseCovariance = data.pose.covariance

        self.odomLinVelX = data.twist.twist.linear.x
        self.odomLinVelY = data.twist.twist.linear.y
        self.odomLinVelZ = data.twist.twist.linear.z

        self.odomAngVelX = data.twist.twist.angular.x
        self.odomAngVelY = data.twist.twist.angular.y
        self.odomAngVelZ = data.twist.twist.angular.z

        self.odomTwistCovariance = data.twist.covariance

    def imucallBack(self, data):

        self.imu_pub.publish(data)

        self.imuOrtX = data.orientation.x
        self.imuOrtY = data.orientation.y
        self.imuOrtZ = data.orientation.z
        self.imuOrtW = data.orientation.w

        self.imuOrtCovariance = data.orientation_covariance

        self.imuAngVelX = data.angular_velocity.x
        self.imuAngVelY = data.angular_velocity.y
        self.imuAngVelZ = data.angular_velocity.z

        self.imuAngVelCovariance = data.angular_velocity_covariance

        self.imuLinAccX = data.linear_acceleration.x
        self.imuLinAccY = data.linear_acceleration.y
        self.imuLinAccZ = data.linear_acceleration.z

        self.imuLinAccCovariance = data.linear_acceleration_covariance

if __name__ == '__main__':
    rospy.init_node("Collector")
    Ndt()
    rospy.spin()
