"""

Extended kalman filter (EKF) localization sample

author: Atsushi Sakai (@Atsushi_twi)

"""
import sys
import pathlib
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

sys.path.append(str(pathlib.Path(__file__).parent.parent.parent))

import math
import matplotlib.pyplot as plt
import numpy as np

from utils.plot import plot_covariance_ellipse
import rospy


class Ekf():
    def __init__(self):
        
        # Covariance for EKF simulation
        self.Q = np.diag([
            0.1,  # variance of location on x-axis
            0.1,  # variance of location on y-axis
            np.deg2rad(1.0),  # variance of yaw angle
            1.0  # variance of velocity
        ]) ** 2  # predict state covariance
        self.R = np.diag([1.0, 1.0]) ** 2  # Observation x,y position covariance

        #  Simulation parameter
        self.INPUT_NOISE = np.diag([1.0, np.deg2rad(30.0)]) ** 2
        self.GPS_NOISE = np.diag([0.5, 0.5]) ** 2

        self.DT = 0.1  # time tick [s]
        self.SIM_TIME = 40.0  # simulation time [s]
        self.LinVel = 0.0
        self.YawRate = 0.0
        self.sub1 = rospy.Subscriber("/odom_pub", Odometry, self.callBackOdom)
        self.sub2 = rospy.Subscriber("/imu_pub", Imu, self.callBackImu)
        self.show_animation = True
        self.main()
        


    def callBackOdom(self, data):

        self.Pos = data.pose.pose.position
        self.Ort = data.pose.pose.orientation
        self.LinVel = data.twist.twist.linear.x
        #print(f"callBackOdom: {self.LinVel}")

        return self.Pos, self.Ort, self.LinVel

    def callBackImu(self, data):
        self.LinAcc = data.linear_acceleration
        self.YawRate = data.angular_velocity.z
        #print(f"callBackImu: {self.LinAcc}")

        return self.YawRate

    def calc_input(self):
        self.v = self.LinVel # [m/s]
        print(f"calc_input: {self.v}")
        self.yawrate = self.YawRate  # [rad/s]
        self.u = np.array([[self.v], [self.yawrate]])

        return self.u


    def observation(self, xTrue, xd, u):
        xTrue = self.motion_model(xTrue, u)

        # add noise to gps x-y
        self.z = self.observation_model(xTrue) + self.GPS_NOISE @ np.random.randn(2, 1)

        # add noise to input
        self.ud = u + self.INPUT_NOISE @ np.random.randn(2, 1)

        xd = self.motion_model(xd, self.ud)

        return xTrue, self.z, xd, self.ud


    def motion_model(self, x, u):
        F = np.array([[1.0, 0, 0, 0],
                    [0, 1.0, 0, 0],
                    [0, 0, 1.0, 0],
                    [0, 0, 0, 0]])

        B = np.array([[self.DT * math.cos(x[2, 0]), 0],
                    [self.DT * math.sin(x[2, 0]), 0],
                    [0.0, self.DT],
                    [1.0, 0.0]])
        
        x = F @ x + B @ u

        return x


    def observation_model(self, x):
        H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ])

        z = H @ x

        return z


    def jacob_f(self, x, u):
        """
        Jacobian of Motion Model

        motion model
        x_{t+1} = x_t+v*dt*cos(yaw)
        y_{t+1} = y_t+v*dt*sin(yaw)
        yaw_{t+1} = yaw_t+omega*dt
        v_{t+1} = v{t}
        so
        dx/dyaw = -v*dt*sin(yaw)
        dx/dv = dt*cos(yaw)
        dy/dyaw = v*dt*cos(yaw)
        dy/dv = dt*sin(yaw)
        """
        yaw = x[2, 0]
        v = u[0, 0]
        jF = np.array([
            [1.0, 0.0, -self.DT * v * math.sin(yaw), self.DT * math.cos(yaw)],
            [0.0, 1.0, self.DT * v * math.cos(yaw), self.DT * math.sin(yaw)],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]])

        return jF


    def jacob_h(self):
        # Jacobian of Observation Model
        jH = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ])

        return jH


    def ekf_estimation(self, xEst, PEst, z, u):
        #  Predict
        xPred = self.motion_model(xEst, u)
        jF = self.jacob_f(xEst, u)
        PPred = jF @ PEst @ jF.T + self.Q

        #  Update
        jH = self.jacob_h()
        zPred = self.observation_model(xPred)
        y = z - zPred
        S = jH @ PPred @ jH.T + self.R
        K = PPred @ jH.T @ np.linalg.inv(S)
        xEst = xPred + K @ y
        PEst = (np.eye(len(xEst)) - K @ jH) @ PPred
        return xEst, PEst


    def main(self):
        print(__file__ + " start!!")
       
        time = 0.0

        # State Vector [x y yaw v]'
        xEst = np.zeros((4, 1))
        xTrue = np.zeros((4, 1))
        PEst = np.eye(4)

        xDR = np.zeros((4, 1))  # Dead reckoning

        # history
        hxEst = xEst
        hxTrue = xTrue
        hxDR = xTrue
        hz = np.zeros((2, 1))

        while self.SIM_TIME >= time:
            time += self.DT
            self.u = self.calc_input()

            xTrue, z, xDR, ud = self.observation(xTrue, xDR, self.u)

            xEst, PEst = self.ekf_estimation(xEst, PEst, z, ud)

            # store data history
            hxEst = np.hstack((hxEst, xEst))
            hxDR = np.hstack((hxDR, xDR))
            hxTrue = np.hstack((hxTrue, xTrue))
            hz = np.hstack((hz, z))

            if self.show_animation:
                plt.cla()
                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect('key_release_event',
                        lambda event: [exit(0) if event.key == 'escape' else None])
                plt.plot(hz[0, :], hz[1, :], ".g")
                plt.plot(hxTrue[0, :].flatten(),
                        hxTrue[1, :].flatten(), "-b")
                plt.plot(hxDR[0, :].flatten(),
                        hxDR[1, :].flatten(), "-k")
                plt.plot(hxEst[0, :].flatten(),
                        hxEst[1, :].flatten(), "-r")
                plot_covariance_ellipse(xEst[0, 0], xEst[1, 0], PEst)
                plt.axis("equal")
                plt.grid(True)
                plt.pause(0.001)
                print("True:",xTrue[0])
                print("Est",xEst[0] )


if __name__ == '__main__':
    rospy.init_node("ekf_deneme")
    Ekf()
    rospy.spin()
