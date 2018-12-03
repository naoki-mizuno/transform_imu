from tf2_geometry_msgs import *
from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import QuaternionStamped
from sensor_msgs.msg import Imu

import copy


def do_transform_quaternion(quaternion_msg, transform):
    # Use PoseStamped to transform the Quaternion
    src_ps = PoseStamped()
    src_ps.header = quaternion_msg.header
    src_ps.pose.orientation = quaternion_msg.quaternion
    tgt_ps = do_transform_pose(src_ps, transform)
    tgt_o = QuaternionStamped()
    tgt_o.header = tgt_ps.header
    tgt_o.quaternion = tgt_ps.pose.orientation
    return tgt_o


def do_transform_imu(imu_msg, transform):
    """
    Transforms the given Imu message into the given target frame
    :param imu_msg:
    :type imu_msg: Imu
    :param transform:
    :type transform: TransformStamped
    :return: Imu message transformed into the target frame
    :rtype: Imu
    """
    src_l = Vector3Stamped()
    src_l.header = imu_msg.header
    src_l.vector = imu_msg.linear_acceleration
    src_a = Vector3Stamped()
    src_a.header = imu_msg.header
    src_a.vector = imu_msg.angular_velocity
    src_o = QuaternionStamped()
    src_o.header = imu_msg.header
    src_o.quaternion = imu_msg.orientation

    tgt_l = do_transform_vector3(src_l, transform)
    tgt_a = do_transform_vector3(src_a, transform)
    tgt_o = do_transform_quaternion(src_o, transform)

    tgt_imu = copy.deepcopy(imu_msg)
    tgt_imu.header.frame_id = transform.child_frame_id
    tgt_imu.linear_acceleration = tgt_l
    tgt_imu.angular_velocity = tgt_a
    tgt_imu.orientation = tgt_o
    return tgt_imu


tf2_ros.TransformRegistration().add(Imu, do_transform_quaternion)
tf2_ros.TransformRegistration().add(Imu, do_transform_imu)
