#!/usr/bin/env python3
"""Feed FAST-LIO a synthetic but physically consistent scene and check that it
actually produces odometry -- i.e. the migrated ESKF/ikd-Tree core runs, not
just links. Gravity on +z, a static textured box cloud, IMU at 200 Hz,
lidar at 10 Hz."""
import math, os, struct, sys, time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, PointCloud2, PointField
# ROS1 nested PointCloud2::Field became a standalone message in ROS2
from nav_msgs.msg import Odometry
from builtin_interfaces.msg import Time as RosTime

G = 9.80665


def make_cloud(n=1200, seed=7):
    rng = np.random.default_rng(seed)
    pts = []
    # a room: floor + 4 walls, so the map has structure from every heading
    for _ in range(n):
        f = rng.integers(0, 5)
        x, y, z = rng.uniform(-8, 8, 3)
        if f == 0:
            z = -0.5
        elif f == 1:
            x = 8.0
        elif f == 2:
            x = -8.0
        elif f == 3:
            y = 8.0
        else:
            y = -8.0
        # exactly 4 floats: point_step is 16 bytes, and PCL rejects the message
        # when width*height*point_step != len(data). A 5th value here silently
        # produced an empty cloud on the C++ side.
        pts.append((x, y, z, rng.uniform(0.1, 0.9)))
    a = np.array(pts, dtype=np.float32)
    msg = PointCloud2()
    # unorganised cloud: height=1, width=N. count is the number of ELEMENTS in
    # the field (1 float), not its byte size -- with count=4 PCL reads every
    # field as a 16-byte run, nothing fits in point_step=16, and the cloud
    # comes back empty. Caught by the node logging "Too few input point cloud".
    msg.height, msg.width = 1, a.shape[0]
    msg.is_bigendian = False
    msg.is_dense = True
    F = 7  # PointField.FLOAT32
    msg.fields = [
        PointField(name="x", offset=0, datatype=F, count=1),
        PointField(name="y", offset=4, datatype=F, count=1),
        PointField(name="z", offset=8, datatype=F, count=1),
        PointField(name="intensity", offset=12, datatype=F, count=1)]
    msg.point_step = 16
    msg.row_step = 16 * a.shape[0]
    msg.data = a.tobytes()
    return msg


class Feeder(Node):
    def __init__(self):
        super().__init__("t4_feeder")
        self.pub_i = self.create_publisher(Imu, "/imu", 10)
        self.pub_l = self.create_publisher(PointCloud2, "/lidar_points", 10)
        self.got = []
        self.clouds = []
        self.create_subscription(Odometry, "/Odometry", self.cb, 10)
        # /cloud_registered is the topic the migrated node used to create but
        # never publish (see fastlio_verify.md 7.3); counting it here makes
        # the regression assertable from the same process that drives the data.
        self.create_subscription(PointCloud2, "/cloud_registered", self.cb_cloud, 10)

    def cb(self, m):
        p = m.pose.pose.position
        self.got.append((p.x, p.y, p.z))

    def cb_cloud(self, m):
        self.clouds.append(m.width * m.height)


def stamp(sec):
    s = int(sec)
    return RosTime(sec=s, nanosec=int((sec - s) * 1e9))


rclpy.init()
nd = Feeder()
cloud = make_cloud()
# FEED_HOLD=1 keeps publishing for the full window instead of breaking on the
# 5th odom message -- needed for rviz2 screenshots, where the point cloud has to
# accumulate for tens of seconds. The default (unset) keeps the fast functional
# check used by build_and_verify.sh.
HOLD = os.environ.get("FEED_HOLD", "") == "1"
WINDOW = float(os.environ.get("FEED_SECONDS", "40.0"))
t0 = time.time()
scan = 0
last_pub = 0.0
while rclpy.ok() and time.time() - t0 < WINDOW:
    now = time.time() - t0
    for _ in range(4):
        m = Imu()
        m.header.frame_id = "imu"
        m.header.stamp = stamp(now)
        m.angular_velocity.x = m.angular_velocity.y = m.angular_velocity.z = 0.0
        m.linear_acceleration.x = 0.0
        m.linear_acceleration.y = 0.0
        m.linear_acceleration.z = G
        nd.pub_i.publish(m)
    if now - last_pub >= 0.1:
        last_pub = now
        scan += 1
        cloud.header.frame_id = "lidar"
        cloud.header.stamp = stamp(now)
        nd.pub_l.publish(cloud)
    rclpy.spin_once(nd, timeout_sec=0.002)
    if len(nd.got) >= 5 and not HOLD:
        break

print("SCANS_SENT", scan)
print("ODOM_MSGS", len(nd.got))
print("CLOUD_MSGS", len(nd.clouds))
print("CLOUD_MAX_PTS", max(nd.clouds) if nd.clouds else 0)
for g in nd.got[:5]:
    print("  odom pos = (%.4f, %.4f, %.4f)" % g)
ok = (len(nd.got) >= 1 and len(nd.clouds) >= 1
      and max(nd.clouds) > 100
      and all(math.isfinite(v) for t in nd.got for v in t))
print("VERDICT", "PASS" if ok else "FAIL")
nd.destroy_node()
rclpy.shutdown()
