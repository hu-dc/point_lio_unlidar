#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
把 Go2 机身 IMU（/lowstate，unitree_go/LowState，500 Hz）转成标准
sensor_msgs/Imu 发布到 /go2/imu，供 point_lio_unilidar 使用。

为什么要做这一步（实测结论，见 README §8.4）：
  /utlidar/imu 是 L1 雷达模组内部的低成本 IMU（250 Hz）。机器人开始走动以后，
  它的噪声/零偏漂移会让 Point-LIO 的传播很快失准，典型表现是"高度一路往下掉、
  位置越跑越远"（四个 walk_308 bag 全部在 t+27 s 开始发散）。换成机身 IMU
  （LowState.imu_state，500 Hz）后同一段 bag 不再自由落体。

时间戳是怎么定的（关键，v4）：
  LowState 没有 header/stamp，但它有一个 **毫秒级 MCU tick（uint32，每 2 ms
  加 2，实测稳定）**。tick 是机器人内部时钟，与雷达时间轴（/utlidar/cloud、
  /utlidar/imu 的 header.stamp）只差一个随时间缓慢变化的小量 C：
        stamp = tick * 0.001 + C

  C 的在线估计：每收到一帧 /utlidar/imu，就把它的 header.stamp 与"当前最新的
  LowState tick"配成一对，得到候选 C_i。这里有两个坑（都是实测出来的）：
    1. 不能用"收到时刻"打时间戳：rosbag 回放慢于实时 / DDS 成批到达时，
       到达时间会来回跳几百毫秒到两秒。
    2. 也不能用候选的中位数：DDS 成批到达会让"最新 tick"滞后，而滞后只会让
       C_i 偏大（单向偏差）。实测 3 s 窗口中位数抖动 std≈340 ms，10 s 窗口
       仍有 190 ms，只有 30 s 窗口才稳（0.1 ms）。
       反过来取**窗口内低分位数（p01/min）**很稳：10 s 窗口 std≈4.7 ms、
       range≈12 ms —— 因为"最小延迟"那批样本才最接近真值。
  所以本节点对最近 window_sec(默认 10 s) 的候选取 p01 作为目标 C，
  再做限速平滑（≤0.2 ms/次，10 Hz 更新）并锁在标定值 ±10 ms 之内：
  C 物理上只随晶振慢漂；当回放速度跟不上、/lowstate 入队越来越慢时，
  所有候选 C_i 会一起被抬高，不限幅的话时间轴会整体漂几百毫秒（bag3 实测）。
  输出 stamp 严格单调递增（Point-LIO 收到回退的时间戳会打印
  "imu loop back, clear deque" 并丢样本）。

  C 标定完成之前（默认 2 s、≥200 对样本）的 LowState 先缓存，标定完成后
  按顺序补发，不丢样本。

用法：
  ros2 run point_lio_unilidar lowstate_to_imu
  # 或者由 launch/go2_body_imu.launch.py、go2_bag_replay_body_imu.launch.py 拉起

参数：
  input_topic    默认 /lowstate     机身 IMU 来源（unitree_go/LowState）
  sync_topic     默认 /utlidar/imu  雷达时钟参考（与雷达同源 header.stamp）
  out_topic      默认 /go2/imu      输出话题（需与 yaml 里 common.imu_topic 一致）
  frame_id       默认 body          输出 Imu 的 frame_id
  window_sec     默认 10.0          计算 C 的滑动窗口长度（秒）
  calib_sec      默认 2.0           标定开始前至少积累多少秒的配对样本
  calib_samples  默认 200           标定完成所需的最少配对数
"""

import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu

from unitree_go.msg import LowState

TICK_WRAP = 2 ** 32
TICK_HALF = 2 ** 31


class LowStateToImu(Node):
    def __init__(self):
        super().__init__('lowstate_to_imu')
        self.declare_parameter('input_topic', '/lowstate')
        self.declare_parameter('sync_topic', '/utlidar/imu')
        self.declare_parameter('out_topic', '/go2/imu')
        self.declare_parameter('frame_id', 'body')
        self.declare_parameter('window_sec', 10.0)
        self.declare_parameter('calib_sec', 2.0)
        self.declare_parameter('calib_samples', 200)

        input_topic = self.get_parameter('input_topic').value
        sync_topic = self.get_parameter('sync_topic').value
        out_topic = self.get_parameter('out_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.window_sec = float(self.get_parameter('window_sec').value)
        self.calib_sec = float(self.get_parameter('calib_sec').value)
        self.calib_samples = int(self.get_parameter('calib_samples').value)

        sub_qos = QoSProfile(depth=2000, reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST)
        # 发布端必须 RELIABLE：Point-LIO 的 IMU 订阅是 RELIABLE/KEEP_LAST(200000)
        pub_qos = QoSProfile(depth=2000, reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST)

        self.pub = self.create_publisher(Imu, out_topic, pub_qos)
        self.create_subscription(LowState, input_topic, self.on_lowstate, sub_qos)
        self.create_subscription(Imu, sync_topic, self.on_sync, sub_qos)
        self.create_timer(0.1, self.refresh_offset)
        self.create_timer(5.0, self.report)

        self.last_tick = None
        self.pending = deque()
        self.candidates = deque()      # (wall_time, C_i)
        self.first_pair_wall = None
        self.c_offset = None
        self.c_calib = None
        self.last_stamp = None
        self.n_clamped = 0
        self.n_lowstate = 0
        self.n_sync = 0
        self.n_out = 0
        self.n_tick_skip = 0
        self.n_hold = 0

        self.get_logger().info(
            f'lowstate_to_imu: {input_topic} -> {out_topic}，时间基准取自 '
            f'{sync_topic}（{self.window_sec:.1f}s 窗口 p01 估计 + 限速平滑）')

    # ------------------------------------------------------------- callbacks
    def on_sync(self, m: Imu):
        """记录一对 (当前最新 LowState tick, 雷达时间戳) → 候选偏移 C"""
        if self.last_tick is None:
            return
        self.n_sync += 1
        s = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        wall = self.now_sec()
        if self.first_pair_wall is None:
            self.first_pair_wall = wall
        self.candidates.append((wall, s - self.last_tick * 0.001))

    def on_lowstate(self, m: LowState):
        self.n_lowstate += 1
        tick = int(m.tick)
        if self.last_tick is not None:
            if tick < self.last_tick:
                if self.last_tick - tick > TICK_HALF:
                    self.get_logger().warn('LowState tick 回绕，重置时间轴')
                    self.c_offset = None
                    self.c_calib = None
                    self.candidates.clear()
                    self.first_pair_wall = None
                    self.last_stamp = None
                else:
                    self.n_tick_skip += 1
                    return      # 乱序，丢弃
        self.last_tick = tick

        if self.c_offset is None:
            if len(self.pending) < 20000:     # 最多缓存 40 s，足够标定
                self.pending.append(m)
            return
        self.publish_one(m)

    # --------------------------------------------------------------- helpers
    def now_sec(self):
        return time.monotonic()

    def calibrate(self, now):
        """用低分位数（p01）估计 C —— DDS 迟滞只让 C_i 偏大，p01 最接近真值"""
        while self.candidates and now - self.candidates[0][0] > self.window_sec:
            self.candidates.popleft()
        vals = sorted(c for _, c in self.candidates)
        n = len(vals)
        if n < self.calib_samples:
            return None
        if n < 20:
            return vals[0]
        k = max(1, int(round(0.01 * n)))
        return 0.5 * (vals[k - 1] + vals[k])

    def refresh_offset(self):
        """10 Hz：重新估计 C 并做限速平滑（避免时间轴抖动/回退）"""
        if not self.candidates:
            return
        now = self.now_sec()
        target = self.calibrate(now)

        if self.c_offset is None:
            span = now - self.first_pair_wall if self.first_pair_wall else 0.0
            if target is None or span < self.calib_sec:
                return
            self.c_offset = target
            self.c_calib = target
            self.get_logger().info(
                f'时钟标定完成: C={self.c_offset:.6f} s '
                f'({len(self.candidates)} 对样本, 跨度 {span:.2f}s)，'
                f'补发缓存 {len(self.pending)} 帧')
            for m in self.pending:
                self.publish_one(m)
            self.pending.clear()
            return

        if target is None:
            return
        # C 是"机器人 MCU 时钟 ↔ 雷达时钟"的差，物理上只会随晶振慢漂（~百 ppm），
        # 绝不可能在几十秒内变化几百毫秒。实测（bag3）当回放速度跟不上时，
        # /lowstate 数据流入队会越来越慢，所有候选 C_i 一起被抬高，估计器会以
        # 限速一路"爬升"——那样时间轴就整体偏了，轨迹随即发散。
        # 因此把 C 锁在标定值附近：只允许 ±10 ms 的慢速修正（≤0.2 ms/s）。
        if self.c_calib is not None and abs(target - self.c_calib) > 0.010:
            self.n_clamped += 1
            return
        err = target - self.c_offset
        if err > 0.0002:                 # 向上最多 0.2 ms/次（2 ms/s）
            self.c_offset += 0.0002
        elif err < -0.0002:              # 向下最多 0.2 ms/次
            self.c_offset -= 0.0002
        else:
            self.c_offset = target

    def publish_one(self, m: LowState):
        stamp = int(m.tick) * 0.001 + self.c_offset
        if self.last_stamp is not None and stamp <= self.last_stamp:
            # 时间轴不允许回退：宁可让这一帧贴着上一帧（最多几十微秒的挤压）
            self.n_hold += 1
            stamp = self.last_stamp + 0.0002
        self.last_stamp = stamp

        sec = int(stamp)
        out = Imu()
        out.header.stamp.sec = sec
        out.header.stamp.nanosec = int((stamp - sec) * 1e9)
        out.header.frame_id = self.frame_id

        imu = m.imu_state
        out.angular_velocity.x = float(imu.gyroscope[0])
        out.angular_velocity.y = float(imu.gyroscope[1])
        out.angular_velocity.z = float(imu.gyroscope[2])
        out.linear_acceleration.x = float(imu.accelerometer[0])
        out.linear_acceleration.y = float(imu.accelerometer[1])
        out.linear_acceleration.z = float(imu.accelerometer[2])
        # unitree 的四元数顺序是 [w, x, y, z]，与 ROS 一致
        out.orientation.w = float(imu.quaternion[0])
        out.orientation.x = float(imu.quaternion[1])
        out.orientation.y = float(imu.quaternion[2])
        out.orientation.z = float(imu.quaternion[3])
        # rpy 协方差置零（Point-LIO 不使用 orientation）
        for i in range(9):
            out.orientation_covariance[i] = 0.0
            out.angular_velocity_covariance[i] = 0.0
            out.linear_acceleration_covariance[i] = 0.0

        self.pub.publish(out)
        self.n_out += 1

    def report(self):
        c = 'None' if self.c_offset is None else f'{self.c_offset:.6f}'
        cand = len(self.candidates)
        win = 'n/a'
        if cand > 0:
            span = self.now_sec() - self.candidates[0][0]
            win = f'{span:.1f}s'
        self.get_logger().info(
            f'lowstate={self.n_lowstate} sync={self.n_sync} out={self.n_out} '
            f'hold={self.n_hold} 乱序丢弃={self.n_tick_skip} 越界丢弃={self.n_clamped} C={c} '
            f'候选窗口={win}/{cand}')


def main(args=None):
    rclpy.init(args=args)
    node = LowStateToImu()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
