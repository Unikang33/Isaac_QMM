#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, Signal, Slot
from python_qt_binding.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, 
                                         QGroupBox, QSlider, QLabel, 
                                         QDoubleSpinBox, QPushButton, QScrollArea)
from rqt_gui_py.plugin import Plugin


class QuadManiController(Plugin):
    """RQT 플러그인으로 QuadMani 로봇(GO1 + K1 Arm)의 조인트를 제어합니다."""
    
    def __init__(self, context):
        super(QuadManiController, self).__init__(context)
        self.setObjectName('QuadManiController')
        
        # ROS2 노드 생성
        self._node = context.node
        
        # Joint 정의 - GO1 (12개) + K1 Arm (8개)
        self.go1_joints = [
            # Front Right
            'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
            # Front Left
            'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
            # Rear Right
            'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint',
            # Rear Left
            'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint',
        ]
        
        self.k1_joints = [
            'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6',
            'joint_gripper_left', 'joint_gripper_right'
        ]
        
        # Joint limits (from URDF)
        self.joint_limits = {
            # GO1 limits
            'FR_hip_joint': (-0.863, 0.863),
            'FR_thigh_joint': (-0.686, 4.501),
            'FR_calf_joint': (-2.818, -0.888),
            'FL_hip_joint': (-0.863, 0.863),
            'FL_thigh_joint': (-0.686, 4.501),
            'FL_calf_joint': (-2.818, -0.888),
            'RR_hip_joint': (-0.863, 0.863),
            'RR_thigh_joint': (-0.686, 4.501),
            'RR_calf_joint': (-2.818, -0.888),
            'RL_hip_joint': (-0.863, 0.863),
            'RL_thigh_joint': (-0.686, 4.501),
            'RL_calf_joint': (-2.818, -0.888),
            # K1 Arm limits
            'joint1': (-2.0, 2.0),
            'joint2': (-1.57, 1.4),
            'joint3': (-1.48, 1.8),
            'joint4': (-2.90, 2.90),
            'joint5': (-1.8, 1.60),
            'joint6': (-3.1, 3.1),
            'joint_gripper_left': (-0.034, 0.0),
            'joint_gripper_right': (-0.034, 0.0),
        }
        
        self.all_joints = self.go1_joints + self.k1_joints
        
        # 현재 조인트 위치 저장
        self.current_positions = {joint: 0.0 for joint in self.all_joints}
        
        # UI 위젯들을 저장할 딕셔너리
        self.sliders = {}
        self.spinboxes = {}
        
        # UI 생성
        self._widget = QWidget()
        self._create_ui()
        
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)
        
        # ROS2 Publishers
        self.joint_cmd_pub = self._node.create_publisher(
            JointState, '/joint_command', 10)
        
        # ROS2 Subscribers
        self.joint_state_sub = self._node.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        
        # 타이머 설정 (UI 업데이트용)
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_ui)
        self.timer.start(100)  # 100ms마다 업데이트
        
        self._node.get_logger().info('QuadMani Controller 플러그인이 시작되었습니다.')
    
    def _create_ui(self):
        """UI 생성"""
        main_layout = QVBoxLayout()
        
        # 스크롤 영역 생성
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll_widget = QWidget()
        scroll_layout = QVBoxLayout()
        
        # GO1 다리 제어 그룹
        go1_group = self._create_joint_group("GO1 Legs", self.go1_joints)
        scroll_layout.addWidget(go1_group)
        
        # K1 Arm 제어 그룹
        k1_group = self._create_joint_group("K1 Arm", self.k1_joints)
        scroll_layout.addWidget(k1_group)
        
        scroll_widget.setLayout(scroll_layout)
        scroll.setWidget(scroll_widget)
        main_layout.addWidget(scroll)
        
        # 제어 버튼들
        button_layout = QHBoxLayout()
        
        self.zero_btn = QPushButton('모든 조인트 0으로')
        self.zero_btn.clicked.connect(self.zero_all_joints)
        button_layout.addWidget(self.zero_btn)
        
        self.home_btn = QPushButton('홈 포지션')
        self.home_btn.clicked.connect(self.go_to_home)
        button_layout.addWidget(self.home_btn)
        
        self.publish_btn = QPushButton('현재 위치 전송')
        self.publish_btn.clicked.connect(self.publish_joint_commands)
        button_layout.addWidget(self.publish_btn)
        
        main_layout.addLayout(button_layout)
        
        self._widget.setLayout(main_layout)
        self._widget.setWindowTitle('QuadMani Controller')
    
    def _create_joint_group(self, group_name, joints):
        """조인트 그룹 생성"""
        group_box = QGroupBox(group_name)
        layout = QVBoxLayout()
        
        for joint_name in joints:
            joint_layout = self._create_joint_control(joint_name)
            layout.addLayout(joint_layout)
        
        group_box.setLayout(layout)
        return group_box
    
    def _create_joint_control(self, joint_name):
        """개별 조인트 제어 위젯 생성"""
        layout = QHBoxLayout()
        
        # 조인트 이름 라벨
        label = QLabel(joint_name)
        label.setMinimumWidth(150)
        layout.addWidget(label)
        
        # 슬라이더
        min_val, max_val = self.joint_limits[joint_name]
        slider = QSlider(Qt.Horizontal)
        slider.setMinimum(int(min_val * 1000))
        slider.setMaximum(int(max_val * 1000))
        slider.setValue(0)
        slider.valueChanged.connect(
            lambda value, jn=joint_name: self.slider_changed(jn, value))
        self.sliders[joint_name] = slider
        layout.addWidget(slider)
        
        # 스핀박스
        spinbox = QDoubleSpinBox()
        spinbox.setMinimum(min_val)
        spinbox.setMaximum(max_val)
        spinbox.setValue(0.0)
        spinbox.setSingleStep(0.01)
        spinbox.setDecimals(3)
        spinbox.valueChanged.connect(
            lambda value, jn=joint_name: self.spinbox_changed(jn, value))
        self.spinboxes[joint_name] = spinbox
        layout.addWidget(spinbox)
        
        # 현재 값 라벨
        current_label = QLabel('0.000')
        current_label.setMinimumWidth(60)
        layout.addWidget(current_label)
        
        return layout
    
    @Slot(str, int)
    def slider_changed(self, joint_name, value):
        """슬라이더 값이 변경되었을 때"""
        real_value = value / 1000.0
        self.current_positions[joint_name] = real_value
        
        # 스핀박스 업데이트 (신호 차단)
        spinbox = self.spinboxes[joint_name]
        spinbox.blockSignals(True)
        spinbox.setValue(real_value)
        spinbox.blockSignals(False)
    
    @Slot(str, float)
    def spinbox_changed(self, joint_name, value):
        """스핀박스 값이 변경되었을 때"""
        self.current_positions[joint_name] = value
        
        # 슬라이더 업데이트 (신호 차단)
        slider = self.sliders[joint_name]
        slider.blockSignals(True)
        slider.setValue(int(value * 1000))
        slider.blockSignals(False)
    
    def joint_state_callback(self, msg):
        """조인트 상태 콜백"""
        for i, name in enumerate(msg.name):
            if name in self.all_joints and i < len(msg.position):
                # 피드백 값으로 UI 업데이트는 선택적으로
                pass
    
    def update_ui(self):
        """UI 주기적 업데이트"""
        pass
    
    @Slot()
    def zero_all_joints(self):
        """모든 조인트를 0으로 설정"""
        for joint_name in self.all_joints:
            min_val, max_val = self.joint_limits[joint_name]
            if min_val <= 0.0 <= max_val:
                self.current_positions[joint_name] = 0.0
                self.sliders[joint_name].setValue(0)
                self.spinboxes[joint_name].setValue(0.0)
        
        self._node.get_logger().info('모든 조인트를 0으로 설정했습니다.')
    
    @Slot()
    def go_to_home(self):
        """홈 포지션으로 이동"""
        # GO1 홈 포지션 (서있는 자세)
        home_positions = {
            'FR_hip_joint': 0.0, 'FR_thigh_joint': 0.9, 'FR_calf_joint': -1.8,
            'FL_hip_joint': 0.0, 'FL_thigh_joint': 0.9, 'FL_calf_joint': -1.8,
            'RR_hip_joint': 0.0, 'RR_thigh_joint': 0.9, 'RR_calf_joint': -1.8,
            'RL_hip_joint': 0.0, 'RL_thigh_joint': 0.9, 'RL_calf_joint': -1.8,
            # K1 Arm 홈 포지션 (수직)
            'joint1': 0.0, 'joint2': 0.0, 'joint3': 0.0,
            'joint4': 0.0, 'joint5': 0.0, 'joint6': 0.0,
            'joint_gripper_left': 0.0, 'joint_gripper_right': 0.0,
        }
        
        for joint_name, position in home_positions.items():
            min_val, max_val = self.joint_limits[joint_name]
            if min_val <= position <= max_val:
                self.current_positions[joint_name] = position
                self.sliders[joint_name].setValue(int(position * 1000))
                self.spinboxes[joint_name].setValue(position)
        
        self._node.get_logger().info('홈 포지션으로 이동합니다.')
    
    @Slot()
    def publish_joint_commands(self):
        """현재 조인트 위치를 퍼블리시"""
        msg = JointState()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.name = self.all_joints
        msg.position = [self.current_positions[joint] for joint in self.all_joints]
        msg.velocity = []
        msg.effort = []
        
        self.joint_cmd_pub.publish(msg)
        self._node.get_logger().info(
            f'조인트 명령을 전송했습니다: {len(msg.name)}개 조인트')
    
    def shutdown_plugin(self):
        """플러그인 종료"""
        self.timer.stop()
        self._node.destroy_subscription(self.joint_state_sub)
        self._node.destroy_publisher(self.joint_cmd_pub)
    
    def save_settings(self, plugin_settings, instance_settings):
        """설정 저장"""
        pass
    
    def restore_settings(self, plugin_settings, instance_settings):
        """설정 복원"""
        pass

