# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for the SO ARM100 8J Robotics arms.

The following configuration parameters are available:

* :obj:`SO_ARM100_8J_CFG`: The Lerobot SO ARM100 8J (6-Dof) arm with a 2-finger gripper.

Reference: https://github.com/Kinovarobotics/kinova-ros
"""

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg
import os

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
usd_dir_path = os.path.join(BASE_DIR, "../usd/")

robot_usd = "so_arm100_8j.usd"

##
# Configuration
##

SO_ARM100_8J_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=usd_dir_path + robot_usd,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=True,
            max_depenetration_velocity=5.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True, solver_position_iteration_count=8, solver_velocity_iteration_count=0,
            fix_root_link = True
        ),
        activate_contact_sensors=False,
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        joint_pos={
            "Rotation": 0.1,
            "Pitch": 0.1,
            "Elbow": 0.1,
            "Wrist_Pitch": 0.1,
            "Wrist_Roll": 0.1,
            "Jaw": 0.0,
        },
    ),
    actuators={
        "arm": ImplicitActuatorCfg(
            joint_names_expr=["joint_[1-6]"],
            velocity_limit=1.2,
            effort_limit={
                "joint_[1-4]": 39.0,
                "joint_[5-6]": 9.0,
            },
            stiffness={
                "joint_[1-4]": 300.0,  #100
                "joint_[5-6]": 200.0,   # 15
            },
            damping={
                "joint_[1-4]": 10.0,    # 1.0
                "joint_[5-6]": 5.0,    #0.5
            },
        ),
        "hand": ImplicitActuatorCfg(
            joint_names_expr=["Jaw"],
            effort_limit=150.0,
            velocity_limit=2.24,
            stiffness=10.0,
            damping=10.0,
        ),
        "hand_mimic": ImplicitActuatorCfg(
            joint_names_expr=["left_inner_knuckle_joint","left_inner_finger_joint",
            "right_inner_knuckle_joint","right_inner_finger_joint"],
            effort_limit=150.0,
            velocity_limit=2.24,
            stiffness=0.0,
            damping=5000.0,
        ),
    },
)

"""Configuration of robot with stiffer PD control."""
SO_ARM100_8J_PD_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=usd_dir_path + robot_usd,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=True,
            max_depenetration_velocity=5.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True, solver_position_iteration_count=8, solver_velocity_iteration_count=0,
            fix_root_link = True
        ),
        activate_contact_sensors=False,
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        joint_pos={
            "Rotation": 0.1,
            "Pitch": 0.1,
            "Elbow": 0.1,
            "Wrist_Pitch": 0.1,
            "Wrist_Roll": 0.1,
            "Jaw": 0.0,
        },
    ),
    actuators={
        "arm": ImplicitActuatorCfg(
            joint_names_expr=["joint_[1-6]"],
            velocity_limit=2.0,
            effort_limit={
                "joint_[1-4]": 39.0,
                "joint_[5-6]": 9.0,
            },
            stiffness={
                "joint_[1-4]": 600.0,  #100
                "joint_[5-6]": 450.0,   # 15
            },
            damping={
                "joint_[1-4]": 30.0,    # 1.0
                "joint_[5-6]": 15.0,    #0.5
            },
        ),
        "hand": ImplicitActuatorCfg(
            joint_names_expr=["left_inner_knuckle_joint","left_inner_finger_joint",
            "right_inner_knuckle_joint","right_outer_knuckle_joint","right_inner_finger_joint","finger_joint"],
            effort_limit=150.0,
            velocity_limit=2.24,
            stiffness=5.0,
            damping=5.0,
        ),
    },
)
