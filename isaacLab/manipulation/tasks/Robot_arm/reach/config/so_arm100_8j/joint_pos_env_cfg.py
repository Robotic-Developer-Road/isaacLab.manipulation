# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import math

from isaaclab.utils import configclass

#import isaaclab_tasks.manager_based.manipulation.reach.mdp as mdp
import isaacLab.manipulation.tasks.Robot_arm.reach.mdp as mdp
from isaacLab.manipulation.tasks.Robot_arm.reach.reach_env_cfg import ReachEnvCfg
#from isaaclab_tasks.manager_based.manipulation.reach.reach_env_cfg import ReachEnvCfg

##
# Pre-defined configs
##
from isaacLab.manipulation.assets.config.so_arm100_8j import SO_ARM100_8J_CFG # isort: skip


##
# Environment configuration
##


@configclass
class SOARM100ReachEnvCfg(ReachEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # switch robot to so arm100 8j
        self.scene.robot = SO_ARM100_8J_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        # override events
        self.events.reset_robot_joints.params["position_range"] = (0.75, 1.25)
        # override rewards
        self.rewards.end_effector_position_tracking.params["asset_cfg"].body_names = ["Moving_Jaw"]
        self.rewards.end_effector_position_tracking_fine_grained.params["asset_cfg"].body_names = ["Moving_Jaw"]
        self.rewards.end_effector_orientation_tracking.params["asset_cfg"].body_names = ["Moving_Jaw"]
        # override actions
        self.actions.arm_action = mdp.JointPositionActionCfg(
            asset_name="robot", joint_names=[".*"], scale=0.5, use_default_offset=True
        )
        # override command generator body
        # end-effector is along x-direction
        self.commands.ee_pose.body_name = "Moving_Jaw"
        self.commands.ee_pose.ranges.pitch = (math.pi / 2, math.pi / 2)


@configclass
class SOARM100ReachEnvCfg_PLAY(SOARM100ReachEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()
        # make a smaller scene for play
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        # disable randomization for play
        self.observations.policy.enable_corruption = False
