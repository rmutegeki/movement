#!/usr/bin/env python
'''
Created on 2020. 11. 19.
@author: macroact
'''

class MaicatConfig:

    JOINT_LIMITS = {
        'front_left_pan': (-1.57, 0.8), #(-1.41, 1.41)
        'front_left_shoulder': (-0.628, 0.251), #zero : 0.109
        'front_left_leg': (-1.57, 0.47),

        'front_right_pan': (-0.77, 1.57), #(-1.41, 1.41)
        'front_right_shoulder': (-0.361, 0.549), #zero : -0.078
        'front_right_leg': (-0.157, 1.57),

        'rear_left_pan': (-1.1, 1.57), #(-1.41, 1.41)
        'rear_left_shoulder': (-0.533, 0.314), #zero : 0.0
        'rear_left_bridge': (-0.785, 0.926),
        'rear_left_leg': (-1.1, 0.314),

        'rear_right_pan': (-1.57, 0.706), #(-1.41, 1.41)
        'rear_right_shoulder': (-0.392, 0.518), #zero : 0.0
        'rear_right_bridge': (-0.8, 0.863),
        'rear_right_leg': (-0.628, 1.02),

        'neck_lift': (-1.412, 1.412), #Down ~ Up
        'neck_pan': (-1.57, 1.57), #Right ~ Left
        'head_lift': (-1.412, -0.91), #Up ~ Down

        'tail_lift': (-0.785, 0.785) #Down ~ Up
    }

    JOINT_READY = {
        'front_left_pan': 0.36,
        'front_left_shoulder': 0.0,
        'front_left_leg': -0.785,

        'front_right_pan': -0.26,
        'front_right_shoulder': 0.0,
        'front_right_leg': 0.785,

        'rear_left_pan': -0.58,
        'rear_left_shoulder': 0.0,
        'rear_left_bridge': 0.1,
        'rear_left_leg': -0.1,

        'rear_right_pan': 0.58,
        'rear_right_shoulder': -0.1,
        'rear_right_bridge': -0.1,
        'rear_right_leg': 0.1,

        'neck_lift': 0.0,
        'neck_pan': 0.1,
        'head_lift': -0.3,

        'tail_lift': 0.3
    }

    JOINT_SIT = {
        'front_left_pan': 0.9,
        'front_left_shoulder': 0.15,
        'front_left_leg': -1.41,

        'front_right_pan': -0.9,
        'front_right_shoulder': -0.15,
        'front_right_leg': 1.41,

        'rear_left_pan': -1.41,
        'rear_left_shoulder': -0.3,
        'rear_left_bridge': 0.0,
        'rear_left_leg': 0.0,

        'rear_right_pan': 1.41,
        'rear_right_shoulder': 0.3,
        'rear_right_bridge': 0.0,
        'rear_right_leg': 0.0,

        'neck_lift': -1.412,
        'neck_pan': 0.1,
        'head_lift': -0.785,

        'tail_lift': -0.785
    }

    JOINT_LIE = {
        'front_left_pan': 0.1,
        'front_left_shoulder': 0.15,
        'front_left_leg': -0.1,

        'front_right_pan': -0.1,
        'front_right_shoulder': -0.15,
        'front_right_leg': 0.1,

        'rear_left_pan': -1.57,
        'rear_left_shoulder': 0.0,
        'rear_left_bridge': -0.1,
        'rear_left_leg': 0.0,

        'rear_right_pan': 1.57,
        'rear_right_shoulder': 0.0,
        'rear_right_bridge': 0.1,
        'rear_right_leg': 0.0,

        'neck_lift': 0.0,
        'neck_pan': 0.1,
        'head_lift': 0.785,

        'tail_lift': 0.785
    }

    JOINT_LIE_PRONE = {
        'front_left_pan': 0.1,
        'front_left_shoulder': 0.15,
        'front_left_leg': -0.1,

        'front_right_pan': -0.1,
        'front_right_shoulder': -0.15,
        'front_right_leg': 0.1,

        'rear_left_pan': -1.0,
        'rear_left_shoulder': 0.0,
        'rear_left_bridge': -0.1,
        'rear_left_leg': 0.0,

        'rear_right_pan': 1.0,
        'rear_right_shoulder': 0.0,
        'rear_right_bridge': 0.1,
        'rear_right_leg': 0.0,

        'neck_lift': 0.0,
        'neck_pan': 0.1,
        'head_lift': 0.785,

        'tail_lift': 0.785
    }

    JOINT_NAMES = [
        'front_left_pan', 'front_left_shoulder', 'front_left_leg',
        'front_right_pan', 'front_right_shoulder', 'front_right_leg',
        'rear_left_pan', 'rear_left_shoulder', 'rear_left_bridge', 'rear_left_leg',
        'rear_right_pan', 'rear_right_shoulder', 'rear_right_bridge', 'rear_right_leg',
        'neck_lift', 'neck_pan', 'head_lift', 'tail_lift'
    ]
