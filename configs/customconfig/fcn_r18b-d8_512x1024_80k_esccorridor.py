"""
Dataset: ESC CORRIDOR (VOC Type)
Method: FCN
Backbone:R-18b-D8 (We load the R50-d8 config and make modifications to it) - 50 and 18 determines the depth
Crop Size: 512x1024
Lr Schd: 80000
"""

_base_ = [
    '../_base_/models/fcn_r50-d8.py', '../_base_/datasets/pascal_esccorridor.py',
    '../_base_/default_runtime.py', '../_base_/schedules/schedule_80k.py'
]

model = dict(
    pretrained='open-mmlab://resnet18_v1c',
    backbone=dict(type='ResNet', depth=18),
    decode_head=dict(
        in_channels=512,
        channels=128,
        num_classes=2,
    ),
    auxiliary_head=dict(in_channels=256, channels=64, num_classes=2))
