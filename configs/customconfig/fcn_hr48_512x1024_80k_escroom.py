"""
Dataset: ESC ROOM (VOC Type)
Method: FCN (hrnet)
Backbone:hr48
Crop Size: 512x1024
Lr Schd: 80000
"""

_base_ = [
    '../_base_/models/fcn_hr18.py', '../_base_/datasets/pascal_escroom.py',
    '../_base_/default_runtime.py', '../_base_/schedules/schedule_80k.py'
]

model = dict(
    pretrained='open-mmlab://msra/hrnetv2_w48',
    backbone=dict(
        extra=dict(
            stage2=dict(num_channels=(48, 96)),
            stage3=dict(num_channels=(48, 96, 192)),
            stage4=dict(num_channels=(48, 96, 192, 384)))),
    decode_head=dict(
        in_channels=[48, 96, 192, 384], channels=sum([48, 96, 192, 384]), num_classes=2))
