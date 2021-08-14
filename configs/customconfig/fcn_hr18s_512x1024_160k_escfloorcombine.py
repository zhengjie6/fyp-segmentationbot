"""
Dataset: ESC Floor Combine (VOC Type)
Method: FCN (hrnet)
Backbone:hr18s (We load the hr18 config and make modifications to it)- s stands for small
Crop Size: 512x1024
Lr Schd: 160000
"""

_base_ = [
    '../_base_/models/fcn_hr18.py', '../_base_/datasets/pascal_escfloorcombine.py',
    '../_base_/default_runtime.py', '../_base_/schedules/schedule_160k.py'
]

model = dict(
    pretrained='open-mmlab://msra/hrnetv2_w18_small',
    backbone=dict(
        extra=dict(
            stage1=dict(num_blocks=(2, )),
            stage2=dict(num_blocks=(2, 2)),
            stage3=dict(num_modules=3, num_blocks=(2, 2, 2)),
            stage4=dict(num_modules=2, num_blocks=(2, 2, 2, 2)))),
    decode_head=dict(num_classes=3))

# runtime settings
checkpoint_config = dict(by_epoch=False, interval=8000)
evaluation = dict(interval=8000, metric='mIoU')