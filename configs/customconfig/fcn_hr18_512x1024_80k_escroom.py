"""
Dataset: ESC ROOM (VOC Type)
Method: FCN (hrnet)
Backbone:hr18
Crop Size: 512x1024
Lr Schd: 80000
"""

_base_ = [
    '../_base_/models/fcn_hr18.py', '../_base_/datasets/pascal_escroom.py',
    '../_base_/default_runtime.py', '../_base_/schedules/schedule_80k.py'
]

model = dict(decode_head=dict(num_classes=2))