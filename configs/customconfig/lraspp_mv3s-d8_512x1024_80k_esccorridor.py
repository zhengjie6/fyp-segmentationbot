"""
Dataset: ESC CORRIDOR (VOC Type)
Method: MobilenetV3 (lraspp)
Backbone:M-V3s-D8 (We load the M-V3-D8 config and make modifications to it)- s stands for small
Crop Size: 512x1024
Lr Schd: 160000
"""

_base_ = [
    '../_base_/models/lraspp_m-v3-d8.py', '../_base_/datasets/pascal_esccorridor.py',
    '../_base_/default_runtime.py', '../_base_/schedules/schedule_80k.py'
]

norm_cfg = dict(type='BN', eps=0.001, requires_grad=True)
model = dict(
    type='EncoderDecoder',
    pretrained='open-mmlab://contrib/mobilenet_v3_small',
    backbone=dict(
        type='MobileNetV3',
        arch='small',
        out_indices=(0, 1, 12),
        norm_cfg=norm_cfg),
    decode_head=dict(
        type='LRASPPHead',
        in_channels=(16, 16, 576),
        in_index=(0, 1, 2),
        channels=128,
        input_transform='multiple_select',
        dropout_ratio=0.1,
        num_classes=19,
        norm_cfg=norm_cfg,
        act_cfg=dict(type='ReLU'),
        align_corners=False,
        loss_decode=dict(
            type='CrossEntropyLoss', use_sigmoid=False, loss_weight=1.0)))

# Re-config the data sampler.
data = dict(samples_per_gpu=4, workers_per_gpu=4)