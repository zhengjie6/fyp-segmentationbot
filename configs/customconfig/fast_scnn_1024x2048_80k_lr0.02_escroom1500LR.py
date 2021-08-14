_base_ = [
    '../_base_/models/fast_scnn.py', '../_base_/datasets/pascal_escroom_1500.py',
    '../_base_/default_runtime.py', '../_base_/schedules/schedule_80k.py'
]

# Re-config the data sampler.
data = dict(samples_per_gpu=4, workers_per_gpu=4)

# Re-config the optimizer.
optimizer = dict(type='SGD', lr=0.005, momentum=0.9, weight_decay=4e-5)

optimizer=dict(
    paramwise_cfg = dict(
        custom_keys={
            'head': dict(lr_mult=10.)}))