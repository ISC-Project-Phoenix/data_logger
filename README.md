# data_logger
From package '[data_logger](https://github.com/ISC-Project-Phoenix/data_logger)'
# File
`./src/data_logger.cpp`

## Summary 
 Project Phoenix training data collection node. This pairs images with various car statuses and writes these to a CSV
for use as labels during NN training, then writes those images to disk as compressed jpegs.

## Topics

### Publishes
- `/run_folder`:  Path to the folder we are currently writing images to. That is `data_path`/something. Qos: This uses reliable transport and transient local durability

### Subscribes
- `/odom_ack`: AckermannDrive messages representing the current state of the kart. See design docs for info on format.
- `/camera/mid/rgb`: Camera data, to be synced with training data

## Params
- `data_path`: Directory to write data into. Note that log data will be in `data_path`/"%Y-%m-%d-%H-%M-%S"/ not in
this folder directly. Default: ./training_data
- `max_throttle_speed`: Velocity at which we consider the throttle fully pressed.
- `max_braking_speed`: Negative velocity at which we consider the brake fully pressed.
- `max_steering_rad`:  Max steering wheel angle, in radians. Assumed to be symmetrical.

# Misc 
 see [the design for more info](https://github.com/ISC-Project-Phoenix/design/blob/main/software/ros/data_logger.md) 
