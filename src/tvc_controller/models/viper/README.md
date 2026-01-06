# Viper Model (Gazebo)

This folder contains the `viper` Gazebo model and its SDF definition (`model.sdf`).

## TVC Plugin SDF Parameters

The model includes a plugin block that loads the TVC simulation plugin (`libTVCPlugin.so`). The plugin accepts the following SDF elements (defaults shown):

- `<roll_joint>`: name of the roll joint (string). Default: `servo0_roll_joint`.
- `<pitch_joint>`: name of the pitch joint (string). Default: `servo1_pitch_joint`.
- `<engine_link>`: link name to apply thrust/attach propellers (string). Default: `motor_housing`.
- `<topic>`: Gazebo transport topic for servo length commands (string). Default: `/tvc_servo_command`.
- `<servo_max_speed>`: maximum linear speed of the servo (m/s). Default: `0.112`.
- `<servo_time_constant>`: first-order time constant (seconds) for servo dynamics (tau). If `<= 0` the plugin falls back to a rate-limiter. Default: `0.10`.
- `<servo_min_len>`: minimum physical servo length (m). Default: `0.085`.
- `<servo_max_len>`: maximum physical servo length (m). Default: `0.115`.

Notes:
- The plugin maps two commanded servo lengths into roll/pitch joint positions using a simplified kinematic model. Geometry (anchor/clevis points) is currently hard-coded in the plugin source.
- If you change the geometry in `model.sdf`, update the plugin constants or (preferably) extend the plugin to read link poses and compute lengths dynamically.

## Example plugin block (from `model.sdf`)

```xml
<plugin filename="libTVCPlugin.so" name="TVCPlugin">
  <roll_joint>servo0_roll_joint</roll_joint>
  <pitch_joint>servo1_pitch_joint</pitch_joint>
  <engine_link>motor_housing</engine_link>
  <servo_max_speed>0.122</servo_max_speed>
  <servo_time_constant>0.10</servo_time_constant>
  <servo_min_len>0.085</servo_min_len>
  <servo_max_len>0.115</servo_max_len>
  <topic>/tvc_servo_command</topic>
</plugin>
```

If you need the plugin to use different limits or dynamics, change the SDF values or update the plugin source in `src/tvc_controller/assets/TVCPlugin2.cc`.

---
Last updated: 2026-01-06
