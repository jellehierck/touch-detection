# Touch detection experiment

## Build

```bash
docker compose build
```

## Start launch file

```bash
docker compose run --rm --remove-orphans touch_detection_robot ros2 launch touch_detection_bringup bringup.launch.py robot_ip:=172.16.0.2 load_gripper:=false use_rviz:=false
```

## Activate fwd controller

```bash
docker compose run --rm --remove-orphans touch_detection_robot ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController "{activate_controllers: [fwd_linear_velocity_controller], deactivate_controllers: [bwd_linear_velocity_controller], strictness: 1, activate_asap: true, timeout: {sec: 1} }"
```

## Activate bwd controller

```bash
docker compose run --rm --remove-orphans touch_detection_robot ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController "{activate_controllers: [bwd_linear_velocity_controller], deactivate_controllers: [fwd_linear_velocity_controller], strictness: 1, activate_asap: true, timeout: {sec: 1} }"
```

## Start launch file without plotjuggler

```bash
docker compose run --rm --remove-orphans touch_detection_robot ros2 launch touch_detection_bringup bringup.launch.py robot_ip:=172.16.0.2 load_gripper:=false use_rviz:=false use_plotjuggler:=false
```

## Only plotjuggler

```bash
docker compose run --rm --remove-orphans touch_detection_robot ros2 launch touch_detection_bringup plotjuggler.launch.py
```

## Poses

```yaml
ee_pose_contact:
  position:
    x: 0.5225977022306527
    y: -0.05650916132908832
    z: 0.18889776892312335
  orientation:
    x: 0.0003485597591467567
    y: 0.7073287633243067
    z: 0.7068846306152924
    w: 0.00013446219851332073

joint_positions_contact:
- 0.3444753550985544
- 0.8261501144614802
- -0.8445879539940282
- -1.9736597304670214
- 1.5864327898543251
- 0.92192898265104
- 2.0273283264260917

---

ee_pose_start:
  position:
    x: 0.5227362166734849
    y: -0.1997278083478338
    z: 0.18877797389567766
  orientation:
    x: -0.00012436684465579571
    y: 0.7070618075176488
    z: 0.7071516253538871
    w: -0.00040452703111627705

joint_positions_start:
- 0.023986806770280953
- 0.9940544733406931
- -0.8440979213284878
- -1.663454944128066
- 1.4983364365146772
- 0.7414966371331451
- 2.0321190305310366
```