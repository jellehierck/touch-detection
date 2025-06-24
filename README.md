# Touch detection experiment

```bash
ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController "{activate_controllers: [linear_velocity_controller], strictness: 2, activate_asap: true, timeout: {sec: 1} }"
```

```bash
ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController "{deactivate_controllers: [linear_velocity_controller], strictness: 2, activate_asap: true, timeout: {sec: 1} }"
```
