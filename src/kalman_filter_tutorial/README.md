
[base on the construct course](https://bitbucket.org/theconstructcore/kalman_filters_files/)


```
ros2 service call /gazebo/get_entity_state gazebo_msgs/GetEntityState '{name: my_robot_name, reference_frame: world}'
```

```
ros2 service call /get_model_list gazebo_msgs/srv/GetModelList "{}"
```