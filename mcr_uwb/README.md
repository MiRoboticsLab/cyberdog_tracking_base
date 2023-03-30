#mcr_uwb

UWB原始数据协议如下： 
```
topic name: uwb_raw
topic type: protocol/msg/UwbRaw.msg
```

UwbRaw:
```
std_msgs/Header header

float32 dist  //m
float32 angle  //rad
float32 n_los
float32 rssi_1
float32 rssi_2
```

UwbRaw原始数据转换为PoseStamped:
```
  geometry_msgs::msg::PoseStamped pose;
  pose.header = uwb->header;

  RCLCPP_DEBUG(
    get_logger(), "received uwb raw data, frame id: %s, dist: %f, angle: %f.",
    uwb->header.frame_id.c_str(), uwb->dist, uwb->angle);

  pose.pose.position.x = uwb->dist * cos(-uwb->angle);
  pose.pose.position.y = uwb->dist * sin(-uwb->angle);

```

数据转换完成之后，在后续数据使用上，根据位置关系利用tf transformer进行调整。
