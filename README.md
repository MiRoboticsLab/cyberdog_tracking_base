# cyberdog tracking base

主要存放了基于navigation2实现的docking， navigation， tracking功能相关的参数，附加模块等。

## mcr_bringup

存放docking，navigation， tracking三个功能所用到的参数，行为树，launch等，详细内容可参考：[mcr_bringup](mcr_bringup/README.md)

## mcr_voice

订阅tracking功能执行任务时的feedback信息，并根据feedback进行语音播报，详细内容可参考：[mcr_voice](mcr_voice/README.md)

## mcr_uwb

订阅uwb(Ultra Wide Band)驱动数据，并转换为geometry_msgs::PoseStamped数据，详细内容可参考：[mcr_uwb](mcr_uwb/README.md)

## mcr_msgs

定义了跟随功能相关的接口，详细可参考：[mcr_msgs](mcr_msgs/README.md)

## bt_navigators

修改自nav2_bt_navigators，做了功能拆分，目的是为了缓解功能启动时加载时间过长的问题，详细可参考：[bt_navigators](bt_navigators/README.md)
