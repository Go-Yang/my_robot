# models/

本地 Gazebo 模型目录，`sim.launch.py` 会把它加入 `GZ_SIM_RESOURCE_PATH`。

目前机器人直接复用系统安装的 TurtleBot3 模型（`turtlebot3_gazebo/models`），
这里预留给后续自定义模型，例如：

- 自定义机器人底盘 / 传感器布局
- 室内家具、门、充电桩等场景模型

新增模型时保持标准 Gazebo 目录结构：

```
models/<model_name>/
├── model.config
└── model.sdf
```

然后在 world 里用 `model://<model_name>` 引用即可。
