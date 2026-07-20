# uav_navigation

> EGO 导航黑盒：点云处理 + Ego-Planner 规划 + 轨迹跟踪控制。
> 支持**正常模式**（ego 自己避障控飞）和**影子模式**（只出 RViz 可视化，队友控飞）。

---

## 1. 仓库配置与编译

将 `uav_navigation` 放入 catkin 工作空间 `src` 目录，直接编译：

```bash
cd ~/raicom26_ws
catkin_make --pkg uav_navigation
source devel/setup.bash
```

地图边界等参数在 [launch/advanced_param.xml](launch/advanced_param.xml) 中（Z 轴已锁死防撞天花板，勿动）。
上层目标点（巡航点、投物点）由主控包自行维护 YAML，本包不干涉。

---

## 2. 启动方式

### 2.1 正常模式（ego 自己避障 + 控制飞控）

```bash
roslaunch uav_navigation ego_nav.launch
```

完整链路：

```
/fsm/ego_goal (队友发目标)
       │
       ▼
ego_controller_node ──→ /goal, /uav1/goal, /ego_planner/goal
       │                        │
       │                        ▼
       │              ego_planner_node (规划)
       │                        │
       │                        ▼
       │              traj_server (B样条→轨迹)
       │                        │
       │                  /position_cmd
       │                        │
       ▼                        ▼
ego_controller_node ──→ /mavros/setpoint_raw/local ──→ PX4 飞控
```

### 2.2 影子模式（ego 只出 RViz，队友避障控飞） ⭐

```bash
roslaunch uav_navigation ego_nav.launch shadow_mode:=true
```

影子模式下 ego 的行为：

| 模块 | 状态 |
|------|------|
| 接收 `/fsm/ego_goal` 目标 | ✅ 正常 |
| 转发目标到 ego_planner（`/goal` `/uav1/goal` `/ego_planner/goal`） | ✅ 正常 |
| ego_planner 规划 + 膨胀栅格 | ✅ 正常 |
| traj_server 生成 B 样条轨迹 → `/position_cmd` | ✅ 正常 |
| RViz 可视化（点云 / 栅格 / 路径 / 目标点） | ✅ 正常 |
| `/ego_controller/status` 状态反馈 | ✅ 正常 |
| `/finish_ego` 控制 traj_server 启停 | ✅ 正常 |
| **往飞控发 `/mavros/setpoint_raw/local`** | ❌ **不发** |
| 实际避障 | 🤝 **队友节点负责** |

**队友要做的事**：
1. 往 `/fsm/ego_goal` 发目标点 → ego 开始规划并在 RViz 显示
2. 监听 `/position_cmd` 获取 ego 规划的轨迹（参考用）
3. 队友自己的节点往 `/mavros/setpoint_raw/local` 发指令完成实际飞行和避障
4. 监听 `/ego_controller/status` 判断是否到达（`2 = ARRIVED`）

> **红线**：影子模式下 `ego_controller_node` 绝不往 `/mavros/setpoint_raw/local` 发一个字节，不会和队友抢控制权。

---

## 3. API 接口

### 3.1 下发目标点（队友 → ego）

| 项目 | 内容 |
|------|------|
| 话题 | `/fsm/ego_goal` |
| 类型 | `geometry_msgs/PoseStamped` |
| 用法 | 发**一次**即可，ego_controller 收到后自动唤醒 ego_planner + traj_server |

```cpp
geometry_msgs::PoseStamped goal;
goal.header.frame_id = "world";
goal.header.stamp = ros::Time::now();
goal.pose.position.x = 5.0;
goal.pose.position.y = 0.0;
goal.pose.position.z = 1.3;   // 必须 > 0，且在 ground_height ~ ground_height+map_size_z_ 之间
goal.pose.orientation.w = 1.0;
ego_goal_pub.publish(goal);
```

### 3.2 监听导航状态（ego → 队友）

| 项目 | 内容 |
|------|------|
| 话题 | `/ego_controller/status` |
| 类型 | `std_msgs/Int8` |

| 值 | 含义 |
|----|------|
| `0` | IDLE — 待机，等待目标 |
| `1` | FLYING — 正在飞行中 |
| `2` | ARRIVED — 已到达目标点附近（误差 < 0.3m） |

### 3.3 ego 规划的轨迹（参考用）

| 项目 | 内容 |
|------|------|
| 话题 | `/position_cmd` |
| 类型 | `quadrotor_msgs/PositionCommand` |
| 说明 | traj_server 输出的 B 样条轨迹，队友可监听作为参考，但不强制使用 |

### 3.4 RViz 可视化话题

| 话题 | 内容 |
|------|------|
| `/cloud_pcl` | 加厚后的点云（cloud_extruder 输出） |
| `/ego_planner/grid_map/occupancy_inflate` | 膨胀栅格地图 |
| `/ego_planner/optimal_list` | 优化后的 B 样条轨迹 |
| `/goal` `/uav1/goal` `/ego_planner/goal` | 当前目标点 |

---

## 4. 调参

在 [ego_nav.launch](launch/ego_nav.launch) 中调整：

```xml
<!-- 地图Z轴范围：ground_height ~ ground_height+map_size_z_ -->
<arg name="map_size_z_" value="2.0"/>
<arg name="ground_height" value="0.0"/>

<!-- 飞行性能 -->
<arg name="max_vel" value="0.9" />
<arg name="max_acc" value="0.9" />
<arg name="planning_horizon" value="5.0" />

<!-- B样条优化权重 -->
<arg name="lambda_smooth" value="0.05" />
<arg name="lambda_collision" value="2.2" />
<arg name="lambda_feasibility" value="0.2" />
<arg name="lambda_fitness" value="3.3" />

<!-- 碰撞检测 -->
<arg name="swarm_clearance" value="0.2" />
<arg name="dist" value="0.17" />
```

---

## 5. 目标点约束（红线）

- Z 轴必须是**正数**（如 1.3）
- Z 必须在 `ground_height` ~ `ground_height + map_size_z_` 之间，超出范围 ego_planner 拒绝规划并原地锁死
- **正常模式**：ego 飞行期间队友**禁止**往 `/mavros/setpoint_raw/local` 发指令
- **影子模式**：ego 绝不往 `/mavros/setpoint_raw/local` 发指令，队友全权负责

---

## 6. 队友调用伪代码

```cpp
// 1. 初始化
ros::Publisher  ego_goal_pub  = nh.advertise<geometry_msgs::PoseStamped>("/fsm/ego_goal", 1);
ros::Subscriber nav_status_sub = nh.subscribe("/ego_controller/status", 10, &navStatusCallback);
ros::Subscriber traj_sub       = nh.subscribe("/position_cmd", 10, &trajCallback);  // 可选，参考轨迹

int nav_status = 0;

void navStatusCallback(const std_msgs::Int8::ConstPtr& msg) {
    nav_status = msg->data;
}

// 2. 下发目标
void sendGoal(double x, double y, double z) {
    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = "world";
    goal.header.stamp = ros::Time::now();
    goal.pose.position.x = x;
    goal.pose.position.y = y;
    goal.pose.position.z = z;
    goal.pose.orientation.w = 1.0;
    ego_goal_pub.publish(goal);
    ROS_INFO("下发ego目标: (%.1f, %.1f, %.1f)", x, y, z);
}

// 3. 主循环
void tick() {
    switch (state) {
        case NAV:
            // 等待 ego 到达
            if (nav_status == 2) {
                state = HOVER;
            }
            // 队友自己的避障控制 → /mavros/setpoint_raw/local
            break;

        case HOVER:
            publishHoverSetpoint();  // 队友自己悬停
            // 做识别、投货等...
            if (done) {
                sendGoal(next_x, next_y, cruise_z);
                state = NAV;
            }
            break;
    }
}
```

---

## 7. 节点清单

| 节点 | 可执行文件 | 作用 |
|------|-----------|------|
| `ego_controller_node` | `src/ego_controller_node.cpp` | 目标转发、轨迹跟踪、MAVROS 指令（影子模式下不发 MAVROS） |
| `cloud_extruder` | `src/cloud_extruder.cpp` | 点云加厚 + 强度过滤，产出 `/cloud_pcl` 给 ego_planner |
| `ego_planner_node` | ego_planner 包 | EGO 规划器本体（不在本包源码中） |
| `traj_server` | ego_planner 包 | B 样条轨迹服务器（不在本包源码中） |
