# MADDPG 多无人机协同系统 (Qt5 集成版)

本项目实现了基于 MADDPG (Multi-Agent Deep Deterministic Policy Gradient) 算法的多无人机协同搜索与围捕系统，并提供了与 Qt5 框架的集成接口。

## 环境依赖

### Python 环境 (训练部分)

- Python: 3.9.0+
- numpy: 1.26.0+
- gym: 0.26.2+
- pillow: 10.2.0+
- torch: 2.4.0+cu124
- matplotlib: 用于可视化

### Qt 环境 (集成部分)

- Qt5.12+ (推荐 Qt5.15)
- C++11 及以上

## 项目结构

### Python 训练模块

- `agent.py`/`buffer.py`/`maddpg.py`/`networks.py`: MADDPG 算法核心实现
- `qt5_sim_env.py`: 针对 Qt5 环境定制的多无人机搜索-围捕仿真环境
- `main_qt5_sim.py`: 训练主循环
- `evaluate_qt5_sim.py`: 模型评估与可视化

### Qt 集成模块

- `maddpg_agent.h`/`maddpg_agent.cpp`: 用于在 Qt 项目中加载和使用训练好的模型
- `agent_X_actor_inference.json`: 训练好的模型参数文件

## 环境说明

### 仿真环境参数

- 地图大小: 1280×800 像素
- 我方无人机(蓝方): 初始位置在左上角区域，最大速度 50 像素/秒
- 敌方无人机(红方): 初始位置在右下角区域，速度 50 像素/秒
- 探测半径: 地图宽度的 1/6 (约 213 像素)
- 攻击半径: 探测半径的一半 (约 106 像素)
- 时间步长: 0.1 秒

### 观测空间 (28 维)

- 自身信息 (5 维): [pos_x, pos_y, vel_x, vel_y, health]
- 队友信息 (6 维): 2 名队友 × [rel_pos_x, rel_pos_y, health]
- 敌机信息 (9 维): 3 架敌机 × [rel_pos_x, rel_pos_y, health]
- 搜索目标 (2 维): [rel_pos_x, rel_pos_y]
- 障碍物信息 (6 维): 2 个最近障碍物 × [rel_pos_x, rel_pos_y, radius]

### 动作空间

- 2 维连续空间: [a_x, a_y] ∈ [-1, 1]²，表示归一化的加速度

## 训练方法

```bash
# 激活Python环境
cd MADDPG_Multi_UAV_Roundup
python -m venv venv_maddpg
.\venv_maddpg\Scripts\activate

# 安装依赖
pip install -r requirements.txt

# 开始训练
python main_qt5_sim.py

# 评估模型
python evaluate_qt5_sim.py
```

训练过程中的学习曲线和模型检查点保存在 `tmp/maddpg_qt5/UAV_Qt5_3v3_SensorCircle/` 目录下。

## Qt 项目集成指南

### 在 Qt 项目中使用 UAVModel 类

```cpp
// 在您的MainWindow或其他类中
#include "maddpg_agent.h"
#include <QMqttClient>

// 创建模型实例（指定无人机ID）
UAVModel *uavModel = new UAVModel(0, this); // 0是无人机ID

// 设置运动参数
uavModel->setMaxVelocity(50.0f); // 最大速度(像素/秒)
uavModel->setTimeStep(0.1f);     // 时间步长(秒)

// 加载模型参数
bool success = uavModel->loadModel("path/to/agent_0_actor_inference.json");
if (success) {
    qDebug() << "模型加载成功";

    // 准备观察数据（28维向量）
    QVector<float> observation = {
        // 自身信息 (5维)
        0.5f, 0.3f, 0.01f, 0.02f, 1.0f,

        // 队友信息 (6维)
        0.1f, 0.2f, 1.0f, 0.3f, 0.4f, 1.0f,

        // 敌机信息 (9维)
        0.5f, 0.6f, 0.8f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,

        // 搜索目标 (2维)
        0.7f, 0.8f,

        // 障碍物信息 (6维)
        0.2f, 0.3f, 0.05f, 0.4f, 0.5f, 0.04f
    };

    // 方法1: 获取目标速度 (直接映射加速度方向为速度方向)
    QPointF targetVelocity = uavModel->getTargetVelocity(observation);

    // 方法2: 获取考虑加速度和当前速度的新速度
    QPointF velocity = uavModel->getVelocity(observation);

    qDebug() << "推理结果 - 速度向量:" << velocity;

    // 使用速度向量控制无人机
    // 例如，通过MQTT发送到控制系统
    QMqttClient *mqttClient = new QMqttClient(this);
    mqttClient->setHostname("192.168.1.100");
    mqttClient->setPort(1883);
    mqttClient->connectToHost();

    // 构造JSON消息
    QString message = QString("{\"uid\":\"B%1\",\"vx\":%2,\"vy\":%3}")
                      .arg(0)
                      .arg(velocity.x())
                      .arg(velocity.y());

    // 发布消息
    mqttClient->publish("uav/control", message.toUtf8());
}
```

### 从 Python 训练到 Qt 集成的工作流

1. 使用 Python 训练模块训练 MADDPG 模型
2. 导出 Actor 网络参数为 JSON 格式
3. 在 Qt 项目中使用`UAVModel`类加载模型参数
4. 通过`getVelocity`或`getTargetVelocity`方法获取控制指令
5. 将控制指令发送到实际或仿真的无人机系统

## 许可证

MIT License

## 致谢

- 本项目的 MADDPG 实现部分参考了[Phil Tabor 的多智能体强化学习代码](https://github.com/philtabor/Multi-Agent-Reinforcement-Learning)

# UAV 战术系统

## 策略 3 优化总结

我们对"车轮战"策略（策略 3）进行了以下优化：

### 1. 敌机探测与追击

- **敌机记忆功能**：当探测到敌机后，即使短时间内失去探测，仍会继续追击上次目标位置（5 秒内有效）
- **立即响应**：一旦探测到敌机，所有无人机立即切换到攻击模式，不再受到路径规划时间间隔的限制
- **队友协同支援**：当任何一架无人机探测到敌机时，立即通知其他无人机前往支援

### 2. 攻击策略优化

- **直接攻击模式**：考虑到比赛时间紧迫（1 分钟），所有无人机直接攻击目标，不再区分攻击者和支援者角色
- **敌机重叠处理**：当多架敌机位置重叠时，所有无人机直接攻击，提高火力密度

### 3. 巡逻与路径规划

- **分散巡逻**：实现了分散式巡逻点分配，确保无人机能够覆盖整个地图
- **路径规划优化**：
  - 实现了简化版 A\*算法，提高路径规划效率
  - 优化了避障逻辑，确保无人机能够绕过障碍物
  - 当无法找到有效路径时，会尝试直接向目标方向移动

### 4. 坐标系统与边界处理

- **坐标转换**：正确处理像素坐标和栅格坐标之间的转换
- **边界检查**：确保所有坐标都在有效范围内，防止无人机飞出地图

### 5. 调试与日志

- **增强日志系统**：添加详细的调试信息，便于问题排查
- **状态追踪**：记录无人机状态变化，包括目标点到达情况、敌机探测情况等

这些优化使得策略 3 更加高效、灵活，能够快速响应战场态势变化，提高了无人机的作战效能。
