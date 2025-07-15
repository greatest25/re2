#ifndef STRATEGY3_H
#define STRATEGY3_H

#include <QObject>
#include <QPoint>
#include <QMap>
#include <QString>
#include <QVector>
#include <QTime>

// 用于存储无人机状态的简化结构
struct DroneState {
    QPoint position;
    int hp;
};

// 用于存储敌机记忆信息的结构
struct EnemyMemory {
    QPoint position;  // 敌机位置
    QTime lastSeen;   // 上次看到敌机的时间
    int hp;           // 敌机血量
};

class Strategy3 : public QObject
{
    Q_OBJECT
public:
    explicit Strategy3(QObject *parent = nullptr);
    virtual ~Strategy3();

    // 从主窗口更新战场上所有无人机的状态
    void updateGameState(const QMap<QString, DroneState>& friendlyDrones, const QMap<QString, DroneState>& enemyDrones);

    // 为指定的我方无人机获取计算后的目标点
    QPoint getTargetForDrone(const QString& droneId);

    // 重置策略状态，用于新一局游戏
    void reset();

    // 辅助函数，尝试调整目标点位置，避免目标点位于障碍物区域
    QPoint adjustTargetPoint(const QPoint& originalTarget, bool isGrid = true) const;

    // 常量定义
    static const int GRID_SIZE;  // 栅格大小
    static const int MAP_WIDTH;  // 地图宽度（栅格）
    static const int MAP_HEIGHT; // 地图高度（栅格）
    static const int MAP_PIXEL_WIDTH = 1280;   // 地图像素宽度
    static const int MAP_PIXEL_HEIGHT = 800;   // 地图像素高度
    static const int MEMORY_DURATION = 5000;  // 敌机记忆持续时间（毫秒）
    static const int TARGET_CONFIRM_DISTANCE = 5;  // 目标确认距离（栅格）
    static const int TARGET_REACHED_THRESHOLD = 2; // 到达目标点阈值（栅格）
    static const int MIN_TARGET_UPDATE_DISTANCE = 3; // 最小目标更新距离（栅格）
    static const int PATH_UPDATE_THRESHOLD = 2;  // 路径更新阈值（栅格）
    static const int ATTACK_RANGE = 5;  // 攻击范围（栅格）

    // 巡逻点和阵型偏移定义
    static const QVector<QPoint> PATROL_POINTS;  // 巡逻点列表
    static const QMap<QString, QPoint> FORMATION_OFFSETS;  // 阵型偏移
    
signals:
    // 添加重新规划路径的信号
    void needReplanPath(const QString& droneId, const QPoint& targetPoint);

private:
    // 寻找全局最优攻击目标（血量最低的敌人）
    void findWeakestEnemy();

    // 分配攻击者和支援者角色，并计算各自的目标点
    void assignRolesAndTargets();
    
    // 分配巡逻目标点
    void assignPatrolTargets();

    // 新增：检查是否需要重新规划路径
    bool needReplanning(const QString& droneId, const QPoint& enemyPos);

    // 新增：获取最近的敌机
    QString getNearestEnemy(const QPoint& position) const;

    // 新增：共享敌机信息
    void shareEnemyInfo();

    // 新增：检查是否有队友正在追击敌机
    bool hasTeammateEngaging(const QString& excludeDroneId) const;

    // 新增：获取正在交战的队友位置
    QPoint getEngagingTeammatePosition() const;

    // 新增：计算支援位置
    QPoint calculateSupportPosition(const QPoint& enemyPos, const QPoint& teammatePos) const;

    // 新增：计算阵型位置
    QPoint calculateFormationPosition(const QString& droneId, const QPoint& centerPos) const;

    // 新增：获取阵型中心点
    QPoint getFormationCenter() const;
    
    // 新增：判断是否到达目标点
    bool hasReachedTarget(const QString& droneId) const;
    
    // 新增：判断是否需要更新目标点
    bool needUpdateTarget(const QString& droneId, const QPoint& newTarget) const;
    
    // 新增：更新无人机目标点
    void updateDroneTarget(const QString& droneId, const QPoint& newTarget);
    
    // 新增：获取最近的队友位置
    QPoint getNearestTeammatePosition(const QString& excludeDroneId) const;
    
    // 新增：更新巡逻目标点
    void updatePatrolTargets();
    
    // 新增：判断敌机是否是队友的追击目标
    bool isEnemyTeammateTarget(const QPoint& enemyPos, const QPoint& teammatePos) const;

private:
    // 我方无人机状态 - 键是无人机ID，值是状态信息
    QMap<QString, DroneState> m_friendlyDrones;

    // 敌方无人机状态 - 键是无人机ID，值是状态信息
    QMap<QString, DroneState> m_enemyDrones;
    
    // 敌方无人机历史位置 - 用于位置预测
    QMap<QString, QPoint> m_lastEnemyPositions;

    // 敌方无人机记忆 - 用于存储敌机记忆信息
    QMap<QString, EnemyMemory> m_lastKnownEnemyPositions;

    // 当前目标敌机ID
    QString m_currentTargetedEnemyId;

    // 当前攻击者ID
    QString m_currentAttackerId;

    // 无人机目标点 - 键是无人机ID，值是目标栅格坐标
    QMap<QString, QPoint> m_droneTargets;

    // 无人机目标敌机 - 键是无人机ID，值是目标敌机ID
    QMap<QString, QString> m_droneTargetEnemies;

    // 无人机交战状态 - 键是无人机ID，值是是否正在交战
    QMap<QString, bool> m_droneEngagingStatus;

    // 上次更新目标点的时间 - 键是无人机ID，值是上次更新时间
    QMap<QString, QTime> m_lastTargetUpdateTime;

    // 上次规划路径的时间 - 键是无人机ID，值是上次规划时间
    QMap<QString, QTime> m_lastPathPlanTime;

    // 上次目标点 - 键是无人机ID，值是上次目标点
    QMap<QString, QPoint> m_lastTargets;

    // 是否检测到敌机
    bool m_hasEnemyDetected;

    // 巡逻点索引
    int m_patrolIndex;
};

#endif // STRATEGY3_H
