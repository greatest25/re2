#ifndef STRATEGY3_H
#define STRATEGY3_H

#include <QObject>
#include <QPoint>
#include <QMap>
#include <QString>
#include <QVector>

// 用于存储无人机状态的简化结构
struct DroneState {
    QPoint position;
    int hp;
};

class Strategy3 : public QObject
{
    Q_OBJECT
public:
    explicit Strategy3(QObject* parent = nullptr);

    // 从主窗口更新战场上所有无人机的状态
    void updateGameState(const QMap<QString, DroneState>& friendlyDrones, const QMap<QString, DroneState>& enemyDrones);

    // 为指定的我方无人机获取计算后的目标点
    QPoint getTargetForDrone(const QString& droneId);

    // 重置策略状态，用于新一局游戏
    void reset();

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

    // 战场态势感知
    QMap<QString, DroneState> m_friendlyDrones;
    QMap<QString, DroneState> m_enemyDrones;

    // 策略决策结果
    QString m_currentTargetedEnemyId; // 当前集火的敌机ID
    QString m_currentAttackerId;      // 当前负责攻击的我方无人机ID
    QMap<QString, QPoint> m_droneTargets; // 我方每架无人机的目标点
    
    // 新增：每架无人机的交战状态
    QMap<QString, bool> m_droneEngagingStatus;
    
    // 巡逻相关
    int m_patrolIndex;                // 当前巡逻点索引
    bool m_hasEnemyDetected;          // 是否探测到敌机

    // "车轮战"策略的核心参数
    static const int RETREAT_HP_THRESHOLD = 35; // 当攻击者血量低于此值时后撤
    static const int SAFE_DISTANCE = 150;      // 支援者与目标的保持距离
    
    // 支援相关参数
    static const int SUPPORT_DISTANCE = 80;    // 支援者与主攻击者的距离
    static const int ENGAGE_RANGE = 250;       // 交战距离阈值，增加到250以提高发现敌机的灵敏度
    
    // 目标判定参数（调整为更合理的值）
    static const int TARGET_CONFIRM_DISTANCE = 200; // 敌机和队友的距离阈值，用于判断是否为追击目标
    static const int TEAMMATE_SUPPORT_RANGE = 250;  // 前往支援队友的最大距离

    // 新增：目标点更新相关参数
    static const int TARGET_REACHED_THRESHOLD = 2;  // 到达目标点的距离阈值（格数）
    static const int MIN_TARGET_UPDATE_DISTANCE = 2; // 最小目标点更新距离（格数），减小以提高目标更新灵敏度
    static const int TARGET_UPDATE_COOLDOWN = 5;    // 目标点更新冷却时间（帧数），减小以提高更新频率

    // 新增：阵型相关参数
    static const int FORMATION_SPACING = 100;   // 阵型中无人机之间的基础间距
    static const int FORMATION_OFFSET = 60;     // 阵型偏移量，用于错开位置
    
    // 地图实际像素尺寸
    static const int MAP_PIXEL_WIDTH = 1280;   // 地图像素宽度
    static const int MAP_PIXEL_HEIGHT = 800;   // 地图像素高度
    
    // 新增：阵型位置映射（用于确定每架无人机在阵型中的相对位置）
    const QMap<QString, QPoint> FORMATION_OFFSETS = {
        {"B1", QPoint(-FORMATION_OFFSET, -FORMATION_OFFSET)},   // 左上
        {"B2", QPoint(FORMATION_OFFSET, -FORMATION_OFFSET)},    // 右上
        {"B3", QPoint(0, FORMATION_OFFSET)}                     // 下中
    };

    // 栅格大小常量，与GridMap保持一致
    static const int GRID_SIZE;
    
    // 地图尺寸常量
    static const int MAP_WIDTH;  // 栅格地图宽度
    static const int MAP_HEIGHT; // 栅格地图高度
    
    // 巡逻点
    static const QVector<QPoint> PATROL_POINTS;

    // 每架无人机的当前路径
    QMap<QString, QVector<QPoint>> m_dronePaths;
    
    // 每架无人机的当前目标敌机
    QMap<QString, QString> m_droneTargetEnemies;

    // "车轮战"策略的核心参数
    static const int PATH_UPDATE_THRESHOLD = 5; // 当敌机移动超过此格数时重新规划
    static const int ATTACK_RANGE = 8;         // 攻击范围（格数）

    // 新增：判断敌机是否是队友的追击目标
    bool isEnemyTeammateTarget(const QPoint& enemyPos, const QPoint& teammatePos) const;

    // 新增：获取最近的队友位置
    QPoint getNearestTeammatePosition(const QString& excludeDroneId) const;

    // 新增：判断是否到达目标点
    bool hasReachedTarget(const QString& droneId) const;

    // 新增：判断是否需要更新目标点
    bool needUpdateTarget(const QString& droneId, const QPoint& newTarget) const;

    // 新增：更新无人机目标点
    void updateDroneTarget(const QString& droneId, const QPoint& newTarget);

    // 新增：目标点更新相关状态
    QMap<QString, int> m_targetUpdateCooldown;  // 每架无人机的目标点更新冷却计时
    QMap<QString, QPoint> m_lastTargets;        // 每架无人机的上一个目标点
};

#endif // STRATEGY3_H
