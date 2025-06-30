#include "Strategy1.h"

Strategy1::Strategy1(QObject* parent) : QObject(parent), m_isTrackingMode(false)
{
    // 初始化无人机追踪模式
    m_droneTrackingMode["B1"] = false;
    m_droneTrackingMode["B2"] = false;
    m_droneTrackingMode["B3"] = false;
}

// 设置追踪模式或巡逻模式
void Strategy1::setTrackingMode(bool isTracking)
{
    if (m_isTrackingMode != isTracking) {
        m_isTrackingMode = isTracking;
        qDebug() << "设置全局模式为:" << (isTracking ? "追踪模式" : "巡逻模式");

        // 同时更新所有无人机的模式
        QStringList droneIds = {"B1", "B2", "B3"};
        for (const QString& droneId : droneIds) {
            setDroneTrackingMode(droneId, isTracking);
        }
    }
}

// 获取当前模式(追踪或巡逻)
bool Strategy1::isTrackingMode() const
{
    return m_isTrackingMode;
}

// 更新敌方无人机位置信息
void Strategy1::updateEnemyPositions(const QMap<QString, QPoint>& enemyPositions)
{
    m_enemyPositions = enemyPositions;
}

// 更新敌方无人机位置和血量信息
void Strategy1::updateEnemyInfo(const QMap<QString, QPoint>& enemyPositions, const QMap<QString, int>& enemyHp)
{
    m_enemyPositions = enemyPositions;
    m_enemyHp = enemyHp;
}

// 为特定无人机单独设置追踪模式
void Strategy1::setDroneTrackingMode(const QString& droneId, bool isTracking)
{
    if (m_droneTrackingMode.contains(droneId) && m_droneTrackingMode[droneId] != isTracking) {
        m_droneTrackingMode[droneId] = isTracking;
        qDebug() << droneId << "设置为" << (isTracking ? "追踪模式" : "巡逻模式");
    } else if (!m_droneTrackingMode.contains(droneId)) {
        m_droneTrackingMode[droneId] = isTracking;
    }
}

// 获取特定无人机的追踪模式
bool Strategy1::isDroneTracking(const QString& droneId) const
{
    return m_droneTrackingMode.value(droneId, false);
}

// 获取最优追踪目标点 - 优先选择血量高的敌方无人机，血量相同时优先选择R1>R2>R3
QPoint Strategy1::getBestTrackingTarget(const QString& droneId) const
{
    QPoint bestTarget(0, 0);

    if (m_enemyPositions.isEmpty()) {
        return bestTarget;
    }

    // 创建一个记录敌方无人机ID、位置和血量的列表
    struct EnemyInfo {
        QString id;
        QPoint position;
        int hp;
    };

    QVector<EnemyInfo> enemies;

    // 收集所有敌方无人机信息
    for (auto it = m_enemyPositions.constBegin(); it != m_enemyPositions.constEnd(); ++it) {
        const QString& enemyId = it.key();
        if (enemyId.startsWith("R")) {
            EnemyInfo enemy;
            enemy.id = enemyId;
            enemy.position = it.value();
            enemy.hp = m_enemyHp.value(enemyId, 100); // 如果没有血量信息，默认为100
            enemies.append(enemy);
        }
    }

    // 如果没有敌方无人机，返回默认目标点
    if (enemies.isEmpty()) {
        return bestTarget;
    }

    // 对敌方无人机进行排序：首先按血量降序排序，然后按优先级排序(R1>R2>R3)
    std::sort(enemies.begin(), enemies.end(), [](const EnemyInfo& a, const EnemyInfo& b) {
        // 首先比较血量，血量高的优先
        if (a.hp != b.hp) {
            return a.hp > b.hp;
        }

        // 血量相同时，按R1>R2>R3的优先级
        return a.id < b.id; // R1排在R2和R3前面，因为字符串比较 "R1" < "R2" < "R3"
    });

    // 选择排序后的第一个敌方无人机作为目标
    bestTarget = enemies.first().position;
    qDebug() << droneId << "选择" << enemies.first().id << "作为追踪目标，血量:" << enemies.first().hp;

    return bestTarget;
}
