#include "TargetManager.h"

TargetManager::TargetManager(QObject* parent) : QObject(parent)
{
    // 初始化预设目标点
    initPresetTargets();
}

void TargetManager::initPresetTargets()
{
    // 为每个无人机设置3个循环巡逻目标点（栅格坐标）
    QVector<QPoint> b1Targets = {
        QPoint(865/10, 295/10),  // B1的目标点1 (86, 29)
        QPoint(290/10, 315/10),  // B1的目标点2 (29, 31)
        QPoint(295/10, 489/10)   // B1的目标点3 (29, 48)
    };

    QVector<QPoint> b2Targets = {
        QPoint(290/10, 315/10),  // B2的目标点1 (29, 31)
        QPoint(295/10, 489/10),  // B2的目标点2 (29, 48)
        QPoint(865/10, 295/10)   // B2的目标点3 (86, 29)
    };

    QVector<QPoint> b3Targets = {
        QPoint(295/10, 489/10),  // B3的目标点1 (29, 48)
        QPoint(865/10, 295/10),  // B3的目标点2 (86, 29)
        QPoint(290/10, 315/10)   // B3的目标点3 (29, 31)
    };

    m_patrolTargets["B1"] = b1Targets;
    m_patrolTargets["B2"] = b2Targets;
    m_patrolTargets["B3"] = b3Targets;

    // 初始化所有无人机的目标点索引为0
    m_currentTargetIndex["B1"] = 0;
    m_currentTargetIndex["B2"] = 0;
    m_currentTargetIndex["B3"] = 0;

    // 初始化所有无人机的目标点状态为未到达
    m_targetReached["B1"] = false;
    m_targetReached["B2"] = false;
    m_targetReached["B3"] = false;

//    qDebug() << "循环巡逻目标点已初始化";
}

QPoint TargetManager::getCurrentTarget(const QString& droneId) const
{
    if (m_patrolTargets.contains(droneId) && m_currentTargetIndex.contains(droneId)) {
        int index = m_currentTargetIndex[droneId];
        const QVector<QPoint>& targets = m_patrolTargets[droneId];
        if (index >= 0 && index < targets.size()) {
            return targets[index];
        }
    }
    return QPoint(0, 0);
}

QPoint TargetManager::getNextPatrolTarget(const QString& droneId)
{
    if (m_patrolTargets.contains(droneId) && m_currentTargetIndex.contains(droneId)) {
        const QVector<QPoint>& targets = m_patrolTargets[droneId];
        if (!targets.isEmpty()) {
            // 切换到下一个目标点（循环）
            m_currentTargetIndex[droneId] = (m_currentTargetIndex[droneId] + 1) % targets.size();

            // 重置到达状态
            m_targetReached[droneId] = false;

            QPoint nextTarget = targets[m_currentTargetIndex[droneId]];
            qDebug() << droneId << "切换到下一个巡逻目标点:" << nextTarget;
            return nextTarget;
        }
    }

    // 如果没有找到有效的目标点，返回一个默认的有效目标点
    // 避免返回(0,0)导致无人机停止移动
    if (droneId == "B1") {
        return QPoint(865/10, 295/10);
    } else if (droneId == "B2") {
        return QPoint(290/10, 315/10);
    } else if (droneId == "B3") {
        return QPoint(295/10, 489/10);
    }

    return QPoint(50, 50); // 返回一个默认的有效目标点
}

bool TargetManager::hasPresetTarget(const QString& droneId) const
{
    return m_patrolTargets.contains(droneId) && !m_patrolTargets[droneId].isEmpty();
}

void TargetManager::setTargetReached(const QString& droneId, bool reached)
{
    m_targetReached[droneId] = reached;
//    qDebug() << droneId << (reached ? "已到达" : "未到达") << "目标点";
}

bool TargetManager::isTargetReached(const QString& droneId) const
{
    if (m_targetReached.contains(droneId)) {
        return m_targetReached[droneId];
    }
    return false;
}

// 获取巡逻目标点
QPoint TargetManager::getPatrolPoint(const QString& droneId)
{
    // 获取巡逻目标
    if (isTargetReached(droneId)) {
        // 已到达当前目标点，获取下一个巡逻目标点
        return getNextPatrolTarget(droneId);
    } else {
        // 未到达，继续使用当前目标点
        QPoint currentTarget = getCurrentTarget(droneId);

        // 检查当前目标点是否有效，如果无效则获取下一个目标点
        if (currentTarget.isNull() || (currentTarget.x() == 0 && currentTarget.y() == 0)) {
            return getNextPatrolTarget(droneId);
        }
        return currentTarget;
    }
}
