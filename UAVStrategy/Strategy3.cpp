#include "Strategy3.h"
#include <QVector>
#include <QtMath>
#include <QDebug>

// 添加栅格大小常量，与GridMap保持一致
const int Strategy3::GRID_SIZE = 20; // 栅格大小为20像素

// 添加地图尺寸常量
const int Strategy3::MAP_WIDTH = 64; // 栅格地图宽度
const int Strategy3::MAP_HEIGHT = 40; // 栅格地图高度

// 添加巡逻点
const QVector<QPoint> Strategy3::PATROL_POINTS = {
    QPoint(10, 10),   // 左上
    QPoint(50, 10),   // 右上
    QPoint(50, 30),   // 右下
    QPoint(10, 30)    // 左下
};

Strategy3::Strategy3(QObject* parent) : QObject(parent), m_patrolIndex(0)
{
    reset();
}

void Strategy3::reset()
{
    m_currentTargetedEnemyId.clear();
    m_currentAttackerId.clear();
    m_friendlyDrones.clear();
    m_enemyDrones.clear();
    m_droneTargets.clear();
    m_patrolIndex = 0;
    m_hasEnemyDetected = false;
    m_droneEngagingStatus.clear();
    m_targetUpdateCooldown.clear();
    m_lastTargets.clear();
}

// 新增：检查是否有队友正在追击敌机
bool Strategy3::hasTeammateEngaging(const QString& excludeDroneId) const
{
    for (auto it = m_droneEngagingStatus.constBegin(); it != m_droneEngagingStatus.constEnd(); ++it) {
        if (it.key() != excludeDroneId && it.value()) {
            return true;
        }
    }
    return false;
}

// 新增：获取正在交战的队友位置
QPoint Strategy3::getEngagingTeammatePosition() const
{
    for (auto it = m_droneEngagingStatus.constBegin(); it != m_droneEngagingStatus.constEnd(); ++it) {
        if (it.value() && m_friendlyDrones.contains(it.key())) {
            return m_friendlyDrones[it.key()].position;
        }
    }
    return QPoint(0, 0);
}

// 新增：计算支援位置
QPoint Strategy3::calculateSupportPosition(const QPoint& enemyPos, const QPoint& teammatePos) const
{
    // 计算敌机和队友的中点作为支援位置
    QPoint midPoint((enemyPos.x() + teammatePos.x()) / 2, (enemyPos.y() + teammatePos.y()) / 2);
    
    // 转换为栅格坐标
    QPoint gridPos(midPoint.x() / GRID_SIZE, midPoint.y() / GRID_SIZE);
    
    // 确保坐标在有效范围内
    gridPos.setX(qBound(0, gridPos.x(), MAP_WIDTH - 1));
    gridPos.setY(qBound(0, gridPos.y(), MAP_HEIGHT - 1));
    
    return gridPos;
}

// 新增：判断敌机是否是队友的追击目标
bool Strategy3::isEnemyTeammateTarget(const QPoint& enemyPos, const QPoint& teammatePos) const
{
    int dx = qAbs(enemyPos.x() - teammatePos.x());
    int dy = qAbs(enemyPos.y() - teammatePos.y());
    return (dx <= TARGET_CONFIRM_DISTANCE && dy <= TARGET_CONFIRM_DISTANCE);
}

// 新增：获取最近的队友位置
QPoint Strategy3::getNearestTeammatePosition(const QString& excludeDroneId) const
{
    QPoint nearestPos(0, 0);
    int minDistance = INT_MAX;
    
    // 如果没有自己的位置信息，返回默认值
    if (!m_friendlyDrones.contains(excludeDroneId)) {
        return nearestPos;
    }
    
    const QPoint& myPos = m_friendlyDrones[excludeDroneId].position;
    
    // 遍历所有队友
    for (auto it = m_friendlyDrones.constBegin(); it != m_friendlyDrones.constEnd(); ++it) {
        if (it.key() != excludeDroneId) {
            const QPoint& teammatePos = it.value().position;
            int dx = qAbs(teammatePos.x() - myPos.x());
            int dy = qAbs(teammatePos.y() - myPos.y());
            int distance = dx + dy;
            
            if (distance < minDistance) {
                minDistance = distance;
                nearestPos = teammatePos;
            }
        }
    }
    
    return nearestPos;
}

// 新增：获取阵型中心点
QPoint Strategy3::getFormationCenter() const
{
    if (m_friendlyDrones.isEmpty()) {
        return QPoint(0, 0);
    }

    // 计算所有无人机的平均位置
    int totalX = 0, totalY = 0;
    for (auto it = m_friendlyDrones.constBegin(); it != m_friendlyDrones.constEnd(); ++it) {
        totalX += it.value().position.x();
        totalY += it.value().position.y();
    }

    QPoint center(totalX / m_friendlyDrones.size(), totalY / m_friendlyDrones.size());
    return center;
}

// 新增：计算阵型位置
QPoint Strategy3::calculateFormationPosition(const QString& droneId, const QPoint& centerPos) const
{
    if (!FORMATION_OFFSETS.contains(droneId)) {
        return centerPos;
    }

    // 获取该无人机在阵型中的偏移量
    QPoint offset = FORMATION_OFFSETS[droneId];
    
    // 计算目标位置（像素坐标）
    QPoint targetPos = centerPos + offset;
    
    // 确保位置在地图范围内
    targetPos.setX(qBound(0, targetPos.x(), MAP_PIXEL_WIDTH - 1));
    targetPos.setY(qBound(0, targetPos.y(), MAP_PIXEL_HEIGHT - 1));
    
    // 转换为栅格坐标
    QPoint gridPos(targetPos.x() / GRID_SIZE, targetPos.y() / GRID_SIZE);
    gridPos.setX(qBound(0, gridPos.x(), MAP_WIDTH - 1));
    gridPos.setY(qBound(0, gridPos.y(), MAP_HEIGHT - 1));
    
    return gridPos;
}

// 新增：判断是否到达目标点
bool Strategy3::hasReachedTarget(const QString& droneId) const
{
    if (!m_friendlyDrones.contains(droneId) || !m_droneTargets.contains(droneId)) {
        return false;
    }

    const DroneState& droneState = m_friendlyDrones[droneId];
    QPoint currentPos(droneState.position.x() / GRID_SIZE, droneState.position.y() / GRID_SIZE);
    QPoint targetPos = m_droneTargets[droneId];

    int dx = qAbs(currentPos.x() - targetPos.x());
    int dy = qAbs(currentPos.y() - targetPos.y());

    return (dx <= TARGET_REACHED_THRESHOLD && dy <= TARGET_REACHED_THRESHOLD);
}

// 新增：判断是否需要更新目标点
bool Strategy3::needUpdateTarget(const QString& droneId, const QPoint& newTarget) const
{
    // 如果无人机没有当前目标点，需要更新
    if (!m_droneTargets.contains(droneId)) {
        return true;
    }

    // 检查冷却时间
    if (m_targetUpdateCooldown.contains(droneId) && m_targetUpdateCooldown[droneId] > 0) {
        // 如果是追击敌机的目标点，即使在冷却中也允许更新
        if (m_enemyDrones.isEmpty()) {
            return false;
        }
        
        // 检查新目标点是否对应敌机位置
        for (auto enemyIt = m_enemyDrones.constBegin(); enemyIt != m_enemyDrones.constEnd(); ++enemyIt) {
            const DroneState& enemyState = enemyIt.value();
            QPoint enemyGridPos(enemyState.position.x() / GRID_SIZE, enemyState.position.y() / GRID_SIZE);
            
            if (enemyGridPos == newTarget) {
                return true;  // 如果是追击敌机，允许更新目标点
            }
        }
        return false;
    }

    // 如果已经到达当前目标点，允许更新
    if (hasReachedTarget(droneId)) {
        return true;
    }

    // 计算新目标点与当前目标点的距离
    QPoint currentTarget = m_droneTargets[droneId];
    int dx = qAbs(newTarget.x() - currentTarget.x());
    int dy = qAbs(newTarget.y() - currentTarget.y());

    // 如果新目标点对应敌机位置，降低更新距离阈值
    for (auto enemyIt = m_enemyDrones.constBegin(); enemyIt != m_enemyDrones.constEnd(); ++enemyIt) {
        const DroneState& enemyState = enemyIt.value();
        QPoint enemyGridPos(enemyState.position.x() / GRID_SIZE, enemyState.position.y() / GRID_SIZE);
        
        if (enemyGridPos == newTarget) {
            return (dx >= 1 || dy >= 1);  // 如果是追击敌机，只要移动了1格就更新
        }
    }

    // 只有当新目标点与当前目标点距离超过阈值时才更新
    return (dx >= MIN_TARGET_UPDATE_DISTANCE || dy >= MIN_TARGET_UPDATE_DISTANCE);
}

// 新增：更新无人机目标点的成员函数实现
void Strategy3::updateDroneTarget(const QString& droneId, const QPoint& newTarget)
{
    m_droneTargets[droneId] = newTarget;
    m_targetUpdateCooldown[droneId] = TARGET_UPDATE_COOLDOWN;
    m_lastTargets[droneId] = newTarget;
}

void Strategy3::updateGameState(const QMap<QString, DroneState>& friendlyDrones, const QMap<QString, DroneState>& enemyDrones)
{
    m_friendlyDrones = friendlyDrones;
    m_enemyDrones = enemyDrones;
    
    // 更新目标点冷却时间
    for (auto it = m_targetUpdateCooldown.begin(); it != m_targetUpdateCooldown.end(); ++it) {
        if (it.value() > 0) {
            it.value()--;
        }
    }
    
    // 检查是否探测到敌机
    m_hasEnemyDetected = !enemyDrones.isEmpty();
    
    if (m_hasEnemyDetected) {
        // 更新每架无人机的状态
        for (auto it = m_friendlyDrones.constBegin(); it != m_friendlyDrones.constEnd(); ++it) {
            const QString& droneId = it.key();
            const DroneState& droneState = it.value();
            bool hasNewTarget = false;
            
            // 1. 检查是否直接发现敌机
            for (auto enemyIt = m_enemyDrones.constBegin(); enemyIt != m_enemyDrones.constEnd(); ++enemyIt) {
                const QString& enemyId = enemyIt.key();
                const DroneState& enemyState = enemyIt.value();
                
                int dx = qAbs(enemyState.position.x() - droneState.position.x());
                int dy = qAbs(enemyState.position.y() - droneState.position.y());
                
                if (dx <= ENGAGE_RANGE && dy <= ENGAGE_RANGE) {
                    // 计算新的目标点
                    QPoint gridTargetPos(enemyState.position.x() / GRID_SIZE, enemyState.position.y() / GRID_SIZE);
                    gridTargetPos.setX(qBound(0, gridTargetPos.x(), MAP_WIDTH - 1));
                    gridTargetPos.setY(qBound(0, gridTargetPos.y(), MAP_HEIGHT - 1));
                    
                    // 检查是否需要更新目标点
                    if (needUpdateTarget(droneId, gridTargetPos)) {
                        m_droneTargetEnemies[droneId] = enemyId;
                        updateDroneTarget(droneId, gridTargetPos);
                        m_droneEngagingStatus[droneId] = true;
                        hasNewTarget = true;
                        qDebug() << "[Strategy3] 无人机" << droneId << "更新追击目标" << enemyId 
                                << "，新目标点:" << gridTargetPos;
                    }
                    break;
                }
            }
            
            // 2. 如果没有直接发现敌机，考虑支援队友
            if (!hasNewTarget && !hasReachedTarget(droneId)) {
                // 获取最近队友的位置
                QPoint teammatePos = getNearestTeammatePosition(droneId);
                if (teammatePos != QPoint(0, 0)) {
                    // 计算与队友的距离
                    int dx = qAbs(teammatePos.x() - droneState.position.x());
                    int dy = qAbs(teammatePos.y() - droneState.position.y());
                    
                    if (dx <= TEAMMATE_SUPPORT_RANGE && dy <= TEAMMATE_SUPPORT_RANGE) {
                        // 检查是否有敌机在队友附近
                        for (auto enemyIt = m_enemyDrones.constBegin(); enemyIt != m_enemyDrones.constEnd(); ++enemyIt) {
                            const QString& enemyId = enemyIt.key();
                            const DroneState& enemyState = enemyIt.value();
                            
                            // 判断这个敌机是否是队友的追击目标
                            if (isEnemyTeammateTarget(enemyState.position, teammatePos)) {
                                QPoint gridTargetPos(enemyState.position.x() / GRID_SIZE, enemyState.position.y() / GRID_SIZE);
                                gridTargetPos.setX(qBound(0, gridTargetPos.x(), MAP_WIDTH - 1));
                                gridTargetPos.setY(qBound(0, gridTargetPos.y(), MAP_HEIGHT - 1));
                                
                                if (needUpdateTarget(droneId, gridTargetPos)) {
                                    m_droneTargetEnemies[droneId] = enemyId;
                                    updateDroneTarget(droneId, gridTargetPos);
                                    hasNewTarget = true;
                                    qDebug() << "[Strategy3] 无人机" << droneId << "更新支援目标，追击敌机" << enemyId;
                                }
                                break;
                            }
                        }
                        
                        // 如果没有找到队友的追击目标，保持阵型跟随
                        if (!hasNewTarget) {
                            QPoint centerPos = getFormationCenter();
                            QPoint formationPos = calculateFormationPosition(droneId, centerPos);
                            
                            if (needUpdateTarget(droneId, formationPos)) {
                                updateDroneTarget(droneId, formationPos);
                                m_droneEngagingStatus[droneId] = false;
                                qDebug() << "[Strategy3] 无人机" << droneId << "更新阵型位置:" << formationPos;
                            }
                        }
                    }
                }
            }
        }
        
        // 共享敌机信息
        shareEnemyInfo();
        
        // 如果还有未分配任务的无人机，执行车轮战策略
        findWeakestEnemy();
        assignRolesAndTargets();
    } else {
        // 无敌机时，保持三角形阵型巡逻
        m_droneTargetEnemies.clear();
        m_droneEngagingStatus.clear();
        
        // 获取巡逻中心点
        QPoint patrolCenter = PATROL_POINTS[m_patrolIndex];
        patrolCenter.setX(patrolCenter.x() * GRID_SIZE);
        patrolCenter.setY(patrolCenter.y() * GRID_SIZE);
        
        // 为每架无人机分配阵型位置
        for (auto it = m_friendlyDrones.constBegin(); it != m_friendlyDrones.constEnd(); ++it) {
            const QString& droneId = it.key();
            QPoint formationPos = calculateFormationPosition(droneId, patrolCenter);
            
            // 只有到达当前目标点或距离足够远时才更新目标点
            if (needUpdateTarget(droneId, formationPos)) {
                updateDroneTarget(droneId, formationPos);
            }
        }
        
        // 只有当所有无人机都接近目标点时才更新巡逻点
        bool allNearTarget = true;
        for (auto it = m_friendlyDrones.constBegin(); it != m_friendlyDrones.constEnd(); ++it) {
            if (!hasReachedTarget(it.key())) {
                allNearTarget = false;
                break;
            }
        }
        
        if (allNearTarget) {
            m_patrolIndex = (m_patrolIndex + 1) % PATROL_POINTS.size();
            qDebug() << "[Strategy3] 所有无人机到达目标点，更新巡逻中心点:" << patrolCenter;
        }
    }
}

QPoint Strategy3::getTargetForDrone(const QString& droneId)
{
    // 返回为该无人机计算好的目标点，如果不存在则返回默认点(0,0)
    return m_droneTargets.value(droneId, QPoint(0, 0));
}

void Strategy3::findWeakestEnemy()
{
    // 如果当前目标不存在或已被击毁，则寻找新目标
    if (!m_enemyDrones.contains(m_currentTargetedEnemyId)) {
        m_currentTargetedEnemyId.clear();
    }

    // 如果已经有目标，暂时不切换，以保证火力集中
    if (!m_currentTargetedEnemyId.isEmpty()) {
        return;
    }

    int minHp = INT_MAX;
    QString weakestEnemyId;

    for (auto it = m_enemyDrones.constBegin(); it != m_enemyDrones.constEnd(); ++it) {
        if (it.value().hp < minHp) {
            minHp = it.value().hp;
            weakestEnemyId = it.key();
        }
    }

    if (!weakestEnemyId.isEmpty()) {
        m_currentTargetedEnemyId = weakestEnemyId;
        qDebug() << "[Strategy3] New weakest enemy targeted:" << weakestEnemyId << "with HP:" << minHp;
    }
}

// 新增：分配巡逻目标点
void Strategy3::assignPatrolTargets()
{
    if (m_friendlyDrones.isEmpty()) {
        return;
    }
    
    // 为每架无人机分配不同的巡逻点
    int droneCount = m_friendlyDrones.size();
    int pointsPerDrone = PATROL_POINTS.size() / droneCount;
    
    if (pointsPerDrone < 1) pointsPerDrone = 1;
    
    int droneIndex = 0;
    for (auto it = m_friendlyDrones.constBegin(); it != m_friendlyDrones.constEnd(); ++it, ++droneIndex) {
        const QString& droneId = it.key();
        
        // 计算这架无人机的巡逻点索引
        int patrolIndex = (m_patrolIndex + droneIndex) % PATROL_POINTS.size();
        QPoint targetPoint = PATROL_POINTS[patrolIndex];
        
        m_droneTargets[droneId] = targetPoint;
        qDebug() << "[Strategy3] 无人机" << droneId << "分配巡逻点:" << targetPoint;
    }
    
    // 更新巡逻索引，下次分配时使用下一个巡逻点
    m_patrolIndex = (m_patrolIndex + 1) % PATROL_POINTS.size();
}

void Strategy3::assignRolesAndTargets()
{
    if (m_currentTargetedEnemyId.isEmpty() || !m_enemyDrones.contains(m_currentTargetedEnemyId) || m_friendlyDrones.isEmpty()) {
        // 如果没有目标或没有我方无人机，则执行巡逻
        assignPatrolTargets();
        return;
    }

    const DroneState& targetEnemy = m_enemyDrones[m_currentTargetedEnemyId];

    // 检查当前攻击者是否需要后撤
    if (m_friendlyDrones.contains(m_currentAttackerId) && m_friendlyDrones[m_currentAttackerId].hp <= RETREAT_HP_THRESHOLD) {
        qDebug() << "[Strategy3] 攻击者" << m_currentAttackerId << "血量过低，后撤";
        m_currentAttackerId.clear(); // 清除攻击者，迫使下面重新选择
    }

    // 如果没有攻击者 (初始状态，或原攻击者已后撤/被击毁)
    if (!m_friendlyDrones.contains(m_currentAttackerId)) {
        QString bestAttackerId;
        int maxHp = -1;
        // 选择血量最高的无人机作为新的攻击者
        for (auto it = m_friendlyDrones.constBegin(); it != m_friendlyDrones.constEnd(); ++it) {
            const QString& droneId = it.key();
            const DroneState& droneState = it.value();
            
            // 计算与目标敌机的距离
            int dx = qAbs(droneState.position.x() / GRID_SIZE - targetEnemy.position.x() / GRID_SIZE);
            int dy = qAbs(droneState.position.y() / GRID_SIZE - targetEnemy.position.y() / GRID_SIZE);
            int distance = dx + dy;
            
            // 必须是血量健康且距离合适的无人机
            if (droneState.hp > RETREAT_HP_THRESHOLD && 
                droneState.hp > maxHp && 
                distance <= ATTACK_RANGE * 2) { // 在攻击范围的两倍内选择攻击者
                maxHp = droneState.hp;
                bestAttackerId = droneId;
            }
        }
        if(!bestAttackerId.isEmpty()){
             m_currentAttackerId = bestAttackerId;
             qDebug() << "[Strategy3] 选择新的攻击者:" << m_currentAttackerId;
        }
    }

    // 为每架无人机分配目标点
    for (auto it = m_friendlyDrones.constBegin(); it != m_friendlyDrones.constEnd(); ++it) {
        const QString& droneId = it.key();
        const DroneState& droneState = it.value();
        
        if (droneId == m_currentAttackerId) {
            // 攻击者：目标是敌人
            // 将敌人像素坐标转换为栅格坐标
            QPoint enemyPixelPos = targetEnemy.position;
            QPoint gridTargetPos(enemyPixelPos.x() / GRID_SIZE, enemyPixelPos.y() / GRID_SIZE);
            
            // 确保坐标在有效范围内
            gridTargetPos.setX(qBound(0, gridTargetPos.x(), MAP_WIDTH - 1));
            gridTargetPos.setY(qBound(0, gridTargetPos.y(), MAP_HEIGHT - 1));
            
            m_droneTargets[droneId] = gridTargetPos;
            m_droneTargetEnemies[droneId] = m_currentTargetedEnemyId;
            
            qDebug() << "[Strategy3] 攻击者" << droneId << "追踪敌机" << m_currentTargetedEnemyId 
                     << "目标位置:" << gridTargetPos;
        } else {
            // 支援者：根据敌人和自身位置选择最佳支援位置
            QPoint enemyGridPos(targetEnemy.position.x() / GRID_SIZE, targetEnemy.position.y() / GRID_SIZE);
            QPoint droneGridPos(droneState.position.x() / GRID_SIZE, droneState.position.y() / GRID_SIZE);
            
            // 计算从敌人到我方支援者的方向向量
            QPointF vector = QPointF(droneGridPos) - QPointF(enemyGridPos);
            
            // 如果向量长度为0（两者位置重合），给一个默认方向
            if (vector.x() == 0 && vector.y() == 0) {
                vector = QPointF(1, 0); // 默认向右
            } else {
                // 单位化向量
                qreal length = qSqrt(vector.x() * vector.x() + vector.y() * vector.y());
                vector = vector / length;
            }
            
            // 计算支援位置（在敌机周围SAFE_DISTANCE/GRID_SIZE格的位置）
            QPointF safePoint = QPointF(enemyGridPos) + vector * (SAFE_DISTANCE / GRID_SIZE);
            QPoint gridSafePoint(qRound(safePoint.x()), qRound(safePoint.y()));
            
            // 确保坐标在有效范围内
            gridSafePoint.setX(qBound(0, gridSafePoint.x(), MAP_WIDTH - 1));
            gridSafePoint.setY(qBound(0, gridSafePoint.y(), MAP_HEIGHT - 1));
            
            m_droneTargets[droneId] = gridSafePoint;
            
            // 支援者也要记录当前追踪的敌机
            m_droneTargetEnemies[droneId] = m_currentTargetedEnemyId;
            
            qDebug() << "[Strategy3] 支援者" << droneId << "协助追踪敌机" << m_currentTargetedEnemyId 
                     << "支援位置:" << gridSafePoint;
        }
    }
}

// 新增：检查是否需要重新规划路径
bool Strategy3::needReplanning(const QString& droneId, const QPoint& enemyPos)
{
    // 如果无人机没有当前路径，需要规划
    if (!m_dronePaths.contains(droneId) || m_dronePaths[droneId].isEmpty()) {
        return true;
    }
    
    // 获取当前目标点
    QPoint currentTarget = m_droneTargets.value(droneId);
    QPoint enemyGridPos(enemyPos.x() / GRID_SIZE, enemyPos.y() / GRID_SIZE);
    
    // 计算敌机移动距离
    int dx = qAbs(enemyGridPos.x() - currentTarget.x());
    int dy = qAbs(enemyGridPos.y() - currentTarget.y());
    
    // 如果敌机移动超过阈值，需要重新规划
    return (dx > PATH_UPDATE_THRESHOLD || dy > PATH_UPDATE_THRESHOLD);
}

// 新增：获取最近的敌机
QString Strategy3::getNearestEnemy(const QPoint& position) const
{
    QString nearestEnemyId;
    int minDistance = INT_MAX;
    
    for (auto it = m_enemyDrones.constBegin(); it != m_enemyDrones.constEnd(); ++it) {
        const QString& enemyId = it.key();
        const DroneState& enemyState = it.value();
        
        // 计算栅格距离
        int dx = qAbs(enemyState.position.x() / GRID_SIZE - position.x() / GRID_SIZE);
        int dy = qAbs(enemyState.position.y() / GRID_SIZE - position.y() / GRID_SIZE);
        int distance = dx + dy; // 曼哈顿距离
        
        if (distance < minDistance) {
            minDistance = distance;
            nearestEnemyId = enemyId;
        }
    }
    
    return nearestEnemyId;
}

// 新增：共享敌机信息
void Strategy3::shareEnemyInfo()
{
    // 遍历所有我方无人机
    for (auto it = m_friendlyDrones.constBegin(); it != m_friendlyDrones.constEnd(); ++it) {
        const QString& droneId = it.key();
        const DroneState& droneState = it.value();
        
        // 检查是否有目标敌机
        QString targetEnemyId = m_droneTargetEnemies.value(droneId);
        if (!targetEnemyId.isEmpty() && m_enemyDrones.contains(targetEnemyId)) {
            const DroneState& enemyState = m_enemyDrones[targetEnemyId];
            
            // 如果目标敌机在攻击范围内
            int dx = qAbs(enemyState.position.x() / GRID_SIZE - droneState.position.x() / GRID_SIZE);
            int dy = qAbs(enemyState.position.y() / GRID_SIZE - droneState.position.y() / GRID_SIZE);
            
            if (dx <= ATTACK_RANGE && dy <= ATTACK_RANGE) {
                qDebug() << "[Strategy3] 无人机" << droneId << "发现敌机" << targetEnemyId 
                         << "在攻击范围内，位置:" << enemyState.position << "血量:" << enemyState.hp;
                
                // 如果这个敌机血量较低，将其设为集火目标
                if (enemyState.hp < 50) {
                    m_currentTargetedEnemyId = targetEnemyId;
                    qDebug() << "[Strategy3] 发现低血量敌机" << targetEnemyId << "，设为集火目标";
                }
                
                // 通知其他无人机支援
                for (auto supportIt = m_friendlyDrones.constBegin(); supportIt != m_friendlyDrones.constEnd(); ++supportIt) {
                    const QString& supporterId = supportIt.key();
                    if (supporterId != droneId && !m_droneEngagingStatus[supporterId]) {
                        QPoint supportPos = calculateSupportPosition(enemyState.position, droneState.position);
                        m_droneTargets[supporterId] = supportPos;
                        m_droneTargetEnemies[supporterId] = targetEnemyId;
                        qDebug() << "[Strategy3] 无人机" << supporterId << "收到支援请求，前往支援位置:" << supportPos;
                    }
                }
            }
        }
    }
}
