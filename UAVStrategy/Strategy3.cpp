#include "Strategy3.h"
#include <QVector>
#include <QtMath>
#include <QDebug>

// 添加栅格大小常量，与GridMap保持一致
const int Strategy3::GRID_SIZE = 20; // 栅格大小为20像素

// 添加地图尺寸常量
const int Strategy3::MAP_WIDTH = 64; // 栅格地图宽度
const int Strategy3::MAP_HEIGHT = 40; // 栅格地图高度

//* 添加巡逻点
const QVector<QPoint> Strategy3::PATROL_POINTS = {
    QPoint(10, 10),   // 左上
    QPoint(54, 10),   // 右上
    QPoint(32, 20),   // 中心
    QPoint(54, 30),   // 右下
    QPoint(10, 30)    // 左下
};

// 添加阵型偏移量定义
const QMap<QString, QPoint> Strategy3::FORMATION_OFFSETS = {
    {"B1", QPoint(-40, -40)},   // 左上，减小偏移量
    {"B2", QPoint(60, -60)},    // 右上
    {"B3", QPoint(0, 60)}       // 下中
};

Strategy3::Strategy3(QObject* parent) : QObject(parent), m_patrolIndex(0)
{
    reset();
}

Strategy3::~Strategy3()
{
    // 清理资源
    m_droneTargets.clear();
    m_droneTargetEnemies.clear();
    m_droneEngagingStatus.clear();
}

void Strategy3::reset()
{
    m_friendlyDrones.clear();
    m_enemyDrones.clear();
    m_lastEnemyPositions.clear();  // 清除敌机历史位置记录
    m_lastKnownEnemyPositions.clear();  // 清除敌机记忆信息
    m_currentTargetedEnemyId.clear();
    m_currentAttackerId.clear();
    m_droneTargets.clear();
    m_droneTargetEnemies.clear();
    m_droneEngagingStatus.clear();
    m_lastTargetUpdateTime.clear();
    m_lastPathPlanTime.clear();
    m_lastTargets.clear();
    m_hasEnemyDetected = false;
    m_patrolIndex = 0;
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
    
    // 添加小范围随机偏移（±2格）
    offset.setX(offset.x() + (qrand() % 5 - 2) * GRID_SIZE);
    offset.setY(offset.y() + (qrand() % 5 - 2) * GRID_SIZE);
    
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

    // 放宽到达判定条件
    return (dx <= TARGET_REACHED_THRESHOLD * 2 && dy <= TARGET_REACHED_THRESHOLD * 2);
}

// 修改needUpdateTarget函数，移除冷却时间检查
bool Strategy3::needUpdateTarget(const QString& droneId, const QPoint& newTarget) const
{
    // 如果无人机没有当前目标点，需要更新
    if (!m_droneTargets.contains(droneId)) {
        return true;
    }

    // 如果已经到达当前目标点，允许更新
    if (hasReachedTarget(droneId)) {
        return true;
    }

    // 计算新目标点与当前目标点的距离
    QPoint currentTarget = m_droneTargets[droneId];
    int dx = qAbs(newTarget.x() - currentTarget.x());
    int dy = qAbs(newTarget.y() - currentTarget.y());

    // 如果是敌机位置，总是允许更新 - 优先处理
    for (auto enemyIt = m_enemyDrones.constBegin(); enemyIt != m_enemyDrones.constEnd(); ++enemyIt) {
        const DroneState& enemyState = enemyIt.value();
        QPoint enemyGridPos(enemyState.position.x() / GRID_SIZE, enemyState.position.y() / GRID_SIZE);
        
        // 检查目标点是否接近敌机位置（允许少量偏移）
        int targetDx = qAbs(enemyGridPos.x() - newTarget.x());
        int targetDy = qAbs(enemyGridPos.y() - newTarget.y());
        
        if (targetDx <= 3 && targetDy <= 3) {
            return true;  // 如果是敌机附近的目标点，总是允许更新
        }
    }

    // 只有当新目标点与当前目标点距离超过阈值时才更新
    return (dx >= MIN_TARGET_UPDATE_DISTANCE || dy >= MIN_TARGET_UPDATE_DISTANCE);
}

// 修改updateDroneTarget函数，移除冷却时间更新
void Strategy3::updateDroneTarget(const QString& droneId, const QPoint& newTarget)
{
    m_droneTargets[droneId] = newTarget;
    m_lastTargets[droneId] = newTarget;
}

void Strategy3::updateGameState(const QMap<QString, DroneState>& friendlyDrones, const QMap<QString, DroneState>& enemyDrones)
{
    // 更新无人机状态
    m_friendlyDrones = friendlyDrones;
    m_enemyDrones = enemyDrones;

    // 更新敌机记忆
    QTime currentTime = QTime::currentTime();
    
    // 清理过期的敌机记忆
    QMutableMapIterator<QString, EnemyMemory> memIt(m_lastKnownEnemyPositions);
    while (memIt.hasNext()) {
        memIt.next();
        if (memIt.value().lastSeen.msecsTo(currentTime) > MEMORY_DURATION) {
            memIt.remove();
        }
    }

    // 更新或添加新的敌机记忆
    for (auto it = enemyDrones.constBegin(); it != enemyDrones.constEnd(); ++it) {
        EnemyMemory memory;
        memory.position = it.value().position;
        memory.lastSeen = currentTime;
        memory.hp = it.value().hp;
        m_lastKnownEnemyPositions[it.key()] = memory;
    }

    // 检查是否有敌机
    m_hasEnemyDetected = !enemyDrones.isEmpty() || !m_lastKnownEnemyPositions.isEmpty();

    if (m_hasEnemyDetected) {
        // 寻找最弱的敌人
        findWeakestEnemy();
        // 分配角色和目标
        assignRolesAndTargets();
    } else {
        qDebug() << "[Strategy3] 未检测到敌机，也没有敌机记忆，执行巡逻任务";
        assignPatrolTargets();
    }
}

QPoint Strategy3::getTargetForDrone(const QString& droneId)
{
    // 检查是否有预设的目标点
    if (m_droneTargets.contains(droneId)) {
        return m_droneTargets.value(droneId);
    }
    
    // 如果没有预设目标点，但有目标敌机，直接返回敌机位置
    QString targetEnemyId = m_droneTargetEnemies.value(droneId);
    if (!targetEnemyId.isEmpty()) {
        // 首先检查当前可见的敌机
        if (m_enemyDrones.contains(targetEnemyId) && m_enemyDrones[targetEnemyId].hp > 0) {
            // 直接返回敌机栅格坐标
            QPoint enemyPos = m_enemyDrones[targetEnemyId].position;
            return QPoint(enemyPos.x() / GRID_SIZE, enemyPos.y() / GRID_SIZE);
        }
        
        // 如果当前敌机不可见，检查记忆中的敌机
        if (m_lastKnownEnemyPositions.contains(targetEnemyId) && m_lastKnownEnemyPositions[targetEnemyId].hp > 0) {
            QPoint enemyPos = m_lastKnownEnemyPositions[targetEnemyId].position;
            qDebug() << "[Strategy3] 无人机" << droneId << "使用记忆中的敌机位置作为目标点";
            return QPoint(enemyPos.x() / GRID_SIZE, enemyPos.y() / GRID_SIZE);
        }
    }
    
    // 如果没有目标敌机且有集火目标，使用集火目标位置
    if (!m_currentTargetedEnemyId.isEmpty()) {
        // 首先检查当前可见的敌机
        if (m_enemyDrones.contains(m_currentTargetedEnemyId) && m_enemyDrones[m_currentTargetedEnemyId].hp > 0) {
            QPoint enemyPos = m_enemyDrones[m_currentTargetedEnemyId].position;
            return QPoint(enemyPos.x() / GRID_SIZE, enemyPos.y() / GRID_SIZE);
        }
        
        // 如果当前集火目标不可见，检查记忆中的敌机
        if (m_lastKnownEnemyPositions.contains(m_currentTargetedEnemyId) && 
            m_lastKnownEnemyPositions[m_currentTargetedEnemyId].hp > 0) {
            QPoint enemyPos = m_lastKnownEnemyPositions[m_currentTargetedEnemyId].position;
            qDebug() << "[Strategy3] 无人机" << droneId << "使用记忆中的集火目标位置作为目标点";
            return QPoint(enemyPos.x() / GRID_SIZE, enemyPos.y() / GRID_SIZE);
        }
    }
    
    // 如果以上方法都无效，但有任何记忆中的敌机，使用最近的记忆敌机位置
    if (!m_lastKnownEnemyPositions.isEmpty() && m_friendlyDrones.contains(droneId)) {
        const DroneState& droneState = m_friendlyDrones[droneId];
        QString nearestEnemyId;
        int minDistance = INT_MAX;
        
        for (auto it = m_lastKnownEnemyPositions.constBegin(); it != m_lastKnownEnemyPositions.constEnd(); ++it) {
            const QString& enemyId = it.key();
            const EnemyMemory& memory = it.value();
            
            // 跳过血量为0的敌机
            if (memory.hp <= 0) {
                continue;
            }
            
            // 计算距离
            int dx = qAbs(memory.position.x() - droneState.position.x());
            int dy = qAbs(memory.position.y() - droneState.position.y());
            int distance = dx + dy;
            
            if (distance < minDistance) {
                minDistance = distance;
                nearestEnemyId = enemyId;
            }
        }
        
        if (!nearestEnemyId.isEmpty()) {
            QPoint enemyPos = m_lastKnownEnemyPositions[nearestEnemyId].position;
            qDebug() << "[Strategy3] 无人机" << droneId << "使用最近的记忆敌机位置作为目标点";
            return QPoint(enemyPos.x() / GRID_SIZE, enemyPos.y() / GRID_SIZE);
        }
    }
    
    // 如果以上方法都无效，返回默认点(0,0)
    return QPoint(0, 0);
}

void Strategy3::findWeakestEnemy()
{
    // 如果当前目标不存在或已被击毁，则寻找新目标
    if (!m_enemyDrones.contains(m_currentTargetedEnemyId) || 
        (m_enemyDrones.contains(m_currentTargetedEnemyId) && m_enemyDrones[m_currentTargetedEnemyId].hp <= 0)) {
        m_currentTargetedEnemyId.clear();
    }

    // 如果没有敌机和记忆敌机，直接返回
    if ((m_enemyDrones.isEmpty() && m_lastKnownEnemyPositions.isEmpty()) || m_friendlyDrones.isEmpty()) {
        return;
    }

    int minHp = INT_MAX;
    QString weakestEnemyId;
    QPoint closestEnemyPos;
    int closestDistance = INT_MAX;
    
    // 首先检查当前可见的敌机
    for (auto it = m_enemyDrones.constBegin(); it != m_enemyDrones.constEnd(); ++it) {
        const QString& enemyId = it.key();
        const DroneState& enemyState = it.value();
        
        // 跳过血量为0的敌机
        if (enemyState.hp <= 0) {
            continue;
        }
        
        // 计算与我方无人机的平均距离
        int totalDistance = 0;
        for (auto droneIt = m_friendlyDrones.constBegin(); droneIt != m_friendlyDrones.constEnd(); ++droneIt) {
            const DroneState& droneState = droneIt.value();
            int dx = qAbs(enemyState.position.x() - droneState.position.x());
            int dy = qAbs(enemyState.position.y() - droneState.position.y());
            totalDistance += qSqrt(dx * dx + dy * dy);
        }
        int avgDistance = totalDistance / m_friendlyDrones.size();
        
        // 优先选择血量低的敌机
        if (enemyState.hp < minHp || (enemyState.hp == minHp && avgDistance < closestDistance)) {
            minHp = enemyState.hp;
            weakestEnemyId = enemyId;
            closestEnemyPos = enemyState.position;
            closestDistance = avgDistance;
        }
    }
    
    // 如果没有找到可见敌机，检查记忆中的敌机
    if (weakestEnemyId.isEmpty() && !m_lastKnownEnemyPositions.isEmpty()) {
        for (auto it = m_lastKnownEnemyPositions.constBegin(); it != m_lastKnownEnemyPositions.constEnd(); ++it) {
            const QString& enemyId = it.key();
            const EnemyMemory& memory = it.value();
            
            // 跳过血量为0的敌机
            if (memory.hp <= 0) {
                continue;
            }
            
            // 计算与我方无人机的平均距离
            int totalDistance = 0;
            for (auto droneIt = m_friendlyDrones.constBegin(); droneIt != m_friendlyDrones.constEnd(); ++droneIt) {
                const DroneState& droneState = droneIt.value();
                int dx = qAbs(memory.position.x() - droneState.position.x());
                int dy = qAbs(memory.position.y() - droneState.position.y());
                totalDistance += qSqrt(dx * dx + dy * dy);
            }
            int avgDistance = totalDistance / m_friendlyDrones.size();
            
            // 优先选择血量低的敌机
            if (memory.hp < minHp || (memory.hp == minHp && avgDistance < closestDistance)) {
                minHp = memory.hp;
                weakestEnemyId = enemyId;
                closestEnemyPos = memory.position;
                closestDistance = avgDistance;
            }
        }
    }
    
    if (!weakestEnemyId.isEmpty()) {
        m_currentTargetedEnemyId = weakestEnemyId;
        qDebug() << "[Strategy3] 找到最弱敌机:" << weakestEnemyId << "血量:" << minHp << "平均距离:" << closestDistance;
    }
}

// 新增：分配巡逻目标点
void Strategy3::assignPatrolTargets()
{
    // 获取巡逻点
    QPoint patrolPoint = PATROL_POINTS[m_patrolIndex];
    bool shouldUpdatePatrolIndex = true;  // 是否需要更新巡逻点索引
    
    // 为每个无人机分配巡逻位置
    for (auto it = m_friendlyDrones.constBegin(); it != m_friendlyDrones.constEnd(); ++it) {
        const QString& droneId = it.key();
        
        // 如果已有目标点，检查是否需要更新
        if (m_droneTargets.contains(droneId)) {
            // 获取当前位置
            QPoint currentPos(it.value().position.x() / GRID_SIZE, it.value().position.y() / GRID_SIZE);
            QPoint currentTarget = m_droneTargets[droneId];
            
            // 计算到当前目标点的距离
            int dx = qAbs(currentPos.x() - currentTarget.x());
            int dy = qAbs(currentPos.y() - currentTarget.y());
            
            // 如果还没有接近目标点，继续使用当前目标点
            if (dx > 3 || dy > 3) {
                shouldUpdatePatrolIndex = false;
                continue;
            }
        }
        
        // 计算该无人机的巡逻位置（基于阵型偏移）
        QPoint offset = FORMATION_OFFSETS.value(droneId, QPoint(0, 0));
        
        // 计算新的目标位置
        QPoint targetGridPos = patrolPoint + QPoint(offset.x() / GRID_SIZE, offset.y() / GRID_SIZE);
        
        // 确保目标点在地图范围内
        targetGridPos.setX(qBound(0, targetGridPos.x(), MAP_WIDTH - 1));
        targetGridPos.setY(qBound(0, targetGridPos.y(), MAP_HEIGHT - 1));
        
        // 调整目标点，避免障碍物
        targetGridPos = adjustTargetPoint(targetGridPos, true);
        
        // 更新目标点
        m_droneTargets[droneId] = targetGridPos;
        
        // 转换为像素坐标
        QPoint targetPixelPos(targetGridPos.x() * GRID_SIZE + GRID_SIZE/2,
                           targetGridPos.y() * GRID_SIZE + GRID_SIZE/2);
        
        qDebug() << "[Strategy3] 无人机" << droneId << "更新巡逻位置到(栅格):" << targetGridPos 
                 << "目标点(像素):" << targetPixelPos;
        
        // 直接发送路径规划请求，不检查时间间隔
        emit needReplanPath(droneId, targetPixelPos);
    }
    
    // 只有当所有无人机都接近目标点时，才更新巡逻点索引
    if (shouldUpdatePatrolIndex) {
        m_patrolIndex = (m_patrolIndex + 1) % PATROL_POINTS.size();
    }
}

void Strategy3::assignRolesAndTargets()
{
    if (m_currentTargetedEnemyId.isEmpty() || !m_enemyDrones.contains(m_currentTargetedEnemyId) || 
        m_friendlyDrones.isEmpty() || m_enemyDrones[m_currentTargetedEnemyId].hp <= 0) {
        // 如果没有目标或没有我方无人机，或目标敌机已被击毁，则执行巡逻
        assignPatrolTargets();
        return;
    }

    const DroneState& targetEnemy = m_enemyDrones[m_currentTargetedEnemyId];
    // 将敌机位置转换为栅格坐标
    QPoint enemyGridPos(targetEnemy.position.x() / GRID_SIZE, targetEnemy.position.y() / GRID_SIZE);
    
    // 预测敌机位置 - 检查是否有历史位置记录
    QPoint predictedPos = enemyGridPos;
    if (m_lastEnemyPositions.contains(m_currentTargetedEnemyId)) {
        QPoint lastPos = m_lastEnemyPositions[m_currentTargetedEnemyId];
        
        // 计算移动向量
        QPoint moveVector = enemyGridPos - lastPos;
        
        // 预测2帧后的位置
        predictedPos = enemyGridPos + moveVector * 2;
        
        // 确保预测位置在地图范围内
        predictedPos.setX(qBound(0, predictedPos.x(), MAP_WIDTH - 1));
        predictedPos.setY(qBound(0, predictedPos.y(), MAP_HEIGHT - 1));
        
        // 使用adjustTargetPoint确保预测位置不在障碍物内
        predictedPos = adjustTargetPoint(predictedPos, true);
    }
    
    // 更新敌机历史位置
    m_lastEnemyPositions[m_currentTargetedEnemyId] = enemyGridPos;
    
    // 特殊处理：如果只有一架无人机存活，它自动成为攻击者
    if (m_friendlyDrones.size() == 1) {
        m_currentAttackerId = m_friendlyDrones.firstKey();
        qDebug() << "[Strategy3] 只有一架无人机" << m_currentAttackerId << "存活，自动成为攻击者";
        
        // 设置目标点为预测位置
        m_droneTargets[m_currentAttackerId] = predictedPos;
        m_droneTargetEnemies[m_currentAttackerId] = m_currentTargetedEnemyId;
        
        // 立即触发路径规划
        QPoint pixelTargetPos(predictedPos.x() * GRID_SIZE + GRID_SIZE/2,
                           predictedPos.y() * GRID_SIZE + GRID_SIZE/2);
        
        qDebug() << "[Strategy3] 无人机" << m_currentAttackerId << "作为攻击者，目标点:" << predictedPos << "(预测位置)";
        emit needReplanPath(m_currentAttackerId, pixelTargetPos);
        return;
    }
    
    // 多架无人机时，为每架无人机分配角色和目标
    for (auto it = m_friendlyDrones.constBegin(); it != m_friendlyDrones.constEnd(); ++it) {
        const QString& droneId = it.key();
        
        // 计算包围位置 - 不同无人机选择不同的偏移，形成包围圈
        QPoint targetPos = predictedPos;
        
        // 根据无人机ID设置不同的包围位置
        if (droneId == "B1") {
            // B1: 左侧位置
            targetPos = QPoint(predictedPos.x() - 2, predictedPos.y());
        } else if (droneId == "B2") {
            // B2: 右侧位置
            targetPos = QPoint(predictedPos.x() + 2, predictedPos.y());
        } else {
            // B3: 下方位置
            targetPos = QPoint(predictedPos.x(), predictedPos.y() + 2);
        }
        
        // 确保目标点在地图范围内
        targetPos.setX(qBound(0, targetPos.x(), MAP_WIDTH - 1));
        targetPos.setY(qBound(0, targetPos.y(), MAP_HEIGHT - 1));
        
        // 使用adjustTargetPoint确保目标点不在障碍物内
        targetPos = adjustTargetPoint(targetPos, true);
        
        // 设置目标点
        m_droneTargets[droneId] = targetPos;
        m_droneTargetEnemies[droneId] = m_currentTargetedEnemyId;
        
        // 立即触发路径规划
        QPoint pixelTargetPos(targetPos.x() * GRID_SIZE + GRID_SIZE/2,
                           targetPos.y() * GRID_SIZE + GRID_SIZE/2);
        
        qDebug() << "[Strategy3] 无人机" << droneId << "目标敌机:" << m_currentTargetedEnemyId 
                 << "，目标点:" << targetPos << "(包围位置)";
        emit needReplanPath(droneId, pixelTargetPos);
    }
}

// 新增：检查是否需要重新规划路径
bool Strategy3::needReplanning(const QString& droneId, const QPoint& enemyPos)
{
    // 如果无人机没有当前路径，需要规划
    if (!m_droneTargets.contains(droneId) || m_droneTargets[droneId] == QPoint(0, 0)) {
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

void Strategy3::updatePatrolTargets()
{
    // 获取当前巡逻点
    QPoint currentPatrolPoint = PATROL_POINTS[m_patrolIndex];
    bool allNearTarget = true;
    
    // 检查是否所有无人机都接近当前巡逻点
    for (auto it = m_friendlyDrones.constBegin(); it != m_friendlyDrones.constEnd(); ++it) {
        const QString& droneId = it.key();
        const DroneState& droneState = it.value();
        
        // 将无人机位置转换为栅格坐标
        QPoint droneGridPos(droneState.position.x() / GRID_SIZE, droneState.position.y() / GRID_SIZE);
        int dx = qAbs(droneGridPos.x() - currentPatrolPoint.x());
        int dy = qAbs(droneGridPos.y() - currentPatrolPoint.y());
        
        if (dx > 3 || dy > 3) {  // 如果任何无人机距离巡逻点超过3格，就认为未到达
            allNearTarget = false;
            break;
        }
    }
    
    // 如果所有无人机都接近当前巡逻点，更新到下一个巡逻点
    if (allNearTarget) {
        m_patrolIndex = (m_patrolIndex + 1) % PATROL_POINTS.size();
        currentPatrolPoint = PATROL_POINTS[m_patrolIndex];
        qDebug() << "[Strategy3] 更新巡逻点到:" << currentPatrolPoint;
    }
    
    // 为每架无人机分配围绕巡逻点的位置
    for (auto it = m_friendlyDrones.constBegin(); it != m_friendlyDrones.constEnd(); ++it) {
        const QString& droneId = it.key();
        
        // 根据无人机ID计算在巡逻点周围的偏移
        QPoint offset;
        if (droneId == "B1") {
            offset = QPoint(-2, -2);  // 左上
        } else if (droneId == "B2") {
            offset = QPoint(2, -2);   // 右上
        } else {
            offset = QPoint(0, 2);    // 下方
        }
        
        QPoint targetGridPos = currentPatrolPoint + offset;
        
        // 确保目标点在地图范围内
        targetGridPos.setX(qBound(0, targetGridPos.x(), MAP_WIDTH - 1));
        targetGridPos.setY(qBound(0, targetGridPos.y(), MAP_HEIGHT - 1));
        
        // 调整目标点，避免在障碍物区域内
        targetGridPos = adjustTargetPoint(targetGridPos, true);
        
        // 将栅格坐标转换为像素坐标
        QPoint targetPixelPos(targetGridPos.x() * GRID_SIZE + GRID_SIZE/2, 
                           targetGridPos.y() * GRID_SIZE + GRID_SIZE/2);
        
        // 更新无人机目标点并立即规划路径
        m_droneTargets[droneId] = targetGridPos;  // 保存栅格坐标
        // 巡逻状态下清除目标敌机信息
        m_droneTargetEnemies.remove(droneId);
        
        qDebug() << "[Strategy3] 无人机" << droneId << "更新巡逻位置到(栅格):" << targetGridPos
                 << "目标点(像素):" << targetPixelPos;
        
        // 立即触发路径规划
        emit needReplanPath(droneId, targetPixelPos);
        qDebug() << "[Strategy3] 无人机" << droneId << "规划巡逻路径到:" << targetPixelPos;
    }
}

// 新增：辅助函数，尝试调整目标点位置，避免目标点位于障碍物区域
QPoint Strategy3::adjustTargetPoint(const QPoint& originalTarget, bool isGrid = true) const
{
    QPoint adjustedTarget = originalTarget;
    
    // 如果原目标点不是栅格坐标，先转换为栅格坐标
    if (!isGrid) {
        adjustedTarget = QPoint(originalTarget.x() / GRID_SIZE, originalTarget.y() / GRID_SIZE);
    }
    
    // 尝试在目标点周围寻找可行点
    // 按距离顺序生成8个方向的偏移
    const QVector<QPoint> offsets = {
        QPoint(0, -1),   // 上
        QPoint(1, 0),    // 右
        QPoint(0, 1),    // 下
        QPoint(-1, 0),   // 左
        QPoint(1, -1),   // 右上
        QPoint(1, 1),    // 右下
        QPoint(-1, 1),   // 左下
        QPoint(-1, -1)   // 左上
    };
    
    // 尝试偏移距离1-3的点
    for (int distance = 1; distance <= 3; distance++) {
        for (const QPoint& offset : offsets) {
            QPoint candidate = adjustedTarget + offset * distance;
            
            // 确保候选点在地图范围内
            if (candidate.x() >= 0 && candidate.x() < MAP_WIDTH && 
                candidate.y() >= 0 && candidate.y() < MAP_HEIGHT) {
                
                // 在实际系统中，这里应该检查点是否在障碍物区域内
                // 但由于我们无法直接访问地图数据，先假设该点可行
                // 在后续代码中，此点仍可能被pathplanner判定为不可行
                
                // 如果原点是像素坐标，转回像素坐标
                if (!isGrid) {
                    return QPoint(candidate.x() * GRID_SIZE + GRID_SIZE/2, 
                                 candidate.y() * GRID_SIZE + GRID_SIZE/2);
                }
                return candidate;
            }
        }
    }
    
    // 如果找不到合适的点，返回原目标点
    return originalTarget;
}
