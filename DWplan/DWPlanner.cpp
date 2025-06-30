#include "DWPlanner.h"
#include <QDebug>
#include <limits>
#include <QLineF>

DWPlanner::DWPlanner(QObject *parent) : QObject(parent),
    m_dw(nullptr), m_gridMap(nullptr),
    m_considerObstacles(true), m_considerMovingObstacles(true),
    m_movingObstacleExtension(20.0)
{
    // 创建动态窗口算法实例
    m_dw = new DynamicWindow(this);
    
    // 设置合适的默认参数
    m_dw->setMotionParams(50.0, 10.0, 40.0); // 最大速度、加速度、偏航角速度
    m_dw->setWeights(0.15, 0.2, 0.15, 0.5);  // 朝向、距离、速度、障碍物权重
}

DWPlanner::~DWPlanner()
{
    // DynamicWindow由Qt的父子关系自动管理和释放
}

void DWPlanner::initialize(GridMap *gridMap)
{
    m_gridMap = gridMap;
}

void DWPlanner::setCurrentState(const QString &droneId, const QPointF &position, const QPointF &velocity)
{
    DroneState &state = m_droneStates[droneId];
    state.position = position;
    state.velocity = velocity;
    state.lastUpdate = QDateTime::currentDateTime();
}

void DWPlanner::setTargetPoint(const QString &droneId, const QPointF &targetPoint)
{
    if (m_droneStates.contains(droneId)) {
        m_droneStates[droneId].targetPoint = targetPoint;
    } else {
        DroneState state;
        state.targetPoint = targetPoint;
        state.position = QPointF(0, 0);
        state.velocity = QPointF(0, 0);
        state.lastUpdate = QDateTime::currentDateTime();
        m_droneStates[droneId] = state;
    }
}

void DWPlanner::updateObstacles(const QMap<QString, ObstacleInfo> &staticObstacles, 
                              const QMap<QString, ObstacleInfo> &movingObstacles)
{
    m_staticObstacles = staticObstacles;
    m_movingObstacles = movingObstacles;
}

QPointF DWPlanner::planVelocity(const QString &droneId)
{
    // 检查无人机状态是否存在
    if (!m_droneStates.contains(droneId)) {
        qDebug() << "DWPlanner: 无人机状态未找到，droneId=" << droneId;
        return QPointF(0, 0);
    }
    
    const DroneState &state = m_droneStates[droneId];
    
    // 准备障碍物数据
    QVector<QPair<QPointF, double>> obstacles;
    if (m_considerObstacles) {
        obstacles = prepareObstaclesForDW(droneId);
    }
    
    // 执行动态窗口算法规划
    QPointF bestVelocity = m_dw->plan(state.position, state.velocity, state.targetPoint, obstacles);
    
    // 获取评分详情
    QVector<double> scores = m_dw->getLastEvaluation();
    
    // 发送计算结果信号
    emit velocityComputed(droneId, bestVelocity, scores);
    
    return bestVelocity;
}

DynamicWindow* DWPlanner::getDWInstance() const
{
    return m_dw;
}

void DWPlanner::setConsiderObstacles(bool value)
{
    m_considerObstacles = value;
}

void DWPlanner::setConsiderMovingObstacles(bool value)
{
    m_considerMovingObstacles = value;
}

void DWPlanner::setMovingObstacleExtension(double extendValue)
{
    m_movingObstacleExtension = extendValue;
}

QVector<QPair<QPointF, double>> DWPlanner::prepareObstaclesForDW(const QString &droneId)
{
    QVector<QPair<QPointF, double>> obstacles;
    
    // 添加静态障碍物
    for (auto it = m_staticObstacles.constBegin(); it != m_staticObstacles.constEnd(); ++it) {
        const ObstacleInfo &info = it.value();
        obstacles.append(qMakePair(QPointF(info.x, info.y), info.r));
    }
    
    // 添加移动障碍物（雷云）
    if (m_considerMovingObstacles) {
        for (auto it = m_movingObstacles.constBegin(); it != m_movingObstacles.constEnd(); ++it) {
            const ObstacleInfo &info = it.value();
            // 为移动障碍物扩大影响范围
            double extendedRadius = info.r + m_movingObstacleExtension;
            obstacles.append(qMakePair(QPointF(info.x, info.y), extendedRadius));
        }
    }
    
    // 添加其他无人机作为动态障碍物（可选）
    /*
    for (auto it = m_droneStates.constBegin(); it != m_droneStates.constEnd(); ++it) {
        if (it.key() != droneId) { // 不把自己算作障碍物
            const DroneState &otherState = it.value();
            obstacles.append(qMakePair(otherState.position, 20.0)); // 无人机半径20像素
        }
    }
    */
    
    return obstacles;
}

bool DWPlanner::isTargetSafe(const QPointF &targetPoint)
{
    // 检查目标点是否与任何障碍物重叠
    for (auto it = m_staticObstacles.constBegin(); it != m_staticObstacles.constEnd(); ++it) {
        const ObstacleInfo &info = it.value();
        QPointF obstaclePos(info.x, info.y);
        double distance = QLineF(targetPoint, obstaclePos).length();
        
        if (distance < info.r + 10.0) { // 10像素安全余量
            return false;
        }
    }
    
    // 检查移动障碍物
    for (auto it = m_movingObstacles.constBegin(); it != m_movingObstacles.constEnd(); ++it) {
        const ObstacleInfo &info = it.value();
        QPointF obstaclePos(info.x, info.y);
        double distance = QLineF(targetPoint, obstaclePos).length();
        
        if (distance < info.r + m_movingObstacleExtension) {
            return false;
        }
    }
    
    return true;
} 