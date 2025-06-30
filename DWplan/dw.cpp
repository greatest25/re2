#include "dw.h"
#include <QLineF>

DynamicWindow::DynamicWindow(QObject *parent) : QObject(parent)
{
    // 初始化默认参数
    m_params.maxSpeed = 50.0;      // 最大速度 50像素/秒
    m_params.maxAccel = 10.0;      // 最大加速度 10像素/秒^2
    m_params.maxYawRate = 40.0;    // 最大偏航角速度 40度/秒
    m_params.dt = 0.1;             // 时间步长 0.1秒
    m_params.vSamples = 5;         // 线速度采样点数
    m_params.wSamples = 7;         // 角速度采样点数
    m_params.safetyMargin = 10.0;  // 安全余量 10像素
    m_params.lookAheadTime = 1.0;  // 前瞻时间 1秒
    
    // 初始化默认权重
    m_weights.heading = 0.15;      // 朝向权重
    m_weights.dist = 0.2;          // 距离权重
    m_weights.velocity = 0.15;     // 速度权重
    m_weights.obstacle = 0.5;      // 障碍物权重（避障最重要）
    
    // 初始化评估结果向量
    m_lastEval = {0.0, 0.0, 0.0, 0.0};
}

void DynamicWindow::setMotionParams(double maxSpeed, double maxAccel, double maxYawRate)
{
    m_params.maxSpeed = maxSpeed;
    m_params.maxAccel = maxAccel;
    m_params.maxYawRate = maxYawRate;
}

void DynamicWindow::setWeights(double headingWeight, double distWeight, double velocityWeight, double obstacleWeight)
{
    m_weights.heading = headingWeight;
    m_weights.dist = distWeight;
    m_weights.velocity = velocityWeight;
    m_weights.obstacle = obstacleWeight;
}

QPointF DynamicWindow::plan(const QPointF &currentPos, const QPointF &currentVel,
                          const QPointF &targetPos, const QVector<QPair<QPointF, double>> &obstacles)
{
    // 获取动态窗口中的所有速度采样点
    QVector<QPointF> velocityWindow = generateVelocityWindow(currentVel);
    
    QPointF bestVel = currentVel; // 默认保持当前速度
    double bestScore = -std::numeric_limits<double>::max(); // 初始化为最小值
    
    // 评估每个速度样本
    for (const QPointF &vel : velocityWindow) {
        double score = evaluateVelocity(vel, currentPos, targetPos, obstacles);
        
        if (score > bestScore) {
            bestScore = score;
            bestVel = vel;
        }
    }
    
    // 如果目标很近，降低速度
    double distToTarget = QLineF(currentPos, targetPos).length();
    if (distToTarget < 20.0) {
        // 根据距离调整速度
        double scalar = qMax(0.1, distToTarget / 20.0);
        bestVel *= scalar;
    }
    
    return bestVel;
}

QPointF DynamicWindow::plan(const QPointF &currentPos, const QPointF &currentVel,
                          const QPointF &targetPos, const QMap<QString, QPair<QPointF, double>> &obstacleMap)
{
    // 将地图格式转换为向量格式
    QVector<QPair<QPointF, double>> obstacles;
    for (auto it = obstacleMap.constBegin(); it != obstacleMap.constEnd(); ++it) {
        obstacles.append(it.value());
    }
    
    return plan(currentPos, currentVel, targetPos, obstacles);
}

QVector<double> DynamicWindow::getLastEvaluation() const
{
    return m_lastEval;
}

QVector<QPointF> DynamicWindow::generateVelocityWindow(const QPointF &currentVel)
{
    QVector<QPointF> velocities;
    
    // 计算当前速度受限于最大加速度和最大速度的动态窗口
    double vmin_x = qMax(-m_params.maxSpeed, currentVel.x() - m_params.maxAccel * m_params.dt);
    double vmax_x = qMin(m_params.maxSpeed, currentVel.x() + m_params.maxAccel * m_params.dt);
    double vmin_y = qMax(-m_params.maxSpeed, currentVel.y() - m_params.maxAccel * m_params.dt);
    double vmax_y = qMin(m_params.maxSpeed, currentVel.y() + m_params.maxAccel * m_params.dt);
    
    // 采样速度空间
    double dv_x = (vmax_x - vmin_x) / (m_params.vSamples - 1);
    double dv_y = (vmax_y - vmin_y) / (m_params.vSamples - 1);
    
    // 生成速度采样网格
    for (int i = 0; i < m_params.vSamples; ++i) {
        for (int j = 0; j < m_params.vSamples; ++j) {
            double vx = vmin_x + i * dv_x;
            double vy = vmin_y + j * dv_y;
            
            // 确保速度的大小不超过最大速度
            double vMagnitude = qSqrt(vx*vx + vy*vy);
            if (vMagnitude > m_params.maxSpeed) {
                double ratio = m_params.maxSpeed / vMagnitude;
                vx *= ratio;
                vy *= ratio;
            }
            
            velocities.append(QPointF(vx, vy));
        }
    }
    
    // 添加当前速度作为候选
    if (!velocities.contains(currentVel)) {
        velocities.append(currentVel);
    }
    
    // 添加零速度作为候选（紧急停止）
    if (!velocities.contains(QPointF(0, 0))) {
        velocities.append(QPointF(0, 0));
    }
    
    return velocities;
}

double DynamicWindow::evaluateVelocity(const QPointF &vel, const QPointF &currentPos,
                                     const QPointF &targetPos, const QVector<QPair<QPointF, double>> &obstacles)
{
    // 计算各项指标的评分
    double headingScore = calcHeadingScore(vel, currentPos, targetPos);
    double distScore = calcDistScore(vel, currentPos, targetPos);
    double velocityScore = calcVelocityScore(vel);
    double obstacleScore = calcObstacleScore(vel, currentPos, obstacles);
    
    // 保存评估结果
    m_lastEval = {headingScore, distScore, velocityScore, obstacleScore};
    
    // 计算加权总分
    double totalScore = m_weights.heading * headingScore +
                        m_weights.dist * distScore +
                        m_weights.velocity * velocityScore +
                        m_weights.obstacle * obstacleScore;
    
    return totalScore;
}

double DynamicWindow::calcHeadingScore(const QPointF &vel, const QPointF &currentPos, const QPointF &targetPos)
{
    // 如果速度为零，没有方向，给一个中等评分
    if (qAbs(vel.x()) < 0.001 && qAbs(vel.y()) < 0.001) {
        return 0.5;
    }
    
    // 计算目标点的方位角
    QLineF targetLine(currentPos, targetPos);
    double targetAngle = targetLine.angle(); // 0度为指向右方(3点钟方向)，逆时针增加
    
    // 计算当前速度的方位角
    QLineF velocityLine(QPointF(0, 0), vel);
    double velocityAngle = velocityLine.angle();
    
    // 计算两个方位角之间的差（取最小角度）
    double angleDiff = qAbs(targetAngle - velocityAngle);
    if (angleDiff > 180.0) angleDiff = 360.0 - angleDiff;
    
    // 角度差转换为[0, 1]范围的得分，0表示方向完全相反，1表示方向完全一致
    double score = 1.0 - angleDiff / 180.0;
    
    return score;
}

double DynamicWindow::calcDistScore(const QPointF &vel, const QPointF &currentPos, const QPointF &targetPos)
{
    // 预测在lookAheadTime时间后的位置
    QPointF predictedPos = currentPos + vel * m_params.lookAheadTime;
    
    // 计算预测位置到目标位置的距离
    double distance = QLineF(predictedPos, targetPos).length();
    
    // 将距离转换为[0, 1]范围的得分
    // 使用一个衰减函数，距离越近，评分越高
    double maxDist = 500.0; // 假设地图尺寸约为1280x800
    double normalizedDist = qMin(distance / maxDist, 1.0);
    double score = 1.0 - normalizedDist;
    
    return score;
}

double DynamicWindow::calcVelocityScore(const QPointF &vel)
{
    // 计算速度大小
    double velocityMagnitude = QLineF(QPointF(0, 0), vel).length();
    
    // 将速度转换为[0, 1]范围的得分，速度越接近最大速度，评分越高
    double score = velocityMagnitude / m_params.maxSpeed;
    
    return score;
}

double DynamicWindow::calcObstacleScore(const QPointF &vel, const QPointF &currentPos,
                                      const QVector<QPair<QPointF, double>> &obstacles)
{
    if (obstacles.isEmpty()) {
        return 1.0; // 无障碍物，评分最高
    }
    
    // 预测轨迹点
    const int predictionSteps = 10;
    QVector<QPointF> trajectory;
    for (int i = 1; i <= predictionSteps; ++i) {
        double t = i * m_params.dt;
        trajectory.append(currentPos + vel * t);
    }
    
    // 计算轨迹到最近障碍物的最小距离
    double minDistance = std::numeric_limits<double>::max();
    
    for (const QPointF &point : trajectory) {
        for (const QPair<QPointF, double> &obstacle : obstacles) {
            QPointF obstaclePos = obstacle.first;
            double obstacleRadius = obstacle.second;
            
            double distance = QLineF(point, obstaclePos).length() - obstacleRadius - m_params.safetyMargin;
            minDistance = qMin(minDistance, distance);
        }
    }
    
    // 如果轨迹与障碍物碰撞，给出最低评分
    if (minDistance <= 0) {
        return 0.0;
    }
    
    // 计算障碍物评分，距离越远，评分越高
    double maxObstacleDist = 200.0; // 假设障碍物影响范围
    double score = qMin(minDistance / maxObstacleDist, 1.0);
    
    return score;
} 