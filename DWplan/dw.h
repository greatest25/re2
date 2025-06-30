#ifndef DW_H
#define DW_H

#include <QObject>
#include <QPoint>
#include <QPointF>
#include <QVector>
#include <QDebug>
#include <QtMath>
#include <QMap>

/**
 * @brief 动态窗口法类 (Dynamic Window Approach)
 *
 * 该类实现了动态窗口法算法，用于无人机的局部路径规划和避障。
 * 本算法考虑了无人机的运动约束，对环境中的障碍物进行评估，
 * 并选择最优的速度指令，使无人机能够安全地到达目标点。
 */
class DynamicWindow : public QObject
{
    Q_OBJECT

public:
    /**
     * @brief 构造函数
     * @param parent 父对象指针
     */
    explicit DynamicWindow(QObject *parent = nullptr);

    /**
     * @brief 设置无人机运动参数
     * @param maxSpeed 最大速度
     * @param maxAccel 最大加速度
     * @param maxYawRate 最大偏航角速度
     */
    void setMotionParams(double maxSpeed, double maxAccel, double maxYawRate);

    /**
     * @brief 设置算法权重参数
     * @param headingWeight 朝向权重
     * @param distWeight 距离权重
     * @param velocityWeight 速度权重
     * @param obstacleWeight 障碍物权重
     */
    void setWeights(double headingWeight, double distWeight, double velocityWeight, double obstacleWeight);

    /**
     * @brief 规划局部路径
     * @param currentPos 当前位置
     * @param currentVel 当前速度
     * @param targetPos 目标位置
     * @param obstacles 障碍物列表，每个障碍物由位置和半径定义
     * @return 最优速度指令
     */
    QPointF plan(const QPointF &currentPos, const QPointF &currentVel, 
                 const QPointF &targetPos, const QVector<QPair<QPointF, double>> &obstacles);

    /**
     * @brief 规划局部路径 (重载版本，接受障碍物地图)
     * @param currentPos 当前位置
     * @param currentVel 当前速度
     * @param targetPos 目标位置
     * @param obstacleMap 障碍物地图，键为ID，值为包含位置和半径的结构
     * @return 最优速度指令
     */
    QPointF plan(const QPointF &currentPos, const QPointF &currentVel,
                 const QPointF &targetPos, const QMap<QString, QPair<QPointF, double>> &obstacleMap);

    /**
     * @brief 获取上次规划后的评估数据
     * @return 包含各评估指标的向量 [heading_score, dist_score, velocity_score, obstacle_score]
     */
    QVector<double> getLastEvaluation() const;

private:
    // 动态窗口参数
    struct DWParams {
        double maxSpeed;    // 最大速度
        double maxAccel;    // 最大加速度
        double maxYawRate;  // 最大偏航角速度
        double dt;          // 时间步长
        int vSamples;       // 线速度采样点数
        int wSamples;       // 角速度采样点数
        double safetyMargin;// 安全余量
        double lookAheadTime; // 前瞻时间
    };
    
    // 评分权重
    struct DWWeights {
        double heading;     // 朝向权重
        double dist;        // 距离权重
        double velocity;    // 速度权重
        double obstacle;    // 障碍物权重
    };

    DWParams m_params;      // 动态窗口参数
    DWWeights m_weights;    // 评分权重
    QVector<double> m_lastEval; // 上次评估结果

    /**
     * @brief 生成动态窗口的速度空间
     * @param currentVel 当前速度
     * @return 速度采样点列表
     */
    QVector<QPointF> generateVelocityWindow(const QPointF &currentVel);

    /**
     * @brief 评估速度方案
     * @param vel 待评估的速度
     * @param currentPos 当前位置
     * @param targetPos 目标位置
     * @param obstacles 障碍物列表
     * @return 评分
     */
    double evaluateVelocity(const QPointF &vel, const QPointF &currentPos, 
                           const QPointF &targetPos, 
                           const QVector<QPair<QPointF, double>> &obstacles);

    /**
     * @brief 计算朝向评分
     * @param vel 速度
     * @param currentPos 当前位置
     * @param targetPos 目标位置
     * @return 评分
     */
    double calcHeadingScore(const QPointF &vel, const QPointF &currentPos, const QPointF &targetPos);

    /**
     * @brief 计算距离评分
     * @param vel 速度
     * @param currentPos 当前位置
     * @param targetPos 目标位置
     * @return 评分
     */
    double calcDistScore(const QPointF &vel, const QPointF &currentPos, const QPointF &targetPos);

    /**
     * @brief 计算速度评分
     * @param vel 速度
     * @return 评分
     */
    double calcVelocityScore(const QPointF &vel);

    /**
     * @brief 计算障碍物评分
     * @param vel 速度
     * @param currentPos 当前位置
     * @param obstacles 障碍物列表
     * @return 评分
     */
    double calcObstacleScore(const QPointF &vel, const QPointF &currentPos, 
                            const QVector<QPair<QPointF, double>> &obstacles);
};

#endif // DW_H 