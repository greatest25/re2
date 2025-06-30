#ifndef DWPLANNER_H
#define DWPLANNER_H

#include <QObject>
#include <QPoint>
#include <QPointF>
#include <QVector>
#include <QMap>
#include <QString>
#include <QDateTime>
#include "dw.h"
#include "../gridmap.h"
#include "../vsoamanager.h"

/**
 * @brief 动态窗口算法规划器
 * 
 * DWPlanner类是DynamicWindow算法的高级包装器，用于与项目中的其他组件集成。
 * 它处理无人机的局部路径规划，并与栅格地图系统、障碍物管理等进行协作。
 */
class DWPlanner : public QObject
{
    Q_OBJECT
public:
    explicit DWPlanner(QObject *parent = nullptr);
    virtual ~DWPlanner();

    /**
     * @brief 初始化规划器
     * @param gridMap 共享的栅格地图指针
     */
    void initialize(GridMap *gridMap);

    /**
     * @brief 设置当前位置和速度
     * @param droneId 无人机ID
     * @param position 当前位置
     * @param velocity 当前速度
     */
    void setCurrentState(const QString &droneId, const QPointF &position, const QPointF &velocity);

    /**
     * @brief 设置目标点
     * @param droneId 无人机ID
     * @param targetPoint 目标点
     */
    void setTargetPoint(const QString &droneId, const QPointF &targetPoint);

    /**
     * @brief 更新障碍物信息
     * @param staticObstacles 静态障碍物信息(山体、雷达等)
     * @param movingObstacles 移动障碍物信息(雷云等)
     */
    void updateObstacles(const QMap<QString, ObstacleInfo> &staticObstacles, 
                         const QMap<QString, ObstacleInfo> &movingObstacles);

    /**
     * @brief 规划速度指令
     * @param droneId 无人机ID
     * @return 计算出的最优速度指令
     */
    QPointF planVelocity(const QString &droneId);

    /**
     * @brief 获取DW算法实例
     * @return DW算法实例的指针
     */
    DynamicWindow* getDWInstance() const;

    /**
     * @brief 设置是否考虑障碍物的标志
     * @param value true表示考虑障碍物，false表示忽略障碍物
     */
    void setConsiderObstacles(bool value);

    /**
     * @brief 设置是否考虑移动障碍物(雷云)的标志
     * @param value true表示考虑移动障碍物，false表示忽略移动障碍物
     */
    void setConsiderMovingObstacles(bool value);
    
    /**
     * @brief 设置移动障碍物的影响范围扩展值
     * @param extendValue 扩展半径值(像素)
     */
    void setMovingObstacleExtension(double extendValue);

signals:
    /**
     * @brief 当规划器计算出新的速度时发出信号
     * @param droneId 无人机ID
     * @param velocity 计算出的速度指令
     * @param scores DW评价函数的各项评分 [heading, dist, velocity, obstacle]
     */
    void velocityComputed(const QString &droneId, const QPointF &velocity, const QVector<double> &scores);

private:
    /**
     * @brief 将所有障碍物信息转换为DW算法需要的格式
     * @param droneId 无人机ID
     * @return 包含位置和半径信息的障碍物列表
     */
    QVector<QPair<QPointF, double>> prepareObstaclesForDW(const QString &droneId);

    /**
     * @brief 检查目标点是否在安全区域
     * @param targetPoint 目标点
     * @return 如果目标点在安全区域，则返回true
     */
    bool isTargetSafe(const QPointF &targetPoint);

    // 基础算法实例
    DynamicWindow *m_dw;
    
    // 栅格地图引用
    GridMap *m_gridMap;

    // 无人机状态
    struct DroneState {
        QPointF position;    // 当前位置
        QPointF velocity;    // 当前速度
        QPointF targetPoint; // 目标位置
        QDateTime lastUpdate;// 最后更新时间
    };
    
    // 存储每个无人机的状态
    QMap<QString, DroneState> m_droneStates;

    // 障碍物信息
    QMap<QString, ObstacleInfo> m_staticObstacles;
    QMap<QString, ObstacleInfo> m_movingObstacles;

    // 设置项
    bool m_considerObstacles;       // 是否考虑障碍物
    bool m_considerMovingObstacles; // 是否考虑移动障碍物(雷云)
    double m_movingObstacleExtension; // 移动障碍物影响范围扩展值
};

#endif // DWPLANNER_H 