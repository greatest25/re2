#ifndef PATHPLANNER_H
#define PATHPLANNER_H

#include <QObject>
#include <QPoint>
#include <QVector>
#include <QHash>
#include <QSet>
#include <QPair>
#include <QQueue>
#include <functional>
#include <queue>
#include <QDebug>
#include <cmath>
#include <QSharedPointer>
#include <QReadWriteLock>
#include <QThreadPool>
#include <QtConcurrent>

// 引入GridCellState枚举
#include "../gridmap.h"

// 为QPoint添加qHash函数实现（Qt没有为QPoint类型提供默认的qHash函数实现：QHash<QPoint, float> 和 QHash<QPoint, QPoint>）
inline uint qHash(const QPoint &key, uint seed = 0)
{
    return qHash(key.x(), seed) ^ qHash(key.y(), seed);
}


class PathPlanner : public QObject
{
    Q_OBJECT
public:
    explicit PathPlanner(QObject* parent = nullptr);

    //***************共享地图**************
    // 设置共享地图指针
    void setSharedGridMap(SharedGridMap gridMap);
    // 验证当前地图是否和gridmap一致
    void debugCheckMapConsistency(int row, int col);

    //***************A*算法****************
    // A*路径规划方法
    QVector<QPoint> findPath(const QPoint& start, const QPoint& goal, const QString& droneId);
    // 检查点是否有效（不是障碍物且在地图范围内）
    bool isValidPoint(int row, int col) const;
    // 检查是否可以沿对角线移动
    bool canMoveDiagonally(int startRow, int startCol, int endRow, int endCol) const;
    // 计算两点之间的启发式距离（欧几里得距离）
    float heuristic(const QPoint& a, const QPoint& b) const;
    // 获取点的移动成本
    float getMovementCost(int row, int col) const;

    // 重置逃离状态（游戏结束时调用）
    void resetEscapeState();
signals:
    // 路径规划完成信号
    void pathPlanned(const QVector<QPoint>& path, const QString& droneId);

public slots:
    // 地图单元格更新槽函数
    void onMapUpdated(int row, int col, GridCellState newState);

    // 地图重置槽函数
    void onMapReset();

    //栅格地图更新槽函数
    void onUpdateSharedGridMap(SharedGridMap gridMap);

    //调用路径规划
    void onStartfindPath(const QPoint& start, const QPoint& goal, const QString& droneId);

private:
    int m_mapWidth;  // 地图宽度
    int m_mapHeight; // 地图高度

    //***************共享地图**************
    SharedGridMap m_sharedGridMap; // 共享地图指针
    mutable QReadWriteLock m_mapLock; // 用于保护地图访问的读写锁

    //***************线程池**************
    QThreadPool *m_threadPool; // 添加线程池成员变量

    //***************A*算法****************
    // 障碍物成本映射
    QHash<GridCellState, float> m_obstacleCosts;
    // 初始化障碍物成本
    void initObstacleCosts();
    // 寻找最近的非障碍物点作为逃离点
    QPoint findNearestNonObstaclePoint(const QPoint& start, const QString& droneId) const;
    QPoint m_escapePoint;
    QHash<QString, QVector<QPoint>> m_escapePaths; // 存储每个无人机的逃离路径
};

#endif // PATHPLANNER_H
