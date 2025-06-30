#ifndef STRATEGY1_H
#define STRATEGY1_H

#include <QDebug>
#include <QObject>
#include <QPoint>
#include <QMap>
#include <QString>

class Strategy1 : public QObject
{
    Q_OBJECT
public:
    explicit Strategy1(QObject* parent = nullptr);

    // 设置追踪模式
    void setTrackingMode(bool isTracking);

    // 获取当前模式(追踪)
    bool isTrackingMode() const;

    // 更新敌方无人机位置信息
    void updateEnemyPositions(const QMap<QString, QPoint>& enemyPositions);

    // 更新敌方无人机位置和血量信息
    void updateEnemyInfo(const QMap<QString, QPoint>& enemyPositions, const QMap<QString, int>& enemyHp);

    // 为特定无人机单独设置追踪模式
    void setDroneTrackingMode(const QString& droneId, bool isTracking);

    // 获取特定无人机的追踪模式
    bool isDroneTracking(const QString& droneId) const;

    // 获取最优追踪目标点
    QPoint getBestTrackingTarget(const QString& droneId) const;

private:
    // 全局追踪模式标志
    bool m_isTrackingMode;

    // 单个无人机追踪模式标志
    QMap<QString, bool> m_droneTrackingMode;

    // 敌方无人机位置 - 键是敌方无人机ID，值是栅格坐标
    QMap<QString, QPoint> m_enemyPositions;

    // 敌方无人机血量 - 键是敌方无人机ID，值是血量
    QMap<QString, int> m_enemyHp;
};

#endif // STRATEGY1_H
