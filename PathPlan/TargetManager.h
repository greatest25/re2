#ifndef TARGETMANAGER_H
#define TARGETMANAGER_H
#include <QDebug>
#include <QObject>
#include <QPoint>
#include <QMap>
#include <QString>

class TargetManager : public QObject
{
    Q_OBJECT
public:
    explicit TargetManager(QObject* parent = nullptr);

    // 初始化预设目标点
    void initPresetTargets();

    // 获取指定无人机的当前目标点
    QPoint getCurrentTarget(const QString& droneId) const;

    // 获取指定无人机的下一个巡逻目标点
    QPoint getNextPatrolTarget(const QString& droneId);

    // 检查是否有预设目标点
    bool hasPresetTarget(const QString& droneId) const;

    // 设置无人机是否已到达目标点
    void setTargetReached(const QString& droneId, bool reached);

    // 检查无人机是否已到达目标点
    bool isTargetReached(const QString& droneId) const;

    QPoint getPatrolPoint(const QString& droneId);
signals:

private:
    // 存储每个无人机的3个循环目标点
    QMap<QString, QVector<QPoint>> m_patrolTargets;

    // 存储每个无人机当前的目标点索引
    QMap<QString, int> m_currentTargetIndex;

    // 存储无人机是否已到达目标点的状态
    QMap<QString, bool> m_targetReached;
};

#endif // TARGETMANAGER_H
