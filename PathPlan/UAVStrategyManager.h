#ifndef UAVSTRATEGYMANAGER_H
#define UAVSTRATEGYMANAGER_H
#include <QDebug>
#include <QtMath>
#include <QObject>
#include <QPoint>
#include <QVector>
#include <QMap>
#include <QString>
#include <QDebug>


// 前向声明
struct DroneInfo;

class UAVStrategyManager : public QObject
{
    Q_OBJECT

public:
    explicit UAVStrategyManager(QObject *parent = nullptr);

    // 策略方法：输入18维向量，输出6维数据（3个无人机的目标点坐标）
    QVector<float> executeStrategy(const QVector<float> &observation);
};

#endif // UAVSTRATEGYMANAGER_H
