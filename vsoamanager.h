#ifndef VSOAMANAGER_H
#define VSOAMANAGER_H

#include <QObject>
#include <QVsoa>
#include <QJsonObject>
#include <QJsonDocument>
#include <QJsonArray>
#include <QDateTime>
#include <QTimer>
#include <QDebug>
#include <functional>

// 定义无人机信息结构体
struct DroneInfo {
    int hp;
    QString status;
    QString team;
    QString uid;
    double vx;
    double vy;
    double x;
    double y;
};

// 定义障碍物信息结构体
struct ObstacleInfo {
    QString id;       // 障碍物ID，如M1, R1, C1
    double r;         // 障碍物半径
    QString type;     // 障碍物类型：mountain, radar, cloud
    double x;         // 障碍物X坐标
    double y;         // 障碍物Y坐标
};

class VsoaManager : public QObject
{
    Q_OBJECT

public:
    explicit VsoaManager(QObject *parent = nullptr);
    ~VsoaManager();

    // 初始化VSOA连接
    void initConnection();
    // 发送控制命令
    void sendControlCommand(const QString &uavId, const QPointF &velocity);

signals:
    // 连接状态信号
    void connectionStatusChanged(bool connected, QString info);
    // 游戏数据更新信号
    void gameDataUpdated(const QMap<QString, DroneInfo> &dronesInfo,
                        const QMap<QString, ObstacleInfo> &obstaclesInfo,
                        const QMap<QString, ObstacleInfo> &staticObstacles,
                        const QMap<QString, ObstacleInfo> &movingObstacles,
                        int gameLeftTime,
                        QString gameStage);
    // 服务器暂停信号
    void serverPaused(QDateTime pauseStartTime);
    // 服务器恢复信号
    void serverResumed(int pausedSeconds);

private slots:
    void onConnected(bool ok, QString info);
    void onDisconnected();
    void onDatagram(QVsoaClient *client, QString url, QVsoaPayload payload);
    void checkServerStatus();

private:
    QVsoaClient client;
    QTimer *serverStatusTimer;
    QDateTime lastDataReceivedTime;
    bool isServerPaused = false;
    QDateTime serverPauseStartTime;

    // 处理游戏数据
    void processGameData(const QString &jsonData);

    // 游戏状态信息
    int gameLeftTime = 0;
    QString gameStage = "init";

    // 存储无人机信息的Map
    QMap<QString, DroneInfo> dronesInfo;

    // 障碍物信息
    QMap<QString, ObstacleInfo> obstaclesInfo;  // 所有障碍物的合并信息
    // 分类存储障碍物信息
    QMap<QString, ObstacleInfo> staticObstacles;  // 存储固定障碍物（山体、雷达）
    QMap<QString, ObstacleInfo> movingObstacles;  // 存储移动障碍物（雷云）

    // 固定障碍物更新间隔（毫秒）
    const int staticObstacleUpdateInterval = 5000;  // 5秒
};

#endif // VSOAMANAGER_H
