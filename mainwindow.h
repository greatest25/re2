#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include <QTime>
#include <QMainWindow>
#include <QLabel>
#include <functional>
#include <QJsonObject>
#include <QJsonDocument>
#include <QJsonArray>
#include <QDateTime>
#include <QMap>
#include <QtMath>
#include <Qtimer>
#include <QWidget>
#include <QVector>
#include <QPoint>
#include <QPainter>
#include <QColor>
#include "maddpg_agent.h"
#include "gridmap.h"
#include "vsoamanager.h"
#include "PathPlan/pathplanner.h"
#include "PathPlan/TargetManager.h"
#include "PathPlan/UAVStrategyManager.h"
#include "UAVStrategy/Strategy1.h"
#include "UAVStrategy/Strategy2.h"
#include "UAVStrategy/Strategy3.h"
#include "UAVStrategy/Strategy4.h"

// 定义静态障碍物标记结构体
struct StaticObstacleMarker {
    QString id;       // 障碍物ID，如M1, R1
    bool hasAdded;   // 是否已添加到地图中
};

struct CloudVelocityInfo {
    QPointF lastPosition;     // 上一帧位置
    QPointF currentPosition;  // 当前帧位置
    QPointF velocity;         // 计算出的速度向量
    QDateTime lastUpdateTime; // 上次更新时间
    bool hasValidVelocity;    // 是否有有效的速度信息
};

using namespace std::placeholders;

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

signals:
    // 调用路径规划
    void StartfindPath(const QPoint& start, const QPoint& goal, const QString& droneId);
    //更新路径规划中的地图
    void UpdateSharedGridMap(SharedGridMap gridMap);


private slots:
    void setCompetitionMode(bool isCompetition);

    // 添加处理 VsoaManager 信号的槽函数
    void onConnectionStatusChanged(bool connected, QString info);
    void onGameDataUpdated(const QMap<QString, DroneInfo> &dronesInfo,
                          const QMap<QString, ObstacleInfo> &obstaclesInfo,
                          const QMap<QString, ObstacleInfo> &staticObstacles,
                          const QMap<QString, ObstacleInfo> &movingObstacles,
                          int gameLeftTime,
                          QString gameStage);
    void onServerPaused(QDateTime pauseStartTime);
    void onServerResumed(int pausedSeconds);

//*****************算法1 A* 处理路径规划信号的槽函数****************
    //添加处理路径规划信号的槽函数
    void onTargetPointSet(const QPoint& targetPoint, const QString& droneId = "B1"); // 处理目标点设置
    void onPathPlanned(const QVector<QPoint>& path, const QString& droneId);// 处理路径规划完成
    //SO1 SO3处理飞机到达重规划
    void onTargetReached(const QString& droneId);
private:
    // 存储静态障碍物标记的容器
    QMap<QString, StaticObstacleMarker> staticObstacleMarkers;

    Ui::MainWindow *ui;
    VsoaManager *vsoaManager; // VSOA 通信管理器
    // 添加发送控制命令的方法
    void sendControlCommand(const QString &uavId, const QPointF &velocity);
    GridMap *gridMap;

    // 添加更新UI显示的函数
    void updateUIDisplay();
    void clearStaleMovingObjects();

    // 存储无人机信息的Map
    QMap<QString, DroneInfo> dronesInfo;

    // 游戏状态信息
    int gameLeftTime = 0;
    QString gameStage = "init";

    // 障碍物信息
    QMap<QString, ObstacleInfo> obstaclesInfo;  // 所有障碍物的合并信息
    // 分类存储障碍物信息
    QMap<QString, ObstacleInfo> staticObstacles;  // 存储固定障碍物（山体、雷达）
    QMap<QString, ObstacleInfo> movingObstacles;  // 存储移动障碍物（雷云）

    // 标记是否需要重置红方无人机状态更新标签
    bool resetRedDroneUpdateLabels = false;

    // 记录当前这次暂停的时间（秒）
    int currentPauseSeconds = 0;

    // 记录最后一次收到数据的时间
    // 需要移除的成员变量
    bool serverPaused = false;           // 已在 VsoaManager 中实现
    int totalPauseSeconds = 0;          // 应该由 VsoaManager 管理
    QDateTime serverPauseStartTime;      // 已在 VsoaManager 中实现

    //定时清除移动障碍物的栅格显示
    QTimer *movingObjectsTimer;

    // 当前选择的策略（1-4对应SO1-SO4）
    int currentStrategy = 1;
    // 策略按钮槽函数
    void onStrategyButtonClicked();
    void updateStrategyButtonsState();

//*****************策略1、策略3使用 A* 算法公用槽函数等****************
    TargetManager *targetManager;
    PathPlanner *pathPlanner;
    QPoint m_startPoint;           // 存储当前选择的起点和终点
    QPoint m_targetPoint;
    bool m_hasValidStartPoint = false;
    bool m_hasValidTargetPoint = false;
    void initializePathPlanner();
    void cleanupPathPlanner();
    bool isPathPlannerInitialized;

    //先按预设目标点飞行，并构建18维向量得到我方无人机的目标点进行路径规划
    void planPathToPresetTargets();
    // 添加策略管理器
    UAVStrategyManager *strategyManager;

    // 添加构建18维向量的方法
    QVector<float> buildObservationVector();
    // 存储雷云位置信息
    struct stacleInfo {
            int x;
            int y;
            int radius;
        };
    QMap<QString, stacleInfo> staclePositions; // 存储雷云ID和位置信息

    // 添加检查路径是否在雷云范围内的方法
    bool isPathInObstacle(const QString &droneId);

    QMap<QString, QTime> m_lastPathPlanTime; // 记录每个无人机上次路径规划的时间

    // 为单个无人机规划路径
    void planPathForSingleDrone_S1(const QString &droneId);   //SO1
    void planPathForSingleDrone_S3(const QString &droneId);   //SO3
//*****************算法2 maddpg算法uavmodal相关的槽函数****************
    UAVModel *model;
    QMap<QString, UAVModel*> uavModels;        // 添加UAVModel成员变量，用于每个蓝方无人机
    QVector<float> buildObservationForUAV(const QString &uavId);
    void initializeMADDPG();
    void initializeUAVModels();
    void cleanupMADDPG();
    // 添加构建观察向量的辅助方法
    bool isMaddpgInitialized;
    void calculateCloudVelocities();

    // 存储雷云速度信息
    QMap<QString, CloudVelocityInfo> cloudVelocities;

//    // 添加计算目标距离和角度的方法
//    void calculateTargetInfo(const QPointF &selfPos, QVector<QPointF> &enemyPos,
//                            float &targetDistance, float &targetAngle);

    // UAV策略类，处理追踪逻辑
    Strategy1 *SO1;
    Strategy2 *SO2;
    Strategy3 *SO3;
    Strategy4 *SO4;
    int blueAliveCount; // 我方存活数量

    PathPlanner *m_pathPlanner; // 添加路径规划器
    UAVModel *maddpgAgent;

    // 简化版的A*路径规划算法
    QVector<QPoint> findPath(const QPoint& start, const QPoint& goal, const QVector<QPoint>& obstacles, int mapWidth, int mapHeight);
    float heuristic(const QPoint& a, const QPoint& b);
    QVector<QPoint> reconstructPath(const QHash<QPoint, QPoint>& cameFrom, QPoint current);

    // 新增：处理Strategy3的needReplanPath信号
    void onStrategy3NeedReplanPath(const QString& droneId, const QPoint& targetPoint);
};
#endif // MAINWINDOW_H

