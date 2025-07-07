#ifndef GRIDMAP_H
#define GRIDMAP_H
#include <QWidget>
#include <functional>
#include <QDateTime>
#include <QMap>
#include <QtMath>
#include <Qtimer>
#include <QPainter>
#include <QVector>
#include <QPoint>
#include <QColor>
#include <QDebug>
#include <QSharedPointer>
#include <QReadWriteLock>
#include <QMouseEvent>

struct DroneDisplayInfo {
    QPoint pos;  // 无人机位置
    int hp;      // 血量
};

struct movObstaclRange {
    int startRow, endRow, startCol, endCol;
};

// 定义栅格地图的单元格状态
enum GridCellState {
    UNKNOWN = 1,    // 未探测区域 - 白色
    RADAR = 2,      // 雷达障碍物 - 紫色
    MOUNTAIN = 3,   // 山体障碍物 - 绿色
    CLOUD = 4,      // 雷云(移动障碍物) - 黄色
    OVERLAP_CLOUD_RADAR = 5,   // 雷云与雷达重叠 - 新颜色
};

// 定义共享地图类型
typedef QSharedPointer<QVector<QVector<GridCellState>>> SharedGridMap;

// 栅格地图类
class GridMap : public QWidget {
    Q_OBJECT
public:
    explicit GridMap(QWidget *parent = nullptr);
    void initMap();
    void addself(const QString &droneId,double x, double y, int b_hp);
    void addEnemy(QString id, double x, double y,int r_hp);
    void clearDronePositions();
    void addObstacle(QString id,int x, int y, int radius, GridCellState type);
    void clearMovingObjects(QString id);
    void clearStaleObstacles(int timeoutMs = 500);
    bool needsUpdate() const {
            return gridNeedsUpdate || dronesNeedUpdate || obstaclesNeedUpdate || pathNeedsUpdate;//添加路径绘制
        }
    bool showMapInCompetitionMode = true;  //true显示地图（VNC:1921x820），false不显示地图（VNC:642x820）

    //************路径规划相关*************
    // 获取共享地图数据
    SharedGridMap getSharedGridMap() const;
    // 设置路径用于绘制 - 修改为支持指定无人机ID
    void setPath(const QVector<QPoint>& path, const QString& droneId = "B1");
    // 清除路径 - 修改为支持指定无人机ID
    void clearPath(const QString& droneId = "");
    //由私有常量变为公共常量
    static const int MAP_WIDTH = 1280;
    static const int MAP_HEIGHT = 800;
    static const int GRID_SIZE = 10;
    static const int GRID_COLS = 128;
    static const int GRID_ROWS = 80;
    static const int DRONE_VISION_RADIUS = 300;
    static const int ENEMY_ATTACK_RADIUS = 150;
    static const int OBSTACLE_RADIUS = 80;
    static const int CLOUD_RADIUS = 110;  // 雷云特殊半径

    // 存储多个无人机的路径
    QMap<QString, QVector<QPoint>> m_pathMap;
    QMap<QString, QVector<QPointF>> m_smoothedPathMap;


    //**************B样条路径平滑**************

    QVector<QPointF> smoothPathWithBSpline(const QVector<QPoint>& originalPath);
    // B样条辅助方法
    QVector<QPointF> createLinearInterpolation(const QVector<QPointF>& controlPoints);
    QVector<QPointF> createBSplineInterpolation(const QVector<QPointF>& controlPoints);
    QVector<double> generateImprovedKnotVector(int controlPoints, int degree);
    QPointF deBoorAlgorithm(double t, const QVector<QPointF>& controlPoints,
                           const QVector<double>& knots, int degree);

    // 辅助方法：栅格坐标转像素坐标
    QPointF gridToPixel(const QPoint& gridPoint);

    // 静态常量
    static const double TIME_STEP = 0.05;          // 50ms = 0.05s
    static const double MAX_VELOCITY = 2.5;       // 2.5像素/50ms
    static const int LOOKAHEAD_POINTS = 5;      // 前瞻点数
    const int SPLINE_DEGREE = 5;         // B样条阶数

    // 添加计算速度的方法
    QPointF calculateVelocity(const QString &droneId, const QPointF &currentPos);

    bool isPointInCircle(int px, int py, int cx, int cy, int radius);
    // 辅助方法：找到最近的路径点索引
    int findClosestPathPointIndex(const QPointF &currentPos, const QString &droneId);
signals:
    // 地图更新信号
    void mapUpdated(int row, int col, GridCellState newState);
    void mapReset();

    // 目标点设置信号 - 修改为支持无人机ID
    void targetPointSet(const QPoint& targetPoint, const QString& droneId = "B1");

    void targetReached(const QString& droneId);
protected:
    void paintEvent(QPaintEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;//设置目标点

private:

    QVector<QVector<GridCellState>> grid;
    mutable QReadWriteLock gridLock; // 用于保护grid的读写锁

    QMap<QString, DroneDisplayInfo> droneInfos;      // 我方无人机 id -> DroneDisplayInfo
    QMap<QString, DroneDisplayInfo> enemyDroneInfos; // 敌方无人机 id -> DroneDisplayInfo

    bool isCellIntersectCircle(int gridRow, int gridCol, int cx, int cy, int radius);         //计算外接矩形

    QMap<QString, movObstaclRange> movingObstaclRanges; // 存储障碍物范围

    // 分层绘制标志
    bool gridNeedsUpdate = true;  // 栅格层是否需要更新
    bool dronesNeedUpdate = true; // 无人机层是否需要更新
    bool obstaclesNeedUpdate = true; // 障碍物层是否需要更新

    // 缓存的绘制表面
    QPixmap gridLayer;      // 栅格层
    QPixmap dronesLayer;    // 无人机层
    QPixmap staticObstaclesLayer; // 静态障碍物层（山体、雷达）
    QPixmap movingObstaclesLayer; // 移动障碍物层（雷云）

    // 分层绘制方法
    void updateGridLayer(QPainter &painter);
    void updateDronesLayer(QPainter &painter);
    void updateStaticObstaclesLayer(QPainter &painter);
    void updateMovingObstaclesLayer(QPainter &painter);

    QMap<QString, QDateTime> lastObstacleUpdateTime; // 记录每个移动障碍物的最后更新时间

    //*************路径规划相关************
    //路径绘制
    void updatePathLayer(QPainter &painter);//（可以放到分层绘制方法中）
    // 路径绘制层
    QPixmap pathLayer;

    bool pathNeedsUpdate = false;

    //**************B样条路径平滑**************
    // B样条平滑参数
    int m_currentPathIndex;                 // 当前路径索引

    // 辅助方法：计算到下一个路径点的速度
    QPointF calculateVelocityToNextPoint(const QPointF &currentPos, const QPointF &targetPos);

    // 辅助方法：计算前瞻方向
    QPointF calculateLookaheadDirection(int currentIndex, int lookaheadCount, const QString &droneId);

    // 辅助常量
    static const double TARGET_THRESHOLD = 10.0;  // 到达目标点的距离阈值
    static const int LOOKAHEAD = 3;              // 前瞻点数量
    static const double MAX_SPEED = 50.0;         // 最大速度
};
#endif // GRIDMAP_H
