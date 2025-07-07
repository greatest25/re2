#include "gridmap.h"

// GridMap 类实现

GridMap::GridMap(QWidget *parent) : QWidget(parent) {
    // 初始化栅格地图
    grid.resize(GRID_ROWS);
    for (int i = 0; i < GRID_ROWS; i++) {
        grid[i].resize(GRID_COLS);
        for (int j = 0; j < GRID_COLS; j++) {
            grid[i][j] = UNKNOWN;
        }
    }

    // 设置背景色
    setAutoFillBackground(true);
    QPalette pal = palette();
    pal.setColor(QPalette::Window, Qt::black);
    setPalette(pal);

    // 设置大小策略
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    // 初始化图层
    gridLayer = QPixmap(MAP_WIDTH, MAP_HEIGHT);
    dronesLayer = QPixmap(MAP_WIDTH, MAP_HEIGHT);
    staticObstaclesLayer = QPixmap(MAP_WIDTH, MAP_HEIGHT);
    movingObstaclesLayer = QPixmap(MAP_WIDTH, MAP_HEIGHT);


    // 初始化所有图层为透明
    gridLayer.fill(Qt::transparent);
    dronesLayer.fill(Qt::transparent);
    staticObstaclesLayer.fill(Qt::transparent);
    movingObstaclesLayer.fill(Qt::transparent);

    //**************路径规划相关**************
    // 初始化路径层
    pathLayer = QPixmap(MAP_WIDTH, MAP_HEIGHT);
    pathLayer.fill(Qt::transparent);//路径层初始化

    //**************B样条路径平滑**************
    // 重置当前索引
    m_currentPathIndex = 0;
}


void GridMap::initMap() {             // 重置地图为全白色方格

    QWriteLocker locker(&gridLock); // 使用写锁保护修改
    for (int i = 0; i < GRID_ROWS; i++) {
        for (int j = 0; j < GRID_COLS; j++) {
            grid[i][j] = UNKNOWN;
        }
    }
    gridNeedsUpdate = true;
    obstaclesNeedUpdate = true;
    update();  // 这里需要完全重绘

    // 发送地图重置信号
    emit mapReset();
    m_pathMap.clear();
    m_smoothedPathMap.clear();
    pathNeedsUpdate = true;
}


void GridMap::addself(const QString &droneId,double x, double y, int b_hp) {           //实时更新记录我方UAV位置等信息
    droneInfos[droneId] = DroneDisplayInfo{QPoint(x, y), b_hp};
    dronesNeedUpdate = true;  // 只设置标志，不立即重绘
}

void GridMap::addEnemy(QString id, double x, double y,int r_hp) {             //探测到后记录敌方UAV位置等信息
    enemyDroneInfos[id] = DroneDisplayInfo{QPoint(x, y), r_hp};
    dronesNeedUpdate = true;  // 只设置标志，不立即重绘
}

void GridMap::clearDronePositions() {          // 清空所有无人机位置
    droneInfos.clear();
    enemyDroneInfos.clear();
    dronesNeedUpdate = true;
}


void GridMap::addObstacle(QString id,int x, int y, int radius, GridCellState type) {        //在栅格地图中更新障碍物表示
    clearMovingObjects(id);
    // 计算障碍物范围
    movingObstaclRanges[id].startRow = qMax(0, (y - radius) / GRID_SIZE - 1);
    movingObstaclRanges[id].endRow = qMin(GRID_ROWS - 1, (y + radius) / GRID_SIZE + 1);
    movingObstaclRanges[id].startCol = qMax(0, (x - radius) / GRID_SIZE - 1);
    movingObstaclRanges[id].endCol = qMin(GRID_COLS - 1, (x + radius) / GRID_SIZE + 1);

    // 更新障碍物的最后更新时间
    if (type == CLOUD) {
        lastObstacleUpdateTime[id] = QDateTime::currentDateTime();
    }

    QWriteLocker locker(&gridLock); // 使用写锁保护修改
    for (int i = movingObstaclRanges[id].startRow; i <= movingObstaclRanges[id].endRow; i++) {
        for (int j = movingObstaclRanges[id].startCol; j <= movingObstaclRanges[id].endCol; j++) {
            // 检查格子是否与圆相交
            if (isCellIntersectCircle(i, j, x, y, radius)) {
                // 山体优先级最高，如果已经是山体则不改变
                if (grid[i][j] == MOUNTAIN) {
//                    qDebug()<<"设置方格("<<i<<j<<")为山体";
                    continue;
                }


//                // 处理雷云和雷达重合的情况，显示雷达颜色（掉血更多）
//                if (type == CLOUD && grid[i][j] == RADAR) {
//                    // 保持为雷达颜色，不做改变
//                    continue;
//                }

                // 处理雷云和雷达重合的情况
                if (type == CLOUD && grid[i][j] == RADAR) {
                    grid[i][j] = OVERLAP_CLOUD_RADAR; // 设置为重叠状态
//                    qDebug()<<"设置方格("<<i<<j<<")为雷云和雷达重合区域";
                    continue;
                }

                // 其他情况正常设置
                GridCellState oldState = grid[i][j];
                grid[i][j] = type;

                // 只有当状态发生变化时才发送信号
                if (oldState != type) {
                    emit mapUpdated(i, j, type); // 发送单元格更新信号
                }
            }
        }
    }
    obstaclesNeedUpdate = true;  // 只设置标志，不立即重绘
}


void GridMap::clearMovingObjects(QString id) {                  // 清除所有移动物体
    QWriteLocker locker(&gridLock); // 使用写锁保护修改
    for (int i = movingObstaclRanges[id].startRow; i <= movingObstaclRanges[id].endRow; i++) {
        for (int j = movingObstaclRanges[id].startCol; j <= movingObstaclRanges[id].endCol; j++) {
            if (grid[i][j] == CLOUD) {
                grid[i][j] = UNKNOWN;
                emit mapUpdated(i, j, UNKNOWN); // 发送单元格更新信号
            }else if(grid[i][j] == OVERLAP_CLOUD_RADAR)
            {
                grid[i][j] = RADAR;
                emit mapUpdated(i, j, RADAR); // 发送单元格更新信号
            }
        }
    }
    obstaclesNeedUpdate = true;  // 只设置标志，不立即重绘
}

void GridMap::clearStaleObstacles(int timeoutMs) {           //清除显示超时的移动障碍物
    QDateTime currentTime = QDateTime::currentDateTime();
    QList<QString> staleObstacles;

    // 找出所有超时的移动障碍物
    for (auto it = lastObstacleUpdateTime.begin(); it != lastObstacleUpdateTime.end(); ++it) {
        if (it.value().msecsTo(currentTime) > timeoutMs) {
            staleObstacles.append(it.key());
        }
    }

    // 清除超时的移动障碍物
    for (const QString &id : staleObstacles) {
        clearMovingObjects(id);
        lastObstacleUpdateTime.remove(id);
    }
}


void GridMap::paintEvent(QPaintEvent *event) {           //分层绘制栅格地图内容
    if (!showMapInCompetitionMode) {
        // 比赛模式且不显示地图，直接返回
        return;
    }

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);

    // 计算缩放比例
    double scaleX = static_cast<double>(width()) / MAP_WIDTH;
    double scaleY = static_cast<double>(height()) / MAP_HEIGHT;
    double scale = qMin(scaleX, scaleY);

    // 计算偏移量
    int offsetX = (width() - MAP_WIDTH * scale) / 2;
    int offsetY = (height() - MAP_HEIGHT * scale) / 2;

    // 设置变换
    painter.translate(offsetX, offsetY);
    painter.scale(scale, scale);

    // 更新需要重绘的图层
    if (gridNeedsUpdate) {
        updateGridLayer(painter);
        gridNeedsUpdate = false;
    }

    if (obstaclesNeedUpdate) {
        updateStaticObstaclesLayer(painter);
        updateMovingObstaclesLayer(painter);
        obstaclesNeedUpdate = false;
    }

    if (dronesNeedUpdate) {
        updateDronesLayer(painter);
        dronesNeedUpdate = false;
    }
        //新增：路径绘制相关
    if (pathNeedsUpdate) {
        updatePathLayer(painter);
        pathNeedsUpdate = false;
    }

    // 绘制所有图层（注意顺序：先网格，再静态障碍物，然后移动障碍物，路径，最后无人机）
    painter.drawPixmap(0, 0, gridLayer);
    painter.drawPixmap(0, 0, staticObstaclesLayer);
    painter.drawPixmap(0, 0, movingObstaclesLayer);
    painter.drawPixmap(0, 0, pathLayer);//新增：路径绘制相关
    painter.drawPixmap(0, 0, dronesLayer);
}

void GridMap::updateGridLayer(QPainter &painter) {         // 实现分层绘制方法画网格
    // 清空图层
    gridLayer.fill(Qt::transparent);
    QPainter layerPainter(&gridLayer);
    layerPainter.setRenderHint(QPainter::Antialiasing, false);

    // 绘制栅格
    int cellWidth = MAP_WIDTH / GRID_COLS;
    int cellHeight = MAP_HEIGHT / GRID_ROWS;

    for (int i = 0; i < GRID_ROWS; i++) {
        for (int j = 0; j < GRID_COLS; j++) {
            QRect rect(j * cellWidth, i * cellHeight, cellWidth, cellHeight);

            // 只绘制背景色（白色）
            layerPainter.fillRect(rect, Qt::white);

            // 绘制网格线
            layerPainter.setPen(Qt::lightGray);
            layerPainter.drawRect(rect);
        }
    }
}

void GridMap::updateStaticObstaclesLayer(QPainter &painter) {
    // 清空图层
    staticObstaclesLayer.fill(Qt::transparent);
    QPainter layerPainter(&staticObstaclesLayer);
    layerPainter.setRenderHint(QPainter::Antialiasing, false);

    // 绘制静态障碍物（山体、雷达）
    int cellWidth = MAP_WIDTH / GRID_COLS;
    int cellHeight = MAP_HEIGHT / GRID_ROWS;

    for (int i = 0; i < GRID_ROWS; i++) {
        for (int j = 0; j < GRID_COLS; j++) {
            if (grid[i][j] == RADAR) {
                QRect rect(j * cellWidth, i * cellHeight, cellWidth, cellHeight);
                layerPainter.fillRect(rect, Qt::magenta);
            } else if (grid[i][j] == MOUNTAIN) {
                QRect rect(j * cellWidth, i * cellHeight, cellWidth, cellHeight);
                layerPainter.fillRect(rect, Qt::green);
            }
        }
    }


}

void GridMap::updateMovingObstaclesLayer(QPainter &painter) {
    // 清空图层
    movingObstaclesLayer.fill(Qt::transparent);
    QPainter layerPainter(&movingObstaclesLayer);
    layerPainter.setRenderHint(QPainter::Antialiasing, false);

    // 绘制移动障碍物（雷云）
    int cellWidth = MAP_WIDTH / GRID_COLS;
    int cellHeight = MAP_HEIGHT / GRID_ROWS;

    for (int i = 0; i < GRID_ROWS; i++) {
        for (int j = 0; j < GRID_COLS; j++) {
            if (grid[i][j] == CLOUD) {
                QRect rect(j * cellWidth, i * cellHeight, cellWidth, cellHeight);
                layerPainter.fillRect(rect, Qt::yellow);
            } else if (grid[i][j] == OVERLAP_CLOUD_RADAR) {
                QRect rect(j * cellWidth, i * cellHeight, cellWidth, cellHeight);
                // 使用黄色，让雷云在视觉上覆盖雷达
                layerPainter.fillRect(rect, QColor(255, 255, 0, 128)); // 半透明黄色
            }
        }
    }
}

void GridMap::updateDronesLayer(QPainter &painter) {              //绘制无人机层
    // 清空图层
    dronesLayer.fill(Qt::transparent);
    QPainter layerPainter(&dronesLayer);
    layerPainter.setRenderHint(QPainter::Antialiasing, true);

    // 绘制我方无人机位置和范围
    for (auto it = droneInfos.begin(); it != droneInfos.end(); ++it) {
        // 绘制无人机位置（蓝色实心圆）
        layerPainter.setBrush(Qt::blue);
        layerPainter.setPen(Qt::NoPen);
        layerPainter.drawEllipse(it.value().pos, 2, 2);

        // 绘制ID和血量, 位置在无人机旁边(例如右上)
        layerPainter.setFont(QFont("Arial", 10, QFont::Bold));
        layerPainter.setPen(QColor(0, 64, 200)); // 蓝色
        QString text = QString("%1 %2").arg(it.key()).arg(it.value().hp);

        // 在无人机点的右上角10, -10处
        layerPainter.drawText(it.value().pos + QPoint(10, -10), text);

        // 绘制攻击范围（浅红粉色细线圆）
        layerPainter.setBrush(Qt::NoBrush);
        layerPainter.setPen(QPen(QColor(255, 150, 150), 1));
        layerPainter.drawEllipse(it.value().pos, ENEMY_ATTACK_RADIUS, ENEMY_ATTACK_RADIUS);

        // 绘制探测范围（浅蓝绿色虚线圆）
        QPen dottedPen(QColor(0, 200, 200), 1);
        dottedPen.setStyle(Qt::DashLine);
        layerPainter.setBrush(Qt::NoBrush);
        layerPainter.setPen(dottedPen);
        layerPainter.drawEllipse(it.value().pos, DRONE_VISION_RADIUS, DRONE_VISION_RADIUS);
    }

    // 绘制敌方无人机位置和范围
    for (auto it = enemyDroneInfos.begin(); it != enemyDroneInfos.end(); ++it) {
        // 绘制无人机位置（红色实心圆）
        layerPainter.setBrush(Qt::red);
        layerPainter.setPen(Qt::NoPen);
        layerPainter.drawEllipse(it.value().pos, 2, 2);

        layerPainter.setFont(QFont("Arial", 10, QFont::Bold));
        layerPainter.setPen(QColor(200, 0, 0)); // 红色
        QString text = QString("%1 %2").arg(it.key()).arg(it.value().hp);

        layerPainter.drawText(it.value().pos + QPoint(10, -10), text);

        // 绘制攻击范围（红色细线圆）
        layerPainter.setBrush(Qt::NoBrush);
        layerPainter.setPen(QPen(Qt::red, 1));
        layerPainter.drawEllipse(it.value().pos, ENEMY_ATTACK_RADIUS, ENEMY_ATTACK_RADIUS);

        // 绘制探测范围（蓝色虚线圆）
        QPen dottedPen(Qt::blue, 1);
        dottedPen.setStyle(Qt::DashLine);
        layerPainter.setBrush(Qt::NoBrush);
        layerPainter.setPen(dottedPen);
        layerPainter.drawEllipse(it.value().pos, DRONE_VISION_RADIUS, DRONE_VISION_RADIUS);
    }
}


bool GridMap::isPointInCircle(int px, int py, int cx, int cy, int radius) {          // 计算点到圆心的距离
    int dx = px - cx;
    int dy = py - cy;
    return (dx * dx + dy * dy) <= (radius * radius);
}


bool GridMap::isCellIntersectCircle(int gridRow, int gridCol, int cx, int cy, int radius) {        // 检查格子是否与圆相交（圆外接马赛克圆）
    // 计算格子的四个角点坐标
    int cellLeft = gridCol * GRID_SIZE;
    int cellRight = (gridCol + 1) * GRID_SIZE;
    int cellTop = gridRow * GRID_SIZE;
    int cellBottom = (gridRow + 1) * GRID_SIZE;

    // 检查格子的四个角点是否有任何一个在圆内
    if (isPointInCircle(cellLeft, cellTop, cx, cy, radius) ||
        isPointInCircle(cellRight, cellTop, cx, cy, radius) ||
        isPointInCircle(cellLeft, cellBottom, cx, cy, radius) ||
        isPointInCircle(cellRight, cellBottom, cx, cy, radius)) {
        return true;
    }

    // 检查圆心是否在格子内
    if (cx >= cellLeft && cx <= cellRight && cy >= cellTop && cy <= cellBottom) {
        return true;
    }

    // 检查圆是否与格子的边相交
    // 计算圆心到格子最近点的距离
    int closestX = qMax(cellLeft, qMin(cx, cellRight));
    int closestY = qMax(cellTop, qMin(cy, cellBottom));

    return isPointInCircle(closestX, closestY, cx, cy, radius);
}

//***************共享地图**************
// 获取共享地图数据
SharedGridMap GridMap::getSharedGridMap() const {
    QReadLocker locker(&gridLock); // 使用读锁保护访问
    return QSharedPointer<QVector<QVector<GridCellState>>>::create(grid);
}

//************路径绘制相关*****************
// 设置路径用于绘制
void GridMap::setPath(const QVector<QPoint>& path, const QString& droneId) {
    if (path.isEmpty()) {
        return;
    }

    // 更新原始路径
    m_pathMap[droneId] = path;

    // 平滑路径
    m_smoothedPathMap[droneId] = smoothPathWithBSpline(path);

    // 设置更新标志
    pathNeedsUpdate = true;
}

// 清除路径
void GridMap::clearPath(const QString& droneId) {
    if (droneId.isEmpty()) {
        // 清除所有路径
        m_pathMap.clear();
        m_smoothedPathMap.clear();
    } else {
        // 清除指定无人机的路径
        m_pathMap.remove(droneId);
        m_smoothedPathMap.remove(droneId);
    }

    pathNeedsUpdate = true;
    update(); // 触发重绘
}

//// 实现路径层的绘制
//void GridMap::updatePathLayer(QPainter &painter) {
//    // 清空图层
//    pathLayer.fill(Qt::transparent);
//    QPainter layerPainter(&pathLayer);
//    layerPainter.setRenderHint(QPainter::Antialiasing, true);

//    // 如果路径为空，直接返回
//    if (m_currentPath.isEmpty()) {
//        return;
//    }

//    // 设置路径绘制样式
//    QPen pathPen(QColor(0, 128, 255), 2); // 蓝色，宽度为2
//    layerPainter.setPen(pathPen);

//    // 绘制路径线段
//    for (int i = 1; i < m_currentPath.size(); ++i) {
//        // 将路径点从栅格坐标转换为像素坐标
//        QPoint start(m_currentPath[i-1].x() * GRID_SIZE, m_currentPath[i-1].y() * GRID_SIZE);
//        QPoint end(m_currentPath[i].x() * GRID_SIZE, m_currentPath[i].y() * GRID_SIZE);

//        // 绘制线段
//        layerPainter.drawLine(start, end);//每两个路径点连成一条线
//    }

//    // 绘制路径点
//    layerPainter.setBrush(QColor(0, 128, 255));
//    for (const QPoint& point : m_currentPath) {
//        // 将路径点从栅格坐标转换为像素坐标
//        QPoint pixelPoint(point.x() * GRID_SIZE, point.y() * GRID_SIZE);

//        // 绘制点
//        layerPainter.drawEllipse(pixelPoint, 3, 3);//像素坐标绘制为直径为6像素的圆(rx=ry=3)
//    }



//    // 特别标记起点和终点
//    if (m_currentPath.size() > 1) {
//        // 起点（绿色）
//        layerPainter.setBrush(Qt::green);
//        layerPainter.setPen(Qt::black);
//        QPoint startPoint(m_currentPath.first().x() * GRID_SIZE, m_currentPath.first().y() * GRID_SIZE);
//        layerPainter.drawEllipse(startPoint, 5, 5);

//        // 终点（红色）
//        layerPainter.setBrush(Qt::red);
//        QPoint endPoint(m_currentPath.last().x() * GRID_SIZE, m_currentPath.last().y() * GRID_SIZE);
//        layerPainter.drawEllipse(endPoint, 5, 5);
//    }
//}

// 实现鼠标点击事件，用于设置目标点
void GridMap::mousePressEvent(QMouseEvent *event) {
    // 获取鼠标点击位置
    QPoint pos = event->pos();

    // 将像素坐标转换为栅格坐标
    int gridCol = pos.x() / GRID_SIZE;
    int gridRow = pos.y() / GRID_SIZE;

    // 检查坐标是否在有效范围内
    if (gridCol >= 0 && gridCol < GRID_COLS && gridRow >= 0 && gridRow < GRID_ROWS) {
        // 发送目标点设置信号，默认为B1无人机
        emit targetPointSet(QPoint(gridCol, gridRow), "B1");
    }
}

//**************B样条路径平滑**************
//绘制平滑路径
void GridMap::updatePathLayer(QPainter &painter) {
    pathLayer.fill(Qt::transparent);
    QPainter layerPainter(&pathLayer);
    layerPainter.setRenderHint(QPainter::Antialiasing, true);

    // 为每个无人机绘制路径
    QStringList droneIds = m_pathMap.keys();
    for (const QString& droneId : droneIds) {
        // 为不同无人机设置不同颜色
        QColor pathColor;
        if (droneId == "B1") {
            pathColor = QColor(0, 128, 255); // 蓝色
        } else if (droneId == "B2") {
            pathColor = QColor(0, 200, 0);   // 绿色
        } else if (droneId == "B3") {
            pathColor = QColor(200, 0, 200); // 紫色
        } else {
            pathColor = QColor(128, 128, 128); // 灰色
        }

        // 绘制原始路径（虚线）
        const QVector<QPoint>& currentPath = m_pathMap[droneId];
        if (!currentPath.isEmpty()) {
            QPen originalPathPen(pathColor.lighter(), 1, Qt::DashLine);
            layerPainter.setPen(originalPathPen);

            for (int i = 1; i < currentPath.size(); ++i) {
                QPointF start = gridToPixel(currentPath[i-1]);
                QPointF end = gridToPixel(currentPath[i]);
                layerPainter.drawLine(start, end);
            }

            // 绘制原始路径点
            layerPainter.setBrush(pathColor.lighter());
            for (const QPoint& point : currentPath) {
                QPointF pixelPoint = gridToPixel(point);
                layerPainter.drawEllipse(pixelPoint, 2, 2);
            }

//            qDebug() << "无人机" << droneId << "路径终点:" << currentPath.last();
        }

        // 绘制平滑路径（实线）
        const QVector<QPointF>& smoothedPath = m_smoothedPathMap[droneId];
        if (!smoothedPath.isEmpty()) {
            QPen smoothPathPen(pathColor, 2);
            layerPainter.setPen(smoothPathPen);

            for (int i = 1; i < smoothedPath.size(); ++i) {
                layerPainter.drawLine(smoothedPath[i-1], smoothedPath[i]);
            }

            // 绘制起点和终点
            if (smoothedPath.size() > 1) {
                // 起点（绿色）
                layerPainter.setBrush(Qt::green);
                layerPainter.setPen(Qt::black);
                layerPainter.drawEllipse(smoothedPath.first(), 6, 6);

                // 终点（红色）
                layerPainter.setBrush(Qt::red);
                layerPainter.drawEllipse(smoothedPath.last(), 6, 6);
            }
        }
    }
}

// 辅助方法：栅格坐标转像素坐标
QPointF GridMap::gridToPixel(const QPoint& gridPoint) {
    // 假设GRID_SIZE已定义
    const int GRID_SIZE = 10; // 根据实际情况调整
    return QPointF(gridPoint.x() * GRID_SIZE + GRID_SIZE/2,
                   gridPoint.y() * GRID_SIZE + GRID_SIZE/2);
}



// 修复后的B样条路径平滑算法
QVector<QPointF> GridMap::smoothPathWithBSpline(const QVector<QPoint>& originalPath) {
    if (originalPath.size() < 2) {
        return QVector<QPointF>();
    }

    // 将栅格坐标转换为像素坐标作为控制点
    QVector<QPointF> controlPoints;
    for (const QPoint& point : originalPath) {
        controlPoints.append(gridToPixel(point));
    }

//    qDebug() << "控制点数量:" << controlPoints.size();
//    qDebug() << "第一个控制点:" << controlPoints.first();
//    qDebug() << "最后一个控制点:" << controlPoints.last();

    // 如果控制点太少，使用线性插值
    if (controlPoints.size() < SPLINE_DEGREE + 1) {
        return createLinearInterpolation(controlPoints);
    }
//    if (controlPoints.size() < 10 + 1) {
//        return createLinearInterpolation(controlPoints);
//    }

    // 使用改进的B样条算法
    return createBSplineInterpolation(controlPoints);
}

// 线性插值方法（用于控制点不足的情况）
QVector<QPointF> GridMap::createLinearInterpolation(const QVector<QPointF>& controlPoints) {
    QVector<QPointF> linearPath;
    int segmentsPerPoint = 10; // 每段10个插值点

    for (int i = 0; i < controlPoints.size() - 1; ++i) {
        for (int j = 0; j < segmentsPerPoint; ++j) {
            double t = static_cast<double>(j) / segmentsPerPoint;
            QPointF point = controlPoints[i] * (1 - t) + controlPoints[i + 1] * t;
            linearPath.append(point);
        }
    }

    // 确保添加最后一个控制点
    linearPath.append(controlPoints.last());

//    qDebug() << "线性插值完成，路径点数量:" << linearPath.size();
//    qDebug() << "线性插值最后一点:" << linearPath.last();

    return linearPath;
}

// 改进的B样条插值方法
QVector<QPointF> GridMap::createBSplineInterpolation(const QVector<QPointF>& controlPoints) {
    QVector<QPointF> smoothPath;
    int n = controlPoints.size();
    int degree = qMin(SPLINE_DEGREE, n - 1); // 确保阶数不超过控制点数-1
    //int degree = qMin(10, n - 1); // 确保阶数不超过控制点数-1

    // 生成改进的节点向量
    QVector<double> knots = generateImprovedKnotVector(n, degree);

//    qDebug() << "B样条参数 - 控制点数:" << n << "阶数:" << degree;
//    qDebug() << "节点向量:" << knots;

    // 计算样本点数量
    int sampleCount = n * 8; // 每个控制点段8个样本点

    // 计算B样条曲线上的点
    for (int i = 0; i <= sampleCount; ++i) {
        double t = static_cast<double>(i) / sampleCount;

        // 将t映射到有效的节点范围
        double tMin = knots[degree];
        double tMax = knots[knots.size() - degree - 1];
        double mappedT = tMin + t * (tMax - tMin);

        // 使用德布尔算法计算点
        QPointF point = deBoorAlgorithm(mappedT, controlPoints, knots, degree);
        smoothPath.append(point);

        // 调试第一个和最后一个点
        if (i == 0) {
//            qDebug() << "第一个B样条点 t=" << mappedT << "点=" << point;
        }
        if (i == sampleCount) {
//            qDebug() << "最后一个B样条点 t=" << mappedT << "点=" << point;
        }
    }

    // 验证结果
//    qDebug() << "B样条插值完成，路径点数量:" << smoothPath.size();
    if (!smoothPath.isEmpty()) {
//        qDebug() << "B样条第一点:" << smoothPath.first();
//        qDebug() << "B样条最后一点:" << smoothPath.last();
    }

    return smoothPath;
}

// 改进的节点向量生成（使用累积弦长参数化）
QVector<double> GridMap::generateImprovedKnotVector(int controlPoints, int degree) {
    int knotCount = controlPoints + degree + 1;
    QVector<double> knots(knotCount);

    // 前面的重复节点
    for (int i = 0; i <= degree; ++i) {
        knots[i] = 0.0;
    }

    // 后面的重复节点
    for (int i = controlPoints; i < knotCount; ++i) {
        knots[i] = 1.0;
    }

    // 中间的均匀节点
    int internalKnots = controlPoints - degree - 1;
    for (int i = 1; i <= internalKnots; ++i) {
        knots[degree + i] = static_cast<double>(i) / (internalKnots + 1);
    }

    return knots;
}

// 德布尔算法（更稳定的B样条计算方法）
QPointF GridMap::deBoorAlgorithm(double t, const QVector<QPointF>& controlPoints,
                                const QVector<double>& knots, int degree) {
    int n = controlPoints.size();

    // 找到t所在的节点区间
    int k = degree;
    while (k < knots.size() - 1 && t > knots[k + 1]) {
        k++;
    }

    // 确保k在有效范围内
    k = qMax(degree, qMin(k, n - 1));

    // 初始化控制点
    QVector<QPointF> d(degree + 1);
    for (int i = 0; i <= degree; ++i) {
        int idx = k - degree + i;
        if (idx >= 0 && idx < n) {
            d[i] = controlPoints[idx];
        } else {
            // 边界处理：使用最近的有效控制点
            d[i] = controlPoints[qMax(0, qMin(idx, n - 1))];
        }
    }

    // 德布尔递归计算
    for (int r = 1; r <= degree; ++r) {
        for (int i = degree; i >= r; --i) {
            int knotIdx1 = k - degree + i;
            int knotIdx2 = k - degree + i + degree - r + 1;

            // 边界检查
            if (knotIdx1 < 0 || knotIdx2 >= knots.size() ||
                knotIdx1 >= knots.size() || knotIdx2 < 0) {
                continue;
            }

            double alpha = 0.0;
            double denominator = knots[knotIdx2] - knots[knotIdx1];

            if (qAbs(denominator) > 1e-10) { // 避免除零
                alpha = (t - knots[knotIdx1]) / denominator;
            }

            alpha = qBound(0.0, alpha, 1.0); // 限制alpha在[0,1]范围内
            d[i] = d[i - 1] * (1.0 - alpha) + d[i] * alpha;
        }
    }

    return d[degree];
}


// 计算无人机的速度
QPointF GridMap::calculateVelocity(const QString &droneId, const QPointF &currentPos) {
    // 检查是否有该无人机的平滑路径
    if (!m_smoothedPathMap.contains(droneId) || m_smoothedPathMap[droneId].isEmpty()) {
        qDebug() << "[VelocityDebug] " << droneId << " 无平滑路径，返回零速度";
        return QPointF(0, 0); // 无路径，返回零速度
    }

    const QVector<QPointF>& smoothedPath = m_smoothedPathMap[droneId];

    // 如果没有路径或路径点少于2个，返回零速度
    if (smoothedPath.isEmpty() || smoothedPath.size() < 2) {
        qDebug() << "[VelocityDebug] " << droneId << " 路径点不足，返回零速度";
        return QPointF(0, 0);
    }

    // 检查是否已经到达终点
    QPointF endPoint = smoothedPath.last();
    double distToEnd = QVector2D(endPoint - currentPos).length();
    qDebug() << "[VelocityDebug] " << droneId << " 到终点距离: " << distToEnd << " 阈值: " << TARGET_THRESHOLD;

    if (distToEnd < TARGET_THRESHOLD) {
        // 已到达终点，停止移动
        qDebug() << "[VelocityDebug] " << droneId << " 已到达终点，停止移动";
        m_pathMap[droneId].clear();
        m_smoothedPathMap[droneId].clear();
        pathNeedsUpdate = true;

        // 发送到达终点信号
        emit targetReached(droneId);

        return QPointF(0, 0);
    }

    // 找到距离当前位置最近的路径点索引
    int nextPointIndex = findClosestPathPointIndex(currentPos, droneId);
    qDebug() << "[VelocityDebug] " << droneId << " 最近路径点索引: " << nextPointIndex << " 总路径点数: " << smoothedPath.size();

    // 如果已经接近当前目标点，选择再下一个点作为目标
    double minDist = QVector2D(smoothedPath[nextPointIndex] - currentPos).length();
    qDebug() << "[VelocityDebug] " << droneId << " 到最近点距离: " << minDist;
    
    if (minDist < TARGET_THRESHOLD && nextPointIndex < smoothedPath.size() - 1) {
        nextPointIndex++;
        qDebug() << "[VelocityDebug] " << droneId << " 切换到下一个目标点，新索引: " << nextPointIndex;
    }

    // 获取下一个目标点
    QPointF targetPos = smoothedPath[nextPointIndex];
    qDebug() << "[VelocityDebug] " << droneId << " 目标点坐标: " << targetPos << " 当前位置: " << currentPos;

    // 计算到下一个点的基本方向
    QPointF baseDirection = targetPos - currentPos;
    double baseLen = QVector2D(baseDirection).length();
    qDebug() << "[VelocityDebug] " << droneId << " 基本方向向量: " << baseDirection << " 长度: " << baseLen;

    if (baseLen <= 0) {
        qDebug() << "[VelocityDebug] " << droneId << " 基本方向长度为零，返回零速度";
        return QPointF(0, 0);
    }

    // 归一化基本方向
    baseDirection = baseDirection / baseLen;
    qDebug() << "[VelocityDebug] " << droneId << " 归一化基本方向: " << baseDirection;

    // 计算前瞻方向
    QPointF avgDirection = calculateLookaheadDirection(nextPointIndex, LOOKAHEAD, droneId);
    qDebug() << "[VelocityDebug] " << droneId << " 前瞻方向: " << avgDirection;

    // 混合基本方向和平均方向 (70% 基本方向, 30% 平均方向)
    QPointF finalDirection = baseDirection * 0.7 + avgDirection * 0.3;
    double finalLen = QVector2D(finalDirection).length();
    qDebug() << "[VelocityDebug] " << droneId << " 最终方向向量: " << finalDirection << " 长度: " << finalLen;

    if (finalLen <= 0) {
        qDebug() << "[VelocityDebug] " << droneId << " 最终方向长度为零，返回零速度";
        return QPointF(0, 0);
    }

    // 归一化最终方向
    finalDirection = finalDirection / finalLen;
    qDebug() << "[VelocityDebug] " << droneId << " 归一化最终方向: " << finalDirection;

    // 计算vx和vy
    double vx = 0, vy = 0;

    // 根据方向计算速度分量
    if (qAbs(finalDirection.x()) > qAbs(finalDirection.y())) {
        // x方向分量更大
        vx = MAX_SPEED * (finalDirection.x() > 0 ? 1 : -1);
        vy = vx * (finalDirection.y() / finalDirection.x());
    } else {
        // y方向分量更大
        vy = MAX_SPEED * (finalDirection.y() > 0 ? 1 : -1);
        vx = vy * (finalDirection.x() / finalDirection.y());
    }

    qDebug() << "[VelocityDebug] " << droneId << " 最终计算速度: vx=" << vx << " vy=" << vy << " MAX_SPEED=" << MAX_SPEED;
    return QPointF(vx, vy);
}

// 找到最近的路径点索引
int GridMap::findClosestPathPointIndex(const QPointF &currentPos, const QString &droneId) {
    int closestIndex = 0;
    double minDist = std::numeric_limits<double>::max();

    // 确保该无人机有平滑路径
    if (!m_smoothedPathMap.contains(droneId) || m_smoothedPathMap[droneId].isEmpty()) {
        return 0;
    }

    const QVector<QPointF>& smoothedPath = m_smoothedPathMap[droneId];

    // 找到距离当前位置最近的路径点
    for (int i = 0; i < smoothedPath.size(); i++) {
        QPointF pathPoint = smoothedPath[i];
        double dist = QVector2D(pathPoint - currentPos).length();
        if (dist < minDist) {
            minDist = dist;
            closestIndex = i;
        }
    }

    return closestIndex;
}

// 计算前瞻方向
QPointF GridMap::calculateLookaheadDirection(int currentIndex, int lookaheadCount, const QString &droneId) {
    QPointF avgDirection(0, 0);
    int count = 0;

    // 确保该无人机有平滑路径
    if (!m_smoothedPathMap.contains(droneId) || m_smoothedPathMap[droneId].isEmpty()) {
        return avgDirection;
    }

    const QVector<QPointF>& smoothedPath = m_smoothedPathMap[droneId];

    // 计算未来几个点的平均方向
    for (int i = currentIndex; i < qMin(currentIndex + lookaheadCount, smoothedPath.size()); i++) {
        if (i + 1 < smoothedPath.size()) {
            QPointF dir = smoothedPath[i + 1] - smoothedPath[i];
            // 归一化方向向量
            double len = QVector2D(dir).length();
            if (len > 0) {
                dir = dir / len;
                avgDirection += dir;
                count++;
            }
        }
    }

    // 如果有有效的平均方向，进行归一化
    if (count > 0) {
        avgDirection = avgDirection / count;
        double len = QVector2D(avgDirection).length();
        if (len > 0) {
            avgDirection = avgDirection / len;
        }
    }

    return avgDirection;
}
