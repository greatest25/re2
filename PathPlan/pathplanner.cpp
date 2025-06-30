#include "pathplanner.h"

// 优先队列比较函数
struct ComparePoints {
    bool operator()(const QPair<QPoint, float>& a, const QPair<QPoint, float>& b) const {
        return a.second > b.second; // 小的优先级高
    }
};

PathPlanner::PathPlanner(QObject* parent)
    : QObject(parent)
    , m_mapWidth(0)
    , m_mapHeight(0)
{
    // 初始化障碍物成本
    initObstacleCosts();

    // 初始化线程池
    m_threadPool = new QThreadPool(this);
    m_threadPool->setMaxThreadCount(3); // 设置最大线程数为3，对应3个无人机
}

// 初始化障碍物成本
void PathPlanner::initObstacleCosts()
{
    // 设置各种障碍物的成本
//    m_obstacleCosts[UNKNOWN] = 1.0f;       // 未探测区域 - 可通行，基本成本
//    m_obstacleCosts[RADAR] = 500.0f;       // 雷达障碍物 - 高成本，尽量避开（原值100.0f）
//    m_obstacleCosts[MOUNTAIN] = 999999.0f; // 山体障碍物 - 不可通行
//    m_obstacleCosts[CLOUD] = 300.0f;        // 雷云(移动障碍物) - 较高成本（原值50.0f）
//    m_obstacleCosts[ENEMY] = 400.0f;        // 敌方无人机 - 高成本（原值80.0f）
//    m_obstacleCosts[OVERLAP_CLOUD_RADAR] = 600.0f;  // 雷云与雷达重叠 - 非常高成本（原值150.0f）
//    m_obstacleCosts[OVERLAP_ENEMY_CLOUD] = 700.0f;  // 敌方攻击范围与雷云重叠 - 非常高成本（原值130.0f）
//    m_obstacleCosts[OVERLAP_ENEMY_RADAR] = 800.0f;  // 敌方攻击范围与雷达重叠 - 非常高成本（原值180.0f）

    m_obstacleCosts[1] = 1.0f;       // 未探测区域 - 可通行，基本成本
    m_obstacleCosts[2] = 500.0f;       // 雷达障碍物 - 高成本，尽量避开（原值100.0f）
    m_obstacleCosts[3] = 999999.0f;     // 山体障碍物 - 不可通行
    m_obstacleCosts[4] = 400.0f;        // 雷云(移动障碍物) - 较高成本（原值50.0f）
    m_obstacleCosts[5] = 300.0f;        // 敌方无人机 - 高成本（原值80.0f）
    m_obstacleCosts[6] = 600.0f;  // 雷云与雷达重叠 - 非常高成本（原值150.0f）
    m_obstacleCosts[7] = 700.0f;  // 敌方攻击范围与雷云重叠 - 非常高成本（原值130.0f）
    m_obstacleCosts[8] = 800.0f;  // 敌方攻击范围与雷达重叠 - 非常高成本（原值180.0f）
}


//***************共享地图**************
// 设置共享地图指针
void PathPlanner::setSharedGridMap(SharedGridMap gridMap)
{
    QWriteLocker locker(&m_mapLock);
    m_sharedGridMap = gridMap;
    if (m_sharedGridMap) {
        // 只记录地图尺寸，不再创建本地副本
        m_mapWidth = m_sharedGridMap->at(0).size();  // 列数
        m_mapHeight = m_sharedGridMap->size();      // 行数
//        qDebug() << "PathPlanner: 共享地图设置成功，地图大小:" << m_mapWidth << "x" << m_mapHeight;
    }
}

//栅格地图更新槽函数
void PathPlanner::onUpdateSharedGridMap(SharedGridMap gridMap)
{
    setSharedGridMap(gridMap);
}

// 实现地图单元格更新槽函数
void PathPlanner::onMapUpdated(int row, int col, GridCellState newState)
{
    // 只记录日志，不需要更新本地副本
    //qDebug() << "PathPlanner: 收到地图更新通知 (" << row << "," << col << ") = " << newState;

}
//实现地图重置槽函数
void PathPlanner::onMapReset()
{
    qDebug() << "PathPlanner: 收到地图重置通知";
}

// 验证当前地图是否和gridmap一致
void PathPlanner::debugCheckMapConsistency(int row, int col)
{
    if (!m_sharedGridMap) return;

    // 获取GridMap的共享指针中的值
    GridCellState sharedState = m_sharedGridMap->at(row).at(col);

    // 输出日志
    qDebug() << "地图一致性检查 - 位置:(" << row << "," << col << ")";
    qDebug() << "共享指针中的值:" << sharedState;

    // 如果有本地副本，也可以比较
    // qDebug() << "本地副本中的值:" << m_localMap[row][col];
}

//***************调用A*算法槽函数****************
//调用路径规划接口
void PathPlanner::onStartfindPath(const QPoint& start, const QPoint& goal, const QString& droneId)
{
    // 使用QtConcurrent在线程池中异步执行路径规划
    QtConcurrent::run(m_threadPool, [=]() {
        // 在工作线程中执行路径规划
        QVector<QPoint> path = findPath(start, goal, droneId);

        // 使用QMetaObject::invokeMethod在主线程中发送信号，包含无人机ID
        QMetaObject::invokeMethod(this, "pathPlanned", Qt::QueuedConnection,
                                 Q_ARG(QVector<QPoint>, path),
                                 Q_ARG(QString, droneId));
    });
}

//***************A*算法****************
// 检查点是否有效（在地图范围内且不是不可通行的障碍物）
bool PathPlanner::isValidPoint(int row, int col) const
{
    QReadLocker locker(&m_mapLock); // 使用读锁保护访问

    // 检查点是否在地图范围内
    if (row < 0 || row >= m_mapHeight || col < 0 || col >= m_mapWidth) {
        return false;
    }

    // 检查共享指针是否有效
    if (!m_sharedGridMap) {
        qDebug() << "PathPlanner: 共享地图指针无效";
        return false;
    }

    // 检查点是否是不可通行的障碍物
    GridCellState cellState = m_sharedGridMap->at(row).at(col);
    if(cellState != 1)
    {
//         qDebug()<<"GridCellState:"<<cellState;
    }
    return m_obstacleCosts[cellState] < 200.0f; // 将阈值从999.0f降低到200.0f，使高成本区域被视为不可通行
}

// 检查是否可以沿对角线移动（两个相邻的格子都不是障碍物）
bool PathPlanner::canMoveDiagonally(int startRow, int startCol, int endRow, int endCol) const
{
    // 确保是对角线移动
    if (abs(startRow - endRow) != 1 || abs(startCol - endCol) != 1) {
        return false;
    }

    // 检查两个相邻的格子是否都可通行
    // 这里使用isValidPoint方法，该方法已经被修改为对高成本区域更严格
    return isValidPoint(startRow, endCol) && isValidPoint(endRow, startCol);
}

// 计算两点之间的启发式距离（欧几里得距离）
float PathPlanner::heuristic(const QPoint& a, const QPoint& b) const
{
//    // 使用曼哈顿距离作为启发式函数
//    return abs(a.x() - b.x()) + abs(a.y() - b.y());

    // 如果允许对角线移动，使用欧几里得距离
    return sqrt(pow(a.x() - b.x(), 2) + pow(a.y() - b.y(), 2));
}

// 获取点的移动成本
float PathPlanner::getMovementCost(int row, int col) const
{
    QReadLocker locker(&m_mapLock);

    if (!m_sharedGridMap || row < 0 || row >= m_mapHeight || col < 0 || col >= m_mapWidth) {
        return 999999.0f; // 无效点，返回极高成本
    }

    GridCellState cellState = m_sharedGridMap->at(row).at(col);
    return m_obstacleCosts.value(cellState, 999999.0f); // 如果没有定义成本，默认为不可通行
}

// 寻找最近的非障碍物点作为逃离点  策略1-2
QPoint PathPlanner::findNearestNonObstaclePoint(const QPoint& start, const QString& droneId) const
{
    // 定义搜索范围（从小到大扩展）
    const int maxSearchRadius = 20; // 最大搜索半径

    // 定义方向数组（8个方向：上、右、下、左、右上、右下、左下、左上）
    const int dx[8] = {0, 1, 0, -1, 1, 1, -1, -1};
    const int dy[8] = {-1, 0, 1, 0, -1, 1, 1, -1};

    // 使用BFS寻找最近的非障碍物点
    QQueue<QPoint> queue;
    QSet<QPoint> visited;

    queue.enqueue(start);
    visited.insert(start);

    while (!queue.isEmpty()) {
        QPoint current = queue.dequeue();

        // 如果当前点不在障碍物区域内，返回该点
        if (isValidPoint(current.y(), current.x())) {
            return current;
        }

        // 探索所有相邻点
        for (int i = 0; i < 8; ++i) {
            int newRow = current.y() + dy[i];
            int newCol = current.x() + dx[i];
            QPoint neighbor(newCol, newRow);

            // 检查点是否在地图范围内且未访问过
            if (newRow >= 0 && newRow < m_mapHeight &&
                newCol >= 0 && newCol < m_mapWidth &&
                !visited.contains(neighbor)) {

                // 计算与起点的曼哈顿距离，限制搜索范围
                int distance = abs(newRow - start.y()) + abs(newCol - start.x());
                if (distance <= maxSearchRadius) {
                    queue.enqueue(neighbor);
                    visited.insert(neighbor);
                }
            }
        }
    }

    // 如果找不到合适的点，返回起点（保底方案）
    return start;
}
// A*路径规划算法
QVector<QPoint> PathPlanner::findPath(const QPoint& start, const QPoint& goal, const QString& droneId)
{
    QVector<QPoint> path;

    QPoint actualGoal = goal;

    // 检查终点是否有效
    if (!isValidPoint(goal.y(), goal.x())) {
        qDebug() << "PathPlanner: 无人机" << droneId <<"终点无效，寻找目标点最近的非障碍点";
        actualGoal = findNearestNonObstaclePoint(goal,droneId);
        goal = actualGoal;
    }

    // 检查起点是否在障碍物区域内（使用!isValidPoint直接判断）
    bool startInObstacle = !isValidPoint(start.y(), start.x());

    if (startInObstacle) {
        qDebug() << "PathPlanner: 起点在障碍物区域内，寻找逃离点";

        // 寻找最近的非障碍物点作为逃离点
        QPoint escapePoint = findNearestNonObstaclePoint(start,droneId);

        // 如果找不到逃离点或逃离点就是起点，直接尝试规划到目标点的路径
        if (escapePoint == start) {
            qDebug() << "PathPlanner: 无法找到合适的逃离点，尝试直接规划到目标点";
        } else {
            qDebug() << "PathPlanner: 找到逃离点" << escapePoint;

            // 定义方向数组（8个方向）
            const int dx[8] = {0, 1, 0, -1, 1, 1, -1, -1};
            const int dy[8] = {-1, 0, 1, 0, -1, 1, 1, -1};

            // 使用优先队列进行A*搜索
            std::priority_queue<QPair<QPoint, float>, std::vector<QPair<QPoint, float>>, ComparePoints> openSet;

            // 记录每个点的g值和父节点
            QHash<QPoint, float> gScore;
            QHash<QPoint, QPoint> cameFrom;

            // 初始化起点
            openSet.push(qMakePair(start, 0.0f));
            gScore[start] = 0.0f;

            QVector<QPoint> escapePath;
            bool foundEscapePath = false;

            while (!openSet.empty()) {
                // 获取当前f值最小的点
                QPoint current = openSet.top().first;
                openSet.pop();

                // 如果到达逃离点，重建并返回路径
                if (current == escapePoint) {
                    // 重建路径
                    QPoint temp = current;
                    while (temp != start) {
                        escapePath.prepend(temp);
                        temp = cameFrom[temp];
                    }
                    escapePath.prepend(start);
                    foundEscapePath = true;
                    break;
                }

                // 探索所有相邻点
                for (int i = 0; i < 8; ++i) {
                    int newRow = current.y() + dy[i];
                    int newCol = current.x() + dx[i];
                    QPoint neighbor(newCol, newRow);

                    // 检查点是否在地图范围内
                    if (newRow < 0 || newRow >= m_mapHeight || newCol < 0 || newCol >= m_mapWidth) {
                        continue;
                    }

                    // 如果是对角线移动，检查是否可以沿对角线移动
                    if (i >= 4) {
                        // 简化对角线检查，只要两个相邻点在地图范围内即可
                        if (newRow < 0 || newRow >= m_mapHeight ||
                            newCol < 0 || newCol >= m_mapWidth ||
                            current.y() < 0 || current.y() >= m_mapHeight ||
                            current.x() < 0 || current.x() >= m_mapWidth) {
                            continue;
                        }
                    }

                    // 计算移动成本
                    float moveCost = (i < 4) ? 1.0f : 1.414f;

                    // 获取单元格的移动成本，但在障碍物中移动时降低惩罚
                    float cellCost = getMovementCost(newRow, newCol);
                    if (cellCost > 1.0f) {
                        // 在障碍物中移动时，降低惩罚，但仍然有一定惩罚
                        moveCost *= cellCost / 5.0f; // 降低惩罚系数
                    } else {
                        moveCost *= cellCost;
                    }

                    float tentativeGScore = gScore[current] + moveCost;

                    // 如果找到了更好的路径，或者这个点是第一次被访问
                    if (!gScore.contains(neighbor) || tentativeGScore < gScore[neighbor]) {
                        // 更新这个点的信息
                        cameFrom[neighbor] = current;
                        gScore[neighbor] = tentativeGScore;

                        // 计算f值并加入开放集
                        float fScore = tentativeGScore + heuristic(neighbor, escapePoint);
                        openSet.push(qMakePair(neighbor, fScore));
                    }
                }
            }

            // 如果找到了逃离路径，再规划从逃离点到目标点的路径
            if (foundEscapePath) {
                // 使用原始A*算法从逃离点规划到目标点
                QVector<QPoint> goalPath;

                // 重置优先队列和相关数据结构
                std::priority_queue<QPair<QPoint, float>, std::vector<QPair<QPoint, float>>, ComparePoints> openSet2;
                QHash<QPoint, float> gScore2;
                QHash<QPoint, QPoint> cameFrom2;

                // 初始化逃离点
                openSet2.push(qMakePair(escapePoint, 0.0f));
                gScore2[escapePoint] = 0.0f;

                while (!openSet2.empty()) {
                    // 获取当前f值最小的点
                    QPoint current = openSet2.top().first;
                    openSet2.pop();

                    // 如果到达目标点，重建并返回路径
                    if (current == goal) {
                        // 重建路径
                        QPoint temp = current;
                        while (temp != escapePoint) {
                            goalPath.prepend(temp);
                            temp = cameFrom2[temp];
                        }
                        goalPath.prepend(escapePoint);
                        break;
                    }

                    // 探索所有相邻点
                    for (int i = 0; i < 8; ++i) {
                        int newRow = current.y() + dy[i];
                        int newCol = current.x() + dx[i];
                        QPoint neighbor(newCol, newRow);

                        // 检查相邻点是否有效
                        if (!isValidPoint(newRow, newCol)) {
                            continue;
                        }

                        // 如果是对角线移动，检查是否可以沿对角线移动
                        if (i >= 4 && !canMoveDiagonally(current.y(), current.x(), newRow, newCol)) {
                            continue;
                        }

                        // 计算移动成本
                        float moveCost = (i < 4) ? 1.0f : 1.414f;

                        // 获取单元格的移动成本
                        float cellCost = getMovementCost(newRow, newCol);

                        // 如果单元格成本大于基本成本，额外增加权重
                        if (cellCost > 1.0f) {
                            moveCost *= cellCost * cellCost / 10.0f;
                        } else {
                            moveCost *= cellCost;
                        }

                        float tentativeGScore = gScore2[current] + moveCost;

                        // 如果找到了更好的路径，或者这个点是第一次被访问
                        if (!gScore2.contains(neighbor) || tentativeGScore < gScore2[neighbor]) {
                            // 更新这个点的信息
                            cameFrom2[neighbor] = current;
                            gScore2[neighbor] = tentativeGScore;

                            // 计算f值并加入开放集
                            float fScore = tentativeGScore + heuristic(neighbor, goal);
                            openSet2.push(qMakePair(neighbor, fScore));
                        }
                    }
                }

                // 合并两段路径
                if (!goalPath.isEmpty()) {
                    path = escapePath;
                    // 移除第二段路径的第一个点（即逃离点），因为它已经在第一段路径的末尾
                    goalPath.removeFirst();
                    path.append(goalPath);

//                    qDebug() << "PathPlanner: 成功规划从障碍物中逃离并到达目标点的路径，总长度:" << path.size();
                    return path;
                } else {
//                    qDebug() << "PathPlanner: 无法找到从逃离点到目标点的路径，仅返回逃离路径";
                    return escapePath;
                }
            }
        }
    }

    // 如果起点不在障碍物区域内，或者无法找到逃离路径，使用原始A*算法
    // 定义方向数组（8个方向：上、右、下、左、右上、右下、左下、左上）
    const int dx[8] = {0, 1, 0, -1, 1, 1, -1, -1};
    const int dy[8] = {-1, 0, 1, 0, -1, 1, 1, -1};

    // 使用优先队列进行A*搜索
    std::priority_queue<QPair<QPoint, float>, std::vector<QPair<QPoint, float>>, ComparePoints> openSet;

    // 记录每个点的g值（从起点到当前点的实际成本）
    QHash<QPoint, float> gScore;

    // 记录每个点的父节点，用于重建路径
    QHash<QPoint, QPoint> cameFrom;

    // 初始化起点
    openSet.push(qMakePair(start, 0.0f));
    gScore[start] = 0.0f;

    while (!openSet.empty()) {
        // 获取当前f值最小的点
        QPoint current = openSet.top().first;
        openSet.pop();

        // 如果到达终点，重建并返回路径
        if (current == goal) {
            int pathSteps = 0;
            int pathReconstructionLimit = 1000; // 设置一个合理的上限
            // 重建路径
            while (current != start && pathSteps < pathReconstructionLimit) {
                path.prepend(current);
                if (!cameFrom.contains(current)) {
                    qDebug() << "findPath: 路径重建错误，无法找到节点" << current << "的父节点，无人机" << droneId;
                    break; // 中断循环，返回不完整的路径
                }
                current = cameFrom[current];
                pathSteps++;
            }
            path.prepend(start);
            if (pathSteps >= pathReconstructionLimit) {
                qDebug() << "findPath: 路径重建超过最大步数限制，可能存在循环，无人机" << droneId;
            }

            qDebug() << "findPath: 找到路径"<<"无人机" << droneId ;
            return path;
        }

        // 探索所有相邻点
        for (int i = 0; i < 8; ++i) {
            int newRow = current.y() + dy[i];
            int newCol = current.x() + dx[i];
            QPoint neighbor(newCol, newRow);

            // 检查相邻点是否有效
            if (!isValidPoint(newRow, newCol)) {
                continue;
            }

            // 如果是对角线移动，检查是否可以沿对角线移动
            if (i >= 4 && !canMoveDiagonally(current.y(), current.x(), newRow, newCol)) {
                continue;
            }

            // 计算从起点经过当前点到相邻点的成本
            // 对角线移动的成本为1.414（根号2），直线移动的成本为1
            float moveCost = (i < 4) ? 1.0f : 1.414f;

            // 获取单元格的移动成本
            float cellCost = getMovementCost(newRow, newCol);

            // 如果单元格成本大于基本成本，额外增加权重，使算法更倾向于避开高成本区域
            if (cellCost > 1.0f) {
                // 使用平方关系增加高成本区域的权重
                moveCost *= cellCost * cellCost / 10.0f;
            } else {
                moveCost *= cellCost;
            }

            float tentativeGScore = gScore[current] + moveCost;

            // 如果找到了更好的路径，或者这个点是第一次被访问
            if (!gScore.contains(neighbor) || tentativeGScore < gScore[neighbor]) {
                // 更新这个点的信息
                cameFrom[neighbor] = current;
                gScore[neighbor] = tentativeGScore;

                // 计算f值（g值 + 启发式值）并加入开放集
                float fScore = tentativeGScore + heuristic(neighbor, goal);
                openSet.push(qMakePair(neighbor, fScore));
            }
        }
    }

    // 如果无法找到路径，尝试直线规划
    if (path.isEmpty()) {
        qDebug() << "PathPlanner: 无法找到从" << start << "到" << goal << "的路径，尝试直线规划";

        // 使用Bresenham算法进行直线规划
        int x0 = start.x();
        int y0 = start.y();
        int x1 = goal.x();
        int y1 = goal.y();

        int dx = abs(x1 - x0);
        int dy = abs(y1 - y0);
        int sx = (x0 < x1) ? 1 : -1;
        int sy = (y0 < y1) ? 1 : -1;
        int err = dx - dy;

        path.append(start); // 添加起点

        while (x0 != x1 || y0 != y1) {
            int e2 = 2 * err;
            if (e2 > -dy) {
                err -= dy;
                x0 += sx;
            }
            if (e2 < dx) {
                err += dx;
                y0 += sy;
            }

            QPoint p(x0, y0);
            path.append(p);

            // 如果路径点过多，可以适当跳过一些点
            if (path.size() > 100) { // 限制路径点数量
                break;
            }
        }

        if (!path.contains(goal)) {
            path.append(goal); // 确保终点在路径中
        }
    }
    return path;
}
