#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    this->setWindowTitle("自动策略飞行控制终端");
    // 设置标题字体大小
    QFont statusFont("Microsoft YaHei", 14, QFont::Bold);
    QFont strategyFont("Microsoft YaHei", 14, QFont::Bold);
    QFont mapFont("Microsoft YaHei", 15, QFont::Bold);
    // 设置主标题字体
    ui->groupBox_3->setFont(statusFont);
    ui->groupBox_4->setFont(strategyFont);
    ui->groupBox_5->setFont(mapFont);
    // 设置子标题字体
    QFont blueUAVFont("Microsoft YaHei", 11);
    QFont redUAVFont("Microsoft YaHei", 11);
    ui->groupBox->setFont(blueUAVFont);
    ui->groupBox_2->setFont(redUAVFont);
    // 设置游戏状态和剩余时间字体
    QFont timeStatusFont("Microsoft YaHei", 13, QFont::Bold);
    ui->time_status->setFont(timeStatusFont);

    // 创建自定义奖杯图标
    QPixmap trophyPixmap(ui->win_icon->size());
    trophyPixmap.fill(Qt::transparent);

    QPainter painter(&trophyPixmap);
    painter.setRenderHint(QPainter::Antialiasing);

    QColor trophyColor(255, 215, 0);
    QColor baseColor(90, 90, 90);

    int w = trophyPixmap.width();
    int h = trophyPixmap.height();

    // 向上偏移量
    int yOffset = h * 0.30;  // 如需向上再调大些

    // 杯身参数
    const int cupWidth = 36, cupHeight = 38;
    int cLeft = w/2 - cupWidth/2, cTop = h*0.18 - yOffset;
    int cBot = cTop + cupHeight;

    // 杯身路径
    QPainterPath cupPath;
    cupPath.moveTo(cLeft, cTop + 8);
    cupPath.arcTo(QRectF(cLeft, cTop, cupWidth, 16), 180, 180); // 上半椭圆
    cupPath.quadTo(w/2 + cupWidth*0.5, cBot - 10, w/2, cBot + 12);
    cupPath.quadTo(w/2 - cupWidth*0.5, cBot - 10, cLeft, cTop + 8);
    cupPath.closeSubpath();

    // 杯身
    painter.setPen(Qt::NoPen);
    painter.setBrush(trophyColor);
    painter.drawPath(cupPath);

    // 杯口高光（亮黄宽贴边）
    QRectF ellipse(w/2-cupWidth/2-2, cTop-2, cupWidth+4, 10);
    painter.setBrush(QColor(255,255,210)); // 更亮更偏白
    painter.drawEllipse(ellipse);

    // 杯颈 (整体上移)
    int neckW = 12, neckH = 11;
    int neckX = w/2-neckW/2, neckY = cBot + 10;
    painter.setBrush(trophyColor);
    painter.drawRect(neckX, neckY, neckW, neckH);

    // 底座 (整体上移，宽度缩窄)
    painter.setBrush(baseColor);
    painter.setPen(Qt::NoPen);
    // 小底座，收窄/靠下
    int baseY1 = neckY + neckH - 2;         // 贴紧杯颈下沿
    int baseY2 = baseY1 + 7;                // 比例更协调
    int baseW1 = 20, baseW2 = 32;           // 两层宽
    painter.drawRect(w/2 - baseW1/2, baseY1, baseW1, 7);
    painter.drawRect(w/2 - baseW2/2, baseY2, baseW2, 6);
    // WIN字样，完全限制在杯身 （也向上偏移）
    QRect winRect(w/2-cupWidth/2+4, cTop+13, cupWidth-8, cupHeight-16);
    painter.setPen(Qt::white);
    QFont font("Arial", 8, QFont::Bold); // 小字体
    painter.setFont(font);
    painter.drawText(winRect, Qt::AlignCenter, "WIN");
    painter.end();

    ui->win_icon->setPixmap(trophyPixmap);
    ui->win_icon->setText("");

    // 设置胜率文本字体
    QFont winRateFont("Arial", 14, QFont::Bold);
    ui->win_rate->setFont(winRateFont);
    ui->win_rate->setStyleSheet("color: #DAA520;"); // 金色文字


    // 初始化栅格地图
    gridMap = new GridMap(ui->gridmap);
    gridMap->setGeometry(0, 0, ui->gridmap->width(), ui->gridmap->height());
    gridMap->show();
    gridMap->initMap();

    targetManager = new TargetManager(this);

//    //连接栅格地图中计算而得的速度
//    connect(gridMap, &GridMap::velocityUpdated, this, [this](const QString& droneId, const QPointF& velocity) {
//        // 发送控制命令
//        sendControlCommand(droneId, velocity);
//    });

    // 创建并初始化 VsoaManager
    vsoaManager = new VsoaManager(this);

    // 连接 VsoaManager 的信号到 MainWindow 的槽
    connect(vsoaManager, &VsoaManager::connectionStatusChanged, this, &MainWindow::onConnectionStatusChanged);
    connect(vsoaManager, &VsoaManager::gameDataUpdated, this, &MainWindow::onGameDataUpdated);
    connect(vsoaManager, &VsoaManager::serverPaused, this, &MainWindow::onServerPaused);
    connect(vsoaManager, &VsoaManager::serverResumed, this, &MainWindow::onServerResumed);

    // 初始化 VSOA 连接
    vsoaManager->initConnection();

    // 初始化移动物体刷新定时器
    movingObjectsTimer = new QTimer(this);
    connect(movingObjectsTimer, &QTimer::timeout, this, &MainWindow::clearStaleMovingObjects);
    movingObjectsTimer->start(500);

    // 初始化地图刷新定时器
    QTimer *refreshTimer = new QTimer(this);
    connect(refreshTimer, &QTimer::timeout, [this]() {
        if (gridMap->needsUpdate()) {
            gridMap->update();
        }
    });
    refreshTimer->start(50);

    // 连接策略按钮信号
    connect(ui->SO1, &QPushButton::clicked, this, [this]() {
        if (gameStage != "running") {
            currentStrategy = 1;
            ui->SO1->setStyleSheet("background-color: #8FBC8F;");
            ui->SO2->setStyleSheet("");
            ui->SO3->setStyleSheet("");
            ui->SO4->setStyleSheet("");
            qDebug() << "已选择策略1 (PathPlan的A*算法)";

            // 清理MADDPG算法资源
            cleanupMADDPG();

            // 初始化PathPlanner
            initializePathPlanner();
        }
    });

    connect(ui->SO2, &QPushButton::clicked, this, [this]() {
        if (gameStage != "running") {
            currentStrategy = 2;
            ui->SO1->setStyleSheet("");
            ui->SO2->setStyleSheet("background-color: #8FBC8F;");
            ui->SO3->setStyleSheet("");
            ui->SO4->setStyleSheet("");
            qDebug() << "已选择策略2 (MADDPG算法)";

            // 清理PathPlanner资源
            cleanupPathPlanner();

            // 初始化MADDPG
            initializeMADDPG();
        }
    });

    connect(ui->SO3, &QPushButton::clicked, this, [this]() {
        if (gameStage != "running") {
            currentStrategy = 3;
            ui->SO1->setStyleSheet("");
            ui->SO2->setStyleSheet("");
            ui->SO3->setStyleSheet("background-color: #8FBC8F;");
            ui->SO4->setStyleSheet("");
            qDebug() << "已选择策略3 (动态窗口法DW)";

            // 清理其他算法资源
            cleanupPathPlanner();
            cleanupMADDPG();
            
            // 初始化动态窗口规划器
            initializeDWPlanner();
        }
    });

    connect(ui->SO4, &QPushButton::clicked, this, [this]() {
        if (gameStage != "running") {
            currentStrategy = 4;
            ui->SO1->setStyleSheet("");
            ui->SO2->setStyleSheet("");
            ui->SO3->setStyleSheet("");
            ui->SO4->setStyleSheet("background-color: #8FBC8F;");
            qDebug() << "已选择策略4 (暂未实现)";

            // 清理所有算法资源
            cleanupPathPlanner();
            cleanupMADDPG();
        }
    });

    // 默认选中策略1
    ui->SO1->setStyleSheet("background-color: #8FBC8F;");
    // 默认初始化PathPlanner
    initializePathPlanner();

    // 初始化SO1和TargetManager
    SO1 = new Strategy1(this);
    targetManager = new TargetManager(this);
    
    // 初始化DW算法状态
    isDWPlannerInitialized = false;

    // 初始化策略管理器
    strategyManager = new UAVStrategyManager(this);

    setCompetitionMode(gridMap->showMapInCompetitionMode);         //是否显示地图
}

MainWindow::~MainWindow()
{
    // 清理算法资源
    cleanupPathPlanner();
    cleanupMADDPG();
    cleanupDWPlanner();
    delete targetManager;
    delete SO1;

    delete ui;
}

// 处理 VsoaManager 信号的槽函数
void MainWindow::onConnectionStatusChanged(bool connected, QString info)
{
    // 处理连接状态变化
    if (connected) {
        qDebug() << "已连接到服务器:" << info;
    } else {
        qDebug() << "服务器连接失败或断开:" << info;
    }
}

void MainWindow::onServerPaused(QDateTime pauseStartTime)        //服务器暂停游戏
{
    movingObjectsTimer->stop();
    serverPaused = true;
    serverPauseStartTime = pauseStartTime;
}

void MainWindow::onServerResumed(int pausedSeconds)        //暂停后继续，计算本次暂停时间
{
    movingObjectsTimer->start(500);
    serverPaused = false;
    totalPauseSeconds += pausedSeconds;
    qDebug() << "服务器恢复，本次暂停时间：" << pausedSeconds << "秒，自上次检测到红方无人机到现在总暂停时间：" << totalPauseSeconds << "秒";
}

void MainWindow::setCompetitionMode(bool isCompetition) {       //设置比赛模式是否显示地图
   if (isCompetition) {
       ui->groupBox_5->setVisible(true);
   } else {
       ui->groupBox_5->setVisible(false);
       ui->centralwidget->setFixedSize(642, 820);
   }
}

void MainWindow::sendControlCommand(const QString &uavId, const QPointF &velocity)
{
    // 使用 VsoaManager 发送控制命令
    vsoaManager->sendControlCommand(uavId, velocity);
}

void MainWindow::onGameDataUpdated(const QMap<QString, DroneInfo> &updatedDronesInfo,
                                  const QMap<QString, ObstacleInfo> &updatedObstaclesInfo,
                                  const QMap<QString, ObstacleInfo> &updatedStaticObstacles,
                                  const QMap<QString, ObstacleInfo> &updatedMovingObstacles,
                                  int updatedGameLeftTime,
                                  QString updatedGameStage)
{
    // 检查游戏状态是否变化
    bool gameStateChanged = (gameStage != updatedGameStage);

    // 更新本地数据
    dronesInfo = updatedDronesInfo;
    obstaclesInfo = updatedObstaclesInfo;
    staticObstacles = updatedStaticObstacles;
    movingObstacles = updatedMovingObstacles;
    gameLeftTime = updatedGameLeftTime;

    // 计算雷云速度
    calculateCloudVelocities();

    // 检查游戏状态是否变为running
    if (gameStateChanged && updatedGameStage == "running" && gameStage != "running") {
//        pathPlanner->resetEscapeState();
        // 重新初始化巡逻目标点
        targetManager->initPresetTargets();
        // 可以添加一个短暂延迟，等待一段时间后再规划路径
        QTimer::singleShot(10, this, [this]() {
            planPathToPresetTargets();
        });
    }

    // 检查游戏状态是否变为finish
    if (updatedGameStage == "finish" && gameStage != "finish") {
        resetRedDroneUpdateLabels = true;
        // 清空无人机位置
        gridMap->clearDronePositions();
        // 清空所有路径
        gridMap->clearPath();
        gridMap->initMap();
        // 清空静态障碍物标记容器
        staticObstacleMarkers.clear();
        // 重置所有无人机的目标点到达状态
        QStringList droneIds = {"B1", "B2", "B3"};
        for (const QString& droneId : droneIds) {
            targetManager->setTargetReached(droneId, false);
        }
    }
    gameStage = updatedGameStage;

    // 如果游戏状态发生变化，更新策略按钮状态
    if (gameStateChanged) {
        updateStrategyButtonsState();
    }

    // 更新栅格地图
    // 清空无人机位置
    gridMap->clearDronePositions();

    // 更新无人机位置
    for (auto it = dronesInfo.begin(); it != dronesInfo.end(); ++it) {
        const QString &uid = it.key();
        const DroneInfo &info = it.value();

        if (uid.startsWith("B")) {
            if (info.hp > 0) {
                // 蓝方无人机（己方）位置更新
                gridMap->addself(uid, info.x, info.y, info.hp);
            } else {
                // 蓝方无人机死亡，清空其路径
                gridMap->clearPath(uid);
            }
        } else if (uid.startsWith("R") && info.hp > 0) {
            // 红方无人机（敌方）位置更新
            // 检查是否在任何蓝方无人机的探测范围内
            bool inDetectionRange = false;
            for (auto blueIt = dronesInfo.begin(); blueIt != dronesInfo.end(); ++blueIt) {
                if (blueIt.key().startsWith("B") && blueIt.value().hp > 0) {
                    // 计算距离
                    double dx = info.x - blueIt.value().x;
                    double dy = info.y - blueIt.value().y;
                    double distance = sqrt(dx*dx + dy*dy);

                    if (distance <= 300) { // 探测范围为300
                        inDetectionRange = true;
                        gridMap->addEnemy(uid, info.x, info.y, info.hp);
                        break;
                    }
                }
            }
        }
    }

    // 更新障碍物
    // [1] 先处理静态障碍物（山体、雷达）
    for (auto it = staticObstacles.begin(); it != staticObstacles.end(); ++it) {
        const ObstacleInfo &info = it.value();
        QString obstacleId = info.id;

        // 检查该静态障碍物是否已在标记容器中
        if (!staticObstacleMarkers.contains(obstacleId)) {
            // 如果不存在，添加到标记容器中，并设置hasAdded为false
            StaticObstacleMarker marker;
            marker.id = obstacleId;
            marker.hasAdded = false;
            staticObstacleMarkers.insert(obstacleId, marker);

        }

        // 获取该障碍物的标记
        StaticObstacleMarker &marker = staticObstacleMarkers[obstacleId];

        // 如果该静态障碍物尚未添加到地图中，则添加
        if (!marker.hasAdded) {
            qDebug()<<"绘制静态障碍物中";
            // 更新地图
            emit UpdateSharedGridMap(gridMap->getSharedGridMap());
            // 存储静态障碍物位置信息，用于判断重规划
            staclePositions[info.id] = {info.x, info.y, info.r};
            if (info.type == "mountain") {
                gridMap->addObstacle(info.id, info.x, info.y, info.r, MOUNTAIN);
                marker.hasAdded = true;
            } else if (info.type == "radar") {
                gridMap->addObstacle(info.id, info.x, info.y, info.r, RADAR);
                marker.hasAdded = true;
            }
        }
    }
    // [2] 再一次性刷新移动障碍（如云）
    for (auto it = obstaclesInfo.begin(); it != obstaclesInfo.end(); ++it) {
        const ObstacleInfo &info = it.value();
        if (info.type == "cloud") {
            gridMap->addObstacle(info.id, info.x, info.y, gridMap->CLOUD_RADIUS, CLOUD);

            // 存储雷云位置信息
            staclePositions[info.id] = {info.x, info.y, 100};

            // 遍历蓝方无人机，检查路径是否在雷云范围内
            for (auto it = dronesInfo.begin(); it != dronesInfo.end(); ++it) {
                const QString &uid = it.key();

                // 只处理蓝方无人机
                if (!uid.startsWith("B") || it.value().hp <= 0) {
                    continue;
                }

                // 如果有平滑路径且不为空
                if (gridMap->m_pathMap.contains(uid) && !gridMap->m_pathMap[uid].isEmpty() &&
                    gridMap->m_smoothedPathMap.contains(uid) && gridMap->m_smoothedPathMap[uid].size() > 1) {

                    // 检查该无人机的路径是否在雷云范围内
                    if (isPathInObstacle(uid)) {
                        // 检查是否已经过了规划间隔时间
                        QTime currentTime = QTime::currentTime();
                        if (!m_lastPathPlanTime.contains(uid) ||
                            m_lastPathPlanTime[uid].msecsTo(currentTime) >= PATH_PLAN_INTERVAL) {

                            // 更新上次规划时间
                            m_lastPathPlanTime[uid] = currentTime;

                            // 只为这个无人机重规划
                            planPathForSingleDrone(uid);
                            qDebug() << uid << "路径在障碍物中，重新规划路径";
                        }
                        // 不使用break，继续检查其他无人机
                    }
                }
                else if(gridMap->m_pathMap[uid].isEmpty()) {
                    QPointF velocity(0,0);
                    sendControlCommand(uid, velocity);
                }
            }
        }
    }

    // 更新UI显示
    updateUIDisplay();

    // 根据当前策略执行相应算法
    if (gameStage == "running") {
        if (currentStrategy == 1 && isPathPlannerInitialized) {
            // 策略1: A*路径规划
            // 获取蓝方无人机的信息
            for (auto it = dronesInfo.begin(); it != dronesInfo.end(); ++it) {
                const QString &uid = it.key();

                // 只处理蓝方无人机
                if (!uid.startsWith("B") || it.value().hp <= 0) {
                    continue;
                }

                // 如果有平滑路径且不为空
                if (gridMap->m_pathMap.contains(uid) && !gridMap->m_pathMap[uid].isEmpty() &&
                    gridMap->m_smoothedPathMap.contains(uid) && gridMap->m_smoothedPathMap[uid].size() > 1) {
                    // 获取无人机当前位置
                    QPointF currentPos(it.value().x, it.value().y);

                    // 使用GridMap计算速度
                    QPointF velocity = gridMap->calculateVelocity(uid, currentPos);
                    // 发送控制命令
                    sendControlCommand(uid, velocity);
                }  else if(gridMap->m_pathMap[uid].isEmpty()){     //路径为空时飞机暂停
                    QPointF velocity(0,0);
                    sendControlCommand(uid, velocity);
                }
            }
        } else if (currentStrategy == 3 && isDWPlannerInitialized) {
            // 策略3: 动态窗口法(DW)
            // 更新障碍物信息
            dwPlanner->updateObstacles(staticObstacles, movingObstacles);
            
            // 获取蓝方无人机的信息
            for (auto it = dronesInfo.begin(); it != dronesInfo.end(); ++it) {
                const QString &uid = it.key();

                // 只处理蓝方无人机
                if (!uid.startsWith("B") || it.value().hp <= 0) {
                    continue;
                }
                
                // 获取无人机当前位置和速度
                QPointF currentPos(it.value().x, it.value().y);
                QPointF currentVel(it.value().vx, it.value().vy);
                
                // 设置DW算法的当前状态
                dwPlanner->setCurrentState(uid, currentPos, currentVel);
                
                // 获取目标点 - 从当前路径的终点作为目标
                QPointF targetPoint;
                if (gridMap->m_pathMap.contains(uid) && !gridMap->m_pathMap[uid].isEmpty()) {
                    // 从路径终点获取目标点
                    QPoint lastPathPoint = gridMap->m_pathMap[uid].last();
                    targetPoint = QPointF(lastPathPoint.x() * gridMap->GRID_SIZE, 
                                         lastPathPoint.y() * gridMap->GRID_SIZE);
                    
                    dwPlanner->setTargetPoint(uid, targetPoint);
                    
                    // 使用DW算法计算速度
                    QPointF velocity = dwPlanner->planVelocity(uid);
                    
                    // sendControlCommand在DW的信号处理函数中已经处理
                    // 这里不需要再发送
                } else if (targetManager->hasPresetTarget(uid)) {
                    // 尝试从目标管理器获取预设目标点
                    QPoint targetGridPoint = targetManager->getCurrentTarget(uid);
                    targetPoint = QPointF(targetGridPoint.x() * gridMap->GRID_SIZE,
                                         targetGridPoint.y() * gridMap->GRID_SIZE);
                                         
                    dwPlanner->setTargetPoint(uid, targetPoint);
                    
                    // 使用DW算法计算速度
                    QPointF velocity = dwPlanner->planVelocity(uid);
                } else {
                    // 如果没有目标点，停止移动
                    QPointF velocity(0, 0);
                    sendControlCommand(uid, velocity);
                }
            }
        } else if (currentStrategy == 2 && isMaddpgInitialized) {
            // 策略2: MADDPG算法
            // 静态变量，用于记录每个无人机的折射状态和持续时间
            static QMap<QString, int> reflectionCounter;
            static QMap<QString, bool> reflectX;
            static QMap<QString, bool> reflectY;

            for (auto it = dronesInfo.begin(); it != dronesInfo.end(); ++it) {
                const QString &uid = it.key();
                const DroneInfo &info = it.value();

                // 只处理蓝方无人机且HP大于0的
                if (uid.startsWith("B") && info.hp > 0) {
                    // 初始化计数器（如果不存在）
                    if (!reflectionCounter.contains(uid)) {
                        reflectionCounter[uid] = 0;
                        reflectX[uid] = false;
                        reflectY[uid] = false;
                    }

                    // 构建观察向量
                    QVector<float> observation = buildObservationForUAV(uid);

                    if (!observation.isEmpty() && uavModels.contains(uid)) {
                        // 使用UAVModel进行推理
                        QPointF velocity = uavModels[uid]->getTargetVelocity(observation);

                        // 边界检测：当无人机到达地图边界时实现镜面折射效果
                        // 地图边界：x=0或1280，y=0或800
                        bool hitBoundary = false;

                        // 检查是否碰到边界
                        // 左边界 (x=0)
                        if (info.x <= 0 && velocity.x() < 0) {
                            reflectX[uid] = true;
                            hitBoundary = true;
                        }
                        // 右边界 (x=1280)
                        else if (info.x >= 1280 && velocity.x() > 0) {
                            reflectX[uid] = true;
                            hitBoundary = true;
                        }

                        // 上边界 (y=0)
                        if (info.y <= 0 && velocity.y() < 0) {
                            reflectY[uid] = true;
                            hitBoundary = true;
                        }
                        // 下边界 (y=800)
                        else if (info.y >= 800 && velocity.y() > 0) {
                            reflectY[uid] = true;
                            hitBoundary = true;
                        }

                        // 如果碰到边界，重置计数器为20
                        if (hitBoundary) {
                            reflectionCounter[uid] = 80; // 持续20次（约1秒）
                            qDebug() << uid << "碰到边界，应用镜面折射，持续20次";
                        }

                        // 应用折射效果
                        if (reflectionCounter[uid] > 0) {
                            // 根据反射标志应用折射
                            if (reflectX[uid]) {
                                velocity.setX(-velocity.x()); // 水平方向镜面反射
                            }
                            if (reflectY[uid]) {
                                velocity.setY(-velocity.y()); // 垂直方向镜面反射
                            }

                            // 递减计数器
                            reflectionCounter[uid]--;

                            // 如果计数器归零，重置反射标志
                            if (reflectionCounter[uid] == 0) {
                                reflectX[uid] = false;
                                reflectY[uid] = false;
                            }
                        }

                        // 发送控制命令
                        sendControlCommand(uid, velocity);
                    }
                }
            }
        }else if (currentStrategy == 4) {
            // 其他策略暂未实现，不执行算法计算和发送控制命令
        }
    }
}

// 更新策略按钮状态
void MainWindow::updateStrategyButtonsState() {
    // 如果游戏正在运行，禁用所有策略按钮
    bool enabled = (gameStage != "running");

    ui->SO1->setEnabled(enabled);
    ui->SO2->setEnabled(enabled);
    ui->SO3->setEnabled(enabled);
    ui->SO4->setEnabled(enabled);
}

void MainWindow::clearStaleMovingObjects() {        // 清除500ms秒未更新的移动障碍物
    if (gridMap) {
        gridMap->clearStaleObstacles(500);
    }
}

void MainWindow::updateUIDisplay()           //更新自动飞行终端ui界面显示
{
    // 更新游戏状态和时间显示，使用HTML格式增强显示效果
    QString gameStatusText;
    if (gameStage == "running") {
        gameStatusText = "<span style='color:green;'>运行中</span>";
    } else if (gameStage == "pause") {
        gameStatusText = "<span style='color:orange;'>已暂停</span>";
    } else if (gameStage == "finish") {
        gameStatusText = "<span style='color:red;'>已结束</span>";
    } else {
        gameStatusText = "<span style='color:blue;'>初始化</span>";
    }

    ui->time_status->setText(QString("游戏状态: %1, 剩余时间: <span style='color:blue; font-weight:bold;'>%2</span>秒")
                            .arg(gameStatusText)
                            .arg(gameLeftTime));
    ui->time_status->setTextFormat(Qt::RichText);
    // 更新蓝方无人机信息
    QMap<QString, QProgressBar*> blueHpBars = {
        {"B1", ui->Bhp1},
        {"B2", ui->Bhp2},
        {"B3", ui->Bhp3}
    };

    QMap<QString, QLabel*> bluePoints = {
        {"B1", ui->Bponit1},
        {"B2", ui->Bponit2},
        {"B3", ui->Bponit3}
    };

    QMap<QString, QLabel*> blueVelocities = {
        {"B1", ui->Bv1},
        {"B2", ui->Bv1_2},
        {"B3", ui->Bv1_3}
    };

    QMap<QString, QLabel*> blueStatus = {
        {"B1", ui->Bstat1},
        {"B2", ui->Bstat2},
        {"B3", ui->Bstat3}
    };

    // 更新红方无人机信息
    QMap<QString, QProgressBar*> redHpBars = {
        {"R1", ui->Rhp1},
        {"R2", ui->Rhp2},
        {"R3", ui->Rhp3}
    };

    QMap<QString, QLabel*> redPoints = {
        {"R1", ui->Rponit1},
        {"R2", ui->Rponit2},
        {"R3", ui->Rponit3}
    };

//    QMap<QString, QLabel*> redVelocities = {
//        {"R1", ui->Bv1_6},
//        {"R2", ui->Bv1_5},
//        {"R3", ui->Bv1_4}
//    };

    QMap<QString, QLabel*> redStatus = {
        {"R1", ui->Rstat1},
        {"R2", ui->Rstat2},
        {"R3", ui->Rstat3}
    };

    QMap<QString, QLabel*> redUpdateTimes = {
        {"R1", ui->updataetime1},
        {"R2", ui->updataetime2},
        {"R3", ui->updataetime3}
    };

    // 当前时间，用于计算红方无人机信息更新时间
    QDateTime currentTime = QDateTime::currentDateTime();

    // 静态变量，用于记录红方无人机上次被探测到的时间
    static QMap<QString, QDateTime> lastRedDroneDetectionTime;
    // 静态变量，用于记录红方无人机是否曾经被探测到
    static QMap<QString, bool> redDroneEverDetected;
    // 静态变量，用于记录红方无人机最后一次在视野中的时间
    static QMap<QString, QDateTime> lastRedDroneInSightTime;

    // 如果游戏状态变为finish，重置红方无人机状态
    if (resetRedDroneUpdateLabels) {
        // 重置所有红方无人机的状态
        for (auto it = redUpdateTimes.begin(); it != redUpdateTimes.end(); ++it) {
            QString uid = it.key();
            totalPauseSeconds = 0;
            // 重置更新时间标签为"未更新"
            redUpdateTimes[uid]->setText("未更新");
            // 重置血量为0
            redHpBars[uid]->setValue(0);
            // 重置坐标
            redPoints[uid]->setText("unknown");
            // 重置状态为unknown
            redStatus[uid]->setText("无");
            redStatus[uid]->setStyleSheet(""); // 清除背景色样式

            // 清除探测记录
            redDroneEverDetected[uid] = false;
            lastRedDroneDetectionTime.remove(uid);
            lastRedDroneInSightTime.remove(uid);
        }

        // 重置标志
        resetRedDroneUpdateLabels = false;
    }

    // 状态标签样式设置
    QString baseStyle = "min-width: 16px; min-height: 16px; max-width: 16px; max-height: 16px; border-radius: 8px;";

    // 遍历所有无人机信息并更新UI
    for (auto it = dronesInfo.begin(); it != dronesInfo.end(); ++it) {
        QString uid = it.key();
        DroneInfo info = it.value();

        // 根据无人机ID前缀判断是蓝方还是红方
        if (uid.startsWith("B")) {
            // 蓝方无人机
            if (blueHpBars.contains(uid)) {
                // 更新血量
                blueHpBars[uid]->setValue(info.hp);

                // 更新位置
                bluePoints[uid]->setText(QString("(%1,%2)").arg(info.x, 0, 'f', 0).arg(info.y, 0, 'f', 0));

                // 更新速度 - 如果坠毁则清零速度显示
                if (info.hp <= 0) {
                    blueVelocities[uid]->setText(QString("vx:00,vy:00"));
                    // 同时在数据结构中也清零速度值
                    dronesInfo[uid].vx = 0.0;
                    dronesInfo[uid].vy = 0.0;
                } else {
                    blueVelocities[uid]->setText(QString("vx:%1,vy:%2").arg(info.vx, 0, 'f', 0).arg(info.vy, 0, 'f', 0));
                }

                // 判断无人机状态并设置颜色
                QString statusColor = "green"; // 默认安全状态为绿色

                // 1. 检查是否坠毁
                if (info.hp <= 0) {
                    statusColor = "gray"; // 坠毁状态为灰色
                } else {
                    // 2. 检查是否在雷达范围内（紫色）
                    bool inRadarRange = false;
                    // 遍历所有障碍物，查找雷达
                    for (auto obstIt = obstaclesInfo.begin(); obstIt != obstaclesInfo.end(); ++obstIt) {
                        if (obstIt.value().type == "radar") {
                            // 计算与雷达的距离
                            double dx = info.x - obstIt.value().x;
                            double dy = info.y - obstIt.value().y;
                            double distance = sqrt(dx*dx + dy*dy);

                            if (distance <= 80) { // 雷达范围半径为80
                                inRadarRange = true;
                                statusColor = "purple";
                                break;
                            }
                        }
                    }

                    // 3. 检查是否在雷云范围内（黄色）
                    if (!inRadarRange) {
                        bool inCloudRange = false;
                        // 遍历所有障碍物，查找雷云
                        for (auto obstIt = obstaclesInfo.begin(); obstIt != obstaclesInfo.end(); ++obstIt) {
                            if (obstIt.value().type == "cloud") {
                                // 计算与雷云的距离
                                double dx = info.x - obstIt.value().x;
                                double dy = info.y - obstIt.value().y;
                                double distance = sqrt(dx*dx + dy*dy);

                                if (distance <= 80) { // 雷云范围半径为80
                                    inCloudRange = true;
                                    statusColor = "yellow";
                                    break;
                                }
                            }
                        }

                        // 4. 检查是否在敌方无人机攻击范围内（红色）
                        if (!inCloudRange) {
                            // 遍历所有红方无人机
                            for (auto droneIt = dronesInfo.begin(); droneIt != dronesInfo.end(); ++droneIt) {
                                if (droneIt.key().startsWith("R")) {
                                    // 计算与敌方无人机的距离
                                    double dx = info.x - droneIt.value().x;
                                    double dy = info.y - droneIt.value().y;
                                    double distance = sqrt(dx*dx + dy*dy);

                                    if (distance <= 150) { // 敌方攻击范围半径为150
                                        statusColor = "red";
                                        break;
                                    }
                                }
                            }
                        }
                    }
                }

                // 设置圆形状态指示器
                blueStatus[uid]->setText(""); // 清空文本
                blueStatus[uid]->setStyleSheet(baseStyle + QString("background-color: %1;").arg(statusColor));
            }
        } else if (uid.startsWith("R")) {
            // 红方无人机
            if (redHpBars.contains(uid)) {
                // 更新血量
                redHpBars[uid]->setValue(info.hp);

                // 更新位置
                redPoints[uid]->setText(QString("(%1,%2)").arg(info.x, 0, 'f', 0).arg(info.y, 0, 'f', 0));

//                // 更新速度
//                redVelocities[uid]->setText(QString("vx:%1,vy:%2").arg(info.vx, 0, 'f', 2).arg(info.vy, 0, 'f', 2));

                // 判断无人机状态并设置颜色
                QString statusColor = "green"; // 默认安全状态为绿色

                // 检查是否坠毁
                if (info.hp <= 0) {
                    statusColor = "gray"; // 坠毁状态为灰色
                }

                // 设置圆形状态指示器
                redStatus[uid]->setText(""); // 清空文本
                redStatus[uid]->setStyleSheet(baseStyle + QString("background-color: %1;").arg(statusColor));

                // 更新探测时间
                // 标记该无人机已被探测到
                redDroneEverDetected[uid] = true;
                // 更新最后探测时间为当前时间
                lastRedDroneDetectionTime[uid] = currentTime;

                // 更新最后在视野中的时间（这个时间用于计算"多少秒前"）
                lastRedDroneInSightTime[uid] = currentTime;

                totalPauseSeconds = 0;

                // 当前正在探测到，显示"更新：刚刚"
                redUpdateTimes[uid]->setText("更新：刚刚");
            }
        }
    }

    // 检查是否有未更新的无人机，设置默认值
    // 蓝方无人机
    for (auto it = blueHpBars.begin(); it != blueHpBars.end(); ++it) {
        QString uid = it.key();
        if (!dronesInfo.contains(uid)) {
            // 无人机不存在或已被击毁，显示默认值
            blueHpBars[uid]->setValue(0);
            bluePoints[uid]->setText("(00,00)");
            blueVelocities[uid]->setText("vx:00,vy:00");

            // 设置为灰色圆点（坠毁状态）
            blueStatus[uid]->setText("");
            blueStatus[uid]->setStyleSheet(baseStyle + "background-color: gray;");
        }
    }

    // 红方无人机
    for (auto it = redHpBars.begin(); it != redHpBars.end(); ++it) {
        QString uid = it.key();
        if (!dronesInfo.contains(uid)) {
            // 无人机不在当前探测范围内

            // 如果曾经探测到过，使用上次记录的状态信息并显示时间
            if (redDroneEverDetected.value(uid, false)) {
                // 计算自上次探测到的时间差（秒）
                if (lastRedDroneInSightTime.contains(uid)) {
                    QDateTime currentTime = QDateTime::currentDateTime();

                    // 计算时间差（秒）
                    int secsPassed = lastRedDroneInSightTime[uid].secsTo(currentTime);

                    // 减去暂停的时间
                    if (serverPaused) {
                        // 如果当前是暂停状态，减去已上次检测到红方到现在累计的暂停时间和当前正在进行的暂停时间
                        int currentPauseTime = serverPauseStartTime.secsTo(currentTime);
                        secsPassed -= (totalPauseSeconds + currentPauseTime);
                    } else {
                        // 如果当前不是暂停状态，减去已上次检测到红方到现在累计的所有暂停时间
                        secsPassed -= totalPauseSeconds;
                    }

                    // 确保时间不为负数
                    if (secsPassed < 0) {
                        secsPassed = 0;
                    }

                    // 更新显示
                    redUpdateTimes[uid]->setText(QString("更新：%1s前").arg(secsPassed));
                } else {
                    // 如果没有记录最后在视野中的时间（不应该发生），显示未更新
                    redUpdateTimes[uid]->setText("未更新");
                }
            } else {
                // 如果从未探测到过，显示未更新
                redUpdateTimes[uid]->setText("未更新");

                redStatus[uid]->setText("");
                redStatus[uid]->setStyleSheet(baseStyle + "background-color: gray;"); // 清除背景色样式
            }
        }
    }
    ui->time_status->setText(QString("游戏状态: %1, 剩余时间: %2秒").arg(gameStage).arg(gameLeftTime));
}


//*****************算法1 A* 处理路径规划信号的槽函数****************
// 处理目标点设置 - 支持指定无人机ID
void MainWindow::onTargetPointSet(const QPoint& targetPoint, const QString& droneId) {
    // 如果游戏状态为finish，则不处理路径规划请求
    if (gameStage == "finish"||gameStage == "init") {
        qDebug() << "游戏已结束或未开始，无法为无人机" << droneId << "规划路径";
        return;
    }
    // 将点击位置设为终点
    m_targetPoint = targetPoint;
    m_hasValidTargetPoint = true;
    qDebug() << "为无人机" << droneId << "设置终点:" << m_targetPoint;
    
    // 如果使用DW算法，也设置DW规划器的目标点
    if (currentStrategy == 3 && isDWPlannerInitialized) {
        QPointF pixelTarget(m_targetPoint.x() * gridMap->GRID_SIZE, m_targetPoint.y() * gridMap->GRID_SIZE);
        dwPlanner->setTargetPoint(droneId, pixelTarget);
        qDebug() << "DW规划器设置无人机" << droneId << "目标点:" << pixelTarget;
    }

    // 清除指定无人机的路径
    gridMap->clearPath(droneId);

    if (!droneId.isEmpty() && dronesInfo.contains(droneId) && dronesInfo[droneId].hp > 0) {
        // 使用指定的无人机作为起点
        int gridCol = dronesInfo[droneId].x / gridMap->GRID_SIZE;
        int gridRow = dronesInfo[droneId].y / gridMap->GRID_SIZE;
        m_startPoint = QPoint(gridCol, gridRow);
        qDebug() << "使用无人机" << droneId << "位置作为起点:" << m_startPoint;
    } else {
        // 如果指定的无人机不可用，使用第一个找到的蓝方无人机
        for (auto it = dronesInfo.begin(); it != dronesInfo.end(); ++it) {
            const QString &uid = it.key();
            const DroneInfo &info = it.value();

            if (uid.startsWith("B") && info.hp > 0) {
                // 将无人机位置转换为栅格坐标
                int gridCol = info.x / gridMap->GRID_SIZE;
                int gridRow = info.y / gridMap->GRID_SIZE;
                m_startPoint = QPoint(gridCol, gridRow);

                qDebug() << "使用无人机" << uid << "位置作为起点:" << m_startPoint;
                break; // 只使用第一个找到的蓝方无人机
            }
        }
    }

    // 更新地图
    emit UpdateSharedGridMap(gridMap->getSharedGridMap());

    // 调用路径规划
    emit StartfindPath(m_startPoint, m_targetPoint, droneId);
}

// 处理路径规划完成 - 支持指定无人机ID
void MainWindow::onPathPlanned(const QVector<QPoint>& path, const QString& droneId) {
//    qDebug() << "无人机" << droneId << "路径规划完成，路径点数量:" << path.size();

    // 在地图上显示路径，使用传入的无人机ID
    gridMap->setPath(path, droneId);

    // 重置起点和终点标志，准备下一次规划
    m_hasValidStartPoint = false;
    m_hasValidTargetPoint = false;
}

// 初始化PathPlanner
void MainWindow::initializePathPlanner() {
    connect(gridMap, &GridMap::targetReached, this, &MainWindow::onTargetReached);
    if (!isPathPlannerInitialized) {
        // 创建PathPlanner实例
        pathPlanner = new PathPlanner(this);

        // 连接信号和槽，注意使用新的带有无人机ID参数的信号
        connect(gridMap, &GridMap::targetPointSet, this, &MainWindow::onTargetPointSet);
        connect(this, &MainWindow::StartfindPath, pathPlanner, &PathPlanner::onStartfindPath);
        connect(this, &MainWindow::UpdateSharedGridMap, pathPlanner, &PathPlanner::onUpdateSharedGridMap);
        connect(pathPlanner, &PathPlanner::pathPlanned, this, &MainWindow::onPathPlanned);
        connect(gridMap, &GridMap::mapUpdated, pathPlanner, &PathPlanner::onMapUpdated);
        connect(gridMap, &GridMap::mapReset, pathPlanner, &PathPlanner::onMapReset);

        // 设置共享地图
        pathPlanner->setSharedGridMap(gridMap->getSharedGridMap());

        isPathPlannerInitialized = true;
        qDebug() << "PathPlanner初始化完成";
    }
}
// 清理PathPlanner
void MainWindow::cleanupPathPlanner() {
    if (isPathPlannerInitialized) {
        // 断开信号连接
        disconnect(gridMap, &GridMap::targetPointSet, this, &MainWindow::onTargetPointSet);
        disconnect(this, &MainWindow::StartfindPath, pathPlanner, &PathPlanner::onStartfindPath);
        disconnect(this, &MainWindow::UpdateSharedGridMap, pathPlanner, &PathPlanner::onUpdateSharedGridMap);
        disconnect(pathPlanner, &PathPlanner::pathPlanned, this, &MainWindow::onPathPlanned);
        disconnect(gridMap, &GridMap::mapUpdated, pathPlanner, &PathPlanner::onMapUpdated);
        disconnect(gridMap, &GridMap::mapReset, pathPlanner, &PathPlanner::onMapReset);

        // 清除路径
        gridMap->clearPath();

        // 删除PathPlanner实例
        delete pathPlanner;
        pathPlanner = nullptr;

        isPathPlannerInitialized = false;
        qDebug() << "PathPlanner资源已清理";
    }
}

//使用预设目标点进行路径规划循环巡逻
void MainWindow::planPathToPresetTargets() {
    // 检查游戏是否正在运行
    if (gameStage != "running") {
        qDebug() << "游戏未运行，无法规划路径";
        return;
    }

    QStringList blueUAVs = {"B1", "B2", "B3"};

    // 首先更新共享地图，所有无人机共用同一份地图数据
    emit UpdateSharedGridMap(gridMap->getSharedGridMap());

    // 使用QtConcurrent::run并行处理每个无人机的路径规划
    for (const QString& droneId : blueUAVs) {
        if (!dronesInfo.contains(droneId) || dronesInfo[droneId].hp <= 0) {
            continue;
        }

        // 使用QtConcurrent::run在单独的线程中处理每个无人机的路径规划请求
        QtConcurrent::run([=]() {
            int gridCol = dronesInfo[droneId].x / gridMap->GRID_SIZE;
            int gridRow = dronesInfo[droneId].y / gridMap->GRID_SIZE;
            QPoint startPoint(gridCol, gridRow);

            // 确定目标点 - 先尝试追踪，若无法追踪则巡逻
            QPoint targetPoint;

            // 游戏开始先进入巡逻模式
            targetPoint = targetManager->getPatrolPoint(droneId);

            // 在主线程中执行UI相关操作
            QMetaObject::invokeMethod(this, [=]() {
                gridMap->clearPath(droneId);

                // 发送路径规划请求
                emit StartfindPath(startPoint, targetPoint, droneId);
            }, Qt::QueuedConnection);
        });
    }
}
void MainWindow::planPathForSingleDrone(const QString &droneId) {
    // 检查游戏是否正在运行
    if (gameStage != "running") {
        qDebug() << "游戏未运行，无法规划路径";
        return;
    }

    // 检查该无人机是否存在且血量大于0
    if (!dronesInfo.contains(droneId) || dronesInfo[droneId].hp <= 0) {
        return;
    }

    // 更新共享地图
    emit UpdateSharedGridMap(gridMap->getSharedGridMap());

    // 检查敌方无人机位置并更新到SO1
    QMap<QString, QPoint> enemyPositions;
    QMap<QString, int> enemyHp;
    bool hasEnemyUAV = false;

    for (auto it = dronesInfo.begin(); it != dronesInfo.end(); ++it) {
        const QString &id = it.key();
        const DroneInfo &info = it.value();

        if (id.startsWith("R") && info.hp > 0) {
            // 转换为栅格坐标
            QPoint gridPos(info.x / gridMap->GRID_SIZE, info.y / gridMap->GRID_SIZE);
            enemyPositions[id] = gridPos;
            enemyHp[id] = info.hp;
            hasEnemyUAV = true;

            // 如果检测到了R1，R2，R3，则切换到追踪模式
            if (id == "R1"||id == "R2"||id == "R3") {
                if(currentStrategy == 1){
                   SO1->setTrackingMode(true);
                }else if (currentStrategy == 2){

                }else if (currentStrategy == 3){
                    // 当使用DW算法时，设置是否考虑障碍物
                    if (isDWPlannerInitialized) {
                        dwPlanner->setConsiderObstacles(true);
                        dwPlanner->setConsiderMovingObstacles(true);
                    }
                }else if (currentStrategy == 4){

                }
            }
        }
    }

    // 更新敌方无人机位置和血量到SO1
    SO1->updateEnemyInfo(enemyPositions, enemyHp);

    // 如果没有检测到敌方无人机，切换回巡逻模式
    if (!hasEnemyUAV) {
        SO1->setTrackingMode(false);
    }

    // 使用QtConcurrent::run在单独的线程中处理该无人机的路径规划请求
    QtConcurrent::run([=]() {
        int gridCol = dronesInfo[droneId].x / gridMap->GRID_SIZE;
        int gridRow = dronesInfo[droneId].y / gridMap->GRID_SIZE;
        QPoint startPoint(gridCol, gridRow);

        // 确定目标点 - 先尝试追踪，若无法追踪则巡逻
        QPoint targetPoint;
        if (SO1->isDroneTracking(droneId)) {
             // 尝试获取追踪目标
            QPoint trackingTarget = SO1->getBestTrackingTarget(droneId);

            if (trackingTarget != QPoint(0, 0)) {
                // 有可跟踪目标
                targetPoint =trackingTarget;
                qDebug() << droneId << "正在追踪敌方无人机，目标点:" << targetPoint;
            } else {
                // 无可跟踪目标，切换到巡逻+
                targetPoint = targetManager->getPatrolPoint(droneId);
                qDebug() << droneId << "无法追踪敌方，切换到巡逻模式，目标点:" << targetPoint;
            }
        } else {
            // 巡逻模式
            targetPoint = targetManager->getPatrolPoint(droneId);
        }

        // 在主线程中执行UI相关操作和发送信号
        QMetaObject::invokeMethod(this, [=]() {
            gridMap->clearPath(droneId);

            // 发送路径规划请求
            emit StartfindPath(startPoint, targetPoint, droneId);
        }, Qt::QueuedConnection);
    });
}

void MainWindow::onTargetReached(const QString& droneId) {
    // 更新TargetManager中的状态
    targetManager->setTargetReached(droneId, true);
    qDebug() << droneId << "已到达目标点，准备规划下一个目标";

    if (currentStrategy == 3 && isDWPlannerInitialized) {
        // 如果使用DW算法，获取下一个巡逻目标点
        QPoint nextTarget = targetManager->getNextPatrolTarget(droneId);
        QPointF pixelTarget(nextTarget.x() * gridMap->GRID_SIZE, nextTarget.y() * gridMap->GRID_SIZE);
        
        // 设置DW规划器的目标点
        dwPlanner->setTargetPoint(droneId, pixelTarget);
        qDebug() << "DW规划器为无人机" << droneId << "设置下一个巡逻目标点:" << pixelTarget;
    } else {
        // 其他算法：只为到达终点的这个无人机重新规划路径
        planPathForSingleDrone(droneId);
    }
}


bool MainWindow::isPathInObstacle(const QString &droneId) {
    // 如果无人机没有路径，返回false
    if (!gridMap->m_smoothedPathMap.contains(droneId) ||
        gridMap->m_smoothedPathMap[droneId].size() <= 1) {
        return false;
    }

    // 获取无人机的平滑路径
    const QVector<QPointF> &path = gridMap->m_smoothedPathMap[droneId];

    // 获取无人机当前位置
    if (!dronesInfo.contains(droneId)) {
        return false; // 无人机信息不存在
    }
    QPointF currentPos(dronesInfo[droneId].x, dronesInfo[droneId].y);

    // 找到距离当前位置最近的路径点索引
    int currentIndex = gridMap->findClosestPathPointIndex(currentPos, droneId);

    // 遍历所有障碍物
    for (auto it = staclePositions.begin(); it != staclePositions.end(); ++it) {
        const stacleInfo &obstacle = it.value();

        // 只检查当前位置之后的路径点（未飞行的路径）
        for (int i = currentIndex; i < path.size(); i++) {
            const QPointF &point = path[i];
            // 使用GridMap的isPointInCircle方法检查点是否在障碍物范围内
            if (gridMap->isPointInCircle(point.x(), point.y(), obstacle.x, obstacle.y, obstacle.radius)) {
                return true; // 未飞行的路径与障碍物相交
            }
        }
    }

    return false; // 未飞行的路径不与任何障碍物相交
}

// 构建18维向量的方法
QVector<float> MainWindow::buildObservationVector()
{
    QVector<float> observation(18, 0.0f);

    // 前9个元素：我方无人机B1, B2, B3的栅格位置和血量
    QStringList blueUAVs = {"B1", "B2", "B3"};
    for (int i = 0; i < 3; ++i) {
        const QString &droneId = blueUAVs[i];
        int baseIndex = i * 3;

        if (dronesInfo.contains(droneId) && dronesInfo[droneId].hp > 0) {
            // 无人机存在且血量大于0
            observation[baseIndex] = dronesInfo[droneId].x / gridMap->GRID_SIZE;     // gridCol
            observation[baseIndex + 1] = dronesInfo[droneId].y / gridMap->GRID_SIZE; // gridRow
            observation[baseIndex + 2] = dronesInfo[droneId].hp;                     // hp
        } else {
            // 无人机坠机，用0补齐
            observation[baseIndex] = 0.0f;
            observation[baseIndex + 1] = 0.0f;
            observation[baseIndex + 2] = 0.0f;
        }
    }

    // 后9个元素：敌方无人机R1, R2, R3的栅格位置和血量
    QStringList redUAVs = {"R1", "R2", "R3"};
    for (int i = 0; i < 3; ++i) {
        const QString &droneId = redUAVs[i];
        int baseIndex = 9 + i * 3;

        if (dronesInfo.contains(droneId) && dronesInfo[droneId].hp > 0) {
            // 当前探测到敌方无人机且血量大于0
            observation[baseIndex] = dronesInfo[droneId].x / gridMap->GRID_SIZE;     // gridCol
            observation[baseIndex + 1] = dronesInfo[droneId].y / gridMap->GRID_SIZE; // gridRow
            observation[baseIndex + 2] = dronesInfo[droneId].hp;                     // hp
        } else {
            // 未探测到敌方无人机，设置为-1
            observation[baseIndex] = -1.0f;
            observation[baseIndex + 1] = -1.0f;
            observation[baseIndex + 2] = -1.0f;
        }
    }

    return observation;
}

//*****************算法2 maddpg算法uavmodal相关的槽函数****************

QVector<float> MainWindow::buildObservationForUAV(const QString &uavId)
{
    if (!dronesInfo.contains(uavId)) {
        return QVector<float>();
    }

    const DroneInfo &self = dronesInfo[uavId];
    QPointF selfPos(self.x, self.y);
    QPointF selfVel(self.vx, self.vy);

    // 友方信息
    QVector<QPointF> friendlyPos;
    QVector<float> friendlyHp;
    for (auto it = dronesInfo.begin(); it != dronesInfo.end(); ++it) {
        const QString &id = it.key();
        const DroneInfo &info = it.value();
        if (id.startsWith("B") && id != uavId && info.hp > 0) {
            friendlyPos.append(QPointF(info.x, info.y));
            friendlyHp.append(info.hp / 100.0f);
        }
    }

    // 敌方信息
    QVector<QPointF> enemyPos;
    QVector<float> enemyHp;
    for (auto it = dronesInfo.begin(); it != dronesInfo.end(); ++it) {
        const QString &id = it.key();
        const DroneInfo &info = it.value();
        if (id.startsWith("R") && info.hp > 0) {
            enemyPos.append(QPointF(info.x, info.y));
            enemyHp.append(info.hp / 100.0f);
        }
    }

    // 障碍信息
    QVector<QPointF> obstaclePos;
    QVector<float> obstacleRadius;

//    QVector<QPointF> cloudVelocityVectors; // 新增：存储雷云速度向量
//    for (auto it = obstaclesInfo.begin(); it != obstaclesInfo.end(); ++it) {
//        const ObstacleInfo &info = it.value();
//        if (info.type == "radar" || info.type == "cloud") {
//            obstaclePos.append(QPointF(info.x, info.y));
//            obstacleRadius.append(info.r / 100.0f);

//            // 如果是雷云，添加速度信息
//            if (info.type == "cloud" && cloudVelocities.contains(info.id) &&
//                cloudVelocities[info.id].hasValidVelocity) {
//                cloudVelocityVectors.append(cloudVelocities[info.id].velocity);
//            } else {
//                // 如果没有速度信息，添加零向量
//                cloudVelocityVectors.append(QPointF(0, 0));
//            }

//            if (obstaclePos.size() >= 2) break;
//        }
//    }
//    // 5. 障碍物信息
//    off = observation.size();
//    for (int i = 0; i < 2; i++) {
//        if (i < obstaclePos.size()) {
//            float relX = (obstaclePos[i].x() - selfPos.x()) / 1280.0f;
//            float relY = (obstaclePos[i].y() - selfPos.y()) / 800.0f;
//            observation.append(relX); observation.append(relY); observation.append(obstacleRadius[i]);

//            // 添加雷云速度信息（归一化）
//            if (i < cloudVelocityVectors.size()) {
//                observation.append(cloudVelocityVectors[i].x() / 50.0f); // 归一化x速度分量
//                observation.append(cloudVelocityVectors[i].y() / 50.0f); // 归一化y速度分量
//            } else {
//                observation.append(0.0f); observation.append(0.0f);
//            }
//        } else {
//            observation.append(0.0f); observation.append(0.0f); observation.append(0.0f);
//            observation.append(0.0f); observation.append(0.0f); // 雷云速度为0
//        }
//    }

    for (auto it = obstaclesInfo.begin(); it != obstaclesInfo.end(); ++it) {
        const ObstacleInfo &info = it.value();
        if (info.type == "radar" || info.type == "cloud") {
            obstaclePos.append(QPointF(info.x, info.y));
            obstacleRadius.append(info.r / 100.0f);
            if (obstaclePos.size() >= 2) break;
        }
    }

    // 1. 自身信息
    QVector<float> observation;
    observation.append(self.x / 1280.0f);
    observation.append(self.y / 800.0f);
    observation.append(self.vx / 50.0f);
    observation.append(self.vy / 50.0f);
    observation.append(self.hp / 100.0f);
//    qDebug() << "自身向量:" << observation.mid(0, 5)<<"uid:"<<uavId;

    // 2. 队友信息
    int off = observation.size();
    for (int i = 0; i < 2; i++) {
        if (i < friendlyPos.size()) {
            float relX = (friendlyPos[i].x() - selfPos.x()) / 1280.0f;
            float relY = (friendlyPos[i].y() - selfPos.y()) / 800.0f;
            observation.append(relX); observation.append(relY); observation.append(friendlyHp[i]);
        } else {
            observation.append(0.0f); observation.append(0.0f); observation.append(0.0f);
        }
    }
//    qDebug() << "队友向量:" << observation.mid(off, 6);

    // 3. 敌机信息
    off = observation.size();
    for (int i = 0; i < 3; i++) {
        if (i < enemyPos.size()) {
            float relX = (enemyPos[i].x() - selfPos.x()) / 1280.0f;
            float relY = (enemyPos[i].y() - selfPos.y()) / 800.0f;
            observation.append(relX); observation.append(relY); observation.append(enemyHp[i]);
        } else {
            observation.append(0.0f); observation.append(0.0f); observation.append(0.0f);
        }
    }
//    qDebug() << "敌机向量:" << observation.mid(off, 9);

    // 4. 目标向量
    float relX = 0, relY = 0;
    if (!enemyPos.isEmpty()) {
        int nearestIdx = 0; float minDist = std::numeric_limits<float>::max();
        for (int i = 0; i < enemyPos.size(); i++) {
            float dx = enemyPos[i].x() - selfPos.x();
            float dy = enemyPos[i].y() - selfPos.y();
            float dist = sqrt(dx*dx + dy*dy);
            if (dist < minDist) {minDist = dist; nearestIdx = i;}
        }
        relX = (enemyPos[nearestIdx].x() - selfPos.x()) / 1280.0f;
        relY = (enemyPos[nearestIdx].y() - selfPos.y()) / 800.0f;
        observation.append(relX); observation.append(relY);
    } else {
        observation.append(0.0f); observation.append(0.0f);
    }
//    qDebug() << "目标向量:" << relX << relY;

    // 5. 障碍物信息
    off = observation.size();
    for (int i = 0; i < 2; i++) {
        if (i < obstaclePos.size()) {
            float relX = (obstaclePos[i].x() - selfPos.x()) / 1280.0f;
            float relY = (obstaclePos[i].y() - selfPos.y()) / 800.0f;
            observation.append(relX); observation.append(relY); observation.append(obstacleRadius[i]);
        } else {
            observation.append(0.0f); observation.append(0.0f); observation.append(0.0f);
        }
    }
//    qDebug() << "障碍向量:" << observation.mid(off, 6);

    // 6. 新增：边界距离信息（3维）
    off = observation.size();
    // 到左边界的归一化距离
    float distToLeftBoundary = selfPos.x() / 1280.0f;
    observation.append(distToLeftBoundary);

    // 到右边界的归一化距离
    float distToRightBoundary = (1280.0f - selfPos.x()) / 1280.0f;
    observation.append(distToRightBoundary);

    // 到上边界的归一化距离
    float distToTopBoundary = selfPos.y() / 800.0f;
    observation.append(distToTopBoundary);

//    qDebug() << "边界距离向量:" << observation.mid(off, 3);
//    qDebug() << "31维总向量:" << observation;

    return observation;
}

void MainWindow::initializeUAVModels()
{
    // 清理现有的UAVModel实例
    for (auto it = uavModels.begin(); it != uavModels.end(); ++it) {
        delete it.value();
    }
    uavModels.clear();

    // 只有策略2才加载MADDPG模型
    if (currentStrategy == 2) {
        // 为每个蓝方无人机创建模型实例
        QStringList blueUAVs = {"B1", "B2", "B3"};

        for (int i = 0; i < blueUAVs.size(); i++) {
            QString uavId = blueUAVs[i];
            int agentId = i; // 对应agent_0, agent_1, agent_2

            // 创建模型实例
            model = new UAVModel(agentId, this);

            // 设置运动参数
            model->setMaxVelocity(50.0f); // 最大速度(像素/秒)
            model->setTimeStep(0.1f);     // 时间步长(秒)

            // 加载模型参数
            QString modelPath = QString(":/agent_%1_actor_inference.json").arg(agentId);
            bool success = model->loadModel(modelPath);

            if (success) {
                qDebug() << "无人机" << uavId << "模型加载成功";
                uavModels[uavId] = model;
            } else {
                qDebug() << "无人机" << uavId << "模型加载失败";
                delete model;
            }
        }
    } else {
        qDebug() << "策略" << currentStrategy << "暂未实现，不加载模型";
    }
}

// 初始化MADDPG
void MainWindow::initializeMADDPG() {
    if (!isMaddpgInitialized) {
        // 初始化UAV模型
        initializeUAVModels();

        isMaddpgInitialized = true;
        qDebug() << "MADDPG初始化完成";
    }
}
// 清理MADDPG
void MainWindow::cleanupMADDPG() {
    if (isMaddpgInitialized) {
        // 清理UAV模型
        for (auto it = uavModels.begin(); it != uavModels.end(); ++it) {
            delete it.value();
        }
        uavModels.clear();

        isMaddpgInitialized = false;
        qDebug() << "MADDPG资源已清理";
    }
}

// 初始化DW规划器
void MainWindow::initializeDWPlanner() {
    if (isDWPlannerInitialized) {
        return;
    }

    qDebug() << "初始化动态窗口规划器(DWPlanner)...";
    
    // 创建DWPlanner实例
    dwPlanner = new DWPlanner(this);
    
    // 初始化动态窗口规划器，传入栅格地图指针
    dwPlanner->initialize(gridMap);
    
    // 连接速度计算信号
    connect(dwPlanner, &DWPlanner::velocityComputed, 
            this, &MainWindow::onVelocityComputed);
    
    // 设置DW算法参数
    dwPlanner->getDWInstance()->setMotionParams(
        5.0,  // 最大速度
        2.0,  // 最大加速度
        1.0   // 最大偏航角速度
    );
    
    // 设置评价函数权重
    dwPlanner->getDWInstance()->setWeights(
        0.4,  // 朝向权重
        0.2,  // 距离权重
        0.1,  // 速度权重
        0.6   // 障碍物权重
    );
    
    // 设置是否考虑障碍物
    dwPlanner->setConsiderObstacles(true);
    dwPlanner->setConsiderMovingObstacles(true);
    
    // 设置移动障碍物(雷云)的影响范围扩展值
    dwPlanner->setMovingObstacleExtension(10.0);
    
    isDWPlannerInitialized = true;
    qDebug() << "动态窗口规划器(DWPlanner)初始化完成";
}

// 清理DWPlanner
void MainWindow::cleanupDWPlanner() {
    if (!isDWPlannerInitialized) {
        return;
    }
    
    if (dwPlanner) {
        disconnect(dwPlanner, &DWPlanner::velocityComputed, 
                this, &MainWindow::onVelocityComputed);
        delete dwPlanner;
        dwPlanner = nullptr;
    }
    
    isDWPlannerInitialized = false;
    qDebug() << "动态窗口规划器(DWPlanner)资源已清理";
}

// DW算法速度计算结果处理
void MainWindow::onVelocityComputed(const QString &droneId, const QPointF &velocity, const QVector<double> &scores) {
    // 将计算出的速度应用于无人机
    sendControlCommand(droneId, velocity);
    
    // 记录日志
    qDebug() << "DW算法计算结果 - 无人机:" << droneId 
             << "速度:" << velocity 
             << "评分:" << scores;
}
// 计算雷云速度
// 计算雷云移动速度
void MainWindow::calculateCloudVelocities() {
    QDateTime currentTime = QDateTime::currentDateTime();

    // 遍历所有移动障碍物（雷云）
    for (auto it = movingObstacles.begin(); it != movingObstacles.end(); ++it) {
        const QString &cloudId = it.key();
        const ObstacleInfo &cloudInfo = it.value();

        // 当前位置
        QPointF currentPos(cloudInfo.x, cloudInfo.y);

        // 如果这个雷云之前没有记录，创建新记录
        if (!cloudVelocities.contains(cloudId)) {
            CloudVelocityInfo velocityInfo;
            velocityInfo.currentPosition = currentPos;
            velocityInfo.lastPosition = currentPos; // 初始时，上一位置与当前位置相同
            velocityInfo.velocity = QPointF(0, 0); // 初始速度为0
            velocityInfo.lastUpdateTime = currentTime;
            velocityInfo.hasValidVelocity = false;

            cloudVelocities[cloudId] = velocityInfo;
        } else {
            // 获取上一次记录的信息
            CloudVelocityInfo &velocityInfo = cloudVelocities[cloudId];

            // 计算时间差（毫秒）
            qint64 timeDiff = velocityInfo.lastUpdateTime.msecsTo(currentTime);

            // 如果时间差足够大（避免频繁更新导致的误差），更新速度
            if (timeDiff > 100) { // 至少100毫秒
                // 保存上一位置
                velocityInfo.lastPosition = velocityInfo.currentPosition;
                // 更新当前位置
                velocityInfo.currentPosition = currentPos;

                // 计算速度向量（像素/秒）
                float dx = currentPos.x() - velocityInfo.lastPosition.x();
                float dy = currentPos.y() - velocityInfo.lastPosition.y();

                // 转换为每秒速度
                float vx = dx / (timeDiff / 1000.0f);
                float vy = dy / (timeDiff / 1000.0f);

                velocityInfo.velocity = QPointF(vx, vy);
                velocityInfo.lastUpdateTime = currentTime;
                velocityInfo.hasValidVelocity = true;

                // 调试输出
                // qDebug() << "雷云" << cloudId << "速度:" << velocityInfo.velocity;
            }
        }
    }

    // 清理不再存在的雷云记录
    QStringList keysToRemove;
    for (auto it = cloudVelocities.begin(); it != cloudVelocities.end(); ++it) {
        if (!movingObstacles.contains(it.key())) {
            keysToRemove.append(it.key());
        }
    }

    for (const QString &key : keysToRemove) {
        cloudVelocities.remove(key);
    }
}

//void MainWindow::calculateTargetInfo(const QPointF &selfPos, QVector<QPointF> &enemyPos, float &targetDistance, float &targetAngle)    //构建向量计算角度
//{
//    // 默认值
//    targetDistance = 1000.0f; // 一个较大的默认值
//    targetAngle = 0.0f;

//    if (enemyPos.isEmpty()) {
//        return;
//    }

//    // 找到最近的敌方无人机
//    QPointF nearestEnemy = enemyPos[0];
//    float minDistance = QVector2D(nearestEnemy - selfPos).length();

//    for (int i = 1; i < enemyPos.size(); i++) {
//        float distance = QVector2D(enemyPos[i] - selfPos).length();
//        if (distance < minDistance) {
//            minDistance = distance;
//            nearestEnemy = enemyPos[i];
//        }
//    }

//    // 计算距离和角度
//    targetDistance = minDistance;

//    // 计算角度（弧度）
//    float dx = nearestEnemy.x() - selfPos.x();
//    float dy = nearestEnemy.y() - selfPos.y();
//    targetAngle = qAtan2(dy, dx);
//}
