#include "vsoamanager.h"

using namespace std::placeholders;

VsoaManager::VsoaManager(QObject *parent) : QObject(parent)
{
    // 初始化服务器状态检测定时器
    serverStatusTimer = new QTimer(this);
    connect(serverStatusTimer, &QTimer::timeout, this, &VsoaManager::checkServerStatus);
    serverStatusTimer->start(50); // 每50ms检查一次，确保及时检测到暂停

    // 初始化最后数据接收时间
    lastDataReceivedTime = QDateTime::currentDateTime();
}

VsoaManager::~VsoaManager()
{
    // 清理资源
    if (serverStatusTimer) {
        serverStatusTimer->stop();
        delete serverStatusTimer;
        serverStatusTimer = nullptr;
    }
}

void VsoaManager::initConnection()
{
    if (client.isInvalid()) {
        qDebug() << "Can not create VSOA client!";
        return;
    }

    // 连接信号和槽
    QObject::connect(&client, &QVsoaClient::connected,
                    this, &VsoaManager::onConnected);
    QObject::connect(&client, &QVsoaClient::disconnected,
                    this, &VsoaManager::onDisconnected);
    QObject::connect(&client, &QVsoaClient::datagram,
                    std::bind(&VsoaManager::onDatagram, this, &client, _1, _2));

    // Connect to server with password
    client.connect2server("vsoa://127.0.0.1:3005/game_server", "", 1000);
    // Enable automatic connections
    client.autoConnect(1000, 500);
    // Subscribe /ctrl 订阅控制通道
    client.subscribe("/ctrl");
    // 订阅游戏数据通道
    client.subscribe("/game");
    // Enable data consistency on channels
    client.autoConsistent({"/ctrl", "/game"}, 1000);
}

void VsoaManager::onConnected(bool ok, QString info)
{
    if (!ok) {
        qDebug() << "Connected with server failed!";
        emit connectionStatusChanged(false, "连接服务器失败");
        return;
    }
    qDebug() << "Connected with server:" << info;
    emit connectionStatusChanged(true, info);
}

void VsoaManager::onDisconnected()
{
    qDebug() << "Connection break";
    emit connectionStatusChanged(false, "连接断开");
}

void VsoaManager::onDatagram(QVsoaClient *client, QString url, QVsoaPayload payload)
{
    QVariant param = payload.param();
    // 处理接收到的数据
    if (url == "/game") {
        // 处理游戏数据
        QString dataStr;

        // 根据实际类型转换
        if (param.type() == QVariant::String) {
            dataStr = param.toString();
        } else if (param.canConvert<QByteArray>()) {
            dataStr = QString::fromUtf8(param.toByteArray());
        } else {
            qDebug() << "不支持的参数类型:" << param.typeName();
            return;
        }

        processGameData(dataStr);
    } else if (url == "/ctrl") {
        // 处理控制数据的反馈
        qDebug() << "接收到控制数据反馈：" << param;
    }
}

void VsoaManager::sendControlCommand(const QString &uavId, const QPointF &velocity)
{
    // 构造JSON消息
    QJsonObject controlMsg;
    controlMsg["uid"] = uavId;
    controlMsg["vx"] = velocity.x();
    controlMsg["vy"] = velocity.y();

    QJsonDocument doc(controlMsg);
    QString message = doc.toJson(QJsonDocument::Compact);

    // 通过VSOA客户端发送控制命令
    QVsoaPayload payload;
    payload.setParam(message);
    client.sendDatagram("/ctrl", payload);

//    qDebug() << "发送控制命令:" << uavId << "vx:" << velocity.x() << "vy:" << velocity.y();
}

void VsoaManager::checkServerStatus()
{
    // 只在游戏运行状态下检测服务器暂停
    if (gameStage != "running") {
        return;
    }

    QDateTime currentTime = QDateTime::currentDateTime();

    // 如果超过90ms没有收到数据，认为服务器暂停
    int msecsElapsed = lastDataReceivedTime.msecsTo(currentTime);

    if (msecsElapsed > 90) {
        // 如果刚从运行状态变为暂停状态
        if (!isServerPaused) {
            isServerPaused = true;
            serverPauseStartTime = currentTime;
            qDebug() <<"超过"<<msecsElapsed<<"ms没有收到数据，服务器暂停，暂停开始时间：" << serverPauseStartTime;
            emit serverPaused(serverPauseStartTime);
        }
    }
}

void VsoaManager::processGameData(const QString &jsonData)
{
    QJsonDocument doc = QJsonDocument::fromJson(jsonData.toUtf8());
    if (doc.isNull() || !doc.isObject()) {
        qDebug() << "无效的JSON数据";
        return;
    }

    QJsonObject gameObj = doc.object();

    // 更新游戏状态
    if (gameObj.contains("left_time")) {
        gameLeftTime = gameObj["left_time"].toInt();
    }

    if (gameObj.contains("stage")) {
        QString newStage = gameObj["stage"].toString();

        // 检查游戏状态是否变为finish
        if (newStage == "finish" && gameStage != "finish") {
            gameLeftTime = 0;

            // 清空固定障碍物信息
            staticObstacles.clear();
            // 清空所有障碍物信息
            obstaclesInfo.clear();
            // 清空无人机信息
            dronesInfo.clear();
        }
        gameStage = newStage;
    }

    // 如果之前是暂停状态，现在收到数据说明恢复了
    if (isServerPaused) {
        QDateTime currentTime = QDateTime::currentDateTime();
        int pausedSeconds = serverPauseStartTime.secsTo(currentTime);
        isServerPaused = false;
        emit serverResumed(pausedSeconds);
    }

    // 更新无人机信息
    if (gameObj.contains("drones") && gameObj["drones"].isArray()) {
        QJsonArray drones = gameObj["drones"].toArray();

        // 清空之前的数据
        dronesInfo.clear();

        // 更新无人机信息
        for (const QJsonValue &droneValue : drones) {
            if (droneValue.isObject()) {
                QJsonObject drone = droneValue.toObject();

                DroneInfo info;
                info.hp = drone["hp"].toInt();
                info.status = drone["status"].toString();
                info.team = drone["team"].toString();
                info.uid = drone["uid"].toString();
                info.x = drone["x"].toDouble();
                info.y = drone["y"].toDouble();

                // vx和vy可能不存在于所有无人机数据中
                if (drone.contains("vx")) {
                    info.vx = drone["vx"].toDouble();
                }

                if (drone.contains("vy")) {
                    info.vy = drone["vy"].toDouble();
                }

                // 存储无人机信息
                dronesInfo[info.uid] = info;
            }
        }
    }

    // 更新障碍物信息
    if (gameObj.contains("obstacles") && gameObj["obstacles"].isArray()) {
        QJsonArray obstacles = gameObj["obstacles"].toArray();

        // 当前时间
        QDateTime currentTime = QDateTime::currentDateTime();

        // 清空移动障碍物（雷云）信息，因为需要实时更新
        movingObstacles.clear();

        // 更新障碍物信息
        for (const QJsonValue &obstacleValue : obstacles) {
            if (obstacleValue.isObject()) {
                QJsonObject obstacle = obstacleValue.toObject();

                ObstacleInfo info;
                info.id = obstacle["id"].toString();
                info.r = obstacle["r"].toDouble();
                info.type = obstacle["type"].toString();
                info.x = obstacle["x"].toDouble();
                info.y = obstacle["y"].toDouble();

                // 根据类型分别存储
                if (info.type == "cloud") {
                    // 雷云是移动的，需要实时更新
                    movingObstacles[info.id] = info;
                } else if ((info.type == "mountain" || info.type == "radar")) {
                    // 检查是否已存在该障碍物
                    if (!staticObstacles.contains(info.id)) {
                        staticObstacles[info.id] = info;
                    }
                }
            }
        }

        // 将所有障碍物合并到obstaclesInfo中，用于UI显示或其他处理
        obstaclesInfo.clear();
        // 先添加固定障碍物
        for (auto it = staticObstacles.begin(); it != staticObstacles.end(); ++it) {
            obstaclesInfo[it.key()] = it.value();
        }
        // 再添加移动障碍物
        for (auto it = movingObstacles.begin(); it != movingObstacles.end(); ++it) {
            obstaclesInfo[it.key()] = it.value();
        }
    }

    // 更新最后数据接收时间
    lastDataReceivedTime = QDateTime::currentDateTime();

    // 发送游戏数据更新信号
    emit gameDataUpdated(dronesInfo, obstaclesInfo, staticObstacles, movingObstacles, gameLeftTime, gameStage);
}
