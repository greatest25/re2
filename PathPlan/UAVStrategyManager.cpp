#include "UAVStrategyManager.h"

UAVStrategyManager::UAVStrategyManager(QObject *parent)
    : QObject(parent)
{
    // 简化构造函数
}

QVector<float> UAVStrategyManager::executeStrategy(const QVector<float> &observation)
{
    QVector<float> result(6, 0.0f); // 6维输出：B1目标x,y, B2目标x,y, B3目标x,y

    if (observation.size() != 18) {
        qDebug() << "观察向量维度错误，期望18维，实际" << observation.size() << "维";
        return result;
    }

    // 输出观察向量用于调试
    qDebug() << "18维观察向量:" << observation;

    // 围剿策略：优先围剿R1
    // 检查R1是否存在且有效
    int r1BaseIndex = 9; // R1在观察向量中的基础索引
    bool r1Valid = (observation[r1BaseIndex] != -1 && observation[r1BaseIndex + 2] > 0);

    if (r1Valid) {
        float r1X = observation[r1BaseIndex];
        float r1Y = observation[r1BaseIndex + 1];

        qDebug() << "策略处理R1，位置:(" << r1X << "," << r1Y << ")，开始围剿策略";

        // 简化围剿策略：所有无人机直接飞向R1的位置
        for (int i = 0; i < 3; ++i) {
            int obsBaseIndex = i * 3; // 我方无人机在观察向量中的基础索引
            int resultBaseIndex = i * 2; // 结果向量中的基础索引

            // 检查我方无人机是否有效
            if (observation[obsBaseIndex + 2] <= 0) {
                // 无人机已坠机，目标点设为0
                result[resultBaseIndex] = 0.0f;
                result[resultBaseIndex + 1] = 0.0f;
                continue;
            }

            // 简化策略：直接将目标点设为R1的位置
            result[resultBaseIndex] = r1X;
            result[resultBaseIndex + 1] = r1Y;

            qDebug() << "无人机B" << (i+1) << "围剿R1，目标点:("
                     << result[resultBaseIndex] << "," << result[resultBaseIndex + 1] << ")";
        }
    } else {
        qDebug() << "R1无效或已被击毁，策略无效";
        // 如果R1无效，返回全0向量
        for (int i = 0; i < 6; ++i) {
            result[i] = 0.0f;
        }
    }

    qDebug() << "策略输出6维数据:" << result;
    return result;
}
