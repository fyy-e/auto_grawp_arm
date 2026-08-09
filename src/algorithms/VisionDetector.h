#ifndef VISION_DETECTOR_H
#define VISION_DETECTOR_H

#include <Eigen/Dense>
#include <arpa/inet.h>
#include <unistd.h>
#include <iostream>

/**
 * @brief GRCNN 视觉抓取结果（UDP 20 字节协议）
 *
 * 协议：5 个 float32 小端 = 20 字节
 *   [ x, y, z, angle, width ]
 *   x,y,z : 抓取点在相机坐标系下的位置 (m)
 *   angle : 图像平面抓取角 (rad, 约 [-pi/2, pi/2])
 *   width : 建议夹爪开口宽度 (m)
 *
 * 由 vision/grcnn_server.py（GRCNN 推理服务端）发送，端口 5005。
 */
struct VisionTarget
{
    double x = 0, y = 0, z = 0; // 相机系坐标 (m)
    double angle = 0;           // 图像平面抓取角 (rad)
    double width = -1;          // 建议开口 (m)，<=0 表示无效
};

class VisionDetector
{
public:
    VisionDetector(float markerLen = 0.05f);
    ~VisionDetector();

    bool Init();

    /**
     * @brief 非阻塞接收一帧 GRCNN 抓取结果
     * @param out 输出完整抓取信息
     * @return true = 收到一个合法的 20 字节抓取包
     */
    bool GetTargetInCam(VisionTarget &out);

private:
    int sockfd;
    struct sockaddr_in servaddr;
    const int PORT = 5005;
};

#endif
