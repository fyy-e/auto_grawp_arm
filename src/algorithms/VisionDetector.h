#ifndef VISION_DETECTOR_H
#define VISION_DETECTOR_H

#include <Eigen/Dense>
#include <arpa/inet.h>
#include <unistd.h>
#include <iostream>

class VisionDetector {
public:
    VisionDetector(float markerLen = 0.05f); 
    ~VisionDetector();

    bool Init();
    // 修改接口，直接返回原始数据以确保线程安全
    bool GetTargetInCam(int target_id, double& x, double& y, double& z);

private:
    int sockfd;
    struct sockaddr_in servaddr;
    const int PORT = 5005;
};

#endif