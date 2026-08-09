#include "VisionDetector.h"
#include <fcntl.h>
#include <cstring>

VisionDetector::VisionDetector(float markerLen) : sockfd(-1) {}

VisionDetector::~VisionDetector()
{
    if (sockfd != -1)
        close(sockfd);
}

bool VisionDetector::Init()
{
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
    {
        std::cerr << "[VisionDetector] 失败：无法创建 Socket" << std::endl;
        return false;
    }

    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = INADDR_ANY;
    servaddr.sin_port = htons(PORT);

    if (bind(sockfd, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0)
    {
        std::cerr << "[VisionDetector] 失败：无法绑定端口 " << PORT << std::endl;
        return false;
    }

    int flags = fcntl(sockfd, F_GETFL, 0);
    fcntl(sockfd, F_SETFL, flags | O_NONBLOCK);

    std::cout << "[VisionDetector] UDP 接收器初始化成功，监听端口: " << PORT
              << "（GRCNN 20 字节协议）" << std::endl;
    return true;
}

bool VisionDetector::GetTargetInCam(VisionTarget &out)
{
    float buf[5];
    struct sockaddr_in cliaddr;
    socklen_t len = sizeof(cliaddr);

    ssize_t n = recvfrom(sockfd, buf, sizeof(buf), MSG_DONTWAIT,
                         (struct sockaddr *)&cliaddr, &len);

    if (n == 20) // GRCNN 协议: x, y, z, angle, width
    {
        out.x = buf[0];
        out.y = buf[1];
        out.z = buf[2];
        out.angle = buf[3];
        out.width = buf[4];
        return true;
    }
    return false; // 其他字节数一律视为无效包丢弃
}
