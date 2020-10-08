//
// Created by xinyang on 2020/3/21.
//

#ifndef _MINDVISION_HPP_
#define _MINDVISION_HPP_

#include "SDK/CameraApi.h"
#include <opencv2/core.hpp>
#include <string>
#include <cstdint>

class FrameReadError : public std::runtime_error {
public:
    explicit FrameReadError(int code) : runtime_error("frame read error " + std::to_string(code)) {};
};

class MindVision {
private:
    std::string camera_name;
    std::string camera_config;
    CameraHandle h_camera;
    uint8_t *rgb_buffer;
public:
    explicit MindVision(std::string config = "", std::string name = "");

    MindVision(const MindVision &obj) = delete;

    MindVision(MindVision &&obj) noexcept;

    MindVision &operator=(const MindVision &obj) = delete;

    MindVision &operator=(MindVision &&obj) noexcept;

    ~MindVision();

    bool init();

    void read(cv::Mat &image);
};


#endif /* _MINDVISION_HPP_ */
