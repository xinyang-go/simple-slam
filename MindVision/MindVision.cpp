//
// Created by xinyang on 2020/3/21.
//

#include "MindVision.hpp"
#include <opencv2/imgproc.hpp>

#define CHECK_API(action, func, ...)  {\
    CameraSdkStatus status; \
    status = func(__VA_ARGS__); \
    if (status != CAMERA_STATUS_SUCCESS) { \
        action; \
    } \
}
#define CHECK_API_GOTO(tag, func, ...) CHECK_API({fprintf(stderr, #func " fail with %d!", status); goto tag;} , func, ##__VA_ARGS__)
#define CHECK_API_THROW(exc, func, ...) CHECK_API({throw exc(status);} , func, ##__VA_ARGS__)
#define CHECK_API_CONTINUE(func, ...) CHECK_API({continue;}, func, ##__VA_ARGS__)
#define CHECK_API_WARNING(func, ...) CHECK_API({fprintf(stderr, #func " fail with %d!", status);}, func, ##__VA_ARGS__)

/************** implement of class MindVision ******************/
MindVision::MindVision(std::string config, std::string name) : camera_name(std::move(name)),
                                                               camera_config(std::move(config)),
                                                               h_camera(0),
                                                               rgb_buffer(nullptr) {

}

MindVision::~MindVision() {
    delete rgb_buffer;
    rgb_buffer = nullptr;
    if (h_camera != 0) {
        CameraUnInit(h_camera);
        h_camera = false;
    }
}

MindVision::MindVision(MindVision &&obj) noexcept(true): camera_name(obj.camera_name),
                                                         camera_config(obj.camera_config),
                                                         h_camera(obj.h_camera),
                                                         rgb_buffer(obj.rgb_buffer) {
    obj.h_camera = 0;
    obj.rgb_buffer = nullptr;
}

class MindVision &MindVision::operator=(MindVision &&obj) noexcept(true) {
    camera_name = obj.camera_name;
    camera_config = obj.camera_config;
    h_camera = obj.h_camera;
    rgb_buffer = obj.rgb_buffer;
    obj.h_camera = 0;
    obj.rgb_buffer = nullptr;
    return *this;
}

bool MindVision::init() {
    // 变量定义在函数头部，便于使用goto
    tSdkCameraDevInfo camera_info_list[5];
    int camera_cnts = 5;
    bool found = false;
    char name[64];
    tSdkCameraCapbility tCapability;
    double exposure = 0;
    int gain = 0;

    // 如果相机已经打开，则关闭并重新打开
    if (h_camera != 0) {
        CameraUnInit(h_camera);
    }

    // 初始化相机SDK
    CameraSdkInit(1);

    // 枚举所有相机
    CHECK_API_GOTO(err0, CameraEnumerateDevice, camera_info_list, &camera_cnts)
    if (camera_cnts == 0) {
        fprintf(stderr, "No camera device detected!\n");
        return false;
    } else {
        fprintf(stderr, "%d camera device detected!\n", camera_cnts);
    }

    // 根据相机名称查找对应相机
    for (int i = 0; i < camera_cnts; i++) {
        CHECK_API_CONTINUE(CameraInit, camera_info_list + i, -1, -1, &h_camera);
        CHECK_API_CONTINUE(CameraGetFriendlyName, h_camera, name);
        if (camera_name.empty() || camera_name == name) {
            found = true;
            break;
        }
        CHECK_API_GOTO(err0, CameraUnInit, h_camera);
    }
    if (!found) {
        fprintf(stdout, "No camera device named %s or camera device open error!\n", camera_name.data());
        return false;
    }

    // 获取相机特性描述结构体
    CHECK_API_GOTO(err1, CameraGetCapability, h_camera, &tCapability);
    // 判断相机是否为彩色相机
    if (tCapability.sIspCapacity.bMonoSensor) {
        fprintf(stdout, "Camera with only mono mode is not supported!\n");
        goto err1;
    }
    // 申请对应空间大小，用于存放图像rgb数据
    rgb_buffer = new uint8_t[tCapability.sResolutionRange.iHeightMax * tCapability.sResolutionRange.iWidthMax * 3];

    // 设置相机参数
    CHECK_API_GOTO(err2, CameraReadParameterFromFile, h_camera, (char *) camera_config.c_str());
    CHECK_API_WARNING(CameraGetExposureTime, h_camera, &exposure);
    fprintf(stdout, "Camera %s exposure: %f", name, exposure);
    CHECK_API_WARNING(CameraGetAnalogGain, h_camera, &gain);
    fprintf(stdout, "Camera %s analog gain: %d", name, gain);

    CameraSetTriggerMode(h_camera, 0);

    CameraPlay(h_camera);
    return true;
    err2:
    delete[]rgb_buffer;
    rgb_buffer = nullptr;
    err1:
    CameraUnInit(h_camera);
    h_camera = 0;
    err0:

    return false;
}

void MindVision::read(cv::Mat &image) noexcept(false) {
    uint8_t *pby_buffer = nullptr;
    tSdkFrameHead frame_info{};
    CHECK_API_THROW(FrameReadError, CameraGetImageBuffer, h_camera, &frame_info, &pby_buffer, 15000);
    CHECK_API_THROW(FrameReadError, CameraImageProcess, h_camera, pby_buffer, rgb_buffer, &frame_info);
    image = cv::Mat(frame_info.iHeight, frame_info.iWidth, CV_8UC3, rgb_buffer).clone();
    cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
    CHECK_API_WARNING(CameraReleaseImageBuffer, h_camera, pby_buffer);
}
