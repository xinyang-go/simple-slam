# SLAM定位程序

仅定位，不建图的简易SLAM程序。

`config/param.yml`文件中给出了相机内参矩阵和畸变参数，需要根据实际情况修改。

---

项目依赖：

| 依赖库  | 作用               | 安装方式                                                     |
| ------- | ------------------ | ------------------------------------------------------------ |
| OpenCV4 | 机器视觉           | [github地址](https://github.com/opencv/opencv)               |
| Eigen3  | 矩阵运算           | sudo apt install libeigen3-dev                               |
| Sophus  | 李群李代数         | [github地址](https://github.com/strasdat/Sophus)             |
| G2O     | 非线性图优化       | [github地址](https://github.com/RainerKuemmerle/g2o)         |
| Cholmod | 优化算法           | 和g2o一起安装                                                |
| Boost   | C++扩展库          | sudo apt install libboost-all-dev                            |
| MVSDK   | MindVision相机驱动 | [官网下载地址](http://www.mindvision.com.cn/rjxz/list_12.aspx) |

