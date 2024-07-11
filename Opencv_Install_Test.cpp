#include <opencv2/opencv.hpp>  // 包含 OpenCV 的主要功能模块
#include <iostream>            // 包含标准输入输出流功能

using namespace cv;  // 使用 OpenCV 的命名空间，简化代码中对 OpenCV 功能的访问

int main(int argc, char** argv) {
    // 读取图像文件到 Mat 对象中。Mat 是 OpenCV 用于存储图像的主要数据结构。
    // "YOUR_IMAGE_PATH" 应该被替换成实际图像文件的路径，例如 "image.jpg"
    Mat src = imread("YOUR_IMAGE_PATH"); 

    // 检查图像是否成功加载
    // if (src.empty()) 检查 src 对象是否为空，表示图像未成功加载
    if (src.empty()) {
        // 图像加载失败时，输出错误信息
        // printf 用于在控制台输出格式化的字符串
        printf("could not load image...\n");
        return -1;  // 返回 -1 表示程序执行失败
    }
    
    // 创建一个窗口，名为 "test opencv setup"
    // CV_WINDOW_AUTOSIZE 参数表示窗口的大小将自动调整以适应图像的大小
    namedWindow("test opencv setup", WINDOW_AUTOSIZE);  

    // 在创建的窗口中显示图像
    // "test opencv setup" 是窗口的名称，src 是要显示的图像
    imshow("test opencv setup", src);  
    
    // 等待用户按键，0 表示无限等待，直到用户按下任意键
    waitKey(0);
    
    return 0;  // 返回 0 表示程序正常结束
}
