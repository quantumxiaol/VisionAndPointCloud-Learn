# 检测水面涟漪

第一个函数 PointLocalNExistNDetection 读取一个点云文件，并使用KD树（KdTree）进行半径搜索。对于每个点，它会在指定半径内搜索邻近点，并计算这些邻近点的z坐标差异。如果差异超过阈值的邻近点数量达到指定数量n，则认为该点是异常点，并将其添加到过滤后的点云中。最后，过滤后的点云被保存到一个新的文件中。

第二个函数 PointLocalNExistNDetectionMT 是第一个函数的多线程版本。它首先读取点云文件并创建KD树，然后根据硬件线程数将点云数据分配给多个线程进行并行处理。每个线程处理其分配的点云部分，并将检测到的异常点存储在局部点云中。最后，所有线程的局部点云被合并到一个主点云中，并保存到文件中。

第三个函数 PointLocalNExistNDetectionMTv1 在多线程处理之前增加了一个降采样步骤。它首先读取点云文件并使用体素网格（VoxelGrid）滤波器对点云进行降采样，以减少点的数量。然后，它创建KD树并进行多线程处理，类似于第二个函数。降采样后的点云被分配给多个线程进行处理，检测到的异常点被合并到一个主点云中，并保存到文件中。

检测数据
![点云](https://github.com/quantumxiaol/VisionAndPointCloud-Learn/blob/main/png/wavaletDetect00.png)

结果
![点云](https://github.com/quantumxiaol/VisionAndPointCloud-Learn/blob/main/png/wavaletDetect01.png)
