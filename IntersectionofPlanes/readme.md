# 两平面相交

## 计算平面方程

首先，创建一个SACSegmentation对象seg，设置模型类型为SACMODEL_PLANE，方法类型为SAC_RANSAC，最大迭代次数为50000，距离阈值为12。

然后，创建一个指向输入点云的指针remainingCloud，一个指向点云中的内点的指针inliers，一个指向模型系数的指针coefficients。

接下来，对于每个平面，调用SACSegmentation::segment()方法，提取出内点和模型系数。

如果内点的数量小于100，说明无法估计给定数据集的平面模型，函数返回。

将模型系数存储到planeCoefficients中，并创建一个ExtractIndices对象extract，设置输入点云为remainingCloud，内点为inliers。

调用extract.filter()方法，将内点从remainingCloud中提取出来。

如果planeCoefficients的大小小于2，说明在点云中找不到至少两个平面，函数返回。

调用computePlaneIntersection()函数，计算两个平面的交线，将结果存储到linePoint和lineDirection中。

## 计算交线

从给定的平面方程（以齐次坐标表示）中提取出法向量normal1和normal2。

然后，计算这两个法向量的叉乘得到交线的方向向量direction。

如果方向向量的范数为零，这意味着两个平面平行或重合，因此没有唯一的交线，函数返回false。

接下来，创建一个3x3矩阵，其中包含两个法向量和平行于交线的方向向量。

计算常数向量constants，其中包含了平面方程中的常数项（以负号表示）和一个零元素（对应于交线方向向量）。

使用QR分解求解矩阵方程Ax = b，其中A是包含法向量和方向向量的矩阵，b是常数向量。这将给出位于交线上的点point。

将计算出的点赋值给linePoint，并将方向向量规范化后赋值给lineDirection。

## 计算交点

点云是有范围的，因此可以计算实际的交点。

将小块点云的点投射到交线上，记录极端值，即为交点。
