# VisionAndPointCloud-Learn

学习PCL和Opencv

## PCL安装指南

点云数据的处理可以采用获得广泛应用的Point Cloud Library (点云库，PCL库)，它实现了大量点云相关的通用算法和高效的数据管理。

PCL是BSD授权方式，可以免费进行商业和学术应用。

官网：https://pointclouds.org/

GitHub：https://github.com/PointCloudLibrary/pcl

### 安装PCL

安装AllInOne文件，选择第二项Add PCL to the system PATH for all users，也可以不选自己之后手动添加。

OpenNI的安装路径为PCL路径下的3rdParty文件夹下的OpenNI2文件夹。

下面采用默认安装路径。

### 添加系统环境变量。

%PCL_ROOT%\bin

%PCL_ROOT%\3rdParty\VTK\bin

%PCL_ROOT%\3rdParty\FLANN\bin

C:\Program Files\OpenNI2\Tools

%PCL_ROOT%\3rdParty\Qhull\bin

%OPENNI2_REDIST64%

%OPENNI2_LIB64%

%OPENNI2_INCLUDE64%

PATH环境变量指定了操作系统在执行命令时搜索可执行文件的路径。‌如果某个程序的执行文件不在当前目录下，‌但它在PATH中指定的某个路径下，‌那么用户仍然可以通过命令行直接运行该程序，‌而无需指定完整的路径。


### 配置VS

新建项目，编辑视图-其他窗口-属性管理器。

为ReleaseX64添加属性表。

**通用属性-VC++目录-包含目录-编辑**，添加如下内容

C:\Program Files\PCL 1.8.1\include\pcl-1.8

C:\Program Files\PCL 1.8.1\3rdParty\Boost\include\boost-1_64

C:\Program Files\PCL 1.8.1\3rdParty\Eigen\eigen3\Eigen

C:\Program Files\PCL 1.8.1\3rdParty\FLANN\include

C:\Program Files\OpenNI2\Include

C:\Program Files\PCL 1.8.1\3rdParty\Qhull\include

C:\Program Files\PCL 1.8.1\3rdParty\VTK\include\vtk-8.0

为了确保编译器能够找到所有必需的头文件

**C/C++-预处理器-预处理器定义-编辑**，添加以下内容

BOOST_USE_WINDOWS_H

NOMINMAX

_CRT_SECURE_NO_DEPRECATE

Boost 是一个广泛使用的 C++ 库，PCL 的某些组件可能依赖于 Boost。在 Windows 平台上，使用 BOOST_USE_WINDOWS_H 可以确保 Boost 和 Windows 头文件的兼容性。
Windows 的 windows.h 头文件会定义 min 和 max 宏，这可能会与 C++ 标准库中的 std::min 和 std::max 冲突。通过定义 NOMINMAX，可以避免这些冲突。
Microsoft 的编译器会对一些旧的、不安全的 C 标准库函数发出警告。定义 _CRT_SECURE_NO_DEPRECATE 可以防止这些警告，但这通常是为了与旧代码兼容，建议逐步使用安全的库函数。

**C/C++-语言-符合模式** ，选择否

编译器会以较宽松的模式编译代码，这有时可以避免因为严格模式下的语法要求而导致的编译错误。样做通常是为了兼容旧代码或第三方库。

**C/C++-所有选项-SDL检查** ，设置为否

SDL 是 Microsoft 的一组安全功能和检查，用于帮助开发人员编写更安全的代码。将 SDL 检查设置为“否”可以减少编译器对安全问题的检查，这有时是为了避免在编译时遇到与 SDL 检查相关的问题。

**C/C++-代码生成-启用增强指令集** ，选择高级矢量扩展

AVX 是一种现代的 CPU 指令集，用于提高处理性能。启用这个选项可以让编译器生成针对 AVX 指令集优化的代码，从而提高程序的执行效率。

**VC++目录-库目录-编辑**，添加如下内容

C:\Program Files\PCL 1.8.1\lib

C:\Program Files\PCL 1.8.1\3rdParty\Boost\lib

C:\Program Files\PCL 1.8.1\3rdParty\FLANN\lib

C:\Program Files\OpenNI2\Lib

C:\Program Files\PCL 1.8.1\3rdParty\Qhull\lib

C:\Program Files\PCL 1.8.1\3rdParty\VTK\lib

**链接器-输入-附加依赖项-编辑**，添加以下内容（C:\Program Files\PCL 1.8.1\lib下和C:\Program Files\PCL 1.8.1\3rdParty\VTK\lib的文件）：

pcl_common.lib

pcl_features.lib

pcl_filters.lib

pcl_io.lib

pcl_io_ply.lib

pcl_kdtree.lib

pcl_keypoints.lib

pcl_ml.lib

pcl_octree.lib

pcl_outofcore.lib

pcl_people.lib

pcl_recognition.lib

pcl_registration.lib

pcl_sample_consensus.lib

pcl_search.lib

pcl_segmentation.lib

pcl_stereo.lib

pcl_surface.lib

pcl_tracking.lib

pcl_visualization.lib

vtknetcdf-8.0.lib

vtkogg-8.0.lib

vtkParallelCore-8.0.lib

vtkParallelDIY-8.0.lib

vtkpng-8.0.lib

vtkpugixml-8.0.lib

vtkRenderingAnnotation-8.0.lib

vtkRenderingCellGrid-8.0.lib

vtkRenderingContext2D-8.0.lib

vtkRenderingContextOpenGL2-8.0.lib

vtkRenderingCore-8.0.lib

vtkRenderingFreeType-8.0.lib

vtkRenderingGL2PSOpenGL2-8.0.lib

vtkRenderingHyperTreeGrid-8.0.lib

vtkRenderingImage-8.0.lib

vtkRenderingLabel-8.0.lib

vtkRenderingLICOpenGL2-8.0.lib

vtkRenderingLOD-8.0.lib

vtkRenderingOpenGL2-8.0.lib

vtkRenderingSceneGraph-8.0.lib

vtkRenderingUI-8.0.lib

vtkRenderingVolume-8.0.lib

vtkRenderingVolumeOpenGL2-8.0.lib

vtkRenderingVtkJS-8.0.lib

vtksqlite-8.0.lib

vtksys-8.0.lib

vtkTestingRendering-8.0.lib

vtktheora-8.0.lib

vtktiff-8.0.lib

vtkverdict-8.0.lib

vtkViewsContext2D-8.0.lib

vtkViewsCore-8.0.lib

vtkViewsInfovis-8.0.lib

vtkWrappingTools-8.0.lib

vtkzlib-8.0.lib

在编译和链接过程中，编译器和链接器需要找到所有被引用的库文件。将库文件添加到链接器的附加依赖项中，确保链接器能够找到并正确地将这些库文件链接到你的最终可执行文件中。

**调试-环境** 添加

PATH=$(PCL_ROOT)\bin;$(PCL_ROOT)\3rdParty\FLANN\bin;$(PCL_ROOT)\3rdParty\VTK\bin;$(PCL_ROOT)\3rdParty\Qhull\bin;C:\Program Files\OpenNI2\Tools;$(PATH)

运行PCL_Install_Test.cpp。


## Opencv安装

OpenCV（Open Source Computer Vision Library），是一个基于BSD许可（开源）发行的跨平台计算机视觉库，可以运行在Linux、Windows、Android和Mac OS操作系统上。它轻量级而且高效——由一系列 C 函数和少量 C++ 类构成，同时提供了Python、Ruby、MATLAB等语言的接口，实现了图像处理和计算机视觉方面的很多通用算法。

官网：https://opencv.org/

### 安装opencv，可以安装到默认路径，也可以安装到C盘或D盘。

### 配置环境变量

在path中添加C:\opencv\build\x64\vc14\bin

### 配置VS

**通用属性-VC++目录-包含目录-编辑**，添加如下内容

C:\opencv\build\include

C:\opencv\build\include\opencv

C:\opencv\build\include\opencv2

**VC++目录-库目录-编辑**，添加如下内容

C:\opencv\build\x64\vc14\lib

**项目-属性-链接器-输入-附加依赖项-编辑**，添加C:\opencv\build\x64\vc1\lib目录下的依赖项，如果后缀前有d为debug依赖项。

执行Opencv_Install_Test.cpp
