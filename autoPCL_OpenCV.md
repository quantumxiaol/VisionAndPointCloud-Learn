PCL的安装路径是这样的

x64版本

        | - C:\Program Files\PCL 1.8.1

            | - 3rdParty

                | - Boost

                    | - include

                        | - boost-1_64

                    | - lib

                | - Eigen

                    | - eigen3\Eigen

                | - FLANN

                | - OpenNI2

                | - Qhull

                    | - bin

                    | - include

                    | - lib

                | - VTK

                    | - bin

                    | - include

                        | - vtk-8.0

                    | - lib

            | - bin

            | - include

                | - pcl-1.8\pcl

                    | - 2d

                    | - common

                    | - ···其他头文件

            | - lib

x86版本

        | - C:\Program Files (x86)\PCL 1.8.1

            | - 3rdParty

                | - Boost

                    | - include

                    | - lib

                | - Eigen

                | - FLANN

                | - OpenNI2

                | - Qhull

                    | - bin

                    | - include

                    | - lib

                | - VTK

                    | - bin

                    | - include

                    | - lib

            | - bin

            | - include

            | - lib

OpenCV的安装路径是这样的

        | - C:\opencv

            | - build

                | - x64

                    | - vc14

                        | - bin

                        | - lib

                        | - staticlib

                | - x86

                    | - vc14

                        | - bin

                        | - lib

                        | - staticlib

PATH环境变量指定了操作系统在执行命令时搜索可执行文件的路径。‌如果某个程序的执行文件不在当前目录下，‌但它在PATH中指定的某个路径下，‌那么用户仍然可以通过命令行直接运行该程序，‌而无需指定完整的路径。因而Path需要包含bin的目录。如"C:\Program Files\PCL 1.8.1\bin"，里面是exe和dll文件。

通用属性-VC++目录-包含目录需要能找到这些头文件位置。如"C:\Program Files\PCL 1.8.1\include\pcl-1.8"。

VC++目录-库目录需要lib的目录。如"C:\Program Files\PCL 1.8.1\lib"。

链接器-输入-附加依赖项-编辑需要lib下具体的各个文件名。如"pcl_common.lib"等。

特别的，如果配置OpenCV的项目-属性-链接器-输入-附加依赖项，如果后缀前有d为debug依赖项，只能在debug中出现，不能再release中出现，否则程序无法正常执行。

现在这个Python程序，输入PCL的x86和x64的安装路径，以及OpenCV的安装路径，去扫描上面的文件夹，生成release_64、release_32、debug_64、debug_32的配置文件，他们包含库目录需要lib的目录。如"C:\Program Files\PCL 1.8.1\lib"；以及链接器所需的各个文件名。如"pcl_common.lib"；还有VC++目录-包含目录需要能找到这些头文件位置，如"C:\Program Files\PCL 1.8.1\include\pcl-1.8"；可以手动复制填到VS项目属性的配置中去，以免跟着教程填了一些没有的东西进去。




