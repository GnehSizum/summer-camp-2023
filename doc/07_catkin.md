# catkin_make

## 1.catkin编译系统

对于源代码包，编译后才能在系统上运行。Linux下，随着源文件的增多，用 **gcc/g++** 命令的方式显得效率低下，人们开始用 **Makefile** 来编译。然而随着工程体量增大，Makefile也不能满足需求，于是便出现了 **Cmake** 工具。CMake是对make工具的生成器，是更高层的工具，它简化了编译构建过程，能够管理大型项目，具有良好的扩展性。ROS这个大体量的平台，采用的就是CMake，并且ROS对CMake进行了扩展，也就有了 **Catkin** 编译系统。

![](https://jlu-ai-lab.oss-cn-beijing.aliyuncs.com/blog/catkin01.png)

![](https://jlu-ai-lab.oss-cn-beijing.aliyuncs.com/blog/catkin02.png)

### 1-1.catkin工作空间

![](https://jlu-ai-lab.oss-cn-beijing.aliyuncs.com/blog/catkin03.png)

### 1-2.catkin编译流程

**建立工作空间**

```bash
mkdir -p catkin_ws/src
cd catkin_ws/src
catkin_init_workspace  # 可以跳过这步，直接在catkin_ws/下catkin_make
cd ..
catkin_make
```

**编译**

```bash
cd catkin_ws
catkin_make
source ./devel/setup.bash  # 编译完成后要source刷新环境
```

若要手动执行catkin_make操作，等效命令为：

```bash
cd catkin_ws/src
catkin_init_workspace
cd ..
mkdir build
cd build
cmake ../src -DCMAKE_INSTALL_PREFIX=../install -DCATKIN_DEVEL_PREFIX=../devel
make
```

如果要在工作空间中编译指定的包，执行以下命令：

```bash
catkin_make -DCATKIN_WHITELIST_PACKAGES=“package1;package2”
```

如果要还原为编译所有包，执行以下命令：

```bash
catkin_make -DCATKIN_WHITELIST_PACKAGES=“”
```

创建功能包

```bash
cd catkin_ws/src
catkin_create_pkg <package> [depend1] [depend2] [depend3]
# e.g. catkin_create_pkg test_pkg roscpp rospy std_msgs
```

### 1-3.catkin_make命令参数

```bash
catkin_make -h
```

自己看去吧。

## 2.最后

[catkin_make, cmake, catkin build区别](https://blog.csdn.net/qq_23225073/article/details/102825545)