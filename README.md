# An algorithm to automatically generate reach and grasp animation
这个项目实现了 **基于样本的 手臂移动reach 和 手掌抓取grasp 的 自动动画生成算法**。整个算法的实现是基于OGRE的动画系统，算法的核心参考了
```
Feng, Andrew W., Yuyu Xu, and Ari Shapiro. "An example-based motion synthesis technique for locomotion and object manipulation." Proceedings of the ACM SIGGRAPH Symposium on Interactive 3D Graphics and Games. ACM, 2012.
```

## 截屏 ScreenShot
![](https://raw.githubusercontent.com/lealzhan/reachAndGrasp/master/Samples/DualQuaternion/Connection/result/reach_1.gif)

## Building
- 只支持windows系统. Only support windows
- 本仓库已包含ogre的 Dependencies（32bit），无需再次下载. Already include Dependencies(32bit) for ogre.
- 使用cmake来build OGRE. Build with cmake
	- windows vs2013 32bit
- 核心算法都集成在Sample DualQuaternion中
- 对于Sample DualQuaternion的配置
	- 手动添加DualQuaternion下所有hpp和cpp文件到项目中
	- 手动添加opencv的头文件路径
		```
		..\..\..\Dependencies\include\opencv-2.4.13
		```
	- 手动添加opencv的lib文件
		```
		..\..\..\Dependencies\lib\Debug\opencv_core2413d.lib
		..\..\..\Dependencies\lib\Debug\opencv_flann2413d.lib
		```
	- 手动添加opencv的dll文件到bin/debug和bin/release文件夹下
	- 注意DualQuaternion.cpp下的资源路径
	```ogre_old/Samples/Media/models/```
	
