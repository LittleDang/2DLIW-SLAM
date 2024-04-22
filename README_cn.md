# 2DLIW-SLAM

## 简介

这是我硕士毕业论文提出的一个SLAM框架，它紧耦合了2D激光雷达、IMU和轮式里程计的数据。

除此之外，也为了室内移动机器人这个场景做了很多特殊的处理：

* 地面约束
* 一些基本的假设，例如roll和pitch很小之类的
* 基于2D激光雷达的回环检测

绝大部分的工作都集中在前端部分的设计和实现

后端部分使用位姿图，唯一创新的地方在于改进了之前的研一发表的论文[《基于全局特征点匹配的全局定位方法》](https://kns.cnki.net/kcms2/article/abstract?v=3uoqIhG8C44YLTlOAiTRKu87-SJxoEJu6LL9TJzd50nS5QmOCQhBuPsjALI6Lv67j6KttBYx8CQ1P4eBo2UeDgakY4XEWQm3&uniplatform=NZKPT)所提出的重定位算法，将之应用于回环检测。

## 运行效果

### 视频效果

[![](https://i0.hdslb.com/bfs/archive/3820d785c07d34bd47fd24d873163d0df9ee9231.jpg@320w_200h)](https://www.bilibili.com/video/BV1K24y1E7eK/?vd_source=a2e7f1803695cee5e5b621196f5bc8f4)

### 实际场景

#### 室内

![](pic/realtime.png)

> 这两个的建图效果有点差，主要是因为外参数没有标定的很准

### 数据集

![](pic/dataset.png)

> 数据集地址:[OpenLORIS-Scene Datasets](https://shimo.im/docs/HhJj6XHYhdRQ6jjk/read)
> 
> PS:记得用它[官方的工具](https://github.com/lifelong-robotic-vision/openloris-scene-tools/blob/master/dataprocess/merge_imu_topics.py)把IMU数据合并一下。

### 安装:

#### 1. 直接安装

```bash
cd ${yourworkspace}/src
git clone https://github.com/LittleDang/2DLIW-SLAM
cd ../..
catkin_make -j
```

#### 2. 通过docker安装

```bash
- 找到docker/下的dockerfile，构建镜像和容器

- 把本项目挂载进去

- 和1一样。
```

### QA：

1. 为啥配置文件里面有一些参数似乎没有意义，例如`enable_camera`之类的？
   
   > 因为开发的过程中项目经历了比较多次的调整，有一些功能是暂时放弃了，但是一开始的代码接口还保留着，后续可能继续开发。
   > 
   > 这些不需要的参数保持默认值即可。

2. 为什么系统的坐标系选择了IMU？
   
   > 一开始没有考虑清楚就开始写这个项目了，后来发现这属实会带来一些不太方便的地方，例如大量的地方的处理都不是在IMU坐标系上的，再来一次应该会选择使用base_link作为系统的坐标系。

3. 为啥仓库叫做2DLIW-SLAM，而代码里面都是叫做LVIO-2D
   
   > 因为我一开始只想做一个前端，没想着做后端，但是后来为了系统的完整和毕业论文的完整，我添加了后端部分。

4. 外参怎么获得？
   
   > 我也没有特别好的方法，有一些是传感器自带的，有一些是通过在有监督的情况下，通过一些比较曲折的方式标定的，总的来说精度没有很高，这也导致了系统在某些情况下误差会比较大。（**虽然很重要，但是数据集外参已经有了以及外参的标定不是本论文的重点，所以也没有仔细研究**）

5. **论文在哪？**
   
   > 这个等我毕业答辩完吧~，目前只有回环相关检测的论文是发表了的。
   > 
   > [《基于全局特征点匹配的全局定位方法》](https://kns.cnki.net/kcms2/article/abstract?v=3uoqIhG8C44YLTlOAiTRKu87-SJxoEJu6LL9TJzd50nS5QmOCQhBuPsjALI6Lv67j6KttBYx8CQ1P4eBo2UeDgakY4XEWQm3&uniplatform=NZKPT)

## 写这个框架的一些原因

* 从大三开始第一次接触SLAM，到现在硕士毕业，一直想着**从零自己写**一个SLAM框架，想要在毕业之前完成这件事情，同时也算是留个纪念，毕竟读研期间也没作出大的成果。

* 在阅读了VINS-MONO、Cartographer、LIO-SAM等优秀的开源项目的代码之后（前两个框架对我影响可太大了哈，感谢这些作者的奉献，我也想开源hhh），我也逐渐萌生了写一个框架按照我自己的理解去实现我想要的功能，遂有了将这个构思已久的SLAM框架作为我硕士毕业设计的准备（可以说是先想做这个框架，再顺便写的毕业论文）。

## 一些感想

* 在实现的过程中也经历过很多很多的问题，最大的一个便是，一开始是还准备融合单目相机的，但是后续主要由于工作量的考虑，在项目进行到一半的时候，我放弃了单目相机（工作量对我一个人来说实在太大了，我那时候很怕不能按时完成。。），可以看到我代码里面很多设计一开始还是考虑着相机这个传感器的。以后有机会或许会考虑完善它，让它进化成2DLVIW-SLAM

* **当然在代码实现方面、理论方面可能比较稚嫩甚至错误，如果发现了，欢迎和我讨论，我很乐于接受修改意见。**

## Licence

[Apache-2.0 license](https://github.com/LittleDang/2DLIW-SLAM/blob/main/LICENSE)

Email: brucedang2022@163.com
