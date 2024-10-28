VSLAM
1. 概述
本章节介绍 VSLAM 功能以及如何使用 XVSDK 快速开始一个 VSLAM 编程。

2. 特性
2.1 VSLAM 工作模式
Xvisio VSLAM支持两种模式：一种是混合模式(mix mode), 也就是 VSLAM 算法一部分运行在 Xvisio 设备端，一部分运行在 SDK 端 ;另外一种模式是边缘模式(edge mode) ,SLAM 算法完全跑在 Xvisio 设备端。
两种方式由用户根据所使用的场景来自由选择和切换模式，通常大部分场景都使用混合模式，在一些较弱能力的 CPU 环境可以使用边缘模式。

2.2 VSLAM 中心点
VSLAM 是以 Xvisio 设备的 IMU 器件为中心点, VSLAM 启动后会基于设备重力方向建立 VSLAM 坐标系，坐标系原点在每次 VSLAM 启动时的IMU 器件上，启动时原点的 6DOF 的 x、y、z值为0，pith、yaw、roll 的值是当前 IMU 器件的 rotation。
设备以水平放置启动：
image
设备以倾斜放置启动：
image

2.3 VSLAM 坐标系
Xvisio VSLAM坐标系默认右手坐标系，X轴向右正方向，Y轴向下正方向，Z轴向前正方向，示意图如下所示：
image

2.4 6DOF
2.4.1 6DOF Pose 结构体
6DOF Pose 支持3种格式（欧拉角，旋转矩阵，四元数）表示rotation，6DOF结构体如下所示：

struct Pose : public details::PosePred_<double> {
    Pose();
    /**
    * @brief Construct a pose with a translation, rotation, timestamps and confidence.
    */
    Pose(Vector3d const& translation, Matrix3d const& rotation,
          double hostTimestamp = std::numeric_limits<double>::infinity(), std::int64_t edgeTimestamp = (std::numeric_limits<std::int64_t>::min)(), double c=0.);
    /**
     * @brief Prediction of the pose based on angular and linear velocity and acceleration.
     * @param dt amount of prediction (in s)
     * @return The predicted (2nd order extrapolation) of the orientation.
     */
    Pose prediction(double dt) const;
}; 
hostTimestamp 的time base 是host端，以host端boot开始计时（从0开始计时）；
deviceTimestamp 的time base 是device端，以device boot开始计时（从0开始计时）；
confidence 是6dof的可信赖level,值为0表示lost ,1表示信任度最高。

2.4.2 获取 6DOF
Xvisio VSLAM支持三种获取 6DOF 方式:

回调方式
先注册回调函数，由 SDK 根据帧率主动触发回调并传参 6DOF 数据。
实时性要求一般的应用使用，流程简便。
device->slam()->registerCallback( poseCallback );
callback的帧率一般由imu的帧率决定（6DOF使用imu做fusion）。
getpose()方式
由应用主动调用接口函数来获取最新的 6DOF 位姿。
实时性要求高的调用方式，由应用设计流程来实时获取最新位姿。
bool getPose(Pose &pose, double prediction) ;
开发者可以调用这个接口，主动获取6DOF 数据，同时可以设置prediction time，但建议小于0.016（16ms，如果prediction过大，6DOF预测不一定准确）。
getposeAt()方式
由应用主动调用接口函数来获取指定时间戳的 6DOF 位姿，通常用来需要同步功能时，例如传参 RGB 帧的时间戳可以获取对应这一帧 RGB 图像的位姿。
bool getPoseAt(Pose& pose, double timestamp);
2.5 VSLAM 建图模式
2.5.1 VIO
VIO模式不包含地图 loopclosure，随着里程计数增加，累计误差也会增加。VSLAM 启动默认都是 VIO模式，实现VIO功能，主要使用下面几个接口：

bool start();
bool stop();
int registerCallback(std::function<void (xv::Pose const&)>);
bool unregisterCallback(int callbackId);
bool getPose(Pose &pose, double prediction) ;  
接口调用流程如下所示：

注册lost callback；
调用start()，开启SLAM；
调用getPose（）获取6DOF；
调用stop()，停止SLAM。
具体code参考demo-api.cpp（case 2，case3）,demo-api 。
2.5.2 Online Loopclosure
不需要提前建图，使用的过程中，在线建图，并在后台实时检测是否闭环，如果闭环后，会做loopclosure优化，缺点是地图只保存在内存中，重启设备后，并不能复用。
online loopclosure的调用接口和 VIO 模式很接近，只有一个设置的区别，使用到的接口如下：

bool start();
bool stop();
bool xv::SlamEx::setEnableOnlineLoopClosure(bool enable);
int registerCallback(std::function<void (xv::Pose const&)>);
bool unregisterCallback(int callbackId);
bool getPose(Pose &pose, double prediction) ; 
接口调用流程如下所示：

注册lost callback；
调用setEnableOnlineLoopClosure(true)，使能Online LoopClosure；
调用start()，开启SLAM；
调用getPose（）获取6DOF；
调用stop()，停止SLAM。
2.5.3 CSLAM
实现CSLAM功能，主要使用下面几个接口：

bool start();
bool stop();
bool loadMapAndSwitchToCslam(std::streambuf &mapStream, std::function<void (int)> done_callback, std::function<void (float)> localized_on_reference_map);
bool saveMapAndSwitchToCslam(std::streambuf &mapStream, std::function<void (int, int)> done_callback, std::function<void (float)> localized_on_reference_map);  
建图的步骤：
要建立一个好的地图，你必须首先考虑如何使用地图。地图必须包含应用程序所需的视点。如果最终应用程序在另一个房间，则没有理由在当前房间中记录地图。类似地，如果最终应用程序将显示地面上的一些虚拟对象，则没有理由将地图记录到使用摄像头向上看的房间中。应用程序要移动的路径应该是记录地图的最终路径。为了保证一个好的循环闭合，在同一条路径上走两次：例如，从一个起点开始，走开，回到起点，然后再次在同一条路径上走开，然后回到起点结束。在记录过程中，在同一条路径上行走两次，可以保证在循环闭合检测的不同记录视点之间有很好的重叠。在做这个地图记录时，重要的是避免快速移动或面对没有特征的区域。
Sample:
具体code参考demo-api.cpp（case 20，case21），demo-api 。
下面是API调用流程和示意图，按两种使用场景介绍：
场景1：建图后切换到CSLAM

API调用流程：

VSLAM start()
device->slam()->start(xv::Slam::Mode::Mixed);
如果不使用callback方式获取6dof可以不用注册
device->slam()->registerCallback( poseCallback );
调用saveMapAndSwitchToCslam，自动保存地图并切换到cslam,调用时需要传参 map stream，done_callback 函数以及localized_on_reference_map 函数,示例:
device->slam()->saveMapAndSwitchToCslam(mapStream,cslamSavedCallback, cslamLocalizedCallback);
saveMapAndSwitchToCslam详细查询，可以使用C++ 应用接口查询.
建图过程中，6DOF获取会同时生效，已注册6DOF callback的会触发，或者可以通过getPose()方式获取6DOF
调用stop()，停止cslam
device->slam()->stop();
场景2：Load 已有地图后切换到CSLAM

VSLAM 启动 , 注册6DOF callback（如果使用callback方式获取6DOF,需要注册）
device->slam()->start(xv::Slam::Mode::Mixed);
device->slam()->registerCallback( poseCallback );
调用loadMapAndSwitchToCslam ，自动装载地图并切换到cslam,调用时需要传参 map stream，done_callback 函数以及localized_on_reference_map 函数,示例:
device->slam()->loadMapAndSwitchToCslam(mapStream,cslamSwitchedCallback, cslamLocalizedCallback);
loadMapAndSwitchToCslam 详细查询，可以使用C++ 应用接口查询.
建图过程中，6DOF获取会同时生效，已注册6DOF callback的会触发，或者可以通过getPose()方式获取6DOF。
调用stop()，停止CSLAM
device->slam()->stop();
3. VSLAM 代码:
定义 Xvisio Device
std::shared_ptr<xv::Device> device = nullptr;
读取 device_list,超时 10 秒(推荐值，也可以缩短)
auto device_list = xv::getDevices(10., "");
判断获取的 devices 是否为空，为空则失败
if (device_list.empty())`
{
        LOG(LOG_Function,"initDevice faiiled:Timeout for device detection.");
        return false;
}

获取 device （这里只针对单设备，多设备请参考相应文档描述）
 device = device_list.begin()->second;
启动Online LoopClosure，如果不调用这句话那么默认就是 VIO 模式
std::static_pointer_cast<xv::SlamEx>(device->slam())->setEnableOnlineLoopClosure(true);
启动混合模式 VSLAM
device->slam()->start(xv::Slam::Mode::Mixed);
启动边缘模式 VSLAM
device->slam()->start(xv::Slam::Mode::Edge);
回调方式获取 6DOF 位姿
int poseId = -1;
poseId = device->slam()->registerCallback([](const xv::Pose& pose){
    ...  
    });

getPose() 方式获取 6DOF 位姿 ,这里预测设置了10ms
double prediction = 0.010;
bool ok = device->slam()->getPose( pose, prediction );
if ( !ok ) {
...//failed.
}

getPoseAt() 方式获取 6DOF 位姿,这里指定的时间戳为伪代码'spec_TS'
double t = spec_TS;
xv::Pose poseAt;
bool ok = device->slam()->getPoseAt( poseAt, t );
if ( !ok ) {
...//failed.
}
CSlam 建图
std::filebuf mapStream;`
if (mapStream.open(map_filename, std::ios::binary | std::ios::out | std::ios::trunc) == nullptr) {
                std::cout << "open " << map_filename << " failed." << std::endl;
                break;
            }
device->slam()->saveMapAndSwitchToCslam(mapStream, cslamSavedCallback, cslamLocalizedCallback);
注册的callback实现参考:
void cslamSavedCallback(int status_of_saved_map, int map_quality)
{
    std::cout << " Save map (quality is " << map_quality << "/100) and switch to CSlam:";
    switch (status_of_saved_map)
    {
        case  2: std::cout << " Map well saved. " << std::endl; break;
        case -1: std::cout << " Map cannot be saved, an error occured when trying to save it." << std::endl; break;
        default: std::cout << " Unrecognized status of saved map " << std::endl; break;
    }
    mapStream.close();    
}       
void cslamLocalizedCallback(float percent)
{
    static int k = 0;
    if (k++ % 100 == 0) {
        localized_on_reference_percent = static_cast<int>(percent * 100);
        std::cout << "localized: " << localized_on_reference_percent << "%" << std::endl;
    }
}

CSlam 装载已有地图
device->slam()->loadMapAndSwitchToCslam(
                mapStream,
                cslamSwitchedCallback,
                cslamLocalizedCallback
            );

注册的callback实现参考:
void cslamSwitchedCallback(int map_quality)
{
    std::cout << " map (quality is " << map_quality << "/100) and switch to CSlam:";
    mapStream.close();
}
void cslamLocalizedCallback(float percent)
{
    static int k = 0;
    if (k++ % 100 == 0) {
        localized_on_reference_percent = static_cast<int>(percent * 100);
        std::cout << "localized: " << localized_on_reference_percent << "%" << std::endl;
    }
}
SLAM停止
此处用到的 poseId 是注册callback时赋值。
device->slam()->unregisterCallback(poseId);
device->slam()->stop();