# dense-poind-cloud-tips
ORB-SLAM2稠密点云重建:RGBD室内[0]：

众所周知，ORB-SLAM2可以基于特征点的得到稀疏点云:
然而，有时我们需要稠密点云。
不知道怎么做？我会在博客中分别介绍：
1.基于ORB-SLAM2的RGBD稠密点云重建（多用于室内）
2.基于ORB-SLAM2的双目稠密点云重建(多用于室外)
注意！：这里的“重建”并没有改变使用的特征点等行为，对原来的ORB-SLAM2没有进行任何改造，只是利用其输出信息与原始图片进行场景重建而已。
平台:虽然ORB-SLAM2在Linux下使用方便。但我习惯了Win下编程。所以把其输出文件拷贝到了Win下，使用VS2017。
代码：我会在最后一篇单独的博文中提供代码。(因为在开写这篇时还未整理，嘿嘿(●ˇ∀ˇ●))
接下来几篇都会介绍RGBD稠密重建。不介绍原理，仅简单解释代码。
步骤0：
了解流程。作者其实已经在其论文中展示了RGBD稠密重建，并简单说明了原理。github主页上的第二篇论文:ORB-SLAM2: An Open-Source SLAM System for Monocular, Stereo, and RGB-D Cameras。如果你不能打开原文链接，可以直接百度搜，比如此链接。
步骤1：
学习高翔博士的几篇博客。我使用他的一起做RGB-D SLAM(1)开始，到(4)，即点云拼接的部分，借鉴了其思想与代码。在此非常感谢。

ok。第一篇篇幅就到这儿了。下一篇正式开始。

接上一篇.
步骤2：了解PCL
在PCL文档中简单了解其基本用法和工具。我在其官方教程中学习并使用了PCL的一些滤波工具。
步骤3：准备工作
首先，根据ORB-SLAM2的git主页，安装好ORB-SLAM2（注意openCV"干净"安装，即如果你要重装openCV,要完整卸载之前版本，具体百度），选择你的数据集跑RGB-D的例子。此时可以得到KeyFrameTrajectory.txt。我使用了TUM数据集上的rgbd_dataset_freiburg1_room，因为它拥有闭环检测，比较精确。我在D盘创建了“SLAM”文件夹，把数据集放进去了。然后把KeyFrameTrajectory.txt也复制进文件夹，重命名为RGBD_RoomTraj.txt。我把association文件也复制进去，重命名为RGBD_RoomAssociations.txt。至此，程序输入数据就准备完成了。
步骤4：声明主函数
我把主函数称为JoinPC（意思为JoinPointCloud，也许起得不好，将就吧）。
用法：
[cpp] view plain copy
JoinPC("D:\\SLAM\\RGBD_RoomTraj.txt","D:\\SLAM\\RGBD_RoomAssociations.txt","D:\\SLAM\\rgbd_dataset_freiburg1_room");  
原型：

[cpp] view plain copy
void JoinPC(string camTransFile,string associateFile,string dataMainPath)  
步骤5：新建VS项目，添加高翔博士的代码

新建VS C/C++控制台项目，配置好OpenCV和PCL（我使用openCV2411和PCL1.8.1）。添加高翔博士第四讲中的slamBase.h和slamBase.cpp。
步骤6：修改ParameterReader，新建参数文件
对于我使用的TUM数据集room建立对应相机的参数文件，参数我在TUM上找到了，下面贴出参数文件内容：
[plain] view plain copy
detector=ORB  
descriptor=ORB  
good_match_threshold=10  
  
# camera  
camera.cx=318.6;  
camera.cy=255.3;  
camera.fx=517.3;  
camera.fy=516.5;  
camera.scale=5000.0;  
我重命名其为RGBDparameters.txt，放入VS项目代码的同目录下。
由于要使用这个文件，所以对高博的ParameterReader类进行修改:

[cpp] view plain copy
class ParameterReader  
{  
public:  
    ParameterReader(string filename = "RGBDparameters.txt")  
...  

然后就可以用了。向主函数填充代码：
[cpp] view plain copy
void JoinPC(string camTransFile,string associateFile,string dataMainPath) {  
    ParameterReader pd;  
    // 相机内参  
    CAMERA_INTRINSIC_PARAMETERS camera;  
    camera.fx = atof(pd.getData("camera.fx").c_str());  
    camera.fy = atof(pd.getData("camera.fy").c_str());  
    camera.cx = atof(pd.getData("camera.cx").c_str());  
    camera.cy = atof(pd.getData("camera.cy").c_str());  
    camera.scale = atof(pd.getData("camera.scale").c_str());  

步骤7：新建两个类，读取关键帧文件信息，association文件信息
新建类XCTool，并添加3个东西：XCKey类读取关键帧信息，XCAssociationKey类读取关联信息，FindDFileByRGB函数顾名思义。
XCTool.h:
[cpp] view plain copy
#pragma once  
#include <string>  
#include <vector>  
using std::string;  
using std::vector;  
class XCTool  
{  
public:  
  
};  
  
class XCKey {  
public:  
    string frameID;  
    double tx, ty, tz;  
    double qx, qy, qz, qw;  
};  
  
class XCAssociationKey {  
public:  
    string rgb, full_rgb, d, full_d;  
};  
  
class XCKITTIKey {  
public:  
    double r00, r01, r02, r10, r11, r12, r20, r21, r22;  
    double tx, ty, tz;  
};  
  
string FindDFileByRGB(vector<XCAssociationKey>& aKeyVec, string rgb);  

XCTool.cpp:
[cpp] view plain copy
#include "stdafx.h"  
#include "XCTool.h"  
  
string FindDFileByRGB(vector<XCAssociationKey>& aKeyVec, string rgb) {  
    for (auto& iter : aKeyVec) {  
        if (iter.rgb == rgb) {  
            return iter.d;  
        }  
    }  
    abort();  
}  
向主函数添加代码，目前是：
[cpp] view plain copy
void JoinPC(string camTransFile,string associateFile,string dataMainPath) {  
    ParameterReader pd;  
    // 相机内参  
    CAMERA_INTRINSIC_PARAMETERS camera;  
    camera.fx = atof(pd.getData("camera.fx").c_str());  
    camera.fy = atof(pd.getData("camera.fy").c_str());  
    camera.cx = atof(pd.getData("camera.cx").c_str());  
    camera.cy = atof(pd.getData("camera.cy").c_str());  
    camera.scale = atof(pd.getData("camera.scale").c_str());  
  
    ifstream fcamTrans(camTransFile);  
    vector<XCKey> keyVec;  
    while (!fcamTrans.eof()) {  
        XCKey tkey;  
        fcamTrans >> tkey.frameID;  
        fcamTrans >> tkey.tx;  
        fcamTrans >> tkey.ty;  
        fcamTrans >> tkey.tz;  
        fcamTrans >> tkey.qx;  
        fcamTrans >> tkey.qy;  
        fcamTrans >> tkey.qz;  
        fcamTrans >> tkey.qw;  
        keyVec.push_back(tkey);  
    }  
  
    ifstream fAssociation(associateFile);  
    vector<XCAssociationKey> assoKeyVec;  
    while (!fAssociation.eof()) {  
        XCAssociationKey takey;  
        fAssociation >> takey.rgb;  
        fAssociation >> takey.full_rgb;  
        fAssociation >> takey.d;  
        fAssociation >> takey.full_d;  
        assoKeyVec.push_back(takey);  
    }  
  
    vector<string> rgbPathVec, dPathVec;  
    for (int i = 0; i < keyVec.size(); i++) {  
        rgbPathVec.push_back(dataMainPath + "\\rgb\\" + keyVec[i].frameID + ".png");  
        dPathVec.push_back(dataMainPath + "\\depth\\" + FindDFileByRGB(assoKeyVec,keyVec[i].frameID)+".png");  
    }  
这样，keyVec中保存了关键帧信息。rgbPathVec, dPathVec就保存了rgb图路径和深度图路径，是一一对应的。

好了，要用的变量都准备好了，本篇篇幅就到这。

步骤8：从rgb图和d图初始化点云，然后滤波（可选）
[cpp] view plain copy
cout << "Initing...\n";  
    //初始化点云  
    vector<PointCloud::Ptr> pcVec;  
    for (int i = 0; i < keyVec.size(); i++) {  
        FRAME tframe;  
        tframe.rgb = cv::imread(rgbPathVec[i]);  
        tframe.depth = cv::imread(dPathVec[i], -1);  
        pcVec.push_back(image2PointCloud(tframe.rgb, tframe.depth, camera));  
    }  
  
    cout << "Filtering...\n";  
    //滤波  
    bool bFilter = true;  
    if (bFilter) {  
        for (auto& pciter : pcVec) {  
            PointCloud::Ptr cloud_filtered(new PointCloud());  
            pcl::VoxelGrid<pcl::PointXYZRGBA> sor;  
            sor.setInputCloud(pciter);  
            sor.setLeafSize(0.01f, 0.01f, 0.01f);  
            sor.filter(*cloud_filtered);  
            pciter = cloud_filtered;  
  
            PointCloud::Ptr cloud_filtered2(new PointCloud());  
            pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> sor2;  
            sor2.setInputCloud(cloud_filtered);  
            sor2.setMeanK(50);  
            sor2.setStddevMulThresh(1.0);  
            sor2.filter(*cloud_filtered2);  
  
            pciter = cloud_filtered2;  
        }  
    }  
初始化点云使用了高博的image2PointCloud函数，滤波使用了PCL官方教程上的体素滤波和离散值滤波，之前都有提过链接。
现在,pcVec中有了每个滤过波的关键帧的点云了。
步骤9：使用关键帧文件信息进行点云拼接
这一部分最会出坑的地方是对于输出信息的理解，还有四元数到旋转矩阵的转化公式，位移的正负。
从代码上可以看出，ORB-SLAM2的RGBD输出文件每行有8个数字，我以我跑的room出来的关键帧文件第一行为例进行解释：
[plain] view plain copy
1305031910.765238 0.0001796 -0.0002834 0.0001877 -0.0000384 -0.0001034 -0.0001286 1.0000000  
第一个数字是时间戳，在此也就是rgb文件名；后面3个数字是世界位置的负数，注意是负数！这是一个大坑。如果没有闭环检测，应该是0,0,0，但这里有，对第一个点进行了调整，所以不是了；再后面4个数字是表示世界旋转，同理，没有闭环检测应该是0,0,0,1。这里要注意的是每行都是世界坐标与旋转（没有闭环检测的话，世界原点是第一帧），我被其源码注释搞糊涂了，以为是每帧相对上一帧的变换，运行出来发现并不是。
接下来是四元数转换的坑，我就不赘述，使用下面的代码就可以了。
思路就是关键帧既然给了我们世界位置的负数，四元数。我们对pcVec中每个点云，位置直接加上世界位置的负数，旋转使用四元数对应的旋转矩阵的逆，就能将所有点云对齐到同一个原点。代码：
[cpp] view plain copy
for(int i=0;i<keyVec.size();i++){      
        double x = keyVec[i].qx, y = keyVec[i].qy, z = keyVec[i].qz, w = keyVec[i].qw;  
  
        cv::Mat R;  
          
        R = (cv::Mat_<double>(3, 3)<<  
            2*(x*x+w*w)-1,2*(x*y+z*w),2*(x*z-y*w),  
            2*(x*y-z*w), 2*(y*y+w*w)-1, 2*(y*z+x*w),  
            2*(x*z+y*w), 2*(y*z-x*w), 2*(z*z+w*w)-1  
            );  
        R = R.inv();  
        Eigen::Matrix3d r;  
        cv::cv2eigen(R, r);  
  
        // 将平移向量和旋转矩阵转换成变换矩阵  
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();  
  
        Eigen::AngleAxisd angle(r);  
        cout << "translation" << endl;  
  
        T = angle;  
        T(0, 3) = keyVec[i].tx;  
        T(1, 3) = keyVec[i].ty;  
        T(2, 3) = keyVec[i].tz;  
          
        PointCloud::Ptr toutput(new PointCloud());  
        pcl::transformPointCloud(*pcVec[i], *toutput, T.matrix());  
        pcVec[i] = toutput;  
    }  
    cout << "trans over\n";  


步骤10：组合点云，保存，使用PCL展示
在编码的过程中我也尝试了PCL的一些registration算法（比如常见的ICP），最终还是不使用了。有以下几个原因：
闭环检测了的室内ORB-SLAM2的RGBD位姿结果足够精确，registration没有意义
registration的这些算法耗时长，不能保证效果，有些算法还需要不错的初始猜测参数
我虽然放在了代码里，但是没有用。allOutput是最终组合起来的大点云，进行了保存和展示。代码：
[cpp] view plain copy
int algoFlag = 0;  
    bool bNDT = false;  
    int ndtindex = 0;  
    bool bPCH = false;  
    if (algoFlag==0) {  
        for (int i = 0; i < pcVec.size(); i++) {  
            *allOutput += *pcVec[i];  
        }  
    }  
    else if (algoFlag == 1) {  
        allOutput = pcVec[0];  
        for (int i = 0; i < pcVec.size() - 1; i++) {  
            PairwiseICP(pcVec[i + 1], allOutput, allOutput);  
        }  
    }  
    else if(algoFlag==2){  
        //记录每相邻两个点云ICP的矩阵  
        vector<Eigen::Matrix4f> icpMatVec;  
        vector<function<void(void)>> funcVec;  
        vector<thread> taskVec;  
        for (int i = 0; i < pcVec.size()-1; i++) {  
            if (bPCH) {  
                funcVec.push_back([&]() {icpMatVec.push_back(XCPairwisePCH(pcVec[i], pcVec[i + 1])); });  
            }  
            else if(bNDT&&i==ndtindex){  
                funcVec.push_back([&]() {icpMatVec.push_back(XCPairwiseNDT(pcVec[i], pcVec[i + 1])); });  
            }  
            else {  
                funcVec.push_back([&]() {  
                    icpMatVec.push_back(XCPairwiseICP(pcVec[i], pcVec[i + 1]));  
                    /*double t1 = icpMatVec[i](0, 3); 
                    double t2 = icpMatVec[i](1, 3); 
                    double t3 = icpMatVec[i](2, 3); 
                    double r1 = icpMatVec[i](0, 0); 
                    double r2 = icpMatVec[i](0, 1); 
                    double r3 = icpMatVec[i](0, 2); 
                    double r4 = icpMatVec[i](1, 0); 
                    double r5 = icpMatVec[i](1, 1); 
                    double r6 = icpMatVec[i](1, 2); 
                    double r7 = icpMatVec[i](2, 0); 
                    double r8 = icpMatVec[i](2, 1); 
                    double r9 = icpMatVec[i](2, 2); 
                    cout << i+1<<"-"<<i+2<<endl; 
                    cout << "t1:" << t1 << " t2:" << t2 << " t3:" << t3<<endl; 
                    cout << "t：" << sqrt(t1*t1 + t2*t2 + t3*t3)<<endl; 
                    cout << "R:\n"; 
                    cout << r1 << " " << r2 << " " << r3 << endl; 
                    cout << r4 << " " << r5 << " " << r6 << endl; 
                    cout << r7 << " " << r8 << " " << r9 << endl;*/  
                });  
            }  
            taskVec.push_back(thread(funcVec[i]));  
            taskVec[i].join();  
        }  
        cout << "PairOver\n";  
        //同理  
        for (int i = 1; i < pcVec.size() ; i++) {  
            for (int i2 = 0; i2 < i; i2++) {  
                PointCloud::Ptr toutput(new PointCloud());  
                pcl::transformPointCloud(*pcVec[i2], *toutput, icpMatVec[i-1]);  
                pcVec[i2] = toutput;  
                cout << "@fusion:" << i2 + 1 << endl;  
            }  
        }  
  
        for (int i = 0; i < pcVec.size(); i++) {  
            *allOutput += *pcVec[i];  
        }  
    }  
  
    pcl::io::savePCDFile("result.pcd", *allOutput);  
    cout << "Final result saved." << endl;  
  
    pcl::visualization::CloudViewer viewer("viewer");  
    viewer.showCloud(allOutput);  
    while (!viewer.wasStopped())  
    {  
  
    }  

至此JoinPC这个主函数就全部填充完毕。
好了，基于ORB-SLAM2的RGBD室内重建至此就完成了。由于公司不能上传图片与文件，最终效果截图与完整代码将在介绍完“ORB-SLAM2稠密点云重建：双目室外”后的一篇独立博文中给出。

