#include "utility.h"
#include <serial/serial.h>
#include <termios.h>            //termios, TCSANOW, ECHO, ICANON
#include <unistd.h>     //STDIN_FILENO
#include <thread>
#include <std_message.h>
#include <tf2/LinearMath/Matrix3x3.h>



char key;

class USBCAN
{
public:
    ros::NodeHandle nh;
    ros::Subscriber subLane;
    ros::Subscriber subPose;
    pcl::PointCloud<pcl::PointXY> centerLanePoints;
    pcl::KdTreeFLANN<pcl::PointXY> centerLaneKdtree;
    //当前车离centerlan哪几个点最近
    std::vector<int> searchIndex;
    std::vector<float> searchDistance;
    float pose[6];
    float progTime=0.1;
    float minProgDist=1.0;
    bool laneLoad=false;
    int currentRoadSegemnt=0;
    int numSegment=50;
    vector<pcl::PointCloud<pcl::PointXY>> roadSegment;
    //serial
    serial::Serial spCan0 ,spCan1;
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    
    bool serialInitDone=false;
    // ros::Timer timer10ms = nh.createTimer(ros::Duration(0.01), &USBCAN::timer10msCallback,this);
    // ros::Timer timer20ms = nh.createTimer(ros::Duration(0.02), &USBCAN::timer20msCallback,this);
    // ros::Timer timer5ms = nh.createTimer(ros::Duration(0.03), &USBCAN::timer5msCallback,this);
    uint8_t msg080Count=0x00;
    std_message_Light msg303;
    parma_msg303_TypeDef parma_msg303;
    std_message msg175;
    parma_msg175_TypeDef parma_msg175;

    std_message msg125;
    parma_msg125_TypeDef parma_msg125;

    std_message msg080;
    parma_msg080_TypeDef parma_msg080;
    USBCAN()
    {
        subPose=nh.subscribe<nav_msgs::Odometry>("/currPose", 1, &USBCAN::poseHandler, this);
        subLane=nh.subscribe<sensor_msgs::PointCloud2>("/centerLane",1,&USBCAN::centerLaneHandler,this);
        initSerialPort();
        Message_Init();
        thread keyThread(&USBCAN::readKeyThread,this);
        keyThread.detach();
        
    }
    void readKeyThread()
    {
        while((key=getchar())!='q')
        {
            printf("%c\r\n",key);
            
            key='.';
        }
        
    }
    void timer5msCallback(const ros::TimerEvent& event)
    {
        //获取缓冲区内的字节数
        static uint8_t buffer[1024];
        size_t n = spCan1.available();
        printf("resive bytes:%d\r\n",n);
        if(n>17)
        {
            n = spCan1.read(buffer, n);
            int frameNum=1;
            int s=0,e=16,offset=0;
            uint16_t id=0x0000;
            uint8_t data[8];
            while (e+offset<n)
            {
                if(buffer[s+offset]==0xaa&&buffer[e+offset]==0x7a)
                {
                    // printf("correct can frame %d\r\n",frameNum);
                    id=buffer[s+offset+6]<<8|buffer[s+offset+7];
                    for(size_t j=0;j<8;j++)
                    {
                        data[j]=buffer[s+offset+8+j];
                    }
                    printf("ID:%04x,data:",id);
                    for(size_t j=0;j<8;j++)
                    {
                        printf(" %02x ",data[j]);
                    }
                    printf("\r\n");
                    s+=17;
                    e+=17;
                    frameNum++;
                }
                else
                {
                    offset++;
                    printf("offset:%d\r\n",offset);
                }
            }
            
        }
        // if(n>=17)
        // {
            
        //     //读出数据
        //     n = spCan1.read(buffer, n);
        //     if(buffer[0]==0xaa&&buffer[17]==0x07a)
        //     {
        //         for(int i=0; i<n; i++)
        //         {
                    
        //             //16进制的方式打印到屏幕
        //             // std::cout << std::hex << (buffer[i] & 0xff) << " ";

        //             buffer[i]=0;
        //         }
        //     }
            
        //     std::cout << std::endl;
        // }
        // spCan1.flushInput();
    }
    void timer10msCallback(const ros::TimerEvent& event)
    {
        static int ms10count=0;
        static bool s=0;
        if (serialInitDone==false)
        {
            return ;
        }
        msg080Count++;
        if(msg080Count>15)
        {
            msg080Count=0;
        }
        parma_msg080.APA_RollingCount_APA2=msg080Count;
        deal_msg080(&msg080,&parma_msg080);
        parma_msg080.APA_CheckSum_APA2=msg080.data[0]^msg080.data[1]^msg080.data[2]^msg080.data[3]^msg080.data[4]^msg080.data[5]^msg080.data[6]^msg080.data[7];
        deal_msg080(&msg080,&parma_msg080);
        sendCANmessage(msg080);
        ros::Duration(0.002).sleep();
        sendCANmessage(msg125);
        s=!s;
        if(s)
        {
            parma_msg175.APA_Checksum=msg175.data[0]^msg175.data[1]^msg175.data[2]^msg175.data[3]^msg175.data[4]^msg175.data[5]^msg175.data[6];
            deal_msg175(&msg175,&parma_msg175);
            ros::Duration(0.002).sleep();
            sendCANmessage(msg175);
        }
        // printf("10ms %6d\r\n",ms10count++);

        
    }
    void timer20msCallback(const ros::TimerEvent& event)
    {
        static int ms20count=0;
        if (serialInitDone==false)
        {
            return ;
        }
        parma_msg175.APA_Checksum=msg175.data[0]^msg175.data[1]^msg175.data[2]^msg175.data[3]^msg175.data[4]^msg175.data[5]^msg175.data[6];
        deal_msg175(&msg175,&parma_msg175);
        // ros::Duration(0.002).sleep();
        sendCANmessage(msg175);
        // printf("20ms %6d\r\n",ms20count++);

    }
    void initSerialPort()
    {
        spCan0.setPort("/dev/usbcan0");
        spCan0.setBaudrate(256000);//最大支持460800
        spCan0.setTimeout(to);
        spCan0.setFlowcontrol(serial::flowcontrol_t::flowcontrol_hardware);
        spCan1.setPort("/dev/usbcan1");
        spCan1.setBaudrate(256000);//最大支持460800
        spCan1.setTimeout(to);
        spCan1.setFlowcontrol(serial::flowcontrol_t::flowcontrol_hardware);
        try
        {
            //打开串口
            spCan0.open();
            spCan1.open();
        }
        catch(serial::IOException& e)
        {
            ROS_ERROR_STREAM("Unable to open port.");
            cout<<e.what()<<endl;
            // return -1;
        }
        
        //判断串口是否打开成功
        if(spCan0.isOpen()&&spCan1.isOpen())
        {
            ROS_INFO_STREAM("/dev/ttyUSB0 is opened.");
            serialInitDone=true;
        }
        else
        {
            // return -1;
        }
    }
    void Message_Init()
    {
        parma_msg303.HAD_HazardCtrlCmd=1;
        parma_msg303.HAD_HighBeamCtrlCmd=1;
        parma_msg303.HAD_LeftlightCtrlCmd=1;
        parma_msg303.HAD_LowBeamCtrlCmd=1;
        parma_msg303.HAD_RightlightCtrlCmd=1;
        deal_msg303(&msg303,&parma_msg303);//灯光全灭
        
        parma_msg175.APA_TargetGearReq=1;//P档
        parma_msg175.APA_VCUControl=0;//0未激活，1激活
        parma_msg175.APA_RequestedMaxTroque=0;//最大扭矩0
        parma_msg175.APA_RequestedTroque=0;//请求的扭矩
        parma_msg175.APA_RollingCounter=0;//滚动计数值
        parma_msg175.APA_EPB_Request=2;//1驻车释放，2驻车夹紧
        parma_msg175.APA_WorkStatus=0;//APA关闭
        parma_msg175.APA_Checksum=msg175.data[0]^msg175.data[1]^msg175.data[2]^msg175.data[3]^msg175.data[4]^msg175.data[5]^msg175.data[6];
        deal_msg175(&msg175,&parma_msg175);//

        parma_msg125.ACC_RollingCounter=0;
        parma_msg125.ACC_Status=0;//0无制动请求，1制动请求
        parma_msg125.ACC_TgtAx=27*100;//目标减速度 比例为真实：数据=1：100，有-27的偏移量，27时，减速度为0
        parma_msg125.ACC_TgtAxLowerComftBand=0;//没用到
        parma_msg125.ACC_TgtAxUpperComftBand=0;//没用到
        parma_msg125.ACC_UpperComftBandReq=0;//没用到
        parma_msg125.ACC_CheckSum=0;//握手信号已屏蔽
        deal_msg125(&msg125,&parma_msg125);

        parma_msg080.APA_Angle_Wheel=0;
        parma_msg080.APA_Anglular_Velocity=0;
        parma_msg080.APA_AutoDrive_Enable=0;
        parma_msg080.APA_RollingCount_APA2=0;
        deal_msg080(&msg080,&parma_msg080);
        parma_msg080.APA_CheckSum_APA2=msg080.data[0]^msg080.data[1]^msg080.data[2]^msg080.data[3]^msg080.data[4]^msg080.data[5]^msg080.data[6];
        deal_msg080(&msg080,&parma_msg080);
    }
    void sendCANmessage(std_message msg)
    {
        static uint8_t serialBuffer[17];
        serialBuffer[0]=0xaa;
        serialBuffer[1]=0x00;
        serialBuffer[2]=0x00;
        serialBuffer[3]=0x08;
        serialBuffer[4]=0x00;
        serialBuffer[5]=0x00;
        serialBuffer[6]=msg.id>>8;
        serialBuffer[7]=msg.id;
        serialBuffer[8]=msg.data[0];
        serialBuffer[9]=msg.data[1];
        serialBuffer[10]=msg.data[2];
        serialBuffer[11]=msg.data[3];
        serialBuffer[12]=msg.data[4];
        serialBuffer[13]=msg.data[5];
        serialBuffer[14]=msg.data[6];
        serialBuffer[15]=msg.data[7];
        serialBuffer[16]=0x7a;
        
        spCan0.write(serialBuffer,17);
        // spCan0.flushOutput();
    }
    void poseHandler(const nav_msgs::Odometry::ConstPtr& msg)
    {
        // cout<<"?"<<endl;
        // printf("resived");
        tf2::Quaternion quat(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                     msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        tf2::Matrix3x3 matrix(quat);
        double roll, pitch, yaw;
        matrix.getRPY(roll, pitch, yaw);
        pose[0]=roll;
        pose[1]=pitch;
        pose[2]=yaw;
        pose[3]=msg->pose.pose.position.x;
        pose[4]=msg->pose.pose.position.y;
        pose[5]=msg->pose.pose.position.z;

        // cout<<"x:"<<pose[3]<<"y:"<<pose[4]<<endl;
        if(laneLoad)
        {
            double steer=getSteering(roadSegment[currentRoadSegemnt],pose[3],pose[4]);
            printf("x:%6f,y:%6f,steer:%6f,currentSegment:%2d\r\n",pose[3],pose[4],steer,currentRoadSegemnt);
        }
    }
    void centerLaneHandler(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        pcl::fromROSMsg(*msg,centerLanePoints);
        if(centerLanePoints.points.size()>0)
        {
            centerLaneKdtree.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXY>>(centerLanePoints));
            
            int numPreSegment=centerLanePoints.points.size()/numSegment;
            int j=0;
            for(int i=0;i<numSegment;i++)
            {
                pcl::PointCloud<pcl::PointXY> tempP;
                if(i==numSegment-1)
                {
                    for(;j<centerLanePoints.points.size();j++)
                    {
                        tempP.points.push_back(centerLanePoints.points[j]);
                    }
                }
                else
                {
                    for(j=i*numPreSegment;j<(i+1)*numPreSegment;j++)
                    {
                        tempP.points.push_back(centerLanePoints.points[j]);
                    }
                }
                
                roadSegment.push_back(tempP);
            }
            cout<<"Lane loaded! "<<endl;
            laneLoad=true;
        }
    }
    float getVehicleSpeed()
    {
        return 1.0;
    }
    double getSteering(const pcl::PointCloud<pcl::PointXY>& targetPath, const float posex,const float posey)
    {

        std::vector<float> pts;
        for (size_t i = 0; i < targetPath.points.size(); ++i)
        {
            pts.push_back(pow((posex - (float)targetPath.points[i].x), 2) + pow((posey - (float)targetPath.points[i].y), 2));
        }

        size_t index = std::min_element(pts.begin(), pts.end()) - pts.begin();
        size_t forwardIndex = 0;
        
        float mainVehicleSpeed=getVehicleSpeed();
        float progDist = mainVehicleSpeed * progTime > minProgDist ? mainVehicleSpeed * progTime : minProgDist;

        for (; index < targetPath.size(); ++index)
        {
            forwardIndex = index;
            float distance = sqrtf(((float)pow(targetPath.points[index].x - posex, 2) + pow((float)targetPath.points[index].y - posey, 2)));
            if (distance >= progDist)
            {
                break;
            }
        }
        // printf("targetPath.points.size():%d,index:%d\r\n",targetPath.points.size(),index);
        if(targetPath.points.size()-index<3)
        {
            currentRoadSegemnt++;
            if(currentRoadSegemnt>=(numSegment-1))//final segemnt end
            {
                currentRoadSegemnt=(numSegment-1);
                printf("end of the road\r\n");
            }
        }
        double psi = (double)pose[2];
        double alfa = atan2(targetPath.points[forwardIndex].y - posey, targetPath[forwardIndex].x - posex) - psi;
        double ld = sqrt(pow(targetPath[forwardIndex].y - posey, 2) + pow(targetPath[forwardIndex].x - posex, 2));
        double steering = -atan2(2. * (1.3 + 1.55) * sin(alfa), ld) * 36. / (7. * M_PI);
        
        return steering;
    }

};



int main(int argc, char** argv)
{
    ros::init(argc, argv, "lio_sam");
    
    USBCAN usbcan;
       
    static struct termios oldt, newt;

    /*tcgetattr gets the parameters of the current terminal
    STDIN_FILENO will tell tcgetattr that it should write the settings
    of stdin to oldt*/
    tcgetattr( STDIN_FILENO, &oldt);
    /*now the settings will be copied*/
    newt = oldt;

    /*ICANON normally takes care that one line at a time will be processed
    that means it will return if it sees a "\n" or an EOF or an EOL*/
    newt.c_lflag &= ~(ICANON);          

    /*Those new settings will be set to STDIN
    TCSANOW tells tcsetattr to change attributes immediately. */
    tcsetattr( STDIN_FILENO, TCSANOW, &newt);
    
    ros::spin();
    /*restore the old settings*/
    tcsetattr( STDIN_FILENO, TCSANOW, &oldt);
    return 0;
}