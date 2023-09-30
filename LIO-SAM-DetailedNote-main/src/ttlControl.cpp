#include "utility.h"
#include <termios.h>            //termios, TCSANOW, ECHO, ICANON
#include <unistd.h>     //STDIN_FILENO
#include <thread>
#include <std_message.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <fstream>
#include <serial/serial.h>
#include <signal.h>
char key;
vector<float> historySteer;
vector<float> filteredValue;
vector<double> lightState,redLightTime,greenLightTime,timeCount,laserSpeed,boardSpeed;
std::mutex mtx;


class TtlControl : public ParamServer
{
public:
    ros::NodeHandle nh;
    ros::Subscriber subLane;
    ros::Subscriber subPose;
    ros::Publisher pubFollowPoint;
    ros::Subscriber subLightAndSpeed;
    pcl::PointCloud<pcl::PointXY> centerLanePoints;
    pcl::KdTreeFLANN<pcl::PointXY> centerLaneKdtree;
    //当前车离centerlan哪几个点最近
    std::vector<int> searchIndex;
    std::vector<float> searchDistance;
    float pose[6]={0,0,0,0,0,0};
    float progTime=0.1;
    float minProgDist=testMinProgDist;
    bool laneLoad=false;
    int currentRoadSegemnt=0;
    int numSegment=50;
    bool poseInit=false;
    vector<pcl::PointCloud<pcl::PointXY>> roadSegment;
    bool inCheck=false;
    //serial
    serial::Serial carControl;
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    bool serialInitDone=false;
    ros::Timer timer10ms = nh.createTimer(ros::Duration(0.01), &TtlControl::timer10msCallback,this);
    float boardSpeedTemp=0;
	int len;
    float steer;
   	char recv_buf[1024];//数据接收缓冲区
   	uint8_t send_buf[6]={0x5a,0xa5,255,0,0,0xaa};//数据传输缓冲区 [0,1,5]是帧头和帧尾，[2,3,4]是控制通道，控制的量的高位和低位。
    float initial_estimate = kalman_initial_estimate;
    float initial_error = kalman_initial_error;
    float measurement_error = kalman_measurement_error;
    float process_error = kalman_process_error;
    float kalman_gain=1.0;
    visualization_msgs::MarkerArray followPoint;
    TtlControl()
    {
        subPose=nh.subscribe<nav_msgs::Odometry>("/currPose", 1, &TtlControl::poseHandler, this);
        subLane=nh.subscribe<sensor_msgs::PointCloud2>("/centerLane",1,&TtlControl::centerLaneHandler,this);
        subLightAndSpeed=nh.subscribe<std_msgs::Float64MultiArray>("/lightAndSpeed",1,&TtlControl::lightAndSpeedHandle,this);
        pubFollowPoint=nh.advertise<visualization_msgs::MarkerArray>("followPoint",1);
        initSerialPort();
        initFollowPointMsg();
        thread keyThread(&TtlControl::readKeyThread,this);
        keyThread.detach();
        
        
    }
    ~TtlControl()
    {

    }
    void initSerialPort()
    {
        carControl.setPort("/dev/carControl");
        carControl.setBaudrate(115200);//最大支持460800
        carControl.setTimeout(to);
        carControl.setFlowcontrol(serial::flowcontrol_t::flowcontrol_hardware);

        try
        {
            //打开串口
            carControl.open();
        }
        catch(serial::IOException& e)
        {
            ROS_ERROR_STREAM("Unable to open port.");
            cout<<e.what()<<endl;
            // return -1;
        }
        
        //判断串口是否打开成功
        if(carControl.isOpen())
        {
            ROS_INFO_STREAM("/dev/carControl is opened.");
            serialInitDone=true;
        }
        else
        {
            // return -1;
        }
    }
    void initFollowPointMsg()
    {
        followPoint.markers.resize(1);
        followPoint.markers[0].header.frame_id="map";
        followPoint.markers[0].header.stamp=ros::Time::now();
        followPoint.markers[0].ns="followPoint___";//命名空间，防止和其他消息冲突
        followPoint.markers[0].id=0;
        followPoint.markers[0].type=visualization_msgs::Marker::POINTS;//画点
        followPoint.markers[0].action = visualization_msgs::Marker::ADD;//相当于clear清除之前的数据
        followPoint.markers[0].pose.position.x = 0.0;
        followPoint.markers[0].pose.position.y = 0.0;
        followPoint.markers[0].pose.position.z = 0.0;
        followPoint.markers[0].pose.orientation.x = 0.0;
        followPoint.markers[0].pose.orientation.y = 0.0;
        followPoint.markers[0].pose.orientation.z = 0.0;
        followPoint.markers[0].pose.orientation.w = 1.0;
        followPoint.markers[0].scale.x = followPointScale;
        followPoint.markers[0].scale.y = followPointScale;
        followPoint.markers[0].scale.z = followPointScale;
        followPoint.markers[0].color.a = 1.0;
        followPoint.markers[0].color.b = 1.0;
    }
    void timer10msCallback(const ros::TimerEvent& event)
    {
        //获取缓冲区内的字节数
        static uint8_t buffer[1024];
        mtx.lock();
        size_t n = carControl.available();
        if(n>=4)
        {
            n = carControl.read(buffer, n);
            int frameNum=1;
            int s=0,e=3,offset=0;
            while (e+offset<n)
            {
                if(buffer[s+offset]==0x5a&&buffer[e+offset]==0xa5)
                {

                    // printf(" %02x ,%02x\r\n",buffer[1],buffer[2]);
                    boardSpeedTemp=(buffer[1]<<8|buffer[2])/100.0;
                    s+=6;
                    e+=6;
                    frameNum++;
                }
                else
                {
                    offset++;
                    printf("offset:%d\r\n",offset);
                }
            }
            
        }
        mtx.unlock();
    }
    void lightAndSpeedHandle(const std_msgs::Float64MultiArray::ConstPtr& msg)
    {
        if(msg->data[0]>0)
        {
            printf("Green Light, time remain:%3.1f s, laserSpeed:%4.1f km/h, boardSpeed:%4.2f km/h\r\n",(msg->data[2]*10-msg->data[3])/10,msg->data[4]/10,boardSpeedTemp);  
        }
        else
        {
            printf("Red   Light, time remain:%3.1f s, laserSpeed:%4.1f km/h, boardSpeed:%4.2f km/h\r\n",(msg->data[1]*10-msg->data[3])/10,msg->data[4]/10,boardSpeedTemp);
        }
        if(inCheck)
        {
            if((msg->data[1]*10-msg->data[3])/10*(msg->data[4]/10)>miniDistance)
            {
                printf("Pass\r\n");
            }
            else
            {
                printf("Wait\r\n");
            }
        }
        lightState.push_back(msg->data[0]);
        redLightTime.push_back(msg->data[1]);
        greenLightTime.push_back(msg->data[2]);
        timeCount.push_back(msg->data[3]);
        laserSpeed.push_back(msg->data[4]);
        boardSpeed.push_back(boardSpeedTemp);
    }
    void pubFollowPointFunc(float x,float y)
    {
        geometry_msgs::Point tmp;
        tmp.x=x;
        tmp.y=y;
        followPoint.markers[0].points.push_back(tmp);
        pubFollowPoint.publish(followPoint);
    }
    float filter(float obs)
    {
        // # Predict step
        initial_error += process_error;
        // printf("%6f\r\n",initial_error);
        // # Update step
        kalman_gain = initial_error / (initial_error + measurement_error);
        initial_estimate += kalman_gain * (obs - initial_estimate);
        initial_error *= (1 - kalman_gain);
        return initial_estimate;
    }

    void readKeyThread()
    {
        while((key=getchar())!='q')
        {
            printf("%c\r\n",key);
            if(key=='C'||key=='c')//红绿灯决策
            {
                inCheck=true;
            }
            else
            {
                inCheck=false;
            }

            mtx.lock();
            
            if(key=='e'||key=='E')//自动控制1阶段
            {
                send_buf[2]=5;
                send_buf[3]=0;
                send_buf[4]=1;
            }
            else if(key=='v'||key=='V')//自动控制2阶段
            {
                send_buf[2]=5;
                send_buf[3]=0;
                send_buf[4]=2;
            }
            else if(key=='b'||key=='B')//取消自动控制
            {
                send_buf[2]=5;
                send_buf[3]=0;
                send_buf[4]=0;
            }
            else if(key=='r'||key=='R')//拉手刹
            {
                send_buf[2]=4;
                send_buf[3]=0;
                send_buf[4]=1;
            }
            else if(key=='t'||key=='T')//松手刹
            {
                send_buf[2]=4;
                send_buf[3]=0;
                send_buf[4]=2;
            }
            else if(key=='p'||key=='P')//P档
            {
                send_buf[2]=3;
                send_buf[3]=0;
                send_buf[4]=1;
            }
            else if(key=='n'||key=='N')//N档
            {
                send_buf[2]=3;
                send_buf[3]=0;
                send_buf[4]=3;
            }
            else if(key=='d'||key=='D')//D档
            {
                send_buf[2]=3;
                send_buf[3]=0;
                send_buf[4]=4;
            }
            else if(key==' ')//Break
            {
                send_buf[2]=1;
                send_buf[3]=0;
                send_buf[4]=(uint8_t)testBreak;
            }
            else if(key=='w'||key=='W')//Torque
            {
                send_buf[2]=0;
                send_buf[3]=0;
                send_buf[4]=(uint8_t)testTorque;
            }
            else if(key=='j'||key=='J')//turn
            {
                send_buf[2]=0x02;
                send_buf[3]=(uint8_t)(6900>>8);
                send_buf[4]=(uint8_t)6900;
            }
            else if(key=='k'||key=='K')//turn
            {
                send_buf[2]=0x02;
                send_buf[3]=(uint8_t)(7800>>8);
                send_buf[4]=(uint8_t)7800;
            }
            else if(key=='l'||key=='L')//turn
            {
                send_buf[2]=0x02;
                send_buf[3]=(uint8_t)(8700>>8);
                send_buf[4]=(uint8_t)8700;
            }
            else if(key=='a'||key=='A')
            {
                send_buf[2]=0x00;
                send_buf[3]=0;
                testTorque+=20;
                printf("testTorque:%3d\r\n",testTorque);
                send_buf[4]=(uint8_t)testTorque;
            }
            else if(key=='z'||key=='Z')
            {
                send_buf[2]=0x00;
                send_buf[3]=0;
                testTorque-=20;
                printf("testTorque:%3d\r\n",testTorque);
                send_buf[4]=(uint8_t)testTorque;
            }
            else if(key=='-')
            {
                turnScale-=5;
                printf("turnScale:%6f\r\n",turnScale);
            }
            else if(key=='=')
            {
                turnScale+=5;
                printf("turnScale:%6f\r\n",turnScale);
            }
            carControl.write(send_buf,6);
            mtx.unlock();
            key='.';
        }
        
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
        if(poseInit==false)
        {
            vector<int> searchIndex;
            vector<float> searchDistance;
            pcl::PointXY cp;
            cp.x=pose[3];
            cp.y=pose[4];
            centerLaneKdtree.nearestKSearch(cp,1,searchIndex,searchDistance);
            pcl::PointXY np=centerLanePoints.points[searchIndex[0]];
            
            for(size_t i=0;i<roadSegment.size();i++)
            {
                currentRoadSegemnt=i;
                for(size_t j=0;j<roadSegment[i].size();j++)
                {
                    if(np.x==roadSegment[i].points[j].x&&np.y==roadSegment[i].points[j].y)
                    {
                        poseInit=true;
                        break;
                    }
                }
                if(poseInit)
                {
                    break;
                }
            }
            printf("currentSegment:%3d\r\n",currentRoadSegemnt);
        }    
        if(laneLoad&&poseInit)
        {
            if(currentRoadSegemnt<(numSegment-1))
            {
                steer=getSteering(roadSegment[currentRoadSegemnt]+roadSegment[currentRoadSegemnt+1],pose[3],pose[4])*turnScale;
            }
            else
            {
                steer=getSteering(roadSegment[currentRoadSegemnt],pose[3],pose[4])*turnScale;
            }
            
            if(steer>779.0)//方向盘能转-780到779度，车辆能识别的精度为0.1，所以我扩大10倍
            {
                steer=779.0;
            }
            if(steer<-779.0)
            {
                steer=-779.0;
            }
            historySteer.push_back(steer);
            steer=filter(steer);
            filteredValue.push_back(steer);
            mtx.lock();
            uint16_t steerInt=(uint16_t) ((steer+779.0)*10);
            send_buf[2]=0x02;
            send_buf[3]=(uint8_t)(steerInt>>8);
            send_buf[4]=(uint8_t)steerInt;
            
            // printf("channel:%3d,value%3d\r\n",send_buf1[2],send_buf1[3]);
            carControl.write(send_buf,6);
            // len=recv(client_sockfd,recv_buf,1024,0);
            // if(len > 0)
            // {
            //     recv_buf[len]='\0';
            //     printf("recv:");
            //     for(int i=0;i<len;i++)
            //     {
            //         printf(" %2x ",recv_buf[i]);
            //     }
            //     printf("\r\n");
            // }
            mtx.unlock();
            
            printf("x:%6f,y:%6f,steer:%6f,steerInt:%5d,currentSegment:%2d\r\n",pose[3],pose[4],steer,steerInt,currentRoadSegemnt);
            
        }
    }
    void centerLaneHandler(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        pcl::fromROSMsg(*msg,centerLanePoints);
        roadSegment.clear();
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
            printf("Lane loaded, total segemnts:%3d\r\n",roadSegment.size());
            laneLoad=true;
        }
    }
    float getVehicleSpeed()
    {
        return 3.0;
    }
    float getSteering(const pcl::PointCloud<pcl::PointXY>& targetPath, const float posex,const float posey)
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
        pubFollowPointFunc(targetPath.points[index].x,targetPath.points[index].y);
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

        // float steering = -atan2(2. * (1.3 + 1.55) * sin(alfa), ld) * 36. / (7. * M_PI);
        float steering = -atan2(2. * (1.3 + 1.55) * sin(alfa), ld)/M_PI*180.0;
        
        return steering;
    }

};

void myShutdownFunction()
{
    ros::Rate rate(0.5);
    while (ros::ok()){
        rate.sleep();
    }
    ROS_INFO("Exiting...");
    // Add any cleanup code or other actions here
    ofstream outfile("/home/limy/roscode/paperData/historySteer.txt");
    if (outfile.is_open()) {
        for (int i = 0; i < historySteer.size(); i++) {
            outfile << historySteer[i] << endl;
        }
        outfile.close();
        cout << "Data historySteer written to file." << endl;
    }
    else {
        cout << "Unable to open file." << endl;
    }

    ofstream outfile1("/home/limy/roscode/paperData/filteredValue.txt");
    if (outfile1.is_open()) {
        for (int i = 0; i < filteredValue.size(); i++) {
            outfile1 << filteredValue[i] << endl;
        }
        outfile1.close();
        cout << "Data filteredValue written to file." << endl;
    }
    else {
        cout << "Unable to open file." << endl;
    }
}
void shutdownFun()
{
    string subDir="lightAndSpeed";
    ofstream outfile("/home/limy/roscode/paperData/"+subDir+"/greenLightTime.txt");
    if (outfile.is_open()) {
        for (int i = 0; i < greenLightTime.size(); i++) {
            outfile << greenLightTime[i] << endl;
        }
        outfile.close();
        cout << "Data greenLightTime written to file." << endl;
    }
    else {
        cout << "Unable to open file." << endl;
    }

    ofstream outfile1("/home/limy/roscode/paperData/"+subDir+"/redLightTime.txt");
    if (outfile1.is_open()) {
        for (int i = 0; i < redLightTime.size(); i++) {
            outfile1 << redLightTime[i] << endl;
        }
        outfile1.close();
        cout << "Data redLightTime written to file." << endl;
    }
    else {
        cout << "Unable to open file." << endl;
    }

    ofstream outfile2("/home/limy/roscode/paperData/"+subDir+"/timeCount.txt");
    if (outfile2.is_open()) {
        for (int i = 0; i < timeCount.size(); i++) {
            outfile2 << timeCount[i] << endl;
        }
        outfile2.close();
        cout << "Data timeCount written to file." << endl;
    }
    else {
        cout << "Unable to open file." << endl;
    }

    ofstream outfile3("/home/limy/roscode/paperData/"+subDir+"/laserSpeed.txt");
    if (outfile3.is_open()) {
        for (int i = 0; i < laserSpeed.size(); i++) {
            outfile3 << laserSpeed[i] << endl;
        }
        outfile3.close();
        cout << "Data laserSpeed written to file." << endl;
    }
    else {
        cout << "Unable to open file." << endl;
    }

    ofstream outfile4("/home/limy/roscode/paperData/"+subDir+"/lightState.txt");
    if (outfile4.is_open()) {
        for (int i = 0; i < lightState.size(); i++) {
            outfile4 << lightState[i] << endl;
        }
        outfile4.close();
        cout << "Data lightState written to file." << endl;
    }
    else {
        cout << "Unable to open file." << endl;
    }
    ofstream outfile5("/home/limy/roscode/paperData/"+subDir+"/boardSpeed.txt");
    if (outfile5.is_open()) {
        for (int i = 0; i < boardSpeed.size(); i++) {
            outfile5 << boardSpeed[i] << endl;
        }
        outfile5.close();
        cout << "Data boardSpeed written to file." << endl;
    }
    else {
        cout << "Unable to open file." << endl;
    }

}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "lio_sam");
    // Register the shutdown function to be executed when the ROS node is shutting down
    // signal(SIGINT, myShutdownFunction);
    TtlControl ttlControl;
    ROS_INFO("\033[1;32m----> TTLControl Started.\033[0m");
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
    // std::thread exitThread(&myShutdownFunction);
    ros::spin();
    // exitThread.join();
    /*restore the old settings*/
    tcsetattr( STDIN_FILENO, TCSANOW, &oldt);
    shutdownFun();

    return 0;
}