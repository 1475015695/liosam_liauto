#include "utility.h"
#include <termios.h>            //termios, TCSANOW, ECHO, ICANON
#include <unistd.h>     //STDIN_FILENO
#include <thread>
#include <std_message.h>

using namespace std;
vector<double> lightState,redLightTime,greenLightTime,timeCount,speed;
char key;
bool inCheck=false;
class Test:public ParamServer
{
public:
    ros::NodeHandle nh;
    ros::Subscriber subLightAndSpeed;

    Test()
    {
        subLightAndSpeed=nh.subscribe<std_msgs::Float64MultiArray>("/lightAndSpeed",1,&Test::lightAndSpeedHandle,this);
        thread keyThread(&Test::readKeyThread,this);
        keyThread.detach();
    }

    void lightAndSpeedHandle(const std_msgs::Float64MultiArray::ConstPtr& msg)
    {
        if(msg->data[0]>0)
        {
            printf("Green Light, time remain:%3.1f s, laserSpeed:%4.1f km/h, bordSpeed:%4.2f km/h\r\n",(msg->data[2]*10-msg->data[3])/10,msg->data[4]/10,0);  
        }
        else
        {
            printf("Red   Light, time remain:%3.1f s, laserSpeed:%4.1f km/h, bordSpeed:%4.2f km/h\r\n",(msg->data[1]*10-msg->data[3])/10,msg->data[4]/10,0);
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
    }
    void readKeyThread()
    {
        while((key=getchar())!='q')
        {
            printf("%s\r\n","heck");
            if(key=='C'||key=='c')
            {
                inCheck=true;
            }
            else
            {
                inCheck=false;
            }
            
            key='.';
        }
    }

};
void shutdownFun()
{
    ofstream outfile("/home/limy/roscode/paperData/greenLightTime.txt");
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

    ofstream outfile1("/home/limy/roscode/paperData/redLightTime.txt");
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

    ofstream outfile2("/home/limy/roscode/paperData/timeCount.txt");
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

    ofstream outfile3("/home/limy/roscode/paperData/speed.txt");
    if (outfile3.is_open()) {
        for (int i = 0; i < speed.size(); i++) {
            outfile3 << speed[i] << endl;
        }
        outfile3.close();
        cout << "Data speed written to file." << endl;
    }
    else {
        cout << "Unable to open file." << endl;
    }

    ofstream outfile4("/home/limy/roscode/paperData/lightState.txt");
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

}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "lio_sam");

    Test test;
    ROS_INFO("\033[1;32m----> Test Node Started.\033[0m");
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
    shutdownFun();
    tcsetattr( STDIN_FILENO, TCSANOW, &oldt);
    return 0;
}