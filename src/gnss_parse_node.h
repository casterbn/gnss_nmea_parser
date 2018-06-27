#pragma once

#include <ros/ros.h>
#include <iostream>
#include <cstring>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <gnss_parse/GPGGA.h>
#include "global.h"
#include "gnss.h"
#include "ubx_protocal.h"

using namespace std;
    
enum MsgType {  
    Msg_GGA  = 0,  
    Msg_GLL,  
    Msg_GSA, 
    Msg_GSV, 
    Msg_RMC, 
    Msg_VTG, 
    Msg_ZDA, 
    Msg_GST,
    Msg_RAW
};  



class AbstractGnssHandler {
public: 
    string buffer_str;
    
    AbstractGnssHandler();
    virtual ~AbstractGnssHandler() {}
    /// Initialze the baud rate and output type
    virtual void initialGNSS() = 0;
    virtual void setBaudRate(const int& baud_rate) = 0;
    virtual void setMeaRate(const int& _freq) = 0;
    virtual void setOutputType(const MsgType& _msg_type, const int& _rate) = 0;
    virtual void parse(const std::string& _str) = 0;
    
    void stringCallBack(const std_msgs::String::ConstPtr& msg);
    void pubCmd(const std::string& _str); 
    
protected:
    ros::NodeHandle n;
    ros::Subscriber sub_ser;
    ros::Publisher pub_ser;
};

class UbloxMax6Handler : public AbstractGnssHandler {
public:
    bool has_GPGGA;
    bool has_GPGLL;
    bool has_GPGSA;
    bool has_GPGST;
    // NMEA messages
    NMEA::GPGGA msg_GPGGA;
    NMEA::GPGLL msg_GPGLL;
    NMEA::GPGSA msg_GPGSA;
    NMEA::GPGST msg_GPGST;
    // UBX messages
    UBX::RAWX_6 msg_RAWX;
    
    UbloxMax6Handler();
    virtual ~UbloxMax6Handler() {}
    virtual void initialGNSS();
    virtual void setBaudRate(const int& baud_rate);
    virtual void setMeaRate(const int& _freq);
    virtual void setOutputType(const MsgType& _msg_type, const int& _rate);    // message state
    virtual void parse(const std::string& _str);
    
    void pubGPGGA();
    
protected:
    ros::Publisher pub_GPGGA, pub_GPGLL, pub_GPGSA, pub_GPGST;
};

AbstractGnssHandler::AbstractGnssHandler()
{
    sub_ser = n.subscribe("read", 1000, &AbstractGnssHandler::stringCallBack, this);
    pub_ser = n.advertise<std_msgs::String>("write", 1000);
}

void AbstractGnssHandler::pubCmd(const string& _str)
{
    std::stringstream ss;
    ss << _str;
    std_msgs::String msg;
    msg.data = ss.str();
    pub_ser.publish(msg);
}

void UbloxMax6Handler::pubGPGGA()
{
    gnss_parse::GPGGA msg;
    msg.time = msg_GPGGA.time;
    msg.latitude = msg_GPGGA.latitude;
    msg.longitude = msg_GPGGA.longitude;
    msg.quality = msg_GPGGA.quality;
    msg.num_SV = msg_GPGGA.num_SV;
    msg.HDOP = msg_GPGGA.HDOP;
    msg.altitude = msg_GPGGA.altitude;
    msg.sep = msg_GPGGA.sep;
    msg.diff_age = msg_GPGGA.diff_age;
    msg.diff_station = msg_GPGGA.diff_station;
    pub_GPGGA.publish(msg);
}


UbloxMax6Handler::UbloxMax6Handler():
    AbstractGnssHandler()
{
    pub_GPGGA = n.advertise<gnss_parse::GPGGA>("gnss/GPGPA", 1000);
    pub_GPGLL = n.advertise<std_msgs::String>("gnss/GPGLL", 1000);
    pub_GPGSA = n.advertise<std_msgs::String>("gnss/GPGSA", 1000);
    pub_GPGST = n.advertise<std_msgs::String>("gnss/GPGST", 1000);
//     initialGNSS();
}

void UbloxMax6Handler::initialGNSS()
{
//     setBaudRate(115200);
//     usleep(10000);
//     setMeaRate(5);
//     usleep(10000);
    setOutputType(Msg_GLL, 0);
    usleep(10000);
    setOutputType(Msg_GSA, 0);
    usleep(1000);
//     setOutputType(Msg_GSV, 0);
//     usleep(10000);
//     setOutputType(Msg_VTG, 0);
//     usleep(10000);
//     setOutputType(Msg_ZDA, 0);
//     usleep(10000);
//     setOutputType(Msg_RAW, 0);
//     usleep(10000);
    setOutputType(Msg_GLL, 1);
    usleep(10000);
}

void UbloxMax6Handler::setBaudRate(const int& baud_rate)
{
    int index = 0;
    unsigned char ublox_cmd[30]={0xB5,0x62,0x06,0x01,0x03,0x00,0xf0,0x00};
    
    ublox_cmd[index++]=0x00; //  115200bps 
    ublox_cmd[index++]=0x14; // Length 20
    ublox_cmd[index++]=0x00; // 串口1
    ublox_cmd[index++]=0x01;
    ublox_cmd[index++]=0x00; // reserved0
    ublox_cmd[index++]=0x00; // txReady 2位
    ublox_cmd[index++]=0x00;
    ublox_cmd[index++]=0xD0; // mode 8位,1个停止位,无校验位
    ublox_cmd[index++]=0x08;
    ublox_cmd[index++]=0x00;
    ublox_cmd[index++]=0x00;
    ublox_cmd[index++]=0x00; // baudRate 1C200 = 115200, 低位到高位
    ublox_cmd[index++]=0xC2;
    ublox_cmd[index++]=0x01;
    ublox_cmd[index++]=0x00;
    ublox_cmd[index++]=0x07; // 0+1+2 
    ublox_cmd[index++]=0x00;
    ublox_cmd[index++]=0x07; // 0+1+2 
    ublox_cmd[index++]=0x00;
    ublox_cmd[index++]=0x00; // 四位都留0
    ublox_cmd[index++]=0x00;
    ublox_cmd[index++]=0x00;
    ublox_cmd[index++]=0x00;
    ublox_cmd[index++]=0xC4; // CK_A
    ublox_cmd[index++]=0x96; // CK_B
    string str = (char*)ublox_cmd;
    pubCmd(str);
}

void UbloxMax6Handler::setMeaRate(const int& _freq)
{
    int index = 0;
    unsigned char ublox_cmd[30]={0xB5,0x62,0x06,0x01,0x03,0x00,0xf0,0x00};
    
    index = 3;
    ublox_cmd[index++]=0x08; //enable 5Hz  5HZ
    ublox_cmd[index++]=0x06; // 6位
    ublox_cmd[index++]=0x00;
    ublox_cmd[index++]=0xC8; // measRate 2位, 低位,高位, 200ms
    ublox_cmd[index++]=0x00;
    ublox_cmd[index++]=0x01; // navRate 每测量一次都输出一个定位结果
    ublox_cmd[index++]=0x00;
    ublox_cmd[index++]=0x01; // timeRef, GPS time作为参考
    ublox_cmd[index++]=0x00;
    ublox_cmd[index++]=0xDE;
    ublox_cmd[index++]=0x6A;
    string str = (char*)ublox_cmd;
    pubCmd(str);
}

void UbloxMax6Handler::setOutputType(const MsgType& _msg_type, const int& _rate)
{
    assert(_rate >= 0 || _rate <= 20);
    
    int index = 7;  
    unsigned char ublox_cmd[30]={0xB5,0x62,0x06,0x01,0x03,0x00,0xf0,0x00}; //0x03, 0x00 为length 后三位为 msgClass, msgID, rate
    
    string str;
    switch (_msg_type) {
	case Msg_GLL:
	    ublox_cmd[index++]=0x01;
	    ublox_cmd[index++]=(unsigned char)_rate;
	    ublox_cmd[index++]=0xFB;
	    ublox_cmd[index++]=0x11;
	    str = (char*)ublox_cmd;
	    pubCmd(str);
	    break;
	case Msg_GSA:
	    ublox_cmd[index++]=0x02;
	    ublox_cmd[index++]=(unsigned char)_rate;
	    ublox_cmd[index++]=0xFC;
	    ublox_cmd[index++]=0x13;
	    str = (char*)ublox_cmd;
	    pubCmd(str);
	    break;
	case Msg_GSV:
	    ublox_cmd[index++]=0x03; 
	    ublox_cmd[index++]=(unsigned char)_rate;
	    ublox_cmd[index++]=0xFD;
	    ublox_cmd[index++]=0x15;
	    str = (char*)ublox_cmd;
	    pubCmd(str);
	    break;
	case Msg_VTG:
	    ublox_cmd[index++]=0x05; 
	    ublox_cmd[index++]=(unsigned char)_rate;
	    ublox_cmd[index++]=0xFF;
	    ublox_cmd[index++]=0x19;
	    str = (char*)ublox_cmd;
	    pubCmd(str);
	    break;
	case Msg_ZDA:
	    ublox_cmd[index++]=0x08; 
	    ublox_cmd[index++]=(unsigned char)_rate;
	    ublox_cmd[index++]=0x02;
	    ublox_cmd[index++]=0x1F;
	    str = (char*)ublox_cmd;
	    pubCmd(str);
	    break;
	case Msg_RAW:
	    ublox_cmd[index++]=0x02; // class
	    ublox_cmd[index++]=0x15; // ID
	    unsigned char ck_A, ck_B;
	    UBX::checkSum(&ublox_cmd[2], 7, ck_A, ck_B);
	    ublox_cmd[index++]=ck_A;
	    ublox_cmd[index++]=ck_B;
	    str = (char*)ublox_cmd;
	    pubCmd(str);
	default:
	    ROS_INFO_STREAM("setOutputType: undefined message type.");
	    break;
    }
}
