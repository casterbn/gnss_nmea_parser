#include <ros/ros.h>
#include <std_msgs/String.h>
#include "global.h"
#include "gnss_parse_node.h"



void AbstractGnssHandler::stringCallBack(const std_msgs::String::ConstPtr& msg)
{
    parse(msg->data);
}

void UbloxMax6Handler::parse(const string& _str)
{
    ROS_INFO_STREAM("Parsing...----------------------------------------");
    
    // UBX parsing
    size_t res = _str.find(UBX::HEAD_UBX_RAWX);	
    if (string::npos != res) {
// 	ROS_INFO_STREAM("Receive: UBX-RAWX" << UBX::HEAD_UBX_RAWX << "length: " << _str.substr(res).size());	
	size_t length =  (size_t)UBX::hexStr2ushort(_str.substr(res+4, 2));
	string str_ubx_rawx = _str.substr(res+6, length);
	
	if (!UBX::check(_str))
	    return;
	
	msg_RAWX.parseRAWX(str_ubx_rawx);
	ROS_INFO_STREAM("number of measurements: " << msg_RAWX.num_meas << "\n");
	for (int i = 0; i < msg_RAWX.num_meas; ++i) {
	    ROS_INFO_STREAM("sv_id: " << msg_RAWX.svs.at(i)->sv_id);	    
	    ROS_INFO_STREAM("pseudorange: " << msg_RAWX.svs.at(i)->pseudorange);
	    ROS_INFO_STREAM("carrier_phase: " << msg_RAWX.svs.at(i)->carrier_phase);
	    ROS_INFO_STREAM("doppler: " << msg_RAWX.svs.at(i)->doppler);
	    ROS_INFO_STREAM("mes_qulity_idc: " << (int)msg_RAWX.svs.at(i)->mes_qulity_idc << "\n");
	}	
    }
	
    size_t index_NMEA = 0, res_idx_NMEA = 0;
    while (1) {
	// NMEA parsing
	res_idx_NMEA = _str.find("$G", index_NMEA);
	if (string::npos != res_idx_NMEA) {	
	    string head = _str.substr(res_idx_NMEA, 6);
	    if (head == "$GPGGA") {
		msg_GPGGA.parseGPGGA(_str.substr(res_idx_NMEA));  
		ROS_INFO_STREAM("Receive: $GPGGA " << "Lat: " << msg_GPGGA.latitude << " Long: " << msg_GPGGA.longitude);
		has_GPGGA = true;
	    } else if (head == "$GPGSA") {
		msg_GPGSA.parseGPGSA(_str.substr(res_idx_NMEA));
		ROS_INFO_STREAM("Receive: $GPGSA " << "SV number: " << msg_GPGSA.SVs.size() << " " << msg_GPGSA.PDOP << " " << msg_GPGSA.HDOP << " " << msg_GPGSA.VDOP);
		has_GPGSA = true;
	    } else if (head == "$GPGLL") {
		msg_GPGLL.parseGPGLL(_str.substr(res_idx_NMEA));
		ROS_INFO_STREAM("Receive: $GPGLL " << "Lat: " << msg_GPGGA.latitude << " Long: " << msg_GPGGA.longitude);
		has_GPGLL = true;
	    } else if (head == "$GPGST") {
		msg_GPGST.parseGPGST(_str.substr(res_idx_NMEA));
		ROS_INFO_STREAM("Receive: $GPGST " << " Orient: " << msg_GPGST.orient << " " << msg_GPGST.std_lat << " " << msg_GPGST.std_long << " " << msg_GPGST.std_alt);
		has_GPGST = true;
	    }
	    index_NMEA = res_idx_NMEA + 1;	    
	} else {
	    break;
	}    	
    } 
}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "gnss_parse_node");
  
    UbloxMax6Handler ubloxM6;
    //Main Loop
    //----------
    ROS_INFO("GNSS initialization complete...Looping");
    ros::Rate loop_rate(1);
    while (ros::ok()) {
        ros::spinOnce();   
	if (ubloxM6.has_GPGGA) {
	    ubloxM6.pubGPGGA();
	    ubloxM6.has_GPGGA = false;
	} 
	
// 	else if (ubloxM6.has_GPGSA) {
// 	    ubloxM6.pub_GPGSA.publish();
// 	    ubloxM6.has_GPGSA = false;
// 	} else if (ubloxM6.has_GPGLL) {
// 	    ubloxM6.pub_GPGLL.publish();
// 	    ubloxM6.has_GPGLL = false;
// 	} else if (ubloxM6.has_GPGST) {
// 	    ubloxM6.pub_GPGST.publish();
// 	    ubloxM6.has_GPGST = false;
// 	}
        loop_rate.sleep();
    }
    return(0);
}


	
	
	
	
	
	
	