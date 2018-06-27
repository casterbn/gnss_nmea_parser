#pragma once

#include <vector>
#include <algorithm>
#include <sstream>

namespace NMEA {

using namespace std;

void vec2double(const vector<unsigned char>& _vec, double& _val) {
    stringstream ss;
    for_each(_vec.begin(), _vec.end(), [&](const unsigned char& str) {
	ss << str;
    });
    ss >> _val;    
}

void vec2int(const vector<unsigned char>& _vec, int& _val) {
    stringstream ss;
    for_each(_vec.begin(), _vec.end(), [&](const unsigned char& str) {
	ss << str;
    });
    ss >> _val;    
}

struct GPGGA {
    double time;
    double latitude;
    double longitude;
    double quality;
    double num_SV;
    double HDOP;
    double altitude;
    double sep;
    double diff_age;
    double diff_station;
    
    GPGGA():
	time(0.0),
	latitude(0.0),
	longitude(0.0),
	quality(0.0),
	num_SV(0.0),
	HDOP(0.0),
	altitude(0.0),
	sep(0.0),
	diff_age(0.0),
	diff_station(0.0)	
    {}
    
    void parseGPGGA(const string& _str) {
	vector<unsigned char> data_buffer;
	int data_counter = 0;
	int idx = 0;
	bool flag_cs = false;
	for (; data_counter <= 14;) {
	    if (_str.at(idx) == ',' || flag_cs) {
		if (!data_buffer.empty()) {
		    switch (data_counter) {
			case 1:
			    vec2double(data_buffer, time);
			    break;
			case 2:
			    double latitude_tmp;
			    vec2double(data_buffer, latitude_tmp);
			    latitude = latitude_tmp / 100;	
			    break;		    
			case 4:
			    double longitude_tmp;
			    vec2double(data_buffer, longitude_tmp);
			    longitude = longitude_tmp / 100;
			    break;
			case 6:
			    vec2double(data_buffer, quality);
			    break;
			case 7:
			    vec2double(data_buffer, num_SV);
			    break;	
			case 8:
			    vec2double(data_buffer, HDOP);
			    break;
			case 9:
			    vec2double(data_buffer, altitude);
			    break;
			case 11:
			    vec2double(data_buffer, sep);
			    break;
			case 13:
			    vec2double(data_buffer, diff_age);
			    break;
			case 14:
			    vec2double(data_buffer, diff_station);
			    break;		    
			default:
			    break;
		    }	
		}
		data_counter++;
		data_buffer.clear();
	    } else if (_str.at(idx) == '*') {
		flag_cs = true;
	    } else {
		data_buffer.push_back(_str.at(idx));
	    }
	    idx++;
	}
    }
};

struct GPGLL {
    double latitude;
    double longitude;
    double time_UTC;
    unsigned char status;
    unsigned char pos_mode;
    
    GPGLL():
	latitude(0.0),
	longitude(0.0),
	time_UTC(0.0),
	status(' '),
	pos_mode(' ')	
    {}
    
    void parseGPGLL(const string& _str) {
	vector<unsigned char> data_buffer;
	int data_counter = 0;
	int idx = 0;
	bool flag_cs = false;
	for (; data_counter <= 7;) {
	    if (_str.at(idx) == ',' || flag_cs) {
		if (!data_buffer.empty()) {
		    switch (data_counter) {
			case 1:
			    double latitude_tmp;
			    vec2double(data_buffer, latitude_tmp);
			    latitude = latitude_tmp / 100;
			    break;		    
			case 3:
			    double longitude_tmp;
			    vec2double(data_buffer, longitude_tmp);
			    longitude = longitude_tmp / 100;
			    break;
			case 5:
			    vec2double(data_buffer, time_UTC);
			    break;
			case 6:
			    status = data_buffer[0];
			    break;	
			case 7:
			    pos_mode = data_buffer[0];
			    break;	    
			default:
			    break;
		    }
		    if (flag_cs == true)
			return;
		}
		data_counter++;
		data_buffer.clear();
	    } else if (_str.at(idx) == '*') {
		flag_cs = true;
	    } else {
		data_buffer.push_back(_str.at(idx));
	    }
	    idx++;
	}	
    }
};

struct GPGSA {
    unsigned char op_mode;
    unsigned char nav_mode;
    double PDOP;
    double HDOP;
    double VDOP;
    int system_id;
    std::vector<double> SVs;
    
    GPGSA():
	op_mode(' '),
	nav_mode(' '),
	PDOP(0.0),
	HDOP(0.0),
	VDOP(0.0),
	system_id(0)	
    {}
    
    void parseGPGSA(const string& _str) {
	vector<unsigned char> data_buffer;
	int data_counter = 0;
	int idx = 0;
	bool flag_cs = false;
	SVs.clear();
	for (; data_counter <= 18;) {
	    if (_str.at(idx) == ',' || flag_cs) {
		if (!data_buffer.empty()) {
		    if (data_counter >= 3 && data_counter < 15) {
			int SV_num;	
			vec2int(data_buffer, SV_num);
			SVs.push_back(SV_num);			
		    } else {			
			switch (data_counter) {
			    case 1:
				op_mode = data_buffer[0];
				break;		    
			    case 2:
				nav_mode = data_buffer[0];
				break;
			    case 15:
				vec2double(data_buffer, PDOP);
				break;	
			    case 16:
				vec2double(data_buffer, HDOP);
				break;	  
			    case 17:
				vec2double(data_buffer, VDOP);
			    case 18:
				vec2int(data_buffer, system_id);
				break;	 			
			    default:
				break;
			}
		    }
		    if (flag_cs == true)
			return;
		}		       
		data_counter++;
		data_buffer.clear();
	    } else if (_str.at(idx) == '*') {
		flag_cs = true;
	    } else {
		data_buffer.push_back(_str.at(idx));
	    }
	    idx++;
	}	
    }    
    
};

struct GPGST {
    double time;
    double rms_range;
    double std_major;
    double std_minor;
    double orient;
    double std_lat;
    double std_long;
    double std_alt;
    
    GPGST():
	time(0.0),
	rms_range(0.0),
	std_major(0.0),
	std_minor(0.0),
	orient(0.0),
	std_lat(0.0),
	std_long(0.0),
	std_alt(0.0)	
    {}
    
    void parseGPGST(const string& _str) {
	vector<unsigned char> data_buffer;
	int data_counter = 0;
	int idx = 0;
	bool flag_cs = false;
	for (; data_counter <= 8;) {
	    if (_str.at(idx) == ',' || flag_cs) {
		if (!data_buffer.empty()) {
		    switch (data_counter) {
			case 1:
			    vec2double(data_buffer, time);
			    break;
			case 2:
			    vec2double(data_buffer, rms_range);
			    break;		    
			case 3:
			    vec2double(data_buffer, std_major);
			    break;
			case 4:
			    vec2double(data_buffer, std_minor);
			    break;
			case 5:
			    vec2double(data_buffer, orient);
			    break;	
			case 6:
			    vec2double(data_buffer, std_lat);
			    break;
			case 7:
			    vec2double(data_buffer, std_long);
			    break;
			case 8:
			    vec2double(data_buffer, std_alt);
			    break;	    
			default:
			    break;
		    }	
		    if (flag_cs == true)
			return;		    
		}
		data_counter++;
		data_buffer.clear();
	    } else if (_str.at(idx) == '*') {
		flag_cs = true;
	    } else {
		data_buffer.push_back(_str.at(idx));
	    }
	    idx++;
	}
    }
};

    
//     string front = _str.substr(0, 6);
//     if (front == "$GPGLL") { // Latitude and longitude, with time of position fix and status
// 	// $GPGLL,4717.11634,N,00833.91297,E,124923.00,A,A*6E
//     } else if (front == "$GPDTM") { // Datum Reference
// 	// $GPDTM,W84,,0.0,N,0.0,E,0.0,W84*6F
//     } else if (front == "$GPGBS") { // GNSS Satellite Fault Detection
// 	// $GPGBS,235458.00,1.4,1.3,3.1,03,,-21.4,3.8,1,0*5B
//     } else if (front == "$GPGGA") { // Global positioning system fix data
// 	// $GPGGA,092725.00,4717.11399,N,00833.91590,E,1,08,1.01,499.6,M,48.0,M,,*5B
//     } else if (front == "$GPGNS") { // GNSS fix data
// 	// $GPGNS,091547.00,5114.50897,N,00012.28663,W,AA,10,0.83,111.1,45.6,,,V*71
//     } else if (front == "$GPGRS") { // GNSS Range Residuals
// 	// $GPGRS,082632.00,1,0.54,0.83,1.00,1.02,-2.12,2.64,-0.71,-1.18,0.25,,,1,0*70
//     } else if (front == "$GPGSA") { // GNSS DOP and Active Satellites
// 	// $GPGSA,A,3,23,29,07,08,09,18,26,28,,,,,1.94,1.18,1.54,1*0D
//     } else if (front == "$GPGST") { // GNSS Pseudo Range Error Statistics 
// 	// $GPGST,082356.00,1.8,,,,1.7,1.3,2.2*7E
//     } else if (front == "$GPGSV") { // GNSS Satellites in View
// 	// $GPGSV,3,1,10,23,38,230,44,29,71,156,47,07,29,116,41,08,09,081,36,0*7F
//     } else if (front == "$GPRMC") { // Recommended Minimum data
// 	// $GPRMC,083559.00,A,4717.11437,N,00833.91522,E,0.004,77.52,091202,,,A,V*57
//     } else if (front == "$GPTXT") { // Text Transmission
// 	// $GPTXT,01,01,02,u-blox ag - www.u-blox.com*50
//     } else if (front == "$GPVLW") { // Dual ground/water distance
// 	// $GPVLW,,N,,N,15.8,N,1.2,N*06
//     } else if (front == "$GPVTG") { // Course over ground and Ground speed
// 	// $GPVTG,77.52,T,,M,0.004,N,0.008,K,A*06
//     } else if (front == "$GPZDA") { // Time and Date
// 	// $GPZDA,082710.00,16,09,2002,00,00*64
//     }
//     
   
}