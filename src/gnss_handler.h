#pragma once

#include "global.h"

namespace gnss_parse {
//     class AbstractGnssHandler {
//     public:
// 	// positioning result
// 	double latitude;
// 	double longitude;
// 	double speed;
// 	double course;
// 	double height;
// 	double time;
// 	double date;
// 	int num_of_sv;
// 	
// 	// gnss state
// 	int gps_state;
// 	
// 	// raw measurements
// 	double pesudorange;
// 	double doppler_mea;
// 	
// 	AbstractGnssHandler();
// 	virtual ~AbstractGnssHandler() {};
// 	
// 	/// Initialze the baud rate and output type
// 	virtual void init();
// 	virtual void setBaudRate(const int& baud_rate);
// 	virtual void setOutputType();
// 	void parse(const std::string& _str);
//     };
//     
//     class UbloxMax6Handler : public AbstractGnssHandler {
//     public:
// 	UbloxMax6Handler();
// 	virtual ~UbloxMax6Handler() {};
// 	
// 	virtual void init();
// 	virtual void setBaudRate(const int& baud_rate);
// 	virtual void setOutputType();
//     };
}