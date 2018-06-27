#pragma once

#include <vector>
#include <algorithm>
#include <sstream>
#include <assert.h>

namespace UBX {
    
using namespace std;

char HEAD_UBX_RAWX_CHAR[] = {(char)0xB5, (char)0x62, (char)0x02, (char)0x10};
string HEAD_UBX_RAWX(HEAD_UBX_RAWX_CHAR);

void checkSum(unsigned char *buf,unsigned short len,unsigned char& cka,unsigned char& ckb)  {  
    unsigned short i;  
    cka=0;ckb=0;  
    for(i=0;i<len;i++) {  
	cka=cka+buf[i];  
	ckb=ckb+cka;  
    }  
} 

unsigned short hexStr2ushort(const string& _str) { 
    assert(_str.size() == 2);
    unsigned short val;
    char* ch = (char*)(&val);
    for (int i = 0; i < 2; ++i) {
	ch[i] = _str.at(i) & 0xff;
    }
    return val;
}

short hexStr2short(const string& _str) { 
    assert(_str.size() == 2);
    short val;
    char* ch = (char*)(&val);
    for (int i = 0; i < 2; ++i) {
	ch[i] = _str.at(i) & 0xff;
    }
    return val;
}

unsigned long hexStr2ulong(const string& _str) { 
    assert(_str.size() == 4);
    unsigned long val;
    char* ch = (char*)(&val);
    for (int i = 0; i < 4; ++i) {
	ch[i] = _str.at(i) & 0xff;
    }
    return val;
}

long hexStr2long(const string& _str) { 
    assert(_str.size() == 4);
    long val;
    char* ch = (char*)(&val);
    for (int i = 0; i < 4; ++i) {
	ch[i] = _str.at(i) & 0xff;
    }
    return val;
}

float hexStr2float(const string& _str) {
    assert(_str.size() == 4);
    float val;
    char* ch = (char*)(&val);
    for (int i = 0; i < 4; ++i) {
	ch[i] = _str.at(i) & 0xff;
    }
    return val;
}
	
double hexStr2double(const string& _str) {
    assert(_str.size() == 8);
    double val;
    char* ch = (char*)(&val);
    for (int i = 0; i < 8; ++i) {
	ch[i] = _str.at(i) & 0xff;
    }
    return val;
}

bool check(const string& _str) {
    // Vlidate CK_A CK_B
    return true;
}


struct SV {
    long pseudorange;
    long carrier_phase;
    int doppler;
    unsigned char gnss_id;
    unsigned char sv_id;
    unsigned char freq_id;
    unsigned short lock_time;
    unsigned char cno;
    unsigned char stdev_pr; // *0.01 * 2^n
    unsigned char stdev_cp; // *0.04
    unsigned char stdev_do; // *0.002 * 2^n
    unsigned char track_status;
};

struct RAWX {
    long rcv_tow; // 0-7
    unsigned short week; // 8-9
    signed char leap_s; // 10
    unsigned char num_meas; // 11
    unsigned char rec_status; // 12-15
    vector<SV> svs;
    
    RAWX():
	rcv_tow(0.0),
	week(0.0),
	leap_s(0),
	num_meas(0),
	rec_status(0)
    {}  

    void parseRAWX(const string& _str);// length byte is not included
    void parseSV(const string& _str, SV& _sv); 
};

struct SV_6 {
    double pseudorange;
    double carrier_phase;
    float doppler;
    int sv_id;
    signed char mes_qulity_idc;
    signed char cno;
    int lli;
    int update_state;
};

struct RAWX_6 {
    long rcv_tow; // 0-3
    short week; // 4-5
    int num_meas; // 6
    vector<SV_6*> svs;
    
    RAWX_6():
	rcv_tow(0.0),
	week(0.0),
	num_meas(0)
    {}  

    void parseRAWX(const string& _str);// length byte is not included
    void parseSV(const string& _str, SV_6* _sv); 
    SV_6* findSV(const int& _id);
    void resetUpdateState();
    void clearInvisibleSVs();
};

void RAWX_6::resetUpdateState()
{
    for_each(svs.begin(), svs.end(), [&](SV_6* sv) {
	sv->update_state = 0;
    });
}

void RAWX_6::clearInvisibleSVs()
{
    for(auto it=svs.begin(), ite=svs.end(); it!=ite; ++it) {
	if((*it)->update_state == 0) {
	    delete *it;
	    svs.erase(it);
	}
    }   
}

SV_6* RAWX_6::findSV(const int& _id)
{
    for(auto it=svs.begin(), ite=svs.end(); it!=ite; ++it)
	if((*it)->sv_id == _id)
	return *it;
    return NULL;    // no keyframe found
}

void RAWX_6::parseRAWX(const string& _str)
{
    resetUpdateState();
    rcv_tow = hexStr2long(_str.substr(0, 4));
    week = hexStr2short(_str.substr(4, 2));
    num_meas = _str.at(6);
    for (int i = 0; i < num_meas; ++i) {
	int sv_id = (int)_str.at(8 + i*24 + 20);
	SV_6* sv = findSV(sv_id);
	if (sv == NULL) {
	    svs.push_back(new SV_6());
	    sv = svs.back();
	}
	parseSV(_str.substr(8 + i*24, 24), sv);
    }
    clearInvisibleSVs();
}

void RAWX_6::parseSV(const string& _str, SV_6* _sv)
{
    assert(_str.size() == 24);
    _sv->carrier_phase = hexStr2double(_str.substr(0, 8));
    _sv->pseudorange = hexStr2double(_str.substr(8, 8));
    _sv->doppler = hexStr2float(_str.substr(16, 4));
    _sv->sv_id = _str.at(20);
    _sv->mes_qulity_idc = _str.at(21);    
    _sv->cno = _str.at(22);
    _sv->lli = _str.at(23); 
    _sv->update_state = 1;
}


void RAWX::parseRAWX(const string& _str)
{
//     rcv_tow = hexStr2short(_str.substr(0, 8));
//     week = hexStr2short(_str.substr(8, 2));
//     leap_s = _str.at(10);
//     num_meas = _str.at(11);
//     rec_status = _str.at(12);
//     for (size_t i = 0; i < num_meas; ++i) {
// 	SV sv;
// 	parseSV(_str.substr(16 + i*32, 32), sv);
// 	svs.push_back(sv);
//     }
}

void RAWX::parseSV(const string& _str, SV& _sv)
{
//     assert(_str.size() == 32);
//     hexStr2long64(_str.substr(0, 8), _sv.pseudorange);
//     hexStr2long64(_str.substr(8, 8), _sv.carrier_phase);
//     hexStr2int32(_str.substr(16, 4), _sv.doppler);
//     _sv.gnss_id = _str.at(20);
//     _sv.sv_id = _str.at(21);
//     _sv.freq_id = _str.at(23);
//     hexStr2short16(_str.substr(24, 2), _sv.lock_time);
//     _sv.cno = _str.at(26);
//     _sv.stdev_pr = _str.at(27);
//     _sv.stdev_cp = _str.at(28);
//     _sv.stdev_do = _str.at(29);
//     _sv.track_status = _str.at(30);  
}






}