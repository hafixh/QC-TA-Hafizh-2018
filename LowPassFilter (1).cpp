// LowPassFilter.cpp: LowPassFilter クラスのインプリメンテーション
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "SimExp.h"
#include "LowPassFilter.h"

#ifdef _DEBUG
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
#define new DEBUG_NEW
#endif


LowPassFilter::LowPassFilter()
{
	xp1 = 0;
	xp2 = 0;
	xp3 = 0; 
	yp1 = 0;
	yp2 = 0;
	vp1 = 0;
	vp2 = 0;

	omega = 15*2*PI; //値適当 15*2*PI rad/sec = 15 Hz
	T = SampTime;	 //実験装置の値流用

	temp1 = pow(omega * T,2) / pow(omega * T + 2,2) ;
	temp2 = 2 * (omega * T - 2) / (omega * T + 2) ;
	temp3 = pow((omega * T - 2) / (omega * T + 2),2) ;

	temp2v = 2 * (0.5*omega * T - 2) / (0.5*omega * T + 2) ;
	temp3v = pow((0.5*omega * T - 2) / (0.5*omega * T + 2),2) ;
	temp4v = 2 * pow(0.5*omega ,2) * T / pow(0.5*omega * T + 2,2) ;
}

LowPassFilter::~LowPassFilter()
{
}

LowPassFilter::Initialize(double x01)
{
	xp1 = x01;
	xp2 = x01;
	xp3 = x01;
	yp1 = x01;
	yp2 = x01;
	vp1 = 0.0;
	vp2 = 0.0;
}

LowPassFilter::SetOmega(double o)
{
	omega = o*2*PI;

	temp1 = pow(omega * T,2) / pow(omega * T + 2,2) ;
	temp2 = 2 * (omega * T - 2) / (omega * T + 2) ;
	temp3 = pow((omega * T - 2) / (omega * T + 2),2) ;

	temp2v = 2 * (1*omega * T - 2) / (1*omega * T + 2) ;
	temp3v = pow((1*omega * T - 2) / (1*omega * T + 2),2) ;
	temp4v = 2 * pow(1*omega ,2) * T / pow(1*omega * T + 2,2) ;
}

double LowPassFilter::calc_y(double x){

	//LPFの計算
	double y = temp1*(x + 2*xp1 + xp2) - temp2*yp1 - temp3*yp2 ;

	return y;
}

double LowPassFilter::calc_v(double x){

	double v = temp4v*(x + xp1 - xp2 -xp3) - temp2v*vp1 - temp3v*vp2 ;

	return v;
}

double LowPassFilter::update(double x, double y, double v){
	//時点更新
	xp3 = xp2;
	xp2 = xp1;
	xp1 = x;
	yp2 = yp1;
	yp1 = y;
	vp2 = vp1;
	vp1 = v;

	return 0;
}