/*
 * Copyright (C) 2008 Robotics at Maryland
 * Copyright (C) 2008 Michael Levashov
 * All rights reserved.
 *
 * Author: Michael Levashov
 * File:  packages/sonar/src/GetDirEdge.cpp
 */

// STD Includes
#include <cmath>
#include <iostream>

// Project Includes
#include "math/include/MatrixN.h"

#include "sonar/include/Sonar.h"
#include "sonar/include/GetPingChunk.h"
#include "sonar/include/SonarPing.h"
#include "sonar/include/GetDirEdge.h"

#include "drivers/bfin_spartan/include/dataset.h"

namespace ram {
namespace sonar {

adcdata_t myAbs(adcdata_t x)
{
	if (x < 0)
		return -x;
	else
		return x;
}

using namespace ram::math;
using namespace std;

getDirEdge::getDirEdge()
    : chunk(), tdoas(3,1), temp_calc(3,3), hydro_array(3,2), ping_matr(3,2), tdoa_errors(3,1)
{
    for(int j=0; j<NCHANNELS; j++)
        data[j]=new adcdata_t [ENV_CALC_FRAME];

    for(int i=0; i<3; i++)
        for(int j=0; j<2; j++)
            hydro_array[i][j]=hydroPlanarArray[i][j];

    //do inv(H'*H)*H'
    temp_calc=hydro_array.transpose()*hydro_array;
    temp_calc.invert();
    temp_calc=temp_calc*hydro_array.transpose();
}

getDirEdge::~getDirEdge()
{
    for(int j=0; j<NCHANNELS; j++)
        delete[] data[j];
}

/* Resets the parameters that are normally initialized in the constructor
 */
void getDirEdge::zero_values()
{
    for(int i=0; i<NCHANNELS; i++)
    {
        total[i]=0;
        abstotal[i]=0;
        pingpoints[i]=0;
    }
    fit_error=0;
}
    
int getDirEdge::getEdge(sonarPing* ping, dataset *dataSet)
{
    //Zero the appropriate values
    zero_values();

    if((ping_found=chunk.getChunk(data, locations, dataSet))==0)
        return 0;
    else if(ping_found != 1)
        return -1;

    for(int i=0; i<NCHANNELS; i++)
    {
        for(int j=0; j<ENV_CALC_FRAME; j++)
            total[i]+=data[i][j];

        average[i]=total[i]/ENV_CALC_FRAME;

        for(int j=0; j<ENV_CALC_FRAME; j++)
            abstotal[i]+=myAbs(data[i][j]-average[i]);

        absaverage[i]=abstotal[i]/ENV_CALC_FRAME;

        for(int j=0; j<ENV_CALC_FRAME; j++)
            if(data[i][j]-average[i]>absaverage[i])
            {
                pingpoints[i]=locations[i]+j;
                break;
            }
    }
    //cout<<"Positions "<<" "<<pingpoints[0]<<"/"<<dataSet->size<<endl;

    if((pingpoints[0]==0) ||
       (pingpoints[1]==0) ||
       (pingpoints[2]==0) ||
       (pingpoints[3]==0))
        return -1;

    for(int i=0; i<NCHANNELS-1; i++)
        tdoas[i][0]=(SPEED_OF_SOUND* double(pingpoints[i+1]-pingpoints[0]))/SAMPRATE;

    //cout<<"Distance norm: "<<tdoas[1][0]*tdoas[1][0]/(hydro_array[0][1]*hydro_array[0][1])+tdoas[2][0]*tdoas[2][0]/(hydro_array[1][0]*hydro_array[1][0])<<endl;

    ping_matr=temp_calc*tdoas;
    tdoa_errors=hydro_array*ping_matr-tdoas;

    for(int i=0; i<NCHANNELS-1; i++)
        fit_error+=tdoa_errors[i][0]*tdoa_errors[i][0];

    //cout<<"Fit error: "<<fit_error<<endl;
    if(fit_error > PING_FIT_THRESHOLD)
    {
        cout<<"BAD FIT\n";
        return 0;
    }

    //Fill in the sonar ping, as a vector towards the pinger
    for(int i=0; i<2; i++)
        ping->direction[i]=-ping_matr[i][0];
    
    //force it to be positive, since we know that the pinger is below
    
    double temp=ping->direction[0]*ping->direction[0]+ping->direction[1]*ping->direction[1];
    if(temp > 1)
    {
        if(temp>VECTOR_QUAL_THRESHOLD)
        {
            cout<<"BAD VECTOR\n";
            return 0;
        }
        else
            ping->direction[2]=0;
    }
    else
        ping->direction[2]=-std::sqrt(1-temp);

    ping->point_num=pingpoints[0];
    ping->distance=0;

    return 1;
}
} //sonar
} //ram
