#include "basicfusion.h"
#include "statevariable.h"
#include "harlielog.h"
#include "harliealloc.h"
#include <stdio.h>
#include <math.h>
#include "sensorfusion.h"
#include "packets.h"
#include "differentialSteering.h"

Sensor previous;

/**
 * Initializes the struct for the Sensor fusion
 */
SensorFusion * BasicFusion_Init()
{
    /*
     *  TODO Fix allocation locations for varaiables
     */
    SensorFusion * fusion = hMalloc(sizeof(SensorFusion));
    SensorFusion_Init(fusion);
    fusion->updateFilter = &BasicFusion_UpdateFilter;
    return fusion;
}

/**
 * Updates the sensor values based on the sensor count... TODO: need to figure out what sensor count corresponds to
 * which sensor
 */
void BasicFusion_UpdateFilter(SensorFusion * fusion,WheelSpeedCommand speeds)
{
    int i;
    Sensor totalVariance;
    Sensor location = fusion->currentState;
    LOG.VERBOSE("Starting sensor update");
    for(i = 0; i < fusion->sensorCount; i++)
    {
        fusion->sensorList[i]->update(fusion->sensorList[i],location);
        totalVariance.x += fusion->sensorList[i]->standardDeviation->x;
        totalVariance.y += fusion->sensorList[i]->standardDeviation->y;
        totalVariance.theta +=fusion->sensorList[i]->standardDeviation->theta;
    }
    if(fusion->sensorCount>1)
    {
/*
        for(i = 0; i < fusion->sensorCount; i++)
        {
            if(totalVariance.x != fusion->sensorList[i]->standardDeviation->x)
            {
                location.x += fusion->sensorList[i]->updatedState->x * ((totalVariance.x - fusion->sensorList[i]->standardDeviation->x)/totalVariance.x);
            }
            else
            {
                location.x = fusion->sensorList[i]->updatedState->x;
            }
            if(totalVariance.y != fusion->sensorList[i]->standardDeviation->y)
            {
                location.y += fusion->sensorList[i]->updatedState->y * ((totalVariance.y - fusion->sensorList[i]->standardDeviation->y)/totalVariance.y);
            }
            else
            {
                location.y = fusion->sensorList[i]->updatedState->y;
            }
            if(totalVariance.theta != fusion->sensorList[i]->standardDeviation->theta)
            {
                location.theta += fusion->sensorList[i]->updatedState->theta * ((totalVariance.theta - fusion->sensorList[i]->standardDeviation->theta)/totalVariance.theta);
            }
            else
            {
                location.theta = fusion->sensorList[i]->updatedState->theta;
            }
        }
*/
/*
        previous.x = location.x;

        previous.y = location.y;

        previous.theta = location.theta;

        location.x = fusion->sensorList[2]->updatedState->x;
        location.y = fusion->sensorList[2]->updatedState->y;
        if(fusion->sensorList[2]->standardDeviation->x > 20)
        {
            location.x = fusion->sensorList[0]->updatedState->x;
            location.y = fusion->sensorList[0]->updatedState->y;
        }
        else
        {
            location.x = fusion->sensorList[0]->updatedState->x*.8 + location.x*.2;
            location.y = fusion->sensorList[0]->updatedState->y*.8 + location.y*.2;
        }

        location.theta = limitTheta(fusion->sensorList[2]->updatedState->theta);

         if(fusion->sensorList[2]->standardDeviation->x > 1)
        {
            location.theta = fusion->sensorList[1]->updatedState->theta;
        }
         else
         {
            location.theta = fusion->sensorList[1]->updatedState->theta*.99 + location.theta*.01;
         }
*/
        float ox = fusion->sensorList[0]->state->x;
        float oy = fusion->sensorList[0]->state->y;
        float ot = fusion->sensorList[0]->state->theta;
        float yt = fusion->sensorList[1]->state->theta;
        float oxd = fusion->sensorList[0]->standardDeviation->x;
        float oyd = fusion->sensorList[0]->standardDeviation->y;
        float otd = fusion->sensorList[0]->standardDeviation->theta;
        float ytd = fusion->sensorList[0]->standardDeviation->theta;

        LOG.DATA("OX = %f",ox);
        LOG.DATA("OY = %f",oy);
        LOG.DATA("OT = %f",ot);
        LOG.DATA("YT = %f",yt);
        LOG.DATA("OXD = %f",oxd);
        LOG.DATA("OYD = %f",oyd);
        LOG.DATA("OTD = %f",otd);
        LOG.DATA("YTD = %f",ytd);
        LOG.DATA("LWS = %f",speeds.leftWheelSpeed);
        LOG.DATA("RWS = %f",speeds.rightWheelSpeed);
        location.x=0;
        location.y=0;
        location.theta = 0;

       /* printf("%f location.theat\n",location.theta);*/
        /*This fixes theta so that is within 2pi-0 this may or may not be needed*/
    }
    else
    {
        location.x = fusion->sensorList[0]->state->x;
        location.y = fusion->sensorList[0]->state->y;
        location.theta = limitTheta(fusion->sensorList[0]->state->theta);
    }
    fusion->currentState = location;
    LOG.VERBOSE("Location after update\n\tx=%f\n\ty=%f\n\ttheta=%f",location.x,location.y,location.theta);
  /*  printf("%f\n",location.theta);*/
}
