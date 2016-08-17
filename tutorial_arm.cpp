// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <stdio.h>
#include <yarp/os/Network.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>

#include <string>

using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;

int main(int argc, char *argv[])
{
    Network yarp;

    Property params;
    params.fromCommand(argc, argv);


    if (!params.check("robot"))
    {
        fprintf(stderr, "Please specify the name of the robot\n");
        fprintf(stderr, "--robot name (e.g. icub)\n");
        return 1;
    }
    std::string robotName=params.find("robot").asString().c_str();
    std::string remotePorts="/";
    std::string remotePorts2="/"; // for the left_arm
    remotePorts+=robotName;
    remotePorts+="/right_arm";


    remotePorts2+=robotName;    // for the left arm
    remotePorts2+="/left_arm";

    std::string localPorts="/test/client";
    std::string localPorts2="/test/client2";   // for the left arm

    Property options;
    options.put("device", "remote_controlboard");
    options.put("local", localPorts.c_str());   //local port names
    options.put("remote", remotePorts.c_str());         //where we connect to


    // <-- For the left_arm -->
    Property options2;
    options2.put("device", "remote_controlboard");
    options2.put("local", localPorts2.c_str());
    options2.put("remote", remotePorts2.c_str());

    // create a device
    PolyDriver robotDevice(options);
    if (!robotDevice.isValid()) {
        printf("Device not available.  Here are the known devices:\n");
        printf("%s", Drivers::factory().toString().c_str());
        return 0;
    }


    // <-- For the left_arm -->

    PolyDriver robotDevice2(options2);
    if (!robotDevice2.isValid()) {
        printf("robotDevice2 not available.  Here are the known devices:\n");
        printf("%s", Drivers::factory().toString().c_str());
        return 0;
    }

    IPositionControl *pos;
    IEncoders *encs;

 
    // <-- For the left_arm -->
    IPositionControl *pos2;
    IEncoders *encs2;


    bool ok;
    ok = robotDevice.view(pos);
    ok = ok && robotDevice.view(encs);

    if (!ok) {
        printf("Problems acquiring interfaces\n");
        return 0;
    }

    // <-- For the left_arm -->
    bool ok2;
    ok2 = robotDevice2.view(pos2);
    ok2 = ok2 && robotDevice2.view(encs2);


    if (!ok2) {
        printf("Problems acquiring interfaces\n");
        return 0;
    }


    int nj=0;
    pos->getAxes(&nj);
    Vector encoders;
    Vector command;
    Vector tmp;
    encoders.resize(nj);
    tmp.resize(nj);
    command.resize(nj);

    int i;
    for (i = 0; i < nj; i++) {
         tmp[i] = 50.0;
    }
    pos->setRefAccelerations(tmp.data());

    for (i = 0; i < nj; i++) {
        tmp[i] = 10.0;
        pos->setRefSpeed(i, tmp[i]);
    }


    // <-- For the left_arm -->
    int nj2=0;
    pos2->getAxes(&nj2);
    Vector encoders2;
    Vector command2;
    Vector tmp2;
    encoders2.resize(nj2);
    tmp2.resize(nj2);
    command2.resize(nj2);

    int i2;
    for (i2 = 0; i2 < nj2; i2++) {
         tmp2[i2] = 50.0;
    }
    pos2->setRefAccelerations(tmp2.data());

    for (i2 = 0; i2 < nj2; i2++) {
        tmp2[i2] = 10.0;
        pos2->setRefSpeed(i2, tmp2[i2]);
    }





    //pos->setRefSpeeds(tmp.data()))

    //fisrst read all encoders
    //
    printf("waiting for encoders");
    while(!encs->getEncoders(encoders.data()))
    {
        Time::delay(0.1);
        printf(".");
    }
    printf("\n;");

    command=encoders;
    //now set the shoulder to some value
    command[0]=-30;
    command[1]=50;
    command[2]=30;
    command[3]=-50;
    pos->positionMove(command.data());
    printf("%.1lf %.1lf %.1lf %.1lf\n", encoders[0], encoders[1], encoders[2], encoders[3]);

    bool done=false;

    while(!done)
    {
        pos->checkMotionDone(&done);
        Time::delay(0.1);
    }




    // <-- For the left_arm -->

    printf("waiting for encoders22222");
    while(!encs2->getEncoders(encoders2.data()))
    {
        Time::delay(0.1);
        printf(".");
    }
    printf("\n;");

    command2=encoders2;
    //now set the shoulder to some value
    command2[0]=-30;
    command2[1]=50;
    command2[2]=-30;
    command2[3]=100;
    pos2->positionMove(command2.data());
    printf("%.1lf %.1lf %.1lf %.1lf\n", encoders2[0], encoders2[1], encoders2[2], encoders2[3]);

    bool done2=false;

    while(!done2)
    {
        pos2->checkMotionDone(&done2);
        Time::delay(0.1);
    }





    int times=0;
    while(true)
    {
        times++;
        if (times%2)
        {
             command[0]=-10;  //max:0 min:-120
             command[1]=90;   //max:80 min:40
             command[2]=0;   //max:90 min:0
             command[3]=50;  //max:-10 min:-90

             command2[0]=-10;
             command2[1]=90;
             command2[2]=0;
             command2[3]=50;
        }
        else 
        { 
             command[0]=0;  //max:0 min:-120
             command[1]=0;   //max:80 min:40
             command[2]=80;   //max:90 min:0
             command[3]=100;  //max:-10 min:-90

             command2[0]=0;
             command2[1]=0;
             command2[2]=80;
             command2[3]=100;
        }


        pos->positionMove(command.data());
        pos2->positionMove(command2.data());

        int count=50;
        while(count--)
            {
                Time::delay(0.1);
                bool ret=encs->getEncoders(encoders.data());

                Time::delay(0.1);
                bool ret2=encs2->getEncoders(encoders2.data());

                if (!ret)
                {
                    fprintf(stderr, "Error receiving encoders, check connectivity with the robot\n");
                }
                else if(!ret2)
                {
                    fprintf(stderr, "Error receiving encoders22222, check connectivity with the robot\n");
                }
                else
                {
                    printf("%.1lf %.1lf %.1lf %.1lf %.1lf %.1lf %.1lf %.1lf\n", 
                            encoders[0], encoders[1], encoders[2], encoders[3],
                            encoders2[0], encoders2[1], encoders2[2], encoders2[3]);
                }
            }


    }

    
    

    robotDevice.close();
    robotDevice2.close();

    return 0;
}
