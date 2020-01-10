/*
Copyright 2019 Konstantinos Antonakoglou

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at
    http://www.apache.org/licenses/LICENSE-2.0
Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
 */
 
 /* 
  *Special thanks to Professor Marcelo A. C. Fernandez as this code 
  *is based on previous effort for a similar project.
  * 
  * */

#ifdef  _WIN64
#pragma warning (disable:4996)
#endif

#define REMOTE_IP "xx.xx.xx.xx" //set remote machine IP
#define	LOCAL_IP "xx.xx.xx.xx"  //set local machine IP
#define BUFLEN	512 //Max length of buffer
#define PORT "7777"   //The port on which to listen for incoming data
#define PORT2 "8888"

#include <cstdio>
#include <cassert>
#include <iostream>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/chrono.hpp>
#include <boost/asio.hpp>
#include <boost/array.hpp>

#if defined(WIN32)
# include <conio.h>
#else
# include "conio.h"
#endif

#include <HD/hd.h>
#include <HDU/hduVector.h>
#include <HDU/hduError.h>

using namespace std;
using boost::thread;

boost::mutex mutexUDP;

/***********Custom Data Types***************/
typedef union
{
	float f;
	unsigned long i;
}  FIType;

typedef struct
{
    unsigned char b0,b1,b2,b3;
} byteFloat;


struct LocalDeviceStruct {
	HDfloat LocalDeviceAngles[3];
	HDfloat LocalDeviceAnglesRef[3];
	HHD hHDLocal;
	HDSchedulerHandle callbackHandleLocal;
};
typedef LocalDeviceStruct LocalDeviceStructType;


struct TactileLocalDataStruct {
    float Theta1m, Theta2m, Theta3m;
    float Theta1mFB, Theta2mFB, Theta3mFB;
	float PIDkp1,PIDki1,PIDkd1;
	float PIDkp2,PIDki2,PIDkd2;
	float PIDkp3,PIDki3,PIDkd3;
	HDfloat prev_angle_interror[3] = {0.0, 0.0, 0.0};
	HDfloat prev_angle_error[3]= {0.0, 0.0, 0.0};
	float PIDTs, PIDinvTs, PIDSatValue;
	LocalDeviceStructType LocalHapticDevice;
	int ProcessingMarkUDPCommunicationLocal;
};
typedef TactileLocalDataStruct TactileLocalDataType, *PTactileLocalDataType;

int LocalHapticDeviceSetupStart(TactileLocalDataType  *data);
HDCallbackCode HDCALLBACK PIDCallback(void *data);
int LocalHapticDeviceClose(TactileLocalDataType  *data);
void LocalReceiveUDPServerThread(void* lpParam);
void LocalSendUDPClientThread(void* lpParam);
float toFloat(byteFloat bf);
byteFloat toByte(float a);

volatile int appMainOption=1;
volatile int UDPServerLocalStartCommunication=0;
volatile int UDPClientLocalStartCommunication=0;

/**************Local Send UDP thread******************/

//concatenate 4 bytes to create float
float toFloat(byteFloat bf)
{
    FIType x;
    x.i = bf.b3;
    x.i = x.i << 8 | bf.b2;
    x.i = x.i << 8 | bf.b1;
    x.i = x.i << 8 | bf.b0;
    return x.f;
}

byteFloat toByte(float a)
{
        byteFloat bf;
        FIType x;

        x.f = a;

        bf.b0 = x.i;
        x.i = x.i >> 8;
        bf.b1 = x.i;
        x.i = x.i >> 8;
        bf.b2 = x.i;
        x.i = x.i >> 8;
        bf.b3 = x.i;

		return bf;
}

int main(int argc, char* argv[])
{
	PTactileLocalDataType pTactileLocalData[1];
	pTactileLocalData[0] = (PTactileLocalDataType) malloc(sizeof(TactileLocalDataType));
	pTactileLocalData[0] ->ProcessingMarkUDPCommunicationLocal = 0;

	pTactileLocalData[0]->PIDkp1 = 2000;
	pTactileLocalData[0]->PIDki1 = 0.001;
	pTactileLocalData[0]->PIDkd1 = 0.05;

	pTactileLocalData[0]->PIDkp2 = 1000;
	pTactileLocalData[0]->PIDki2 = 0;
	pTactileLocalData[0]->PIDkd2 = 0.001;

	pTactileLocalData[0]->PIDkp3 = 1000;
	pTactileLocalData[0]->PIDki3 = 0;
	pTactileLocalData[0]->PIDkd3 = 0.001;

	pTactileLocalData[0]->PIDTs = 0.000625;
	pTactileLocalData[0]->PIDinvTs = 1/0.000625;

	pTactileLocalData[0]->PIDSatValue = 4.5;
	//pTactileLocalData[0]->PIDNormFactor = 1000;


    HDErrorInfo error;

	std::thread thread_LocalReceive(LocalReceiveUDPServerThread,pTactileLocalData[0]);
	std::thread thread_LocalSend(LocalSendUDPClientThread,pTactileLocalData[0]);

	LocalHapticDeviceSetupStart(pTactileLocalData[0]);

	usleep(1000);

	thread_LocalReceive.join();

	usleep(1000);

	thread_LocalSend.join();

    return 0;
}

/*****************************************************************************/

int LocalHapticDeviceSetupStart(TactileLocalDataType  *data)
{
	HDErrorInfo error;

		data->LocalHapticDevice.hHDLocal = hdInitDevice("Default Device");
	    if (HD_DEVICE_ERROR(error = hdGetError()))
	    {
	        hduPrintError(stderr, &error, "\nFailed to initialize Local haptic device\n");
	        fprintf(stderr, "\nPress any key to quit.\n");
	        getch();
	        return -1;
	    }

	    printf("\nLocal haptic device\n");
	    printf("\nFound device model: %s.\n\n", hdGetString(HD_DEVICE_MODEL_TYPE));

		data->LocalHapticDevice.callbackHandleLocal = hdScheduleAsynchronous(PIDCallback, data, HD_MAX_SCHEDULER_PRIORITY);

		hdMakeCurrentDevice(data->LocalHapticDevice.hHDLocal);
		hdDisable(HD_ONE_FRAME_LIMIT);
		hdEnable(HD_FORCE_OUTPUT);

		//hdSetSchedulerRate(1600);
		hdSetSchedulerRate(1000);
		hdStartScheduler();
		/* Check for errors and abort if so. */
		if (HD_DEVICE_ERROR(error = hdGetError()))
		{
		hduPrintError(stderr, &error, "Failed to start scheduler");
		fprintf(stderr, "\nPress any key to quit.\n");
		return -1;
		}

		printf("Start haptic device \n");
		printf("******************************************\n");
		return 0;

}

HDCallbackCode HDCALLBACK PIDCallback(void *data)
{
		long double timeLocalThread = 0;
		long double timeLocalThreadTemp = 0;

		float dataInBuffer[3];
		float dataOutBuffer[3];

		HDErrorInfo error;

		TactileLocalDataType *Data = (TactileLocalDataType*) data;

		Data->ProcessingMarkUDPCommunicationLocal = 1;

		//Current joint agngles and velocities pointers
		HDfloat *jointAngles = Data->LocalHapticDevice.LocalDeviceAngles;

		//Reference joint angles and velocities pointers
		HDfloat *jointAnglesRef = Data->LocalHapticDevice.LocalDeviceAnglesRef;
		float Ts = Data->PIDTs;


		hdMakeCurrentDevice(Data->LocalHapticDevice.hHDLocal);

		hdBeginFrame(Data->LocalHapticDevice.hHDLocal);

		//Update local angle variables
		//-----------------------------------------------
		jointAngles[0] = Data->Theta1m;
		jointAngles[1] = Data->Theta2m;
		jointAngles[2] = Data->Theta3m;
		//-----------------------------------------------


		//Update local angle variables
		//-----------------------------------------------
		jointAnglesRef[0] =   Data->Theta1mFB;
		jointAnglesRef[1] =   Data->Theta2mFB;
		jointAnglesRef[2] =   Data->Theta3mFB;
		//-----------------------------------------------
		HDfloat angle_error[3],  angle_derror[3], angle_interror[3];

		cout << jointAnglesRef[0] << "\n";
		//error
		angle_error[0] = jointAnglesRef[0] - jointAngles[0];
		angle_error[1] = jointAnglesRef[1] - jointAngles[1];
		angle_error[2] = jointAnglesRef[2] - jointAngles[2];

		// differential error
		angle_derror[0] = angle_error[0] - Data->prev_angle_error[0];
		angle_derror[1] = angle_error[1] - Data->prev_angle_error[1];
		angle_derror[2] = angle_error[2] - Data->prev_angle_error[2];

		//sum of errors (integral)
		angle_interror[0] = Data->prev_angle_interror[0] + angle_error[0];
		angle_interror[1] = Data->prev_angle_interror[1] + angle_error[1];
        angle_interror[2] = Data->prev_angle_interror[2] + angle_error[2];

        for (int i=0; i<3; i++){
        	if (angle_interror[i] > Data->PIDSatValue) angle_interror[i]=Data->PIDSatValue;
        }

		//Send feedback local torque to local haptic device
		//Use PID here:
		HDfloat x[3];
		x[0]=   (Data->PIDkp1 * angle_error[0] + Data->PIDki1*(angle_interror[0]*Data->PIDTs) + Data->PIDkd1*(angle_derror[0]*Data->PIDinvTs));
		x[1]=   (Data->PIDkp2 * angle_error[1] + Data->PIDki2*(angle_interror[1]*Data->PIDTs) + Data->PIDkd2*(angle_derror[1]*Data->PIDinvTs));
		x[2]=   (Data->PIDkp3 * angle_error[2] + Data->PIDki3*(angle_interror[2]*Data->PIDTs) + Data->PIDkd3*(angle_derror[2]*Data->PIDinvTs));

		hdSetFloatv(HD_CURRENT_JOINT_TORQUE,x);

		hdGetFloatv(HD_CURRENT_JOINT_ANGLES, jointAngles);

		Data->prev_angle_error[0] = angle_error[0];
		Data->prev_angle_error[1] = angle_error[1];
		Data->prev_angle_error[2] = angle_error[2];

		Data->prev_angle_interror[0] = angle_interror[0];
		Data->prev_angle_interror[1] = angle_interror[1];
		Data->prev_angle_interror[2] = angle_interror[2];


		//Update local angle variables
		//-----------------------------------------------
		Data->Theta1m = jointAngles[0];
		Data->Theta2m = jointAngles[1];
		Data->Theta3m = jointAngles[2];


		hdEndFrame(Data->LocalHapticDevice.hHDLocal);


		/* Check for errors and abort the callback if a scheduler error
			is detected. */
		if (HD_DEVICE_ERROR(error = hdGetError()))
		{
			hduPrintError(stderr, &error,
							"Error detected while rendering gravity well\n");

			if (hduIsSchedulerError(&error))
			{
				return HD_CALLBACK_DONE;
			}
		}
		/* Signify that the callback should continue running, i.e. that
			it will be called again the next scheduler tick. */
		return HD_CALLBACK_CONTINUE;
}


int LocalHapticDeviceClose(TactileLocalDataType  *data)
{

    // Cleanup and shutdown the haptic device, cleanup all callbacks.

	hdStopScheduler();
	hdUnschedule(data->LocalHapticDevice.callbackHandleLocal);
	hdDisableDevice(data->LocalHapticDevice.hHDLocal);

	printf("\n******************************************\n");
	printf("Stop haptic device \n");
	printf("******************************************\n");

	return 0;
}

void LocalSendUDPClientThread(void* lpParam){
	printf("\n******************************************\n");
	printf("Initialising tactile local client communication...\n");
	boost::asio::io_service io_service;
	boost::system::error_code error;
	boost::asio::ip::udp::resolver resolver(io_service);
	boost::asio::ip::udp::resolver::query query(boost::asio::ip::udp::v4(), REMOTE_IP, PORT2);
	boost::asio::ip::udp::endpoint receiver_endpoint = *resolver.resolve(query);
	boost::asio::ip::udp::socket socket(io_service);
	socket.open(boost::asio::ip::udp::v4());

	PTactileLocalDataType data = (PTactileLocalDataType) lpParam;
	char sendMSG[12];
    //start communication
	int k=0;
    while(appMainOption)
    {
    	//check if data from haptic thread is available for transmission
		if (data->ProcessingMarkUDPCommunicationLocal)
		{
			// convert 3 float angular and velocity values to 24 byte message
			byteFloat bfTheta1m = toByte(data->Theta1m);
			sendMSG[0]=bfTheta1m.b0;
			sendMSG[1]=bfTheta1m.b1;
			sendMSG[2]=bfTheta1m.b2;
			sendMSG[3]=bfTheta1m.b3;

			byteFloat bfTheta2m = toByte(data->Theta2m);
			sendMSG[4]=bfTheta2m.b0;
			sendMSG[5]=bfTheta2m.b1;
			sendMSG[6]=bfTheta2m.b2;
			sendMSG[7]=bfTheta2m.b3;

			byteFloat bfTheta3m = toByte(data->Theta3m);
			sendMSG[8]=bfTheta3m.b0;
			sendMSG[9]=bfTheta3m.b1;
			sendMSG[10]=bfTheta3m.b2;
			sendMSG[11]=bfTheta3m.b3;

		}

//		try {

			socket.send_to(boost::asio::buffer(sendMSG, sizeof(sendMSG)), receiver_endpoint);
			//cout << sendMSG << "\n";
//		} catch (const boost::system::system_error& e) {
//		    std::cerr << e.what() << std::endl;
//		}
    }

}

void LocalReceiveUDPServerThread(void* lpParam){

	PTactileLocalDataType data = (PTactileLocalDataType) lpParam;

	char receiveMSG[12];

	printf("\n******************************************\n");
    printf("Initialising tactile local server communication...\n");
		boost::asio::io_service io_service;
		boost::system::error_code error;
		boost::asio::ip::udp::resolver resolver(io_service);
		boost::asio::ip::udp::resolver::query query(boost::asio::ip::udp::v4(), LOCAL_IP, PORT);
		boost::asio::ip::udp::endpoint receiver_endpoint = *resolver.resolve(query);
		boost::asio::ip::udp::socket socket(io_service);
		socket.open(boost::asio::ip::udp::v4());

		try {
			socket.bind(receiver_endpoint);
		}catch (std::exception const&  ex)
		{
			cout << "Error: " << ex.what();
		}
	    printf("\nInitialised...Socket created.\n");

		//boost::array<char, 100> recv_buf;
		boost::asio::ip::udp::endpoint sender_endpoint;

		 //keep listening for data
	    while(appMainOption)
	    {
	    	//mutexUDP.lock();
	        //printf("\nWaiting for data...\n");
	         fflush(stdout);

	         //clear the buffer by filling null, it might have previously received data
	         memset(receiveMSG,'\0', 12);

	         size_t len = socket.receive_from(boost::asio::buffer(receiveMSG), sender_endpoint);


	        //receive message and convert to 6 float values
		    byteFloat bfTheta1mFB;
		    bfTheta1mFB.b0 = receiveMSG[0];
		    bfTheta1mFB.b1 = receiveMSG[1];
		    bfTheta1mFB.b2 = receiveMSG[2];
		    bfTheta1mFB.b3 = receiveMSG[3];
		    data->Theta1mFB = toFloat(bfTheta1mFB);
		    byteFloat bfTheta2mFB;
		    bfTheta2mFB.b0 = receiveMSG[4];
		    bfTheta2mFB.b1 = receiveMSG[5];
		    bfTheta2mFB.b2 = receiveMSG[6];
		    bfTheta2mFB.b3 = receiveMSG[7];
		    data->Theta2mFB  = toFloat(bfTheta2mFB);

		    byteFloat bfTheta3mFB;
		    bfTheta3mFB.b0 = receiveMSG[8];
		    bfTheta3mFB.b1 = receiveMSG[9];
		    bfTheta3mFB.b2 = receiveMSG[10];
		    bfTheta3mFB.b3 = receiveMSG[11];
		    data->Theta3mFB  = toFloat(bfTheta3mFB);

		    // receive debug
		    cout << "Received: " << data->Theta1mFB << ", " << data->Theta2mFB << ", " << data->Theta3mFB << "\n";
	       // mutexUDP.unlock();
	    }

}
