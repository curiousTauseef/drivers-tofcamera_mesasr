#pragma once

typedef enum {CP_RQ_CAM,     //(UDP) CPRqCam
              CP_AK_CAM,     //(UDP) CPAqCam
              CP_RD_FLASH,   //(TCP) CPRdFlash
              CP_WR_FLASH,   //(TCP) CPWrFlash
              CP_STATUS,     //(TCP) CPStatus
              CP_GET_REG,    //(TCP) CPRegister
              CP_SET_REG,    //(TCP) CPRegister
              CP_RQ_READ,    //(TCP) CPRead
              CP_AK_READ,    //(TCP) CPRead
              CP_USER=0x1000,// User Command space (not handeled in default mesaserver)
              CP_SHUTDOWN=-1 //(UDP/TCP) Shutdown the server
             }CPCmd;

typedef struct
{
	//CPCmd cmd;
	unsigned int cmd;
}CamProtocol;

typedef struct
{
	//CPCmd cmd;
	unsigned int cmd;
	unsigned int size;
}CamUser;

//(UDP) PC broadcast to find all cameras
typedef CamProtocol CPRqCam;

//(UDP) Cameras aknowledge their presence
typedef struct
{
	//CPCmd cmd;
	unsigned int cmd;
	//unsigned long mac[2];
	unsigned int mac[2];
	//time_t buildTime;        /* Current time */
	unsigned int buildTime;        /* Current time */
	unsigned int buildVersion;
	char devStr[32];
}CPAkCam;

//Callback function when seeking cameras
typedef void ( SeekCamCB ) ( CPAkCam* cpAkCam );

//(TCP) PC requests a flash read/write
typedef struct
{
	//CPCmd cmd;
	unsigned int cmd;
	unsigned int addr;
	unsigned int size;
	//Write Payload stream for CPWrFlash
}CPRdFlash,CPWrFlash;

typedef enum {CS_SUCCESS,
              CS_FAILED,
              CS_FLASH_ERASING,
              CS_FLASH_WRITING,
              CS_FLASH_READING,
              CS_CAMERA_REBOOT, //SR4k Old CIM needs to reboot the whole camera after FLASH access
              CS_FPGA_BOOTING,  //SR4k New CIM needs to reboot the only the FPGA after FLASH access
              CS_LAST
             }CamStatus;

typedef struct
{
	//CPCmd cmd;
	unsigned int cmd;
	//CamStatus status;
	unsigned int status;
	unsigned int data; //e.g. 0-100% or checksum
}CPStatus;

typedef struct
{
	//CPCmd cmd;  //CP_GET_REG, CP_SET_REG
	unsigned int cmd;
	unsigned char reg;
	unsigned char val;
}CPRegister;


typedef struct
{
	//CPCmd cmd;  //CP_READ
	unsigned int cmd;
	unsigned int mode;
	unsigned int size;
	// >> followed by the byte stream
}CPRead;
