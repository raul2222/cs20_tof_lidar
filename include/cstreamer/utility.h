#pragma once

#ifndef cs_DEVICE_UTILITY_H
#define cs_DEVICE_UTILITY_H

#if (defined _WIN32) 
#if (defined  CSAPI_EXPORTS)
#define CS_EXPORTS __declspec(dllexport)
#else
#define CS_EXPORTS __declspec(dllimport)
#endif
#else
#define CS_EXPORTS __attribute__((visibility("default")))
#endif

#include <stdint.h>
#include <string>
namespace csapi
{


	enum CSAPI_ERROR {

		SUCCESS = 0,

		INVALID_FORMAT = -1,
		NOT_OPEND = -2,


		UNIMPLEMENTED = -11,
		ERROR_1 = -12,
		ERROR_2 = -13,
		ERROR_3 = -14,
		NODEVICE = -15,
		NOFRAME = -16,

	};

	/*
	enum class DriveType {
		DT_NONE,
		DT_DSHOW,
		DT_SONI,
	};
	*/

	enum class ResolutionType
	{
		type_1280,
		type_640,
	};
	enum class DeviceType {
		CS10,
		CS20,
		CS20L,
		CS30_RGB,
		CS30_TOF,
	};

	struct TemCalibrationCoeF
	{
		float TemCalibrationCoeF1[4] = {};
		float TemCalibrationCoeF2[4] = {};
	};

#pragma pack(push,1)
	//标定参数(注意：添加参数需要往后添加，读写是按照字节位)
	struct CameraParameters
	{
		//double CameraMatrix[9] = { 0.0 };         //0-71
		//double DistCoeffs[5] = { 0.0 };         //72-111
		//double GlobalOffset80 = 0.0;            //112-119
		//double GlobalOffset100 = 0.0;           //120-127
		//double WigglingError80[39] = { 0.0 };   //128-439
		//double WigglingError100[31] = { 0.0 };
		//double Fitting80[6] = { 0.0 };
		//double Fitting100[6] = { 0.0 };
		float CameraMatrix[9] = { 0.0 };         //0-35
		float DistCoeffs[5] = { 0.0 };           //36-55
		float GlobalOffset80 = 0.0;              //56-60
		float GlobalOffset100 = 0.0;             //61-65
		float WigglingError80[39] = { 0.0 };     //66-219
		float WigglingError100[31] = { 0.0 };    //220-343
		float Fitting80[6] = { 0.0 };            //344-367
		float Fitting100[6] = { 0.0 };           //368-391 

	};
#pragma pack(pop)

	struct DeviceTypeInfo
	{
		DeviceType device_type;
		std::wstring product_name;
		int  node_index;
	};

	struct Format {
		int index;
		int width;
		int height;
		int fps;
	};

	/*
	* device 信息结构体
	*/
	struct DeviceInfo
	{
		char product_name[10];
		char product_number[10];
		void* dev;
		int  node_index;
	};

	enum StreamType {
		CS_TOF,
		CS_RGB,
		CS_DEPTH,
		CS_RGBD,
	};

	enum FlipMode {
		NOMirrow,
		HMirrow,
		VMirrow,
		HVMirrow,
	};
} //namespace csapi

#endif