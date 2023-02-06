#pragma once

#if (defined _WIN32)
#if (defined CSAPI_EXPORTS)
#define CS_EXPORTS __declspec(dllexport)
#else
#define CS_EXPORTS __declspec(dllimport)
#endif
#else
#define CS_EXPORTS __attribute__((visibility("default")))
#endif


#include"utility.h"
#include <vector>
#include <set>

namespace csapi
{

    class CS_EXPORTS CStreamer
    {

    public:
        static CStreamer *CreateInstance();
		/*
		*	获取设备类型
		*   传出类型
		*/
        virtual CSAPI_ERROR GetDeviceType(std::vector<DeviceTypeInfo> &vec_device_type) = 0;
        /*
        *	开启设备
        *   传入设备类型
        */
        virtual CSAPI_ERROR OpenDevice(const DeviceTypeInfo &vec_device_type) = 0;
        /*
       *	获取支持的分辨率
       */
        virtual CSAPI_ERROR GetSupportFormat(std::vector<Format>& vec_format, StreamType type) = 0;
        /*
       *	开始读取流
       */
        virtual CSAPI_ERROR StartStream(const Format &format, StreamType type) = 0;
        /*
        * 关闭流
        */
        virtual CSAPI_ERROR StopStream() = 0;
        /*
        * 切换分辨率
        */
        virtual CSAPI_ERROR ChangeResolution(const Format& format) = 0;

        virtual CSAPI_ERROR GetDepth(uint16_t **depth, int &width, int &height) = 0;

		virtual CSAPI_ERROR GetDepth(uint16_t **depth, uint16_t **ir, int &width, int &height) = 0;

        virtual CSAPI_ERROR GetRGB(uint16_t** depth, int& width, int& height) = 0;

       /*
		 *	计算点云图
		 * 传入depth图，传出三维点云
		*/
		virtual CSAPI_ERROR GetCloudPoint(const uint16_t* depth_data, float* point_cloud) = 0;

        /*
           * 计算点云图
           * 获取伪彩色数组和长度
        */
        virtual void GetColorBar(unsigned char** bar, int& length) = 0;
        /*
        * 传入深度图
        * 传出的二位伪彩色深度
        */
        virtual void Convert2RGB(uint8_t** color_data, uint16_t* depth, int& length) = 0;
        /*
        * 获取伪彩色当前颜色范围
        */
        virtual void GetCustomDistanceRange(int& min_val, int& max_val) = 0;
        /*
         *	设置伪彩色当前颜色范围
         */
        virtual void SetCustomDistanceRange(int min, int max) = 0;
        /*
         *	获取伪彩色最大最小范围   0   7500
         */
        virtual void GetDistanceRange(int& min_val, int& max_val) = 0;
        /*
        * 获取积分时间
        */
        virtual CSAPI_ERROR GetIntegralTime(uint16_t& int_tim) = 0;
        /*
        * 获取积分时间范围
        */
        virtual CSAPI_ERROR GetIntegralRange(uint16_t& min_int, uint16_t& max_int) = 0;
        /*
        * 获取固件版本
        */
        virtual CSAPI_ERROR  GetFirmwareVersion(std::string& version) = 0;
        /*
        * 获取设备信息
        */
        virtual CSAPI_ERROR GetDeviceInfo(DeviceInfo& deviceinfo) = 0;
        /*
        * 获取支持类型
        */
        virtual CSAPI_ERROR GetSupportedStream(std::set<StreamType>& stream_type) = 0;
        /*
        * 设置积分时间
        */
        virtual CSAPI_ERROR SetIntegralTime(uint16_t int_tim) = 0;
        /*
        * 开启、关闭滤波算法
        */
        virtual void IsOpenFillter(bool start) = 0;
        /*
        * 镜像、垂直翻转
        * FlipMode 翻转类型
        */
        virtual CSAPI_ERROR FlipAndMirror(FlipMode file_mode) = 0;
        /*
        * 保存原始数据
        * 保存路径（全路径包含名称）
        */
        virtual void Dump(const std::string filename, StreamType type) = 0;

		virtual CSAPI_ERROR GetCameraParameters(CameraParameters &camera_parameters) = 0;

    };

}
