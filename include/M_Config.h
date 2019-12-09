//
//  M_Config.h
//  
//
//  Created by TuLigen on 2019/6/4.
//  Copyright © 2019年 TuLigen. All rights reserved.
//

#ifndef _M_CONFIG_H_H_
#define _M_CONFIG_H_H_

#include "M_Types.h"
/*
 * 配置接口类
 */
class  IConfig
{
public:
	IConfig(){}
	virtual ~IConfig(){}


	/*
	 * 读取配置
	 */
	virtual bool ReadConfig(Camera &cam) = 0;
};

/*
 * 威亚标定参数读取
 */
class  WeiYaConfig : public IConfig
{
public:
	/* 构造函数
	 * @param intrin 内参文件
	 * @param bs     安置参数
	 */
	WeiYaConfig(const std::string &intrin, const std::string &bs) :mIntrinsics(intrin), mBs(bs)
	{
	}
    ~WeiYaConfig()
    {}
	/*
	* 读取配置
	*/
	virtual bool ReadConfig(Camera &cam);

protected:
	std::string mIntrinsics;
	std::string mBs;
};



class ConfigParam
{
public:
    ConfigParam(const std::string &str)
    {
        cv::FileStorage fSettings(str,cv::FileStorage::READ);

        std::cout << "Config file status : " << fSettings.isOpened() << std::endl;

        _BeginNo = fSettings["Sys.BeginNo"];
        _EndNo   = fSettings["Sys.EndNo"];

        fSettings["Sys.VocPath"] >> _VocPath;
        cout << "voc file path : " << _VocPath.c_str() << endl;
        fSettings["Sys.PstPath"] >> _PstPath;
        cout << "pst file path : " << _PstPath.c_str() << endl;
        fSettings["Sys.ImgPath"] >> _ImgPath;
        cout << "img file path : " << _ImgPath.c_str() << endl;
        fSettings["Sys.ImuPath"] >> _ImuPath;
        cout << "imu file path : " << _ImuPath.c_str() << endl;
    }

    
    static std::string _PstPath;
    static std::string _ImgPath;
    static std::string _ImuPath;
    static std::string _VocPath;
    static int         _BeginNo;
    static int         _EndNo;
};



//add more



#endif
