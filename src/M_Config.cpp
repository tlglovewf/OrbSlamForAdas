#include "M_Config.h"

/*
* ��ȡ����
*/
bool WeiYaConfig::ReadConfig(Camera &cam)
{
	//�����ļ���һ��Ϊ���򷵻�
	if (mIntrinsics.empty() || mBs.empty())
	{
		cout << "Config file error >>>>>>>>>>>>>" << endl;
		return false;
	}
		

	
	try
	{
		FileStorage intr(mIntrinsics, FileStorage::READ);
		FileStorage bs(mBs, FileStorage::READ);
		cout << "intrinsics : " << mIntrinsics.c_str() << " " << intr.isOpened() << endl;
		cout << "bs         : " << mBs.c_str() << " " << bs.isOpened() << endl;
		if (intr.isOpened() && bs.isOpened())
		{
			intr["P1"] >> cam.K;

			bs["RotateMatrixCam2IMU"] >> cam.RCam2Imu;
			bs["TranslationVectorCam2IMU"] >> cam.TCam2Imu;

			if (cam.K.cols > 3)
			{
				cam.K = cam.K.colRange(0, 3);
			}
			cout << "Camera params >>>>>>>>>>>>>>>>>>>>>>>>>" << endl;
			cout << "K :" << endl << cam.K << endl;
			cout << "Rcam2imu :" << endl << cam.RCam2Imu << endl;
			cout << "Tcam2imu :" << endl << cam.TCam2Imu << endl;
			cout << "---------------------------------------" << endl;
		}
		else
		{
			cout << "Read config failed!!!" << endl;
			return false;
		}
	}
	catch (...)
	{
		return false;
	}
	
	return true;
}



std::string ConfigParam::_ImgPath;
std::string ConfigParam::_ImuPath;
std::string ConfigParam::_PstPath;
std::string ConfigParam::_VocPath;
std::string ConfigParam::_InsPath;
std::string ConfigParam::_fBsPath;

int         ConfigParam::_BeginNo = 0;
int         ConfigParam::_EndNo   = 0;
