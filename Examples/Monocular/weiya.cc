/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<iomanip>

#include<opencv2/core/core.hpp>

#include"System.h"
#include "dirent.h"
#include<unistd.h>
#include "M_Config.h"
#include "M_DataManager.h"

using namespace std;

#define WRITEFILE 1

int main(int argc, char **argv)
{
    const string cfgpath = "./weiya.yaml";
    
    ConfigParam config(cfgpath);

#if WRITEFILE
    const string realpath = ConfigParam::_OutPath + "/real.txt";
    const string estfpath = ConfigParam::_OutPath + "/est.txt";
    std::ofstream fReal;
    fReal.open(realpath);
    std::ofstream fEst;
    fEst.open(estfpath);
#endif

    vector<double> vTimestamps;
    Camera cam;
    Ptr<IConfig> pCfg = new WeiYaConfig(ConfigParam::_InsPath,ConfigParam::_fBsPath);
    pCfg->ReadConfig(cam);
    if(!M_DataManager::getSingleton()->LoadData(ConfigParam::_PstPath,
                                                ConfigParam::_ImuPath))
    {
        return -1;
    }

    int nImages = M_DataManager::getSingleton()->GetImgSize();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(ConfigParam::_VocPath,cfgpath,ORB_SLAM2::System::MONOCULAR,true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl; 

    // Main loop
    cv::Mat im;
    const int st_no = ConfigParam::_BeginNo;
    ImgInfoVIter it = M_DataManager::getSingleton()->begin() + st_no;
    ImgInfoVIter ed = min(M_DataManager::getSingleton()->end(),M_DataManager::getSingleton()->begin() + ConfigParam::_EndNo);
 
    if(it == ed)
        ed = M_DataManager::getSingleton()->end();

    M_DataManager::getSingleton()->setIndicator(st_no);
    int index = 0;
    PoseData origin  = it->second;
    PoseData predata = it->second;
    cv::Mat abspos = cv::Mat::eye(4,4,CV_64F);
    for(; it != ed; ++it)
    {
        size_t len = it->first.size() - 12;
        std::string picname = it->first.substr(len,6).c_str();
        // Read image from file
        const string imgpath = ConfigParam::_ImgPath + it->first;
        im = cv::imread(imgpath,CV_LOAD_IMAGE_UNCHANGED);
        double tframe = it->second._t;

        if(im.empty())
        {
            cerr << endl << "Failed to load image at: " << it->first << endl;
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif
        cv::Mat velcity = cv::Mat::eye(4,4,CV_64F);
        if(0 != index)
        {
            cv::Mat R,t;

            M_Untils::GetRtFromPose(predata,it->second,cam.RCam2Imu,cam.TCam2Imu,R,t);

            R.copyTo(velcity.rowRange(0,3).colRange(0,3));
            t.copyTo(velcity.rowRange(0,3).col(3));

            // abspos = velcity * abspos;
            // cv::Mat ptmt = -abspos.rowRange(0,3).colRange(0,3).t()*abspos.rowRange(0,3).col(3);
            // cout << "real " << ptmt.at<double>(2) << endl;
            predata = it->second;
        }
        #if WRITEFILE
        M_Untils::WriteRealTrace(fReal,it->second.pos,picname);
        #endif
        // Pass the image to the SLAM system
        SLAM.TrackMonocular(im,velcity,picname,tframe);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[index]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(index < nImages - 1)
            T = (it + 1)->second._t - tframe;
        else if(index > 0)
            T = tframe - (it - 1)->second._t;
        else ;
        
        ++index;
        
        if(ttrack<T)
            usleep((T-ttrack)*1e6);
        // usleep(1.5e6);
    }

    // Stop all threads
    SLAM.Shutdown();
    cout << "Slam Program exe Successfully!" << endl;

#if WRITEFILE
    std::vector<ORB_SLAM2::KeyFrame*> keyframes = SLAM.GetAllKeyFrames();
    cout << "Begin saving estimate trace! " << keyframes.size() << endl;
    for(int i = 0 ; i < keyframes.size() ; ++i)
    {
        cv::Mat pos = keyframes[i]->GetPose();
        cv::Mat dpos;
        pos.convertTo(dpos,CV_64F);
        cv::Mat R = dpos.rowRange(0,3).colRange(0,3);
        cv::Mat t = dpos.rowRange(0,3).col(3);
        BLHCoordinate blh;

        t = -R.t() * t;

        M_Untils::CalcPoseFromRT(origin,Mat::eye(3,3,CV_64F),t,cam.RCam2Imu,cam.TCam2Imu,blh);
        M_Untils::WriteEstTrace(fEst,blh,cv::Point3d(0,0,0),"");
    }
    cout << "Estimate trace write successfully!!! " << endl;
    fReal.close();
    fEst.close();
#endif
    return 0;
}