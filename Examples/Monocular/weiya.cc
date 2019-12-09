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

int main(int argc, char **argv)
{
    const string cfgpath = "./weiya.yaml";
    ConfigParam config(cfgpath);

    vector<double> vTimestamps;
     
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
 
    M_DataManager::getSingleton()->setIndicator(st_no);
    int index = 0;
    for(; it != ed; ++it)
    {
        size_t len = it->first.size() - 12;
        cout << "read " << it->first.substr(len).c_str() << endl;
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

        // Pass the image to the SLAM system
        SLAM.TrackMonocular(im,tframe);

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
        else if(index++ > 0)
            T = tframe - (it - 1)->second._t;

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
        // usleep(1.5e6);
    }

    // Stop all threads
    SLAM.Shutdown();
    cout << "Exe Successfully!" << endl;
    cout << "Save the trace " << endl;
   

    return 0;
}