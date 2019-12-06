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
using namespace std;




void LoadImages(const string &strSequence, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);

int main(int argc, char **argv)
{

    // const string imgpath = "/media/tu/Work/Datas/@@1002-0001-191122-03/gray";
    const string imgpath = "/media/tu/Work/Datas/@@1002-0001-190828-00/Output/gray";
    const string vocpath = "/media/tu/Work/GitHub/OrbSlamForAdas/Vocabulary/ORBvoc.txt";
    const string cfgpath = "/media/tu/Work/GitHub/OrbSlamForAdas/Examples/Monocular/weiya.yaml";

    printf("%s\n",imgpath.c_str());
    printf("%s\n",vocpath.c_str());
    printf("%s\n",cfgpath.c_str());

    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    LoadImages(imgpath, vstrImageFilenames, vTimestamps);

    int nImages = vstrImageFilenames.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(vocpath,cfgpath,ORB_SLAM2::System::MONOCULAR,true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat im;
    for(int ni = 300; ni<nImages; ni++)
    {
        size_t len = vstrImageFilenames[ni].size() - 12;
        cout << "read " << vstrImageFilenames[ni].substr(len).c_str() << endl;
        // Read image from file
        im = cv::imread(vstrImageFilenames[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(im.empty())
        {
            cerr << endl << "Failed to load image at: " << vstrImageFilenames[ni] << endl;
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

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        // if(ttrack<T)
        //     usleep((T-ttrack)*1e6);
        // usleep(1.5e6);
    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");    

    return 0;
}
bool isPicSuffix(const char *pName,size_t len)
{
    const size_t suffix = 3;
    assert(pName);
    if(len < suffix)
    {
        return false;
    }
    const char *pSuffix =  &pName[len - suffix];
    
    return !strcasecmp(pSuffix, "jpg") | !strcasecmp(pSuffix, "png");
}
void LoadImages( const std::string &dirpath, vector<string> &files , vector<double> &vTimestamps)
{
    DIR *dp;
    struct dirent *dirp;
    if((dp = opendir(dirpath.c_str())) == NULL)
    {
        assert(NULL);
    }
    int index = 0;
    while((dirp = readdir(dp)) != NULL)
    {
        if(isPicSuffix(dirp->d_name,strlen(dirp->d_name)))
        {
            std::string filepath(dirpath);
            filepath.append("/");
            filepath.append(dirp->d_name);
            files.emplace_back(filepath);
            // printf("%s\n",filepath.c_str());
            vTimestamps.push_back(1.0);
            ++index;
        }
    }
    closedir(dp);
    std::sort(files.begin(),files.end());
}