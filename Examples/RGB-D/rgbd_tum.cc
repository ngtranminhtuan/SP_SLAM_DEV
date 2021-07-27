#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>
#include "kitti_util.h"
#include <opencv2/core/cuda.inl.hpp>

using namespace std;

int main(int argc, char **argv)
{
    if(argc != 5)
    {
        cerr << endl << "Usage: ./rgbd_tum path_to_vocabulary path_to_settings path_to_sequence path_to_association" << endl;
        return 1;
    }

    // Retrieve paths to images
    string img_seq = "/home/tuan/Downloads/Dataset/10/";
    
    // LoadImages(strAssociationFilename, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);
    // std::string seqId = split_string::getSequenceId(sequence_dir_path);
    kitti_sequence sequence(img_seq);
    const auto frames = sequence.get_frames();

    // Check consistency in the number of images and depthmaps
    int nImages = frames.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    SP_SLAM::System SLAM(argv[1],argv[2],SP_SLAM::System::RGBD,true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat imRGB, imD;
    for(int ni=5; ni<nImages; ni++)
    {
        auto frame = frames[ni];
        // Read image and depthmap from file
        imRGB = cv::imread(frame.left_img_path_,CV_LOAD_IMAGE_UNCHANGED);
        imD = cv::imread(frame.left_depth_path_,CV_LOAD_IMAGE_UNCHANGED);
        // imD.convertTo(imD, CV_32FC1, 1/255.0);
        double tframe = frame.timestamp_;

        if(imRGB.empty())
        {
            cerr << endl << "Failed to load image at: " << "/" << frame.left_img_path_[ni] << endl;
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        SLAM.TrackRGBD(imRGB,imD,tframe);

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
            T = frames[ni+1].timestamp_-tframe;
        else if(ni>0)
            T = tframe-frames[ni+1].timestamp_;

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
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
    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");   

    return 0;
}
