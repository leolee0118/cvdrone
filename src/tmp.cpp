#include "ardrone/ardrone.h"
#include <opencv2/aruco.hpp>
#include "pid.cpp"

using namespace std;
using namespace cv;
using namespace cv::aruco;
// --------------------------------------------------------------------------
// main(Number of arguments, Argument values)
// Description  : This is the entry point of the program.
// Return value : SUCCESS:0  ERROR:-1
// --------------------------------------------------------------------------


int main(int argc, char *argv[])
{
    ARDrone ardrone;
    if (!ardrone.open()) {
        cout << "Failed to initialize." << std::endl;
        return -1;
    }
    cout << "Battery = " << ardrone.getBatteryPercentage() << "[%]" << std::endl;

    while (true) {
        Mat img;
        img = ardrone.getImage();
        
        Mat intrinsic, distortion;
        Mat mapx, mapy;
        FileStorage fs("calibration.xml", FileStorage::READ);
        fs["intrinsic"] >> intrinsic;
        fs["distortion"] >> distortion; 
        initUndistortRectifyMap(intrinsic, distortion, Mat(), intrinsic, img.size(), CV_32F, mapx, mapy);
        remap(img, img, mapx, mapy, INTER_LINEAR);

        Ptr<Dictionary> dictionary = getPredefinedDictionary(DICT_6X6_250);
        vector<int> ids;
        vector<vector<Point2f> > corners;
        detectMarkers(img, dictionary, corners, ids);
        if (ids.size() > 0) {
            drawDetectedMarkers(img, corners, ids);
            vector<Vec3d> rvecs, tvecs;
            estimatePoseSingleMarkers(corners, 12.6, intrinsic, distortion, rvecs, tvecs);
            for (int i = 0; i < ids.size(); i++) {
                drawAxis(img, intrinsic, distortion, rvecs[i], tvecs[i], 10);
                cout << tvecs[i] << '\n'; //水平、垂直、遠近
            }
            imshow("test", img);
            waitKey(33);
        }
    }
    // while (1) {
    //     // Key input
    //     int key = cv::waitKey(300);
    //     if (key == 0x1b) break;
    //     // // Get an image
    //     Mat image = ardrone.getImage();
    //     // Take off / Landing 
    //     if (key == ' ') {
    //         if (ardrone.onGround()) ardrone.takeoff();
    //         else                    ardrone.landing();
    //     }

    //     // Move
    //     double vx = 0.0, vy = 0.0, vz = 0.0, vr = 0.0;
    //     if (key == 'i' || key == CV_VK_UP)    vx =  1.0; //左右
    //     if (key == 'k' || key == CV_VK_DOWN)  vx = -1.0;
    //     if (key == 'u' || key == CV_VK_LEFT)  vr =  1.0;
    //     if (key == 'o' || key == CV_VK_RIGHT) vr = -1.0;
    //     if (key == 'j') vy =  1.0; // 前後
    //     if (key == 'l') vy = -1.0;
    //     if (key == 'q') vz =  1.0;
    //     if (key == 'a') vz = -1.0; //垂直
    //     ardrone.move3D(vx, vy, vz, vr);

    //     // Change camera
    //     static int mode = 0;
    //     if (key == 'c') ardrone.setCamera(++mode % 4);

    //     // Display the image
    //     imshow("camera", image);
    // }

    return 0;
}
