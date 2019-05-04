#include "ardrone/ardrone.h"
#include <opencv2/aruco.hpp>

using namespace std;
using namespace cv;
using namespace cv::aruco;

int main()
{
    ARDrone ardrone;
    if (!ardrone.open()) {
        cout << "Failed to initialize." << endl;
        return -1;
    }
    cout << "Battery = " << ardrone.getBatteryPercentage() << "[%]" << endl;

    while (true) {
        Mat img = ardrone.getImage();
        imshow("original", img);

        // calibrate with matrices (intrinsic, distortion)
        Mat intrinsic, distortion;
        Mat mapx, mapy;
        FileStorage fs("calibration.xml", FileStorage::READ);
        fs["intrinsic"] >> intrinsic;
        fs["distortion"] >> distortion; 
        initUndistortRectifyMap(intrinsic, distortion, Mat(), intrinsic, img.size(), CV_32F, mapx, mapy);
        remap(img, img, mapx, mapy, INTER_LINEAR);

        // marker detection
        Ptr<Dictionary> dictionary = getPredefinedDictionary(DICT_6X6_250);
        vector<int> ids;
        vector<vector<Point2f> > corners;
        detectMarkers(img, dictionary, corners, ids);
        if (ids.size() > 0) {
            drawDetectedMarkers(img, corners, ids);
            vector<Vec3d> rvecs, tvecs;
            estimatePoseSingleMarkers(corners, 12.6, intrinsic, distortion, rvecs, tvecs); // R, t
            for (int i = 0; i < ids.size(); i++) {
                drawAxis(img, intrinsic, distortion, rvecs[i], tvecs[i], 10);
                cout << tvecs[i] << '\n'; //水平、垂直、遠近
            }
            imshow("marker_detection", img);
            waitKey(33);
        }
    }
}
