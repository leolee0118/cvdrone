#include "ardrone/ardrone.h"
#include <opencv2/aruco.hpp>
using namespace std;
using namespace cv;
using namespace cv::aruco;
#include "pid.cpp"

// --------------------------------------------------------------------------
// main(Number of arguments, Argument values)
// Description  : This is the entry point of the program.
// Return value : SUCCESS:0  ERROR:-1
// --------------------------------------------------------------------------
int main(int argc, char *argv[])
{
    /*
    // AR.Drone class
    ARDrone ardrone;

    // Initialize
    if (!ardrone.open()) {
        std::cout << "Failed to initialize." << std::endl;
        return -1;
    }

    // Battery
    std::cout << "Battery = " << ardrone.getBatteryPercentage() << "[%]" << std::endl;

    // Instructions
    std::cout << "***************************************" << std::endl;
    std::cout << "*       CV Drone sample program       *" << std::endl;
    std::cout << "*           - How to play -           *" << std::endl;
    std::cout << "***************************************" << std::endl;
    std::cout << "*                                     *" << std::endl;
    std::cout << "* - Controls -                        *" << std::endl;
    std::cout << "*    'Space' -- Takeoff/Landing       *" << std::endl;
    std::cout << "*    'Up'    -- Move forward          *" << std::endl;
    std::cout << "*    'Down'  -- Move backward         *" << std::endl;
    std::cout << "*    'Left'  -- Turn left             *" << std::endl;
    std::cout << "*    'Right' -- Turn right            *" << std::endl;
    std::cout << "*    'Q'     -- Move upward           *" << std::endl;
    std::cout << "*    'A'     -- Move downward         *" << std::endl;
    std::cout << "*                                     *" << std::endl;
    std::cout << "* - Others -                          *" << std::endl;
    std::cout << "*    'C'     -- Change camera         *" << std::endl;
    std::cout << "*    'Esc'   -- Exit                  *" << std::endl;
    std::cout << "*                                     *" << std::endl;
    std::cout << "***************************************" << std::endl;

    while (1) {
        // Key input
        int key = cv::waitKey(33);
        if (key == 0x1b) break;

        // Get an image
        cv::Mat image = ardrone.getImage();

        // Take off / Landing 
        if (key == ' ') {
            if (ardrone.onGround()) ardrone.takeoff();
            else                    ardrone.landing();
        }

        // Move
        double vx = 0.0, vy = 0.0, vz = 0.0, vr = 0.0;
        if (key == 'i' || key == CV_VK_UP)    vx =  1.0;
        if (key == 'k' || key == CV_VK_DOWN)  vx = -1.0;
        if (key == 'u' || key == CV_VK_LEFT)  vr =  1.0;
        if (key == 'o' || key == CV_VK_RIGHT) vr = -1.0;
        if (key == 'j') vy =  1.0;
        if (key == 'l') vy = -1.0;
        if (key == 'q') vz =  1.0;
        if (key == 'a') vz = -1.0;
        ardrone.move3D(vx, vy, vz, vr);

        // Change camera
        static int mode = 0;
        if (key == 'c') ardrone.setCamera(++mode % 4);

        // Display the image
        cv::imshow("camera", image);
    }

    // See you
    ardrone.close();
    */


    ARDrone ardrone;
    if (!ardrone.open()) {
        cout << "Failed to initialize." << endl;
        return -1;
    }
    cout << "Battery = " << ardrone.getBatteryPercentage() << "[%]" << endl;
        
    double _x[3] = {2.5, 0.3, 0.7};
    double _y[3] = {2.5, 0.3, 0.7};
    double _z[3] = {2.5, 0.3, 0.2};
    double _r[3] = {2.5, 0.3, 0.2};
    Mat x(3, 1, CV_64F, _x), y(3, 1, CV_64F, _y), z(3, 1, CV_64F, _z), r(3, 1, CV_64F, _r);
    PIDManager pid(x, y, z, r);

    double t[4] = {0, 0, 100, 0};
    Mat target(4, 1, CV_64F, t);

    Mat intrinsic, distortion;
    FileStorage fs("calibration.xml", FileStorage::READ);
    fs["intrinsic"] >> intrinsic;
    fs["distortion"] >> distortion;

    bool open = false;
    while (true) {
        Mat img = ardrone.getImage();
        imshow("original", img);
        int key = waitKey(300);

        if (key == ' ') {
            if (ardrone.onGround()) ardrone.takeoff();
            else                    ardrone.landing();
        }
            // Move
        double vx = 0.0, vy = 0.0, vz = 0.0, vr = 0.0;
        if (key == 'i' || key == CV_VK_UP)    vx =  1.0;
        if (key == 'k' || key == CV_VK_DOWN)  vx = -1.0;
        if (key == 'u' || key == CV_VK_LEFT)  vr =  1.0;
        if (key == 'o' || key == CV_VK_RIGHT) vr = -1.0;
        if (key == 'q') vy =  1.0;
        if (key == 'a') vy = -1.0;
        if (key == 'j') vz =  1.0;
        if (key == 'l') vz = -1.0;
        ardrone.move3D(vx, vy, vz, vr);

        if (key == 'p') open = !open;

        // calibrate with matrices (intrinsic, distortion)
        
        Mat mapx, mapy;
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
                                          // y, z, x
            }
            imshow("marker_detection", img);
            if (open) {
                Mat error(4, 1, CV_64F), command(4, 1, CV_64F);
                for (int j = 0; j < 3; j++) error.at<double>(j, 0) = tvecs[0][j] - target.at<double>(j, 0);
                pid.getCommand(error, command);
                cout << command.at<double>(2, 0) << ' ';
                cout << command.at<double>(0, 0) << ' ';
                cout << command.at<double>(1, 0) << ' ';
                cout << command.at<double>(3, 0) << ' ';
                ardrone.move3D(command.at<double>(2, 0), -command.at<double>(0, 0), command.at<double>(1, 0), 0);
            }
        }

    }

    return 0;
}
