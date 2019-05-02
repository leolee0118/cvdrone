#include "ardrone/ardrone.h"

using namespace std;
using namespace cv;

int main(int argc, char *argv[]) {
    ARDrone ardrone;
    if (!ardrone.open()) {
        cout << "Failed to initialize." << std::endl;
        return -1;
    }
    cout << "Opened."
    cout << "Battery " << ardrone.getBatteryPercentage() << "[%]" << std::endl;

    int cnt;
    FileStorage fs("image_counter", FileStorage::READ);
    if (!fs.isOpened()) cnt = 1;
    else fs["count"] >> cnt;
    while (true) {
        Mat img = ardrone.getImage();
        imshow("Press Space to capture.", img);
        int key = waitKey(33);
        if (key == ' ') {
            string filename = "./calibration_image/img_" + to_string(cnt++) + ".jpg";
            imwrite(filename, img);
            cout << filename << '\n';
        } else if (key == 'q') {
            fs << "count" << cnt;
            break;
        }
    }
    fs.release();
}
