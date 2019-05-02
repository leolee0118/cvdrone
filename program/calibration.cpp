#include <iostream>
#include <opencv2/opencv.hpp>
#include <cstring>
#include <vector>

using namespace cv;
using namespace std;

class CameraCalibrator {
private:
    int cal_cnt, img_cnt;
    vector<string> filenames;
    Size borderSize;
    vector<vector<Point3f> > dstPoints;
    vector<vector<Point2f> > srcPoints;
public:
    void setFileNames() {
        filenames.clear();
        for (int i = 1; i < img_cnt; i++) {
            string fname = "./calibration_image/img_" + to_string(i) + ".jpg";
            filenames.push_back(fname);
        }
    }
    void setBorderSizeAndCounts(const Size &bsize, int ccnt, int icnt) {
        borderSize = bsize;
        cal_cnt = ccnt;
        img_cnt = icnt;
    }
    void addBoardPoints() {
        vector<Point3f> dstCorners;
        vector<Point2f> srcCorners;
        for (int i = 0; i < borderSize.height; i++) {
            for (int j = 0; j < borderSize.width; j++) {
                dstCorners.push_back(Point3f(i, j, 0.0));
            }
        }
        for (int i = 0; i < filenames.size(); i++) {
            Mat img = imread(filenames[i], CV_LOAD_IMAGE_GRAYSCALE);
            bool found = findChessboardCorners(img, borderSize, srcCorners);
            if (!found) {
                cout << filenames[i] << '\n';
                continue;
            }
            TermCriteria param(TermCriteria::MAX_ITER + TermCriteria::EPS, 30, 0.1);
            cornerSubPix(img, srcCorners, Size(5, 5), Size(-1, -1), param);
            if (srcCorners.size() == borderSize.area()) {
                srcPoints.push_back(srcCorners);
                dstPoints.push_back(dstCorners);
            }
        }
    }
    void calibrate(const Mat &src, Mat &dst) {
        Size imgSize = src.size();
        Mat cameraMatrix; // 內部參數
        Mat distMatrix; //畸變參數
        Mat mapx, mapy;
        vector<Mat> rvecs, tvecs; // rotation, tranlation of each image.
        calibrateCamera(dstPoints, srcPoints, imgSize, cameraMatrix, distMatrix, rvecs, tvecs);
        initUndistortRectifyMap(cameraMatrix, distMatrix, Mat(), cameraMatrix, imgSize, CV_32F, mapx, mapy);;
        remap(src, dst, mapx, mapy, INTER_LINEAR);

        string filename = "calibration_" + to_string(cal_cnt) + ".xml";
        FileStorage fs(filename, FileStorage::WRITE);
        fs << "intrinsic" << cameraMatrix;
        fs << "distortion" << distMatrix;
    }
    void fsWrite(const Mat &src) {
        Size imgSize = src.size();
        Mat cameraMatrix, distMatrix;
        vector<Mat> rvecs, tvecs;
        calibrateCamera(dstPoints, srcPoints, imgSize, cameraMatrix, distMatrix, rvecs, tvecs);

        string filename = "calibration_" + to_string(cal_cnt) + ".xml";
        FileStorage fs(filename, FileStorage::WRITE);
        fs << "intrinsic" << cameraMatrix;
        fs << "distortion" << distMatrix;
    }
};

int main() {
    ARDrone ardrone;
    if (!ardrone.open()) {
        cout << "Failed to initialize." << std::endl;
        return -1;
    }
    cout << "Opened."
    cout << "Battery " << ardrone.getBatteryPercentage() << "[%]" << std::endl;

    int cal_cnt, img_cnt;
    FileStorage fs("counter.xml", FileStorage::READ);
    if (!fs.isOpened()) {
        cout << "Can't calibrate without initial image.\n";
        return -1;
    } else {
        fs["calibration_count"] >> cal_cnt;
        fs["image_count"] >> img_cnt;
    }

    CameraCalibrator cc;
    cc.setFileNames();
    cc.setBorderSizeAndCount(Size(5, 7), cal_cnt);
    cc.addBoardPoints();

    while (true) {
        Mat src, dst;
        src = ardrone.getImage();
        cc.calibrate(src, dst);
        // cc.fsWrite(src);
        imshow("src", src);
        imshow("dst", dst);
        int key = waitKey(33);
        if (key == 'q') {
            cal_cnt++;
            break;
        }
    }
}