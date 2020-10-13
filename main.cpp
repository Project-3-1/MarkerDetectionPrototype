#include <opencv2/opencv.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <string>
#include <chrono>

/**
 * THIS CODE IS MAINLY BASED ON THE ARUCO MARKER EXAMPLE CODE PROVIDED BY OPENCV.
 */

namespace {
    const char* about = "A tutorial code on charuco board creation and detection of charuco board with and without camera caliberation";
    const char* keys = "{c        |       | Put value of c=1 to create charuco board;\nc=2 to detect charuco board without camera calibration;\nc=3 to detect charuco board with camera calibration and Pose Estimation}";
}
void createBoard();
void detectCharucoBoardWithCalibrationPose();
void detectCharucoBoardWithoutCalibration();
static bool readCameraParameters(std::string filename, cv::Mat& camMatrix, cv::Mat& distCoeffs)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened())
        return false;
    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    return true;
}
void createBoard()
{
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(5, 7, 0.04f, 0.02f, dictionary);
    cv::Mat boardImage;
    board->draw(cv::Size(600, 500), boardImage, 10, 1);
    cv::imwrite("BoardImage.jpg", boardImage);
}

void createMarkers()
{
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::Mat markerImage;
    char buffer[20];
    for(int i = 0; i < 250; i++)
    {
        cv::aruco::drawMarker(dictionary, i, 200, markerImage, 1);

        sprintf(buffer, "markers/marker%03d.png", i);
        cv::imwrite(buffer, markerImage);
    }
}

void detectCharucoBoardWithCalibrationPose()
{
    cv::VideoCapture inputVideo;
    inputVideo.open(0);
    cv::Mat cameraMatrix, distCoeffs;
    std::string filename = "calib.txt";
    bool readOk = readCameraParameters(filename, cameraMatrix, distCoeffs);
    if (!readOk) {
        std::cerr << "Invalid camera file" << std::endl;
    } else {
        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
        cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(5, 7, 0.04f, 0.02f, dictionary);
        cv::Ptr<cv::aruco::DetectorParameters> params = cv::aruco::DetectorParameters::create();
        while (inputVideo.grab()) {
            cv::Mat image;
            cv::Mat imageCopy;
            inputVideo.retrieve(image);
            image.copyTo(imageCopy);
            std::vector<int> markerIds;
            std::vector<std::vector<cv::Point2f> > markerCorners;
            cv::aruco::detectMarkers(image, board->dictionary, markerCorners, markerIds, params);
            // if at least one marker detected
            if (markerIds.size() > 0) {
                cv::aruco::drawDetectedMarkers(imageCopy, markerCorners, markerIds);
                std::vector<cv::Point2f> charucoCorners;
                std::vector<int> charucoIds;
                cv::aruco::interpolateCornersCharuco(markerCorners, markerIds, image, board, charucoCorners, charucoIds, cameraMatrix, distCoeffs);
                // if at least one charuco corner detected
                if (charucoIds.size() > 0) {
                    cv::Scalar color = cv::Scalar(255, 0, 0);
                    cv::aruco::drawDetectedCornersCharuco(imageCopy, charucoCorners, charucoIds, color);
                    cv::Vec3d rvec, tvec;
                    // cv::aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds, board, cameraMatrix, distCoeffs, rvec, tvec);
                    bool valid = cv::aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds, board, cameraMatrix, distCoeffs, rvec, tvec);
                    // if charuco pose is valid
                    if (valid)
                        cv::aruco::drawAxis(imageCopy, cameraMatrix, distCoeffs, rvec, tvec, 0.1f);
                }
            }
            cv::imshow("out", imageCopy);
            char key = (char)cv::waitKey(30);
            if (key == 27)
                break;
        }
    }
}
void detectCharucoBoardWithoutCalibration()
{
    std::cout << "TickA";
    cv::VideoCapture inputVideo; //("test/video.mp4");
    inputVideo.open(0);
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_50);
    cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(5, 7, 0.04f, 0.02f, dictionary);
    cv::Ptr<cv::aruco::DetectorParameters> params = cv::aruco::DetectorParameters::create();
    params->cornerRefinementMethod = cv::aruco::CORNER_REFINE_NONE;

    float frame_rate = 25;
    std::chrono::duration<long, std::ratio<1, 1000000000>> prev = std::chrono::high_resolution_clock::now().time_since_epoch();

    while (inputVideo.grab()) {

        // fix the frame rate
        auto time_elapsed = (std::chrono::high_resolution_clock::now() - prev);
        if(time_elapsed.time_since_epoch().count() < 1000000000.0/frame_rate) {
            continue;
        }
        prev = std::chrono::high_resolution_clock::now().time_since_epoch();

        // retrieve the image
        cv::Mat image, result_image;
        inputVideo.retrieve(image);
        cv::resize(image, image, cv::Size(600, 600));

        image.copyTo(result_image);

        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f> > markerCorners;
        cv::aruco::detectMarkers(image, board->dictionary, markerCorners, markerIds, params);
        //or
        //cv::aruco::detectMarkers(image, dictionary, markerCorners, markerIds, params);
        // if at least one marker detected
        if (markerIds.size() > 0) {
            //std::cout << " Marker detected\n";
            cv::aruco::drawDetectedMarkers(result_image, markerCorners, markerIds);
            std::vector<cv::Point2f> charucoCorners;
            std::vector<int> charucoIds;
            cv::aruco::interpolateCornersCharuco(markerCorners, markerIds, image, board, charucoCorners, charucoIds);
            // if at least one charuco corner detected
            if (charucoIds.size() > 0)
            {
                cv::aruco::drawDetectedCornersCharuco(result_image, charucoCorners, charucoIds, cv::Scalar(255, 0, 0));
            }
        }

        // convert image to gray scale
        cv::Mat gray;
        cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        medianBlur(gray, gray, 3);
        cv::threshold(gray, gray, 0, 255, cv::THRESH_OTSU + cv::THRESH_BINARY);
        // detect circles
        std::vector<cv::Vec3f> circles;
        HoughCircles(gray, circles, cv::HOUGH_GRADIENT, 2,
                     gray.rows/4,  // change this value to detect circles with different distances to each other
                     150, 60, 20, 50 // change the last two parameters
                // (min_radius & max_radius) to detect larger circles
        );

        bool draw_all_circles = true;

        if(circles.size() >= 2 || draw_all_circles) {
            //std::cout << " Circle detected\n";
            bool hot = false;
            cv::Vec3i detected_circles[2];

            if(!draw_all_circles) {
                std::cout << " Circle ratios: ";
                for (size_t i = 0; i < circles.size() && !hot; i++) {
                    for (size_t j = 0; j < circles.size() && !hot; j++) {
                        if (j != i) {
                            float r_max = std::max(circles[i][2], circles[j][2]);
                            float r_min = std::min(circles[i][2], circles[j][2]);
                            float centre_distance = sqrt(
                                    ((circles[i][0] - circles[j][0]) * (circles[i][0] - circles[j][0])) +
                                    ((circles[i][1] - circles[j][1]) * (circles[i][1] - circles[j][1])));
                            if (std::abs(r_max / r_min - 2.1905) <= 0.25 /*&& centre_distance <= 50*/) {
                                std::cout << " HOT<> ";
                                std::cout << centre_distance;
                                std::cout << " <-";
                                hot = true;
                            }

                            detected_circles[0] = circles[i];
                            detected_circles[1] = circles[j];
                            std::cout << r_max / r_min;
                            std::cout << ",";
                        }
                    }
                }
                std::cout << "\n";
            }

            if(hot) {
                for(cv::Vec3i c : detected_circles)
                {
                    cv::Point center = cv::Point(c[0], c[1]);
                    circle(result_image, center, 1, cv::Scalar(0,100,100), 3, cv::LINE_AA);

                    int radius = c[2];
                    circle(result_image, center, radius, cv::Scalar(255,0,255), 3, cv::LINE_AA);
                }
            } else if(draw_all_circles) {
                for(cv::Vec3i c : circles)
                {
                    cv::Point center = cv::Point(c[0], c[1]);
                    circle(result_image, center, 1, cv::Scalar(0,100,100), 3, cv::LINE_AA);

                    int radius = c[2];
                    circle(result_image, center, radius, cv::Scalar(255,0,255), 3, cv::LINE_AA);
                }
            }
        }

        //--- display image

        cv::imshow("marker_image", result_image);
        cv::imshow("gray", gray);
        char key = (char)cv::waitKey(30);
        if (key == 27)
            break;
    }
}
int main(int argc, char* argv[])
{
    cv::CommandLineParser parser(argc, argv, keys);
    parser.about(about);
    if (argc < 2) {
        parser.printMessage();
        return 0;
    }
    int choose = parser.get<int>("c");
    switch (choose) {
        case 1:
            createBoard();
            std::cout << "An image named BoardImg.jpg is generated in folder containing this file" << std::endl;
            break;
        case 2:
            detectCharucoBoardWithoutCalibration();
            break;
        case 3:
            detectCharucoBoardWithCalibrationPose();
            break;
        case 4:
            createMarkers();
            break;
        default:
            break;
    }
    return 0;
}