// Copyright (c) 2023. Minh Nguyen, Lam Luu
// All rights reserved.

// Các trục x,y của ảnh hiển thị trên màn hình window được quy ước như theo chuẩn trong thị giác máy tính
// ----------------------> (Truc x)
// |
// |
// |
// |
// |
// |
// |
// | (Truc y)

// Add thư viện vào
// Đường tròn nhìn thấy bao các đỉnh Visible Points được tạo ra có 
// tâm là ClosestPoint (Đỉnh tòa nhà gần nhất với điểm thất lạc) và bán kính là Visible Radius
// Sau khi vẽ được đường tròn thì các Visible Points lân cận được hiển thị trên màn hình Window

#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include "Simulator.h"

// Thực hiện khai báo trước khi sử dụng hàm
//---------------------------------------------------------------------------------------------------------------------------------------------------
void mouseCallback(int event, int x, int y, int flags, void* userData);
void makeText(const cv::Mat& target, const std::string& text, int line);

std::vector<cv::Point2f> obtainCorners(cv::Mat dst); 
std::vector<Point2D> mesh{}; // Khai báo tập hợp tọa độ các đỉnh trong tòa nhà

Point2D lostPoint{};// Khai báo điểm thất lạc
OfflineStat offlineStat; // Khai báo biến offlineStat 
//---------------------------------------------------------------------------------------------------------------------------------------------------

constexpr auto WINDOW = "Localization"; // Đặt Tên Cửa sổ là "Localization" sau khi biên dịch chương trình

cv::Mat img; // Khai báo biến để đọc hình ảnh img 
auto visibleRadius = 50; // Gán giá trị bán kính phạm vi đường tròn

int main(const int argc, char** argv) {
    if (argc > 1) {
        try {
            visibleRadius = std::stoi(argv[1]); // Convert the string argv[1] to integer and store in visibleRadius
            if (visibleRadius <= 0) {
                visibleRadius = 50;
            }
        }
        catch (const std::exception&) {
            visibleRadius = 50;
        }
    }

    img = cv::imread("C:/Users/nhat0/Pictures/quan2.jpeg"); // Đọc ảnh trong thư mục input 

    cv::Mat gray; // Khai báo biến hình ảnh 
    
    // void cv::cvtColor	(	InputArray 	src,
    // OutputArray 	dst,
    // int 	code,
    // int 	dstCn = 0 
    // )		


    //Convert RGB to Gray Image
    cvtColor(img, gray, cv::COLOR_BGR2GRAY); 
    gray.convertTo(gray, CV_32FC1);

    cv::Mat dst = cv::Mat::zeros(gray.size(), CV_32FC1);
    cornerHarris(gray, dst, 2, 3, 0.04); 
    // gray: Input grayscale image.
    // dst: output image where corner response will be stored.
    // It has the same size as the input image.

    //result is dilated for marking the corners, not important
    dilate(dst, dst, cv::Mat(), cv::Point(-1, -1));
    // the 1st dst: input
    // the 2nd dst: output, có nghĩa là dilate trực tiếp trên ảnh đầu vào.
    // cv::Mat(): This is the kernel or structuring element used for dilation. 
    // dst: This is the input image on which dilation will be performed.
    // Initialize my map

    const auto corners = obtainCorners(dst);  // Tạo database obtainCorners chứa toàn bộ tọa độ các đỉnh trong tòa nhà.

    for (const auto& corner : corners) {
        mesh.emplace_back(corner.x, corner.y); 
        //Convert RGB to Gray Image//emplace_back: This is a function in C++ that constructs and adds a new element to the end of the container. 
        // Sau khi có tọa độ các đỉnh trong tòa nhà, thêm vào database mesh.
    }

    // Threshold for an optimal value, it may vary depending on the image.
    for (int i = 0; i < dst.rows; i++) {
        for (int j = 0; j < dst.cols; j++) {
            if (dst.at<float>(i, j) > 0.01f * dst.at<float>(0, 0)) {
                img.at<cv::Vec3b>(i, j)[0] = 0;
                img.at<cv::Vec3b>(i, j)[1] = 0;
                img.at<cv::Vec3b>(i, j)[2] = 255;
            }
        }
    }

    std::cout << "==================== NO GPS LOCALIZATION ====================== \n";
    std::cout << "Calculating Offline Phase\n";
    std::cout << "Map size: " + std::to_string(mesh.size()) <<std::endl;
    
    // struct OfflineStat {
    //    std::vector<ParamSet> myMap;
    //    double offlineTime;
    //}; 
    offlineStat = generateMyMap(mesh); // tạo ra myMap và các đặc trưng. 
    cv::namedWindow(WINDOW); 
    // The code snippet cv::namedWindow(WINDOW); is using the OpenCV library in C++ to create a named window for displaying images or video frames
    // WINDOW là biến
    cv::setMouseCallback(WINDOW, mouseCallback, nullptr); 
    // cv::setMouseCallback is a function provided by the OpenCV library in C++.
    // It is used to set a callback function that will be called when a mouse event occurs on a window created using OpenCV.
    // winname: Name of the window where the mouse callback will be set.
    // onMouse: Callback function that will be called when a mouse event occurs.
    // userdata: An optional parameter that allows you to pass user data to the callback function.
    
    imshow(WINDOW, img);
    cv::waitKey(0);
    cv::destroyAllWindows();

    return 0;
}

std::vector<cv::Point2f> obtainCorners(cv::Mat dst) {
    // Xác định tọa độ của các đỉnh tòa nhà bằng thuật toán Harris Corner
    double minVal, maxVal;
    minMaxLoc(dst, &minVal, &maxVal);

    const auto thresh = 0.01 * maxVal;
    threshold(dst, dst, thresh, 255, 0);
    dst.convertTo(dst, CV_8UC1);

    // find centroids
    cv::Mat labels, stats, centroids;
    const auto numLabels = connectedComponentsWithStats(dst, labels, stats, centroids);

    // define the criteria to stop and refine the corners
    const auto criteria = cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 100, 0.001);
    const auto winSize = cv::Size(5, 5);
    const auto zeroZone = cv::Size(-1, -1);

    // Point2f: Kiểu dữ liệu tự định nghĩa, trong project dùng để trích xuất tọa độ của 1 đỉnh tòa nhà
    std::vector<cv::Point2f> corners{};

    for (int i = 1; i < numLabels; i++) {
        if (stats.at<int>(i, cv::CC_STAT_AREA) > 10) {  // filter out small blobs
            const auto centroidX = static_cast<float>(centroids.at<double>(i, 0));
            const auto centroidY = static_cast<float>(centroids.at<double>(i, 1));
            cv::Point2f centroid(centroidX, centroidY);
            corners.push_back(centroid);
        }
    }

    cornerSubPix(dst, corners, winSize, zeroZone, criteria);

    return corners;
}

void mouseCallback(const int event, const int x, const int y, const int flags, void* userData) {
    
    if (event == cv::EVENT_LBUTTONDOWN) {
        // Clear the old traces
        imshow(WINDOW, img);

        // Draw on a clone to maintain the clean image
        // Tạo ra một ảnh giả tên clone giống với ảnh gốc và ta sẽ làm việc trên ảnh đó
        auto clone = img.clone();

        std::cout << "Lost at: " << x << ' ' << y << '\n';

        lostPoint = Point2D(static_cast<float>(x), static_cast<float>(y));
        const auto onlineStat = navigate(offlineStat.myMap, mesh, lostPoint, visibleRadius);

        // The lost point
        circle(clone, cv::Point(x, y), 6, cv::Scalar(0, 0, 0), -1); // Tạo dấu chấm hiển thị điểm thất lạc, là con trỏ chuột trên bản đồ 

        // The closest point
        const auto closestPoint = onlineStat.closestPoint;
        circle(
                clone, cv::Point(static_cast<int>(closestPoint.x), static_cast<int>(closestPoint.y)),
                visibleRadius, cv::Scalar(255, 255, 20), 2 
        ); // Tạo dấu chấm hiển thị đỉnh tòa nhà gần nhất với điểm thất lạc

        // The visible points
        for (const auto& visible : onlineStat.visiblePoints) {
            circle(clone, cv::Point(static_cast<int>(visible.x), static_cast<int>(visible.y)),
                   6, cv::Scalar(169, 169, 169), -1
            ); // Tạo dấu chấm hiển thị đỉnh tòa nhà trong tầm nhìn 
        }

        // The found point
        for (const auto& result : onlineStat.suggestedPoints) {
            circle(clone, cv::Point(static_cast<int>(result.x), static_cast<int>(result.y)),
                   6, cv::Scalar(255, 255, 0), -1
            );
        }

        // Hiển thị các thông tin cần thiết lên góc trái màn hình Window sau khi đặt trỏ chuột lên bản đồ 
        const auto mapSizeText = "Map Size: " + std::to_string(mesh.size());
        makeText(clone, mapSizeText, 0);

        const auto offlineTimeText = "[+] Offline phase took: " + std::to_string(offlineStat.offlineTime) + " seconds";
        makeText(clone, offlineTimeText, 1);

        const auto visibleRadiusText = "Visible Radius : " + std::to_string(visibleRadius);
        makeText(clone, visibleRadiusText, 2);

        const auto visiblePointText = "Visible Points: " + std::to_string(onlineStat.visiblePoints.size()-1);
        makeText(clone, visiblePointText, 3);

        const auto onlineTimeText = "[+] Online phase took: " + std::to_string(onlineStat.onlineTime) + " seconds";
        makeText(clone, onlineTimeText, 4);

        const auto separateText = std::string{ "-----------------" };
        makeText(clone, separateText, 5);

        const auto resultText = std::string{ "Suggested Points:" };
        makeText(clone, resultText, 6);

        auto line = 7;

        // SuggestedPoints cũng có thể hiển thị nhiều điểm !
        for (const auto& result : onlineStat.suggestedPoints) {
            const auto pointText = "(" + std::to_string(result.x) + ", " + std::to_string(result.y) + ")";
            makeText(clone, pointText, line++);
        }

        imshow(WINDOW, clone);

        std::cout << "\n==================== NO GPS LOCALIZATION ====================== \n";
    }
}

void makeText(const cv::Mat& target, const std::string& text, const int line) {
    // Cấu hình văn bản thông báo lên cửa sổ Window
    const cv::Point org(0, (line + 1) * 20);
    constexpr int fontFace = cv::FONT_HERSHEY_SIMPLEX;
    constexpr double fontScale = 0.5;
    const cv::Scalar color(0, 0, 0);
    constexpr auto thickness = 2;

    putText(target, text, org, fontFace, fontScale, color, thickness);
}
