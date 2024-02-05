// Copyright (c) 2023. Minh Nguyen, Lam Luu
// All rights reserved.

// OfflineStat tạo ra myMap dùng cho OnlineStat !
#include <chrono>
#include <cmath>
#include <limits>
#include <iostream>
#include "Simulator.h"

class Timer {
    // Tạo ra một Timer để đếm thời gian thực thi khâu Online và khâu Offline
    using Clock = std::chrono::steady_clock;
    using Second = std::chrono::duration<double, std::ratio<1> >;

    std::chrono::time_point<Clock> m_beg{ Clock::now() };

public:

    void reset() {
        m_beg = Clock::now();
    }

    [[nodiscard]] double elapsed() const {
        return std::chrono::duration_cast<Second>(Clock::now() - m_beg).count();
    }
};

/*
struct OnlineStat {
    std::vector<Point2D> visiblePoints;
    Point2D closestPoint;
    std::vector<Point2D> suggestedPoints;
    double onlineTime;
};
*/

OnlineStat navigate(const std::vector<ParamSet>& myMap, const std::vector<Point2D>& mesh, const Point2D& lostPoint, const int visibleRadius) {
    // Hàm này dùng để thhực thi quá trình định vị
    // Hàm trả về các giá trị: return OnlineStat{ visiblePoints, closestPoint, suggestedPoints, onlineTime };

    // Hàm này được gọi khi kích trỏ chuột trên màn hình window được tạo ra sau khi build và chạy chương trình
    
    // Thông tin về các Input của hàm: 
    // mesh: Tọa độ của toàn bộ các đỉnh trong tòa nhà, có được sau khi thực thi thuật toán Harris-Corners
    // lostPoint: Là tọa độ của con trỏ chuột trên màn hình window
    // visibleRadius: Bán kính, được thiết lập trong file main.cpp
    // myMap là Input được sử dụng cho hàm OnlinePhase 

    std::cout << "Map Size: " << mesh.size() << '\n';          // thông báo số đỉnh có trên bản đồ 
    std::cout << "Visible Radius: " << visibleRadius << '\n'; // thông báo bán kính của đường hình tròn - phạm vi xác định điểm thất lạc.

    auto closestPoint = Point2D{};                         // Tọa độ đỉnh tòa nhà có khoảng cách gần nhất so với điểm thất lạc.
    auto minDistance = std::numeric_limits<float>::max(); // Khoảng cách giữa điểm thất lạc với closestPoint.
    
    // LostPoint là điểm thất lạc, trong project, LostPoint được tạo ra bởi con trỏ chuột trên bản đồ. 
    for (const auto& x : mesh) {
        // Thực hiện tính khoảng cách từ điểm thất lạc đến đỉnh tòa nhà gần nhất
        if (minDistance > std::sqrt(std::pow(lostPoint.x - x.x, 2.0f) + std::pow(lostPoint.y - x.y, 2.0f))) {
            minDistance = std::sqrt(std::pow(lostPoint.x - x.x, 2.0f) + std::pow(lostPoint.y - x.y, 2.0f));
            closestPoint = x; // Gán tọa độ của đỉnh tòa nhà gần nhất tìm được vào closestPoint
        }
    }
    std::cout << "Closest point: " << "(" << closestPoint.x << ", " << closestPoint.y << "), with distance: " << minDistance << '\n';
    // Thông báo tọa độ của đỉnh tòa nhà gần nhất và khoảng cách so với điểm thất lạc
    auto visiblePoints = std::vector<Point2D>{}; //visiblePoints là các đỉnh tòa nhà nằm trong bán kính nhìn thấy.
    
    // Cách xác định visblePoints:
    for (auto x : mesh) {
        if (std::sqrt(std::pow(closestPoint.x - x.x, 2.0f) + std::pow(closestPoint.y - x.y, 2.0f)) < visibleRadius) {
            visiblePoints.push_back(x);
        }
    }

    // closestPoint là đỉnh tòa nhà gần với điểm thất lạc nhất.
    // visiblePoints là tập hợp các đỉnh tòa nhà trong vùng bán kính nhìn thấy, trừ điểm closestPoint 
    const auto visiblePs = generateParamSet(closestPoint, visiblePoints);
    std::cout << "Number of visible points: " << visiblePoints.size() - 1 << '\n'; // Số các đỉnh tòa nhà nằm trong phạm vi nhìn thấy
    // Trừ 1 ở đây nghĩa là trừ điểm closestPoint.

    auto timer = Timer{};
    timer.reset();
    const auto suggestedPoints = onlinePhase(myMap, visiblePs, mesh); 
    // Ước tính tọa độ của điểm thất lạc, kết quả cho ra gồm nhiều tọa độ khác nhau.
    // std::vector<Point2D> onlinePhase(const std::vector<ParamSet>& myMap, const ParamSet& visiblePs, const std::vector<Point2D>& mesh) { return resultPoints;}

    const auto onlineTime = timer.elapsed(); // Được sử dụng để đo thời gian thực thi khâu Online

    // Hiển thị thông báo lên màn hình window
    std::cout << "[t] Online phase took: " << onlineTime << " seconds\n";
    std::cout << "----------------------------------------------\n";
    std::cout << "Result:\n";
    
    for (const auto& result : suggestedPoints) {
        //suggestedPoints là một tập hợp gồm nhiều tòa độ ước lượng điểm thất lạc
        std::cout << result.x << " " << result.y << '\n';
    }

    return OnlineStat{ visiblePoints, closestPoint, suggestedPoints, onlineTime };
}


// struct OfflineStat {
//    std::vector<ParamSet> myMap;
//    double offlineTime;
//}; 

OfflineStat generateMyMap(const std::vector<Point2D>& mesh){
    // Thực thi khâu Offline
    auto timer = Timer{};
    const auto myMap = offlinePhase(mesh);
    const auto offlineTime = timer.elapsed(); // Được sử dụng để đo thời gian thực thi khâu Offline

    // Hiển thị thời gian thực thi khâu Offline trên cmd.
    std::cout << "[t] Offline phase took: " << offlineTime << " seconds\n"; 
    return OfflineStat{ myMap, offlineTime };
    // myMap dùng cho OnlineStat, 
    // đóng vai trò là một trong những input của hàm navigate kiểu OnlineStat để tìm suggestPoints (Tạo độ dự đoán điểm thất lạc - con trỏ chuột trên cửa sổ Window)
}
