// Copyright (c) 2023. Minh Nguyen, Lam Luu
// All rights reserved.

#pragma once

#include <vector>

#include "ParamSet.h"

//Chia làm 2 khâu: Online và Offline !

// struct Point2D {
//     float x{ 0.0f }; 
//     float y{ 0.0f };
// };

struct OnlineStat {
    // Khởi tạo kho dữ liệu 
    std::vector<Point2D> visiblePoints;
    // Khởi tạo điểm 
    Point2D closestPoint;
    // Khởi tạo kho dữ liệu
    std::vector<Point2D> suggestedPoints;
    // Định nghĩa thời gian thực thi khâu online thuộc kiểu số thực
    double onlineTime; 
};

// struct ParamSet { 
//     std::vector<float> distances{}; 
//     std::vector<float> angles{}; 
// };

struct OfflineStat {
    // Khởi tạo kho dữ liệu 
    std::vector<ParamSet> myMap; 
    // Định nghĩa thời gian thực thi khâu offline thuộc kiểu số thực 
    double offlineTime;  
};

OnlineStat navigate(const std::vector<ParamSet>& myMap, const std::vector<Point2D>& mesh, const Point2D& lostPoint, int visibleRadius);

OfflineStat generateMyMap(const std::vector<Point2D>& mesh);
