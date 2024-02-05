// Copyright (c) 2023. Minh Nguyen, Lam Luu
// All rights reserved.

#pragma once

#include <vector>

struct ParamSet {
    // Đặc trưng trên tất cả các điểm trong bản đồ là góc và khoảng cách.
    // Được sử dụng cho tính toán ban đầu.
    std::vector<float> distances{}; // Tạo 1 Tập hợp gồm các Khoảng cách.
    std::vector<float> angles{}; // Tạo 1 Tập hợp gồm các Góc.
};

struct Point2D {
    // Định nghĩa tọa độ của một điểm.
    // Định nghĩa tọa độ của một điểm theo 2 trục x và y là số thực, lấy 1 chữ số sau dấu chấm.
    float x{ 0.0f }; 
    float y{ 0.0f };
};

struct ParamSetNode {
    // Đặc trưng của điểm thất lạc đến các đỉnh của tòa nhà.
    Point2D point;     // Định nghĩa biến point, kiểu dữ liệu Point2D.
    float distance{}; // Khoảng cách từ điểm thất lạc đến các đỉnh của tòa nhà.
    float theta{};   // Góc tạo thành từ điểm thất lạc đến các đỉnh của tòa nhà.
};

ParamSet generateParamSet(const Point2D& source, const std::vector<Point2D>& mesh);
// Định nghĩa hàm generateParamSet, theo kiểu dữ liệu tự định nghĩa struct ParamSet.
std::vector<ParamSet> offlinePhase(const std::vector<Point2D>& mesh);
// Định nghĩa kho chứa dữ liệu dạng vector, tên offlinePhase, kiểu dữ liệu tự định nghĩa ParamSet.
std::vector<Point2D> onlinePhase(const std::vector<ParamSet>& myMap, const ParamSet& visiblePs, const std::vector<Point2D>& mesh);
// Định nghĩa kho chứa dữ liệu dạng vector, tên onlinePhase, kiểu dữ liệu tự định nghĩa ParamSet. 