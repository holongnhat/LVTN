// Copyright (c) 2023. Minh Nguyen, Lam Luu
// All rights reserved.

// Trong file thực hiện tính toán ngầm.
#include <algorithm>
#include <cmath>
#include <numbers>

#include "ParamSet.h"

// Khai báo giá trị sai số cho phép bằng 0.00001 !
constexpr auto ANGLE_ERROR = 0.00001f;

// Định nghĩa các hàm trước
bool approx(float x, float y);
bool isSubset(const std::vector<float>& a, const std::vector<float>& b);

// Lập trình chức năng của hai hàm approx và isSubset !
bool approx(float x, float y) {
    // Đảm bảo sai số trong tầm kiểm soát.
    // Nếu nhỏ hơn sai số cho phép, trả giá trị về bằng 1 và ngược lại, bằng 0 nếu vượt quá cho phép !
    return std::abs(x - y) <= ANGLE_ERROR;
}

bool isSubset(const std::vector<float>& a, const std::vector<float>& b) {
    // hàm này dùng để kiểm tra tập hợp a có phải là tập hợp con của tập b hay không.
    auto tmpA = a;
    auto tmpB = b;
    std::ranges::sort(tmpA); // Sắp xếp các giá trị trong tập hợp a từ thấp đến cao
    std::ranges::sort(tmpB); // Sắp xếp các giá trị trong tập hợp b từ thấp đến cao

    auto count = 0;
    for (const auto bNode : tmpB) { // bNode: Các phần tử trong tập tmpB
        if (tmpA[count] == bNode) {
            ++count;
        }
    }
    return count == tmpA.size();
}

//3.1.3 Generate PS in Thesis.

// struct ParamSet { 
//     std::vector<float> distances{}; 
//     std::vector<float> angles{}; 
// };

// struct Point2D {
//     float x{ 0.0f }; 
//     float y{ 0.0f };
// };

ParamSet generateParamSet(const Point2D &source, const std::vector<Point2D> &mesh) {
    // Đầu vào của hàm generateParamSet: 
    // source: 
    // mesh: Tọa độ của toàn bộ các đỉnh tòa nhà có trên bản đồ
    

    // hàm generateParamSet để thực hiện tính toán, trả về các giá trị góc và khoảng cách tương ứng với 1 điểm pi(i = 1->n) trên bản đồ. 
    // hàm generateParamSet -> (p1(d1,d2,...; alpha_1, alpha_2,...); p2(d1,d2,...; alpha_1, alpha_2,...); ... ; pn(d1,d2,...; alpha_1, alpha_2,...))
    
    static constexpr auto TWO_PI = std::numbers::pi_v<float> * 2.0f; //TWO_PI =-3.14

    // Compute the ParamSet help list
    // pHelpList là một vector, là tập các góc và khoảng cách trên map.

    // struct ParamSetNode {
    // Point2D point;     
    // float distance{}; 
    // float theta{};  
    // };
    auto psHelpList = std::vector<ParamSetNode>{};
    // Tập psHelpList là một database có dạng: 
    // (p1(d1,d2,...; alpha_1, alpha_2,...); p2(d1,d2,...; alpha_1, alpha_2,...); ... ; pn(d1,d2,...; alpha_1, alpha_2,...)).

    // source là Điểm thất lạc, trong khâu Offline, source là điểm bất kỳ trên bản đồ.

    for (const auto &point : mesh) {
        //point là một phần tử trong mảng mesh, giống phần tử i trong câu lệnh for(i:array)...
        //Trích xuất 1 điểm trong toàn bộ các điểm có trên bản đồ
        //source.x: Tọa độ điểm source tính theo trục x.
        //source.y: Tọa độ điểm source tính theo trục y.

        if (point.x == source.x && point.y == source.y) {
            // Nếu tọa độ của source trùng với tọa độ của tòa nhà thì vẫn tiếp tục quá trình.
            // Trong khâu Offline, point trùng với source. 
            continue;
        }

        // Thực hiện tính toán các đặc trưng của điểm thất lạc 
        // Đặc trưng là đỉnh của tòa nhà so với toàn bộ các điểm trên bản đồ, chứ không phải ngược lại
        auto node = ParamSetNode{}; 
        // node là đặc trưng của điểm thất lạc (gồm góc và khoảng cách)
        // Trong khâu Offline, node là điểm bất kỳ trên bản đồ

        node.point = point; 
        node.distance = std::sqrt(std::pow(source.x - point.x, 2.0f) + std::pow(source.y - point.y, 2.0f));
        // node.distance: khoảng cách từ điểm thất lạc đến 1 đỉnh của tòa nhà
        
        auto theta = std::atan2(point.y - source.y, point.x - source.x); 
        if (theta < 0.0f) {
            theta += TWO_PI;
        }
        node.theta = theta;
        //node.theta: Góc từ điểm đỉnh tòa nhà đến điểm cần xét

        
        auto haveInserted = false; // Kiểm tra xem điểm đó có được thêm vào kho dữ liệu hay chưa.

        //The biggest difference is their functionality. 
        //push_back always puts a new element at the end of the vector and insert allows you to select new element's position

        // Thêm điểm vào database pHelpList !
        for (unsigned j = 0; j < psHelpList.size(); ++j) {
            if (node.theta == psHelpList[j].theta) {
                if (node.distance < psHelpList[j].distance) {
                    psHelpList.insert(psHelpList.begin() + j, node);
                    haveInserted = true; //Thông báo đã thêm dữ liệu vào psHelpList !
                    break;
                }
            }
            if (node.theta < psHelpList[j].theta) {
                psHelpList.insert(psHelpList.begin() + j, node);
                haveInserted = true; //Thông báo đã thêm dữ liệu vào psHelpList !
                break;
            }
        }

        if (!haveInserted) {
            psHelpList.push_back(node); 
        }
    }

    // Thực hiện tính toán, thực hiện trả về giá trị ParamSet cho hàm generateParamSet !
    auto ps = ParamSet{}; 
    for (unsigned i = 0; i < psHelpList.size(); ++i) {
        ps.distances.push_back(psHelpList[i].distance);
        if (i == psHelpList.size() - 1) {
            ps.angles.push_back(TWO_PI - (psHelpList[i].theta - psHelpList[0].theta));
        } else {
            ps.angles.push_back(psHelpList[i + 1].theta - psHelpList[i].theta);
        }
    }
    return ps; // Giá trị đặc trưng của điểm cần xét, ở đây là biến node
}

    // Làm việc với với hai khâu chính trong Project: Online và Offline 
std::vector<ParamSet> offlinePhase(const std::vector<Point2D>& mesh) {
    // Tạo ra tập vector gồm các điểm dựa vào hàm generateParamSet ở khâu Offline !

    // struct ParamSet { 
    //     std::vector<float> distances{}; 
    //     std::vector<float> angles{}; 
    // };

    // mesh: tọa độ của toàn bộ các đỉnh trên tòa nhà  
    // point: tọa độ của 1 điểm bất kỳ trên bản đồ
    auto result = std::vector<ParamSet>{}; 
    for (const auto& point : mesh) {
        result.push_back(generateParamSet(point, mesh));
    }
    return result;
}

// struct Point2D {
//     float x{ 0.0f }; 
//     float y{ 0.0f };
// };

std::vector<Point2D> onlinePhase(const std::vector<ParamSet>& myMap, const ParamSet& visiblePs, const std::vector<Point2D>& mesh) {

    auto resultPoints = std::vector<Point2D>{}; // Giá trị trả về là tập hợp tọa độ tính tính điểm thất lạc

    if (visiblePs.distances.empty()) {
        return resultPoints;
    }
    
    for (unsigned i = 0; i < myMap.size(); ++i) {
        // Check if target visiblePS's distances is subset of node's distances
        auto found = isSubset(visiblePs.distances, myMap[i].distances);
        if (found) {
            if (visiblePs.angles.empty()) {
                if (!visiblePs.distances.empty())
                    resultPoints.push_back(mesh[i]);
                continue;
            }

            const auto alpha = visiblePs.angles[0];
            auto lPtr  = 0;
            auto rPtr = 0;
            auto windowSum = 0.0f;

            do {
                rPtr %= static_cast<int>(myMap[i].angles.size());
                windowSum += myMap[i].angles[rPtr];
                if (approx(windowSum, alpha)) {
                    auto isMatch = true;
                    auto accumulatedSum = 0.0f;

                    auto l = 1;
                    for (
                        auto k = (rPtr + 1) % myMap[i].angles.size();
                        k != lPtr;
                        k = (k + 1) % myMap[i].angles.size()
                    ) {
                        accumulatedSum += myMap[i].angles[k];
                        if (approx(accumulatedSum, visiblePs.angles[l])) {
                            accumulatedSum = 0;
                            l++;
                        }
                        else if (accumulatedSum > visiblePs.angles[l]) {
                            isMatch = false;
                            break;
                        }
                    }
                    if (accumulatedSum != 0)
                        isMatch = false;
                    if (isMatch) {
                        resultPoints.push_back(mesh[i]);
                        break;
                    }
                    windowSum -= myMap[i].angles[lPtr];
                    // window_sum -= myMap[i].angle[right_pointer];
                    ++rPtr;
                    ++lPtr;
                } else if (windowSum > alpha) {
                    windowSum -= myMap[i].angles[lPtr];
                    windowSum -= myMap[i].angles[rPtr];
                    ++lPtr;
                } else if (windowSum < alpha) {
                    ++rPtr;
                }
            } while (lPtr != myMap[i].angles.size());
        }
    }
    return resultPoints;
}
