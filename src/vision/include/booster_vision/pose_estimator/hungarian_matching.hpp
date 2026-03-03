#pragma once

#include <vector>
#include <string>
#include <utility>

#include <opencv2/opencv.hpp>

#include <munkres-cpp/munkres.h>
#include <munkres-cpp/matrix.h>
#include <munkres-cpp/adapters/matrix_std_2d_vector.h>

namespace booster_vision {

typedef struct MarkerCoordinates {
    cv::Point3f position;
    cv::Point3f ray;
    std::vector<MarkerCoordinates> neighbors;
    std::string name;

    MarkerCoordinates() = default;
    MarkerCoordinates(cv::Point3f position, std::string name) {
        this->position = position;
        this->name = name;
    }
    MarkerCoordinates(cv::Point3f position, cv::Point3f ray,
                      std::vector<MarkerCoordinates> neighbors, std::string name) {
        this->position = position;
        this->ray = ray;
        this->neighbors = neighbors;
        this->name = name;
    }

    double EuclideanDistance(const MarkerCoordinates &other) {
        return cv::norm(this->position - other.position);
    }

    double NameDistance(const MarkerCoordinates &other) {
        if (this->name.find(other.name) != std::string::npos) {
            return 0.5;
        }
        return 0.0;
    }

    double NeighborDistance(const MarkerCoordinates &other) {
        if (this->neighbors.empty() || other.neighbors.empty()) {
            return 2.0;
        }

        double distance = 0.0;
        double neighbour_distance_self = 0.0;
        double neightbour_distance_other = 0.0;

        for (auto &neighbour : this->neighbors) {
            neighbour_distance_self += this->EuclideanDistance(neighbour);
        }
        for (auto &neighbour : other.neighbors) {
            neightbour_distance_other += this->EuclideanDistance(neighbour);
        }
        distance += std::abs(neighbour_distance_self - neightbour_distance_other);

        for (int i = 0; i < this->neighbors.size(); i++) {
            distance += this->neighbors[i].NameDistance(other.neighbors[i]);
        }

        return distance * 0.5;
    }

    double Distance(const MarkerCoordinates &other) {
        return this->EuclideanDistance(other) + this->NameDistance(other) + this->NeighborDistance(other);
    }

} MarkerCoordinates;

void HungarianMatching(std::vector<std::pair<int, int>> *matching, std::vector<MarkerCoordinates> &A, std::vector<MarkerCoordinates> &B) {
    auto FindNeighbors = [](std::vector<MarkerCoordinates> &markers) {
        for (size_t i = 0; i < markers.size(); i++) {
            std::vector<double> distances;
            for (size_t j = 0; j < markers.size(); j++) {
                if (i == j) {
                    distances.push_back(std::numeric_limits<double>::max());
                    continue;
                }
                distances.push_back(markers[i].EuclideanDistance(markers[j]));
            }
            markers[i].neighbors.clear();
            // find the 2 closest markers and added to neighbors
            std::vector<MarkerCoordinates> neighbors;
            for (int k = 0; k < 2; k++) {
                auto min = std::min_element(distances.begin(), distances.end());
                int index = std::distance(distances.begin(), min);
                neighbors.push_back(markers[index]);
                distances[index] = std::numeric_limits<double>::max();
            }
        }
    };

    // find neighbors in point list, create toplogical graph
    FindNeighbors(A);
    FindNeighbors(B);

    std::vector<std::vector<double>> cost_matrix;
    for (size_t i = 0; i < A.size(); i++) {
        std::vector<double> row;
        for (size_t j = 0; j < B.size(); j++) {
            row.push_back(A[i].Distance(B[j]));
        }
        cost_matrix.push_back(row);
    }

    munkres_cpp::matrix_std_2d_vector<double> data(cost_matrix);
    munkres_cpp::Munkres<double, munkres_cpp::matrix_std_2d_vector> solver(data);

    matching->clear();
    for (size_t i = 0; i < cost_matrix.size(); i++) {
        for (size_t j = 0; j < cost_matrix[i].size(); j++) {
            if (cost_matrix[i][j] < 0.5) {
                matching->push_back(std::make_pair(i, j));
            }
        }
    }
}

} // namespace booster_vision
