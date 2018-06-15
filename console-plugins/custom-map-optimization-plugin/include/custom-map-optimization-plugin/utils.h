
#ifndef CUSTOM_MAP_OPTIMIZATION_PLUGIN_UTILS_H
#define CUSTOM_MAP_OPTIMIZATION_PLUGIN_UTILS_H

#include <iostream>
#include <string>
#include <vector>
#include "json.hpp"
#include <kindr/minimal/quat-transformation.h>

typedef kindr::minimal::QuatTransformation Transformation;

inline Transformation transformationFromJson(nlohmann::json j) {
  std::vector<float> translation = j["translation_m"];
  std::vector<float> quaternions = j["rotation_xyzw"];

  kindr::minimal::RotationQuaternion quat(quaternions[3], quaternions[0],
                                          quaternions[1], quaternions[2]);
  kindr::minimal::Position pos;
  pos << translation[0], translation[1], translation[2];

  Transformation transformation(quat, pos);
  return transformation;
}

struct Camera {
  long timestamp_ns;
  Transformation pose;

  static Camera fromJson(nlohmann::json j) {
    Camera cam;
    cam.timestamp_ns = j["timestamp_ns"];
    cam.pose = transformationFromJson(j["pose"]);
    return cam;
  }
};

struct LoopClosureEdge {
  Camera from;
  Camera to;
  Transformation T_from_to;

  static LoopClosureEdge fromJson(nlohmann::json j) {
    LoopClosureEdge edge;
    edge.T_from_to = transformationFromJson(j["T_from_to"]);
    edge.from = Camera::fromJson(j["camera_from"]);
    edge.to = Camera::fromJson(j["camera_to"]);
    return edge;
  }
};
typedef std::vector<LoopClosureEdge> LoopClosureEdges;

inline void loopClosureEdgesFromJsonFile(const std::string& filename,
                                         std::vector<LoopClosureEdge>* edges) {
  nlohmann::json j;
  std::ifstream i(filename);
  i >> j;
  for (const auto& edge_j : j) {
    edges->push_back(LoopClosureEdge::fromJson(edge_j));
  }

}

#endif //CUSTOM_MAP_OPTIMIZATION_PLUGIN_UTILS_H
