#include "custom-map-optimization-plugin/custom_map_optimization_plugin.h"
#include <console-common/console-plugin-base.h>
#include <ctime>
#include <iostream>
#include <map-manager/map-manager.h>
#include <string>

#include "custom-map-optimization-plugin/utils.h"
// Your new plugin needs to derive from ConsolePluginBase.
// (Alternatively, you can derive from ConsolePluginBaseWithPlotter if you need
// RViz plotting abilities for your VI map.)
// Every plugin needs to implement a getPluginId function which returns a
// string that gives each plugin a unique name.
std::string CustomMapOptimizationPlugin::getPluginId() const {
  return "custom_map_optimization_plugin";
}

// The constructor takes a pointer to the Console object which we can forward
// to the constructor of ConsolePluginBase.
CustomMapOptimizationPlugin::CustomMapOptimizationPlugin(
    common::Console* console, visualization::ViwlsGraphRvizPlotter* plotter)
    : common::ConsolePluginBaseWithPlotter(console, plotter) {
addCommand(
      {"relax_external"},

      [this]() -> int {
        // Get the currently selected map.
        std::string selected_map_key;

        // This function will write the name of the selected map key into
        // selected_map_key. The function will return false and print an error
        // message if no map key is selected.
        if (!getSelectedMapKeyIfSet(&selected_map_key)) {
          return common::kStupidUserError;
        }

        // Create a map manager instance.
        vi_map::VIMapManager map_manager;

        // Get and lock the map which blocks all other access to the map.
        vi_map::VIMapManager::MapWriteAccess map =
            map_manager.getMapWriteAccess(selected_map_key);

        // Now run your algorithm on the VI map.
        // E.g., we can get the number of missions and print it.
        std::string filename =
            "/home/sebastian/semester_thesis/loop_closures.json";
        LoopClosureEdges edges;
        loopClosureEdgesFromJsonFile(filename, &edges);
        bool success = addLoopClosureEdgesToMap(map.get(), edges);
        if (success) {
          relaxMap(map.get());
          return common::kSuccess;
        }
        return common::kUnknownError;
      },

      "This command will run pose graph relaxation with the provided loop "
      "closure constraints.",
      common::Processing::Sync);
}

bool CustomMapOptimizationPlugin::addLoopClosureEdgesToMap(
    vi_map::VIMap* map, const LoopClosureEdges& edges) {
  constexpr long kTolerance = 1000000000000; // max 1 micro sec
  bool added_loop_constraint = false;
  for (const auto& edge : edges) {
    clock_t start = clock();
    pose_graph::VertexId id_from;
    // find vertex camera from
    const bool success_from = findVertexIdByTimeStamp(
        *map, edge.from.timestamp_ns, kTolerance, &id_from);
    clock_t stop = clock();
    LOG(INFO) << "Search took " << ((float)(stop - start)) / CLOCKS_PER_SEC << "s";

    start = clock();
    pose_graph::VertexId id_to;
    const bool success_to =
        // find vertex camera to
        findVertexIdByTimeStamp(*map, edge.to.timestamp_ns, kTolerance, &id_to);
    stop = clock();
    LOG(INFO) << "Search took " << ((float)(stop - start)) / CLOCKS_PER_SEC << "s";

    if (success_from && success_to) {
      float error_from = getPoseError(id_from, *map, edge.from);
      float error_to = getPoseError(id_to, *map, edge.to);

      LOG(INFO) << "error from " << error_from;
      LOG(INFO) << "error to " << error_to;
      if (error_to > 0.01 || error_from > 0.01) {
        LOG(ERROR) << "Timestamps found, but corresponding poses are bad!";
        return false;
      }
      LOG(INFO) << "Search was successful! Id_from " << id_from << "ID_to "
                << id_to;

      // add loop closure edge to map
      pose_graph::EdgeId edge_id;
      common::generateId(&edge_id);
      CHECK(edge_id.isValid());

      constexpr double kSwitchVariable = 0.5;
      constexpr double kSwitchVariableVariance = 0.5;
      Eigen::Matrix<double, 6, 6> covariance;
      covariance.setIdentity();
      vi_map::Edge::UniquePtr loop_closure_edge(
          new vi_map::LoopClosureEdge(
              edge_id, id_from, id_to, kSwitchVariable, kSwitchVariableVariance,
              edge.T_from_to, covariance));
      map->addEdge(std::move(loop_closure_edge));
      added_loop_constraint = true;
    } else {
      LOG(ERROR) << "FAIL: Search failed. Timestamp not found!!!";
    }
  }
  return added_loop_constraint;
}

bool CustomMapOptimizationPlugin::findVertexIdByTimeStamp(
    const vi_map::VIMap& map, long timestamp, long tolerance,
    pose_graph::VertexId* result) {
  pose_graph::VertexIdList ids;
  map.getAllVertexIdsAlongGraphsSortedByTimestamp(&ids);

  bool success =
      //binarySearch(timestamp, ids, map, 0, ids.size() - 1, result, tolerance);
      search(timestamp, ids, map, result, tolerance);
  return success;
}

bool CustomMapOptimizationPlugin::search(long find_timestamp,
                                         const pose_graph::VertexIdList &vertice_ids,
                                         const vi_map::VIMap &map,
                                         pose_graph::VertexId *result,
                                         long tolerance) {
  long min_difference = -1;
  pose_graph::VertexId best_vertex_id;
  bool found_timestamp = false;

  LOG(INFO) << "searching for " << find_timestamp;
  for (const auto& vertex_id : vertice_ids) {
    const vi_map::Vertex& vertex = map.getVertex(vertex_id);

    const size_t num_frames = vertex.numFrames();
    for (size_t frame_i = 0u; frame_i < num_frames; ++frame_i) {
      const aslam::VisualFrame& frame = vertex.getVisualFrame(frame_i);
      long difference = labs(find_timestamp - frame.getTimestampNanoseconds());

      if (min_difference < 0 || difference < min_difference) {
        min_difference = difference;
        best_vertex_id = vertex_id;
        if (difference < tolerance) {
          found_timestamp = true;
        }
      }
    }
  }
  if (found_timestamp) {
    LOG(INFO) << "Found timestamp. Difference: " << min_difference;
    *result = best_vertex_id;
  }
  else {
    LOG(INFO) << "Did not find timestamp. Minimal difference found: " << min_difference;
  }
  return found_timestamp;
}

bool CustomMapOptimizationPlugin::binarySearch(
    long find_timestamp, const pose_graph::VertexIdList& vertice_ids,
    const vi_map::VIMap& map, size_t begin, size_t end,
    pose_graph::VertexId* result, long tolerance) {
  LOG(INFO) << "find" << find_timestamp;

  size_t middle = begin + (end - begin) / 2;

  const long timestamp_check = map.getVertex(vertice_ids[middle]).getMinTimestampNanoseconds();
  long distance = timestamp_check - find_timestamp;
  LOG(INFO) << "check" << timestamp_check;

  if (std::abs(distance) < tolerance) {
    LOG(INFO) << "Found timestamp. Distance: " << distance;
    *result = vertice_ids[middle];
    return true;
  }
  if (middle == begin) {
    LOG(INFO) << "DID NOT FIND";
    return false;
  }
  if (distance > 0) {
    return binarySearch(
        find_timestamp, vertice_ids, map, begin, middle, result, tolerance);
  } else {
    return binarySearch(
        find_timestamp, vertice_ids, map, middle, end, result, tolerance);
  }
}

void CustomMapOptimizationPlugin::relaxMap(vi_map::VIMap* map) {

  map_optimization::VIMapRelaxation relaxation(plotter_, kSignalHandlerEnabled);
  vi_map::MissionIdList mission_id_list;
  map->getAllMissionIds(&mission_id_list);

  vi_map::MissionIdSet mission_ids(mission_id_list.begin(), mission_id_list.end());
  ceres::Solver::Options solver_options =

      map_optimization::initSolverOptionsFromFlags();
  relaxation.solveRelaxation(solver_options, mission_ids, map);
}

float
CustomMapOptimizationPlugin::getPoseError(const pose_graph::VertexId &vertex_id,
                                          const vi_map::VIMap &map,
                                          const Camera& camera) {
  const vi_map::Vertex& vertex = map.getVertex(vertex_id);
  const Eigen::Vector3d position = vertex.get_p_M_I();

  LOG(INFO) << "position vertex " << std::endl << position;
  LOG(INFO) << "position camera " << std::endl << camera.pose.getPosition();

  return (position - camera.pose.getPosition()).norm();
}


// Finally, call the MAPLAB_CREATE_CONSOLE_PLUGIN macro to create your console
// plugin.
MAPLAB_CREATE_CONSOLE_PLUGIN_WITH_PLOTTER(CustomMapOptimizationPlugin);
