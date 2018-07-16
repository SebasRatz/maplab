
#ifndef CUSTOM_MAP_OPTIMIZATION_PLUGIN_CUSTOM_MAP_OPTIMIZATION_PLUGIN_H
#define CUSTOM_MAP_OPTIMIZATION_PLUGIN_CUSTOM_MAP_OPTIMIZATION_PLUGIN_H

#include <console-common/console-plugin-base-with-plotter.h>
#include <console-common/console.h>
#include <loop-closure-handler/loop-detector-node.h>
#include <map-optimization/augment-loopclosure.h>
#include <map-optimization/callbacks.h>
#include <map-optimization/outlier-rejection-solver.h>
#include <map-optimization/outlier-rejection-solver.h>
#include <map-optimization/solver-options.h>
#include <map-optimization/solver.h>
#include <map-optimization/vi-map-relaxation.h>
#include <map-optimization/vi-optimization-builder.h>
#include <map-optimization/vi-optimization-builder.h>
#include <maplab-common/file-logger.h>
#include <maplab-common/progress-bar.h>
#include <vi-map/vi-map.h>
#include <vi-map/vi-map.h>
#include "custom-map-optimization-plugin/utils.h"

#include "map-optimization/vi-map-optimizer.h"
enum SearchResult { FAILED, FOUND, FOUND_CLOSEST_ONE };
class CustomMapOptimizationPlugin
    : public common::ConsolePluginBaseWithPlotter {
 public:
  CustomMapOptimizationPlugin(
      common::Console* console, visualization::ViwlsGraphRvizPlotter* plotter);
  std::string getPluginId() const override;

 private:
  static constexpr bool kSignalHandlerEnabled = true;

  void relaxMap(vi_map::VIMap* map);
  bool addLoopClosureEdgesToMap(
      vi_map::VIMap* map, const LoopClosureEdges& edges);

  SearchResult search(
      long find_timestamp, const pose_graph::VertexIdList& vertice_ids,
      const vi_map::VIMap& map, pose_graph::VertexId* result);
  SearchResult findVertexIdByTimeStamp(
      const vi_map::VIMap& map, long timestamp,
      pose_graph::VertexId* result);
  bool binarySearch(
      long find_timestamp, const pose_graph::VertexIdList& vertice_ids,
      const vi_map::VIMap& map, size_t begin, size_t end,
      pose_graph::VertexId* result, long tolerance);
  float getPoseError(
      const pose_graph::VertexId& vertex_id, const vi_map::VIMap& map,
      const Camera& camera);

  /*
   * S source map, T target map, M map. 2 is the one stored in the vertex
   */
  void adaptTransformation(
      const Transformation& T_M_S, const Transformation& T_M_T,
      const Transformation& T_M_S2, const Transformation& T_M_T2,
      const Transformation& T_S_T, Transformation* T_S2_T2);
};
#endif  // CUSTOM_MAP_OPTIMIZATION_PLUGIN_CUSTOM_MAP_OPTIMIZATION_PLUGIN_H
