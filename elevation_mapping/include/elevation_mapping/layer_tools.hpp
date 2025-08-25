#pragma once
#include <grid_map_core/grid_map_core.hpp>
#include <vector>
#include <string>

namespace em::layers {

// ===== ya existentes =====
void addSlope(grid_map::GridMap& map, double h);
void addRoughness(grid_map::GridMap& map, double window_m);
void addStep(grid_map::GridMap& map, double window_m);
void addObstacleBinaryFromGeom(grid_map::GridMap& map,
                               double slope_thresh_rad,
                               double step_thresh_m);
void addTraversabilityFromSlopeRough(grid_map::GridMap& map,
                                     double slope_max_rad,
                                     double rough_max_m,
                                     double w_slope,
                                     double w_rough);
void addCvarTraversability(grid_map::GridMap& map,
                           double alpha,
                           double window_m);

// Versión corta (la que ya tenías)

void addNegativeObstacles(grid_map::GridMap& map,
                          double drop_thresh_m,
                          double ring_m);

// NUEVA: versión “robusta” con más parámetros y nombre distinto
void addNegativeObstaclesRobust(grid_map::GridMap& map,
                                double drop_thresh_m,
                                double ring_m,
                                double min_valid_ratio,
                                double slope_gate_rad,
                                const std::string& elevation_layer,
                                const std::string& slope_layer,
                                const std::string& out_layer);

// Combinar capas en una final
void combineToFinalObstacles(grid_map::GridMap& map,
                             const std::string& obstacles_layer,
                             const std::string& negatives_layer,
                             const std::string& cvar_layer,
                             double cvar_tau,
                             const std::string& out_layer);

} // namespace em::layers
