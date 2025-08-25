
// // #include "elevation_mapping/layer_tools.hpp"
// // #include <grid_map_core/iterators/GridMapIterator.hpp>
// // #include <cmath>
// // #include <algorithm>
// // #include <vector>
// // #include <limits>

// // using grid_map::Index;
// // using grid_map::Matrix;

// // namespace {

// // // --- utilidades ---
// // inline bool isFiniteF(float v) { return std::isfinite(static_cast<double>(v)); }

// // inline float NaNf() { return std::numeric_limits<float>::quiet_NaN(); }

// // inline int metersToCells(const grid_map::GridMap& map, double m)
// // {
// //   const double res = map.getResolution();
// //   int cells = static_cast<int>(std::round(m / std::max(1e-9, res)));
// //   return std::max(1, cells);
// // }

// // inline bool insideIndex(const grid_map::GridMap& map, const Index& i)
// // {
// //   const auto& sz = map.getSize();
// //   return (i(0) >= 0 && i(1) >= 0 && i(0) < sz(0) && i(1) < sz(1));
// // }

// // } // anon

// // // --------------------------- SLOPE (rad) ---------------------------
// // void em::layers::addSlope(grid_map::GridMap& map, double h)
// // {
// //   if (!map.exists("elevation")) return;
// //   const Matrix& Z = map["elevation"];
// //   Matrix slope(Z.rows(), Z.cols());
// //   const double dx = std::max(1e-9, h);
// //   const double dy = std::max(1e-9, h);

// //   for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
// //     Index idx(*it);
// //     const float zc = Z(idx(0), idx(1));
// //     if (!isFiniteF(zc)) { slope(idx(0), idx(1)) = NaNf(); continue; }

// //     Index nx = idx; nx(0)++;  Index px = idx; px(0)--;
// //     Index ny = idx; ny(1)++;  Index py = idx; py(1)--;

// //     auto val = [&](const Index& i)->float {
// //       if (!insideIndex(map, i)) return zc;
// //       const float v = Z(i(0), i(1));
// //       return isFiniteF(v) ? v : zc;
// //     };

// //     const double dzdx = (val(nx) - val(px)) / (2.0 * dx);
// //     const double dzdy = (val(ny) - val(py)) / (2.0 * dy);
// //     const double grad = std::hypot(dzdx, dzdy);

// //     slope(idx(0), idx(1)) = static_cast<float>(std::atan(grad)); // rad
// //   }
// //   if (map.exists("slope")) map["slope"] = slope;
// //   else map.add("slope", slope);
// // }

// // // --------------------------- ROUGHNESS (m) ---------------------------
// // void em::layers::addRoughness(grid_map::GridMap& map, double window_m)
// // {
// //   if (!map.exists("elevation")) return;
// //   const Matrix& Z = map["elevation"];
// //   Matrix R(Z.rows(), Z.cols());

// //   const int win = metersToCells(map, window_m);
// //   const int half = std::max(1, win/2);

// //   for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
// //     Index c(*it);
// //     const float zc = Z(c(0), c(1));
// //     if (!isFiniteF(zc)) { R(c(0), c(1)) = NaNf(); continue; }

// //     double sum=0.0, sum2=0.0; int n=0;
// //     for (int di=-half; di<=half; ++di) {
// //       for (int dj=-half; dj<=half; ++dj) {
// //         Index p(c(0)+di, c(1)+dj);
// //         if (!insideIndex(map, p)) continue;
// //         const float z = Z(p(0), p(1));
// //         if (!isFiniteF(z)) continue;
// //         sum += z; sum2 += double(z)*double(z); ++n;
// //       }
// //     }
// //     if (n<3) { R(c(0), c(1)) = NaNf(); continue; }
// //     const double mean = sum / n;
// //     const double var  = std::max(0.0, (sum2 / n) - mean*mean);
// //     R(c(0), c(1)) = static_cast<float>(std::sqrt(var));
// //   }
// //   if (map.exists("roughness")) map["roughness"] = R;
// //   else map.add("roughness", R);
// // }

// // // --------------------------- STEP (m) ---------------------------
// // void em::layers::addStep(grid_map::GridMap& map, double window_m)
// // {
// //   if (!map.exists("elevation")) return;
// //   const Matrix& Z = map["elevation"];
// //   Matrix S(Z.rows(), Z.cols());

// //   const int win = metersToCells(map, window_m);
// //   const int half = std::max(1, win/2);

// //   for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
// //     Index c(*it);
// //     const float zc = Z(c(0), c(1));
// //     if (!isFiniteF(zc)) { S(c(0), c(1)) = NaNf(); continue; }

// //     double sum=0.0; int n=0;
// //     for (int di=-half; di<=half; ++di) {
// //       for (int dj=-half; dj<=half; ++dj) {
// //         if (di==0 && dj==0) continue;
// //         Index p(c(0)+di, c(1)+dj);
// //         if (!insideIndex(map, p)) continue;
// //         const float z = Z(p(0), p(1));
// //         if (!isFiniteF(z)) continue;
// //         sum += z; ++n;
// //       }
// //     }
// //     if (n<3) { S(c(0), c(1)) = NaNf(); continue; }
// //     const double mean = sum / n;
// //     S(c(0), c(1)) = static_cast<float>(std::fabs(double(zc) - mean));
// //   }
// //   if (map.exists("step")) map["step"] = S;
// //   else map.add("step", S);
// // }

// // // --------------------------- OBSTÁCULOS (0/1) ---------------------------
// // void em::layers::addObstacleBinaryFromGeom(grid_map::GridMap& map,
// //                                            double slope_thresh_rad,
// //                                            double step_thresh_m)
// // {
// //   if (!map.exists("slope") || !map.exists("step")) return;
// //   const Matrix& S = map["slope"];
// //   const Matrix& H = map["step"];
// //   Matrix O(S.rows(), S.cols());

// //   for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
// //     Index idx(*it);
// //     const float s = S(idx(0), idx(1));
// //     const float h = H(idx(0), idx(1));
// //     const bool s_bad = isFiniteF(s) && s >= slope_thresh_rad;
// //     const bool h_bad = isFiniteF(h) && h >= step_thresh_m;
// //     O(idx(0), idx(1)) = (s_bad || h_bad) ? 1.0f : 0.0f;
// //   }
// //   if (map.exists("obstacles")) map["obstacles"] = O;
// //   else map.add("obstacles", O);
// // }

// // // --------------------------- TRAVERSABILIDAD [0..1] ---------------------------
// // void em::layers::addTraversabilityFromSlopeRough(grid_map::GridMap& map,
// //                                                  double slope_max_rad,
// //                                                  double rough_max_m,
// //                                                  double w_slope,
// //                                                  double w_rough)
// // {
// //   if (!map.exists("slope") || !map.exists("roughness")) return;
// //   const Matrix& S = map["slope"];
// //   const Matrix& R = map["roughness"];
// //   Matrix T(S.rows(), S.cols());

// //   const double ws = std::clamp(w_slope, 0.0, 1.0);
// //   const double wr = std::clamp(w_rough, 0.0, 1.0);
// //   const double wsum = std::max(1e-6, ws + wr);
// //   const double smax = std::max(1e-6, slope_max_rad);
// //   const double rmax = std::max(1e-6, rough_max_m);

// //   for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
// //     Index idx(*it);
// //     const float s = S(idx(0), idx(1));
// //     const float r = R(idx(0), idx(1));
// //     if (!isFiniteF(s) || !isFiniteF(r)) { T(idx(0), idx(1)) = NaNf(); continue; }

// //     const double ns = 1.0 - std::clamp(double(s)/smax, 0.0, 1.0);
// //     const double nr = 1.0 - std::clamp(double(r)/rmax, 0.0, 1.0);
// //     const double t  = (ws*ns + wr*nr) / wsum;

// //     T(idx(0), idx(1)) = static_cast<float>(std::clamp(t, 0.0, 1.0));
// //   }
// //   if (map.exists("trav")) map["trav"] = T;
// //   else map.add("trav", T);
// // }

// // // --------------------------- CVaR TRAV ---------------------------
// // void em::layers::addCvarTraversability(grid_map::GridMap& map,
// //                                        double alpha,
// //                                        double window_m)
// // {
// //   if (!map.exists("trav")) return;
// //   const Matrix& TR = map["trav"];
// //   Matrix CV(TR.rows(), TR.cols());

// //   const int win  = metersToCells(map, window_m);
// //   const int half = std::max(1, win/2);
// //   const double a = std::clamp(alpha, 0.0, 0.999);

// //   for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
// //     Index c(*it);
// //     const float tc = TR(c(0), c(1));
// //     if (!isFiniteF(tc)) { CV(c(0), c(1)) = NaNf(); continue; }

// //     std::vector<double> risks;
// //     risks.reserve((2*half+1)*(2*half+1));

// //     for (int di=-half; di<=half; ++di) {
// //       for (int dj=-half; dj<=half; ++dj) {
// //         Index p(c(0)+di, c(1)+dj);
// //         if (!insideIndex(map, p)) continue;
// //         const float t = TR(p(0), p(1));
// //         if (!isFiniteF(t)) continue;
// //         risks.push_back(1.0 - double(t)); // riesgo = 1 - trav
// //       }
// //     }
// //     if (risks.size() < 5) { CV(c(0), c(1)) = NaNf(); continue; }

// //     std::sort(risks.begin(), risks.end()); // asc
// //     const size_t n = risks.size();
// //     const size_t q = std::min(n-1, static_cast<size_t>(std::ceil(a * (n - 1))));
// //     const double thresh = risks[q];

// //     double sumTail=0.0; size_t m=0;
// //     for (size_t k=q; k<n; ++k) { sumTail += risks[k]; ++m; }
// //     const double cvar_risk = (m>0) ? (sumTail / double(m)) : thresh;

// //     const double cvar_trav = std::clamp(1.0 - cvar_risk, 0.0, 1.0);
// //     CV(c(0), c(1)) = static_cast<float>(cvar_trav);
// //   }
// //   if (map.exists("cvar_trav")) map["cvar_trav"] = CV;
// //   else map.add("cvar_trav", CV);
// // }

// // // --------------------------- NEGATIVE OBSTACLES (0/1) ---------------------------
// // void addNegativeObstaclesRobust(grid_map::GridMap& map,
// //                                 double drop_thresh_m,
// //                                 double ring_m,
// //                                 double /*min_valid_ratio*/,
// //                                 double /*slope_gate_rad*/,
// //                                 const std::string& /*elevation_layer*/,
// //                                 const std::string& /*slope_layer*/,
// //                                 const std::string& out_layer)
// // {
// //   // Reutiliza tu versión corta existente (3 args), que crea/llena "negatives".
// //   em::layers::addNegativeObstacles(map, drop_thresh_m, ring_m);

// //   const std::string src = "negatives";
// //   if (!map.exists(src)) return;

// //   if (!map.exists(out_layer)) map.add(out_layer, 0.0f);
// //   map[out_layer] = map[src];
// // }

// // void combineToFinalObstacles(grid_map::GridMap& map,
// //                              const std::string& obstacles_layer,
// //                              const std::string& negatives_layer,
// //                              const std::string& cvar_layer,
// //                              double cvar_tau,
// //                              const std::string& out_layer)
// // {
// //   const bool have_obst = map.exists(obstacles_layer);
// //   const bool have_neg  = map.exists(negatives_layer);
// //   const bool have_cvar = map.exists(cvar_layer);

// //   if (!map.exists(out_layer)) map.add(out_layer, 0.0f);
// //   auto& outM = map[out_layer];

// //   const int nx = map.getSize()(0);
// //   const int ny = map.getSize()(1);

// //   const grid_map::Matrix* obst = have_obst ? &map[obstacles_layer] : nullptr;
// //   const grid_map::Matrix* neg  = have_neg  ? &map[negatives_layer]  : nullptr;
// //   const grid_map::Matrix* cvar = have_cvar ? &map[cvar_layer]       : nullptr;

// //   outM.setZero(nx, ny);

// //   for (int i = 0; i < nx; ++i) {
// //     for (int j = 0; j < ny; ++j) {
// //       float v = 0.0f;
// //       if (obst) v = std::max(v, (*obst)(i,j));
// //       if (neg)  v = std::max(v, (*neg)(i,j));
// //       if (cvar) {
// //         const float gate = ((*cvar)(i,j) > static_cast<float>(cvar_tau)) ? 1.0f : 0.0f;
// //         v = std::max(v, gate);
// //       }
// //       outM(i,j) = v;
// //     }
// //   }
// // }

// // #include "elevation_mapping/layer_tools.hpp"
// // #include <grid_map_core/iterators/GridMapIterator.hpp>
// // #include <cmath>
// // #include <algorithm>
// // #include <vector>
// // #include <limits>

// // using grid_map::Index;
// // using grid_map::Matrix;

// // namespace {  // Utilidades internas

// // inline bool isFiniteF(float v) { return std::isfinite(static_cast<double>(v)); }

// // inline float NaNf() { return std::numeric_limits<float>::quiet_NaN(); }

// // inline int metersToCells(const grid_map::GridMap& map, double m)
// // {
// //   const double res = map.getResolution();
// //   int cells = static_cast<int>(std::round(m / std::max(1e-9, res)));
// //   return std::max(1, cells);
// // }

// // inline bool insideIndex(const grid_map::GridMap& map, const Index& i)
// // {
// //   const auto& sz = map.getSize();
// //   return (i(0) >= 0 && i(1) >= 0 && i(0) < sz(0) && i(1) < sz(1));
// // }

// // } // anon

// // // ============================================================================
// // // ===============================  em::layers  ================================
// // // ============================================================================

// // namespace em::layers {

// // // --------------------------- SLOPE (rad) ---------------------------
// // void addSlope(grid_map::GridMap& map, double h)
// // {
// //   if (!map.exists("elevation")) return;
// //   const Matrix& Z = map["elevation"];
// //   Matrix slope(Z.rows(), Z.cols());
// //   const double dx = std::max(1e-9, h);
// //   const double dy = std::max(1e-9, h);

// //   for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
// //     Index idx(*it);
// //     const float zc = Z(idx(0), idx(1));
// //     if (!isFiniteF(zc)) { slope(idx(0), idx(1)) = NaNf(); continue; }

// //     Index nx = idx; nx(0)++;  Index px = idx; px(0)--;
// //     Index ny = idx; ny(1)++;  Index py = idx; py(1)--;

// //     auto val = [&](const Index& i)->float {
// //       if (!insideIndex(map, i)) return zc;
// //       const float v = Z(i(0), i(1));
// //       return isFiniteF(v) ? v : zc;
// //     };

// //     const double dzdx = (val(nx) - val(px)) / (2.0 * dx);
// //     const double dzdy = (val(ny) - val(py)) / (2.0 * dy);
// //     const double grad = std::hypot(dzdx, dzdy);

// //     slope(idx(0), idx(1)) = static_cast<float>(std::atan(grad)); // rad
// //   }
// //   if (map.exists("slope")) map["slope"] = slope;
// //   else map.add("slope", slope);
// // }

// // // --------------------------- ROUGHNESS (m) ---------------------------
// // void addRoughness(grid_map::GridMap& map, double window_m)
// // {
// //   if (!map.exists("elevation")) return;
// //   const Matrix& Z = map["elevation"];
// //   Matrix R(Z.rows(), Z.cols());

// //   const int win = metersToCells(map, window_m);
// //   const int half = std::max(1, win/2);

// //   for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
// //     Index c(*it);
// //     const float zc = Z(c(0), c(1));
// //     if (!isFiniteF(zc)) { R(c(0), c(1)) = NaNf(); continue; }

// //     double sum=0.0, sum2=0.0; int n=0;
// //     for (int di=-half; di<=half; ++di) {
// //       for (int dj=-half; dj<=half; ++dj) {
// //         Index p(c(0)+di, c(1)+dj);
// //         if (!insideIndex(map, p)) continue;
// //         const float z = Z(p(0), p(1));
// //         if (!isFiniteF(z)) continue;
// //         sum += z; sum2 += double(z)*double(z); ++n;
// //       }
// //     }
// //     if (n<3) { R(c(0), c(1)) = NaNf(); continue; }
// //     const double mean = sum / n;
// //     const double var  = std::max(0.0, (sum2 / n) - mean*mean);
// //     R(c(0), c(1)) = static_cast<float>(std::sqrt(var));
// //   }
// //   if (map.exists("roughness")) map["roughness"] = R;
// //   else map.add("roughness", R);
// // }

// // // --------------------------- STEP (m) ---------------------------
// // void addStep(grid_map::GridMap& map, double window_m)
// // {
// //   if (!map.exists("elevation")) return;
// //   const Matrix& Z = map["elevation"];
// //   Matrix S(Z.rows(), Z.cols());

// //   const int win = metersToCells(map, window_m);
// //   const int half = std::max(1, win/2);

// //   for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
// //     Index c(*it);
// //     const float zc = Z(c(0), c(1));
// //     if (!isFiniteF(zc)) { S(c(0), c(1)) = NaNf(); continue; }

// //     double sum=0.0; int n=0;
// //     for (int di=-half; di<=half; ++di) {
// //       for (int dj=-half; dj<=half; ++dj) {
// //         if (di==0 && dj==0) continue;
// //         Index p(c(0)+di, c(1)+dj);
// //         if (!insideIndex(map, p)) continue;
// //         const float z = Z(p(0), p(1));
// //         if (!isFiniteF(z)) continue;
// //         sum += z; ++n;
// //       }
// //     }
// //     if (n<3) { S(c(0), c(1)) = NaNf(); continue; }
// //     const double mean = sum / n;
// //     S(c(0), c(1)) = static_cast<float>(std::fabs(double(zc) - mean));
// //   }
// //   if (map.exists("step")) map["step"] = S;
// //   else map.add("step", S);
// // }

// // // --------------------------- OBSTÁCULOS (0/1) ---------------------------
// // void addObstacleBinaryFromGeom(grid_map::GridMap& map,
// //                                double slope_thresh_rad,
// //                                double step_thresh_m)
// // {
// //   if (!map.exists("slope") || !map.exists("step")) return;
// //   const Matrix& S = map["slope"];
// //   const Matrix& H = map["step"];
// //   Matrix O(S.rows(), S.cols());

// //   for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
// //     Index idx(*it);
// //     const float s = S(idx(0), idx(1));
// //     const float h = H(idx(0), idx(1));
// //     const bool s_bad = isFiniteF(s) && s >= slope_thresh_rad;
// //     const bool h_bad = isFiniteF(h) && h >= step_thresh_m;
// //     O(idx(0), idx(1)) = (s_bad || h_bad) ? 1.0f : 0.0f;
// //   }
// //   if (map.exists("obstacles")) map["obstacles"] = O;
// //   else map.add("obstacles", O);
// // }

// // // --------------------------- TRAVERSABILIDAD [0..1] ---------------------------
// // void addTraversabilityFromSlopeRough(grid_map::GridMap& map,
// //                                      double slope_max_rad,
// //                                      double rough_max_m,
// //                                      double w_slope,
// //                                      double w_rough)
// // {
// //   if (!map.exists("slope") || !map.exists("roughness")) return;
// //   const Matrix& S = map["slope"];
// //   const Matrix& R = map["roughness"];
// //   Matrix T(S.rows(), S.cols());

// //   const double ws = std::clamp(w_slope, 0.0, 1.0);
// //   const double wr = std::clamp(w_rough, 0.0, 1.0);
// //   const double wsum = std::max(1e-6, ws + wr);
// //   const double smax = std::max(1e-6, slope_max_rad);
// //   const double rmax = std::max(1e-6, rough_max_m);

// //   for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
// //     Index idx(*it);
// //     const float s = S(idx(0), idx(1));
// //     const float r = R(idx(0), idx(1));
// //     if (!isFiniteF(s) || !isFiniteF(r)) { T(idx(0), idx(1)) = NaNf(); continue; }

// //     const double ns = 1.0 - std::clamp(double(s)/smax, 0.0, 1.0);
// //     const double nr = 1.0 - std::clamp(double(r)/rmax, 0.0, 1.0);
// //     const double t  = (ws*ns + wr*nr) / wsum;

// //     T(idx(0), idx(1)) = static_cast<float>(std::clamp(t, 0.0, 1.0));
// //   }
// //   if (map.exists("trav")) map["trav"] = T;
// //   else map.add("trav", T);
// // }

// // // --------------------------- CVaR TRAV + RISK ---------------------------
// // void addCvarTraversability(grid_map::GridMap& map,
// //                            double alpha,
// //                            double window_m)
// // {
// //   if (!map.exists("trav")) return;
// //   const Matrix& TR = map["trav"];
// //   Matrix CV(TR.rows(), TR.cols());

// //   const int win  = metersToCells(map, window_m);
// //   const int half = std::max(1, win/2);
// //   const double a = std::clamp(alpha, 0.0, 0.999);

// //   for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
// //     Index c(*it);
// //     const float tc = TR(c(0), c(1));
// //     if (!isFiniteF(tc)) { CV(c(0), c(1)) = NaNf(); continue; }

// //     std::vector<double> risks;
// //     risks.reserve((2*half+1)*(2*half+1));

// //     for (int di=-half; di<=half; ++di) {
// //       for (int dj=-half; dj<=half; ++dj) {
// //         Index p(c(0)+di, c(1)+dj);
// //         if (!insideIndex(map, p)) continue;
// //         const float t = TR(p(0), p(1));
// //         if (!isFiniteF(t)) continue;
// //         risks.push_back(1.0 - double(t)); // riesgo = 1 - trav
// //       }
// //     }
// //     if (risks.size() < 5) { CV(c(0), c(1)) = NaNf(); continue; }

// //     std::sort(risks.begin(), risks.end()); // asc
// //     const size_t n = risks.size();
// //     const size_t q = std::min(n-1, static_cast<size_t>(std::ceil(a * (n - 1))));
// //     const double thresh = risks[q];

// //     double sumTail=0.0; size_t m=0;
// //     for (size_t k=q; k<n; ++k) { sumTail += risks[k]; ++m; }
// //     const double cvar_risk = (m>0) ? (sumTail / double(m)) : thresh;

// //     const double cvar_trav = std::clamp(1.0 - cvar_risk, 0.0, 1.0);
// //     CV(c(0), c(1)) = static_cast<float>(cvar_trav);
// //   }

// //   // Guarda ambas capas para compatibilidad con tu fusión:
// //   //  - "cvar_trav": alto = bueno
// //   //  - "cvar_risk": alto = malo = 1 - cvar_trav
// //   // if (map.exists("cvar_trav")) map["cvar_trav"] = CV;
// //   // else map.add("cvar_trav", CV);
// //   if (map.exists("cvar_risk")) map["cvar_risk"] = CV;
// //   else map.add("cvar_risk", CV);
// //   Matrix RISK = Matrix::Constant(CV.rows(), CV.cols(), NaNf());
// //   for (int i=0; i<CV.rows(); ++i) {
// //     for (int j=0; j<CV.cols(); ++j) {
// //       const float v = CV(i,j);
// //       RISK(i,j) = isFiniteF(v) ? (1.0f - v) : NaNf();
// //     }
// //   }
// //   if (map.exists("cvar_risk")) map["cvar_risk"] = RISK;
// //   else map.add("cvar_risk", RISK);
// // }

// // // --------------------------- NEGATIVE OBSTACLES (3 args) ---------------------------
// // void addNegativeObstacles(grid_map::GridMap& map,
// //                           double drop_thresh_m,
// //                           double ring_m)
// // {
// //   if (!map.exists("elevation")) return;
// //   const Matrix& Z = map["elevation"];
// //   Matrix NEG(Z.rows(), Z.cols());
// //   NEG.setZero();

// //   const int ring = metersToCells(map, ring_m);
// //   const int rin  = std::max(1, ring / 2);
// //   const int rout = std::max(rin + 1, ring);

// //   for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
// //     Index c(*it);
// //     const float zc = Z(c(0), c(1));
// //     if (!isFiniteF(zc)) { NEG(c(0), c(1)) = 0.0f; continue; }

// //     double sum = 0.0; int n = 0;
// //     for (int di = -rout; di <= rout; ++di) {
// //       for (int dj = -rout; dj <= rout; ++dj) {
// //         const int rmax = std::max(std::abs(di), std::abs(dj));
// //         if (rmax < rin || rmax > rout) continue;   // anillo [rin, rout]
// //         Index p(c(0) + di, c(1) + dj);
// //         if (!insideIndex(map, p)) continue;
// //         const float z = Z(p(0), p(1));
// //         if (!isFiniteF(z)) continue;
// //         sum += z; ++n;
// //       }
// //     }
// //     if (n < 3) { NEG(c(0), c(1)) = 0.0f; continue; }

// //     const double mean_ring = sum / double(n);
// //     NEG(c(0), c(1)) = (mean_ring - double(zc) >= drop_thresh_m) ? 1.0f : 0.0f;
// //   }

// //   if (map.exists("negatives")) map["negatives"] = NEG;
// //   else map.add("negatives", NEG);
// // }

// // // --------------------------- NEGATIVE OBSTACLES (8 args) ---------------------------
// // void addNegativeObstaclesRobust(grid_map::GridMap& map,
// //                                 double drop_thresh_m,
// //                                 double ring_m,
// //                                 double /*min_valid_ratio*/,
// //                                 double /*slope_gate_rad*/,
// //                                 const std::string& /*elevation_layer*/,
// //                                 const std::string& /*slope_layer*/,
// //                                 const std::string& out_layer)
// // {
// //   // Para empezar: reutiliza la versión corta que escribe "negatives".
// //   em::layers::addNegativeObstacles(map, drop_thresh_m, ring_m);

// //   const std::string src = "negatives";
// //   if (!map.exists(src)) return;

// //   if (!map.exists(out_layer)) map.add(out_layer, 0.0f);
// //   map[out_layer] = map[src];
// // }

// // // --------------------------- COMBINAR A CAPA FINAL ---------------------------
// // void combineToFinalObstacles(grid_map::GridMap& map,
// //                              const std::string& obstacles_layer,
// //                              const std::string& negatives_layer,
// //                              const std::string& cvar_layer,
// //                              double cvar_tau,
// //                              const std::string& out_layer)
// // {
// //   const bool have_obst = map.exists(obstacles_layer);
// //   const bool have_neg  = map.exists(negatives_layer);
// //   const bool have_cvar = map.exists(cvar_layer);

// //   if (!map.exists(out_layer)) map.add(out_layer, 0.0f);
// //   auto& outM = map[out_layer];

// //   const int nx = map.getSize()(0);
// //   const int ny = map.getSize()(1);
// //   outM.setZero(nx, ny);

// //   const grid_map::Matrix* obst = have_obst ? &map[obstacles_layer] : nullptr;
// //   const grid_map::Matrix* neg  = have_neg  ? &map[negatives_layer]  : nullptr;
// //   const grid_map::Matrix* cvar = have_cvar ? &map[cvar_layer]       : nullptr;

// //   for (int i = 0; i < nx; ++i) {
// //     for (int j = 0; j < ny; ++j) {
// //       float v = 0.0f;
// //       if (obst) v = std::max(v, (*obst)(i, j));
// //       if (neg)  v = std::max(v, (*neg)(i, j));
// //       if (cvar) {
// //         // Aquí asumimos cvar_layer == "cvar_risk" (alto = malo)
// //         const float gate = ((*cvar)(i, j) > static_cast<float>(cvar_tau)) ? 1.0f : 0.0f;
// //         v = std::max(v, gate);
// //       }
// //       outM(i, j) = v;
// //     }
// //   }
// // }

// // } // namespace em::layers

// /////
// #include "elevation_mapping/layer_tools.hpp"

// #include <grid_map_core/iterators/GridMapIterator.hpp>
// #include <grid_map_core/GridMap.hpp>

// #include <cmath>
// #include <algorithm>
// #include <vector>
// #include <limits>
// #include <numeric>

// using grid_map::Index;
// using grid_map::Matrix;

// namespace { // utilidades internas

// inline bool isFiniteF(float v) { return std::isfinite(static_cast<double>(v)); }
// inline float NaNf() { return std::numeric_limits<float>::quiet_NaN(); }

// inline bool insideIndex(const grid_map::GridMap& map, const Index& i)
// {
//   const auto& sz = map.getSize();
//   return (i(0) >= 0 && i(1) >= 0 && i(0) < sz(0) && i(1) < sz(1));
// }

// inline int metersToCells(const grid_map::GridMap& map, double m)
// {
//   const double res = std::max(1e-9, map.getResolution());
//   int cells = static_cast<int>(std::round(m / res));
//   return std::max(1, cells);
// }

// } // namespace (anon)

// namespace em {
// namespace layers {

// // --------------------------- SLOPE (rad) ---------------------------
// void addSlope(grid_map::GridMap& map, double h)
// {
//   if (!map.exists("elevation")) return;
//   const Matrix& Z = map["elevation"];

//   Matrix slope(Z.rows(), Z.cols());
//   const double dx = std::max(1e-9, h);
//   const double dy = std::max(1e-9, h);

//   for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
//     Index idx(*it);
//     const float zc = Z(idx(0), idx(1));
//     if (!isFiniteF(zc)) { slope(idx(0), idx(1)) = NaNf(); continue; }

//     Index nx = idx; nx(0)++;  Index px = idx; px(0)--;
//     Index ny = idx; ny(1)++;  Index py = idx; py(1)--;

//     auto val = [&](const Index& i)->float {
//       if (!insideIndex(map, i)) return zc;
//       const float v = Z(i(0), i(1));
//       return isFiniteF(v) ? v : zc;
//     };

//     const double dzdx = (val(nx) - val(px)) / (2.0 * dx);
//     const double dzdy = (val(ny) - val(py)) / (2.0 * dy);
//     const double grad = std::hypot(dzdx, dzdy);

//     slope(idx(0), idx(1)) = static_cast<float>(std::atan(grad)); // rad
//   }

//   if (map.exists("slope")) map["slope"] = slope;
//   else map.add("slope", slope);
// }

// // --------------------------- ROUGHNESS (m) ---------------------------
// void addRoughness(grid_map::GridMap& map, double window_m)
// {
//   if (!map.exists("elevation")) return;
//   const Matrix& Z = map["elevation"];
//   Matrix R(Z.rows(), Z.cols());

//   const int win  = metersToCells(map, window_m);
//   const int half = std::max(1, win/2);

//   for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
//     Index c(*it);
//     const float zc = Z(c(0), c(1));
//     if (!isFiniteF(zc)) { R(c(0), c(1)) = NaNf(); continue; }

//     double sum=0.0, sum2=0.0; int n=0;
//     for (int di=-half; di<=half; ++di) {
//       for (int dj=-half; dj<=half; ++dj) {
//         Index p(c(0)+di, c(1)+dj);
//         if (!insideIndex(map, p)) continue;
//         const float z = Z(p(0), p(1));
//         if (!isFiniteF(z)) continue;
//         sum  += z;
//         sum2 += double(z)*double(z);
//         ++n;
//       }
//     }
//     if (n<3) { R(c(0), c(1)) = NaNf(); continue; }
//     const double mean = sum / n;
//     const double var  = std::max(0.0, (sum2 / n) - mean*mean);
//     R(c(0), c(1)) = static_cast<float>(std::sqrt(var));
//   }

//   if (map.exists("roughness")) map["roughness"] = R;
//   else map.add("roughness", R);
// }

// // --------------------------- STEP (m) ---------------------------
// void addStep(grid_map::GridMap& map, double window_m)
// {
//   if (!map.exists("elevation")) return;
//   const Matrix& Z = map["elevation"];
//   Matrix S(Z.rows(), Z.cols());

//   const int win  = metersToCells(map, window_m);
//   const int half = std::max(1, win/2);

//   for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
//     Index c(*it);
//     const float zc = Z(c(0), c(1));
//     if (!isFiniteF(zc)) { S(c(0), c(1)) = NaNf(); continue; }

//     double sum=0.0; int n=0;
//     for (int di=-half; di<=half; ++di) {
//       for (int dj=-half; dj<=half; ++dj) {
//         if (di==0 && dj==0) continue;
//         Index p(c(0)+di, c(1)+dj);
//         if (!insideIndex(map, p)) continue;
//         const float z = Z(p(0), p(1));
//         if (!isFiniteF(z)) continue;
//         sum += z; ++n;
//       }
//     }
//     if (n<3) { S(c(0), c(1)) = NaNf(); continue; }
//     const double mean = sum / n;
//     S(c(0), c(1)) = static_cast<float>(std::fabs(double(zc) - mean));
//   }

//   if (map.exists("step")) map["step"] = S;
//   else map.add("step", S);
// }

// // --------------------------- OBSTÁCULOS (binario) ---------------------------
// void addObstacleBinaryFromGeom(grid_map::GridMap& map,
//                                double slope_thresh_rad,
//                                double step_thresh_m)
// {
//   if (!map.exists("slope") || !map.exists("step")) return;
//   const Matrix& S = map["slope"];
//   const Matrix& H = map["step"];
//   Matrix O(S.rows(), S.cols());

//   for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
//     Index idx(*it);
//     const float s = S(idx(0), idx(1));
//     const float h = H(idx(0), idx(1));
//     const bool s_bad = isFiniteF(s) && s >= slope_thresh_rad;
//     const bool h_bad = isFiniteF(h) && h >= step_thresh_m;
//     O(idx(0), idx(1)) = (s_bad || h_bad) ? 1.0f : 0.0f;
//   }

//   if (map.exists("obstacles")) map["obstacles"] = O;
//   else map.add("obstacles", O);
// }

// // --------------------------- TRAVERSABILIDAD [0..1] ---------------------------
// void addTraversabilityFromSlopeRough(grid_map::GridMap& map,
//                                      double slope_max_rad,
//                                      double rough_max_m,
//                                      double w_slope,
//                                      double w_rough)
// {
//   if (!map.exists("slope") || !map.exists("roughness")) return;
//   const Matrix& S = map["slope"];
//   const Matrix& R = map["roughness"];
//   Matrix T(S.rows(), S.cols());

//   const double ws = std::clamp(w_slope, 0.0, 1.0);
//   const double wr = std::clamp(w_rough, 0.0, 1.0);
//   const double wsum = std::max(1e-6, ws + wr);
//   const double smax = std::max(1e-6, slope_max_rad);
//   const double rmax = std::max(1e-6, rough_max_m);

//   for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
//     Index idx(*it);
//     const float s = S(idx(0), idx(1));
//     const float r = R(idx(0), idx(1));
//     if (!isFiniteF(s) || !isFiniteF(r)) { T(idx(0), idx(1)) = NaNf(); continue; }

//     const double ns = 1.0 - std::clamp(double(s)/smax, 0.0, 1.0);
//     const double nr = 1.0 - std::clamp(double(r)/rmax, 0.0, 1.0);
//     const double t  = (ws*ns + wr*nr) / wsum;

//     T(idx(0), idx(1)) = static_cast<float>(std::clamp(t, 0.0, 1.0));
//   }

//   if (map.exists("trav")) map["trav"] = T;
//   else map.add("trav", T);
// }

// // --------------------------- CVaR TRAV -> cvar_risk ---------------------------
// void addCvarTraversability(grid_map::GridMap& map,
//                            double alpha,
//                            double window_m)
// {
//   if (!map.exists("trav")) return;
//   const Matrix& TR = map["trav"];
//   Matrix CV(TR.rows(), TR.cols());

//   const int win  = metersToCells(map, window_m);
//   const int half = std::max(1, win/2);
//   const double a = std::clamp(alpha, 0.0, 0.999);

//   for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
//     Index c(*it);
//     const float tc = TR(c(0), c(1));
//     if (!isFiniteF(tc)) { CV(c(0), c(1)) = NaNf(); continue; }

//     std::vector<double> risks;
//     risks.reserve((2*half+1)*(2*half+1));

//     for (int di=-half; di<=half; ++di) {
//       for (int dj=-half; dj<=half; ++dj) {
//         Index p(c(0)+di, c(1)+dj);
//         if (!insideIndex(map, p)) continue;
//         const float t = TR(p(0), p(1));
//         if (!isFiniteF(t)) continue;
//         risks.push_back(1.0 - double(t)); // riesgo = 1 - trav
//       }
//     }
//     if (risks.size() < 5) { CV(c(0), c(1)) = NaNf(); continue; }

//     std::sort(risks.begin(), risks.end()); // ascendente
//     const size_t n = risks.size();
//     const size_t q = std::min(n-1, static_cast<size_t>(std::ceil(a * (n - 1))));
//     const double thresh = risks[q];

//     double sumTail=0.0; size_t m=0;
//     for (size_t k=q; k<n; ++k) { sumTail += risks[k]; ++m; }
//     const double cvar_risk = (m>0) ? (sumTail / double(m)) : thresh;

//     CV(c(0), c(1)) = static_cast<float>(std::clamp(cvar_risk, 0.0, 1.0));
//   }

//   // >>> Cambio pedido: escribir en "cvar_risk"
//   if (map.exists("cvar_risk")) map["cvar_risk"] = CV;
//   else map.add("cvar_risk", CV);
// }

// // --------------------------- NEGATIVE OBSTACLES (simple) ---------------------------
// // Marca como 1.0 las celdas cuyo centro es significativamente más bajo
// // que la media del anillo a ~ring_m, usando umbral drop_thresh_m.
// void addNegativeObstacles(grid_map::GridMap& map,
//                           double drop_thresh_m,
//                           double ring_m)
// {
//   if (!map.exists("elevation")) return;
//   const Matrix& Z = map["elevation"];

//   Matrix NEG(Z.rows(), Z.cols());
//   NEG.setZero();

//   const int r = metersToCells(map, ring_m);
//   const double r2_low  = std::pow(std::max(1, r/2), 2);   // anillo interior (suave)
//   const double r2_high = std::pow(std::max(1, r),   2);   // anillo exterior

//   for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
//     Index c(*it);
//     const float zc = Z(c(0), c(1));
//     if (!isFiniteF(zc)) { NEG(c(0), c(1)) = 0.0f; continue; }

//     double sum=0.0; int n=0;
//     for (int di=-r; di<=r; ++di) {
//       for (int dj=-r; dj<=r; ++dj) {
//         if (di==0 && dj==0) continue;
//         const int d2 = di*di + dj*dj;
//         if (d2 < r2_low || d2 > r2_high) continue; // aproximar anillo
//         Index p(c(0)+di, c(1)+dj);
//         if (!insideIndex(map, p)) continue;
//         const float zn = Z(p(0), p(1));
//         if (!isFiniteF(zn)) continue;
//         sum += zn; ++n;
//       }
//     }

//     if (n < 6) { NEG(c(0), c(1)) = 0.0f; continue; }
//     const double ring_mean = sum / n;

//     // Si la media del anillo está "bastante" por encima del centro => hueco/bache
//     NEG(c(0), c(1)) = ((ring_mean - double(zc)) >= drop_thresh_m) ? 1.0f : 0.0f;
//   }

//   if (map.exists("negatives")) map["negatives"] = NEG;
//   else map.add("negatives", NEG);
// }

// // --------------------------- NEGATIVE OBSTACLES (robusto wrapper) ---------------------------
// void addNegativeObstaclesRobust(grid_map::GridMap& map,
//                                 double drop_thresh_m,
//                                 double ring_m,
//                                 double /*min_valid_ratio*/,
//                                 double /*slope_gate_rad*/,
//                                 const std::string& /*elevation_layer*/,
//                                 const std::string& /*slope_layer*/,
//                                 const std::string& out_layer)
// {
//   // Para empezar, reutilizamos la versión simple.
//   addNegativeObstacles(map, drop_thresh_m, ring_m);

//   const std::string src = "negatives";
//   if (!map.exists(src)) return;

//   if (!map.exists(out_layer)) map.add(out_layer, 0.0f);
//   map[out_layer] = map[src];
// }

// // --------------------------- COMBINE TO FINAL ---------------------------
// void combineToFinalObstacles(grid_map::GridMap& map,
//                              const std::string& obstacles_layer,
//                              const std::string& negatives_layer,
//                              const std::string& cvar_layer,
//                              double cvar_tau,
//                              const std::string& out_layer)
// {
//   const bool have_obst = map.exists(obstacles_layer);
//   const bool have_neg  = map.exists(negatives_layer);
//   const bool have_cvar = map.exists(cvar_layer);

//   if (!map.exists(out_layer)) map.add(out_layer, 0.0f);
//   auto& outM = map[out_layer];
//   outM.setZero(); // conservar tamaño, poner a cero

//   const grid_map::Matrix* obst = have_obst ? &map[obstacles_layer] : nullptr;
//   const grid_map::Matrix* neg  = have_neg  ? &map[negatives_layer]  : nullptr;
//   const grid_map::Matrix* cvar = have_cvar ? &map[cvar_layer]       : nullptr;

//   const auto sz = map.getSize();
//   const int nx = sz(0);
//   const int ny = sz(1);

//   for (int i = 0; i < nx; ++i) {
//     for (int j = 0; j < ny; ++j) {
//       float v = 0.0f;
//       if (obst) v = std::max(v, (*obst)(i,j));
//       if (neg)  v = std::max(v, (*neg)(i,j));
//       if (cvar) {
//         const float gate = ((*cvar)(i,j) > static_cast<float>(cvar_tau)) ? 1.0f : 0.0f;
//         v = std::max(v, gate);
//       }
//       outM(i,j) = v;
//     }
//   }
// }

// } // namespace layers
// } // namespace em


// ////


// // // } // namespace em::layers
// // #include "elevation_mapping/layer_tools.hpp"
// // #include <grid_map_core/iterators/GridMapIterator.hpp>
// // #include <cmath>
// // #include <algorithm>
// // #include <vector>
// // #include <limits>

// // using grid_map::Index;
// // using grid_map::Matrix;

// // namespace {  // utilidades internas

// // inline bool isFiniteF(float v) { return std::isfinite(static_cast<double>(v)); }
// // inline float NaNf() { return std::numeric_limits<float>::quiet_NaN(); }

// // inline int metersToCells(const grid_map::GridMap& map, double m)
// // {
// //   const double res = map.getResolution();
// //   int cells = static_cast<int>(std::round(m / std::max(1e-9, res)));
// //   return std::max(1, cells);
// // }

// // inline bool insideIndex(const grid_map::GridMap& map, const Index& i)
// // {
// //   const auto& sz = map.getSize();
// //   return (i(0) >= 0 && i(1) >= 0 && i(0) < sz(0) && i(1) < sz(1));
// // }

// // } // namespace anónimo

// // namespace em::layers {

// // // --------------------------- SLOPE (rad) ---------------------------
// // void addSlope(grid_map::GridMap& map, double h)
// // {
// //   if (!map.exists("elevation")) return;
// //   const Matrix& Z = map["elevation"];
// //   Matrix slope(Z.rows(), Z.cols());
// //   const double dx = std::max(1e-9, h);
// //   const double dy = std::max(1e-9, h);

// //   for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
// //     Index idx(*it);
// //     const float zc = Z(idx(0), idx(1));
// //     if (!isFiniteF(zc)) { slope(idx(0), idx(1)) = NaNf(); continue; }

// //     Index nx = idx; nx(0)++;  Index px = idx; px(0)--;
// //     Index ny = idx; ny(1)++;  Index py = idx; py(1)--;

// //     auto val = [&](const Index& i)->float {
// //       if (!insideIndex(map, i)) return zc;
// //       const float v = Z(i(0), i(1));
// //       return isFiniteF(v) ? v : zc;
// //     };

// //     const double dzdx = (val(nx) - val(px)) / (2.0 * dx);
// //     const double dzdy = (val(ny) - val(py)) / (2.0 * dy);
// //     const double grad = std::hypot(dzdx, dzdy);

// //     slope(idx(0), idx(1)) = static_cast<float>(std::atan(grad)); // rad
// //   }

// //   if (map.exists("slope")) map["slope"] = slope;
// //   else map.add("slope", slope);
// // }

// // // --------------------------- ROUGHNESS (m) ---------------------------
// // void addRoughness(grid_map::GridMap& map, double window_m)
// // {
// //   if (!map.exists("elevation")) return;
// //   const Matrix& Z = map["elevation"];
// //   Matrix R(Z.rows(), Z.cols());

// //   const int win = metersToCells(map, window_m);
// //   const int half = std::max(1, win/2);

// //   for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
// //     Index c(*it);
// //     const float zc = Z(c(0), c(1));
// //     if (!isFiniteF(zc)) { R(c(0), c(1)) = NaNf(); continue; }

// //     double sum=0.0, sum2=0.0; int n=0;
// //     for (int di=-half; di<=half; ++di) {
// //       for (int dj=-half; dj<=half; ++dj) {
// //         Index p(c(0)+di, c(1)+dj);
// //         if (!insideIndex(map, p)) continue;
// //         const float z = Z(p(0), p(1));
// //         if (!isFiniteF(z)) continue;
// //         sum += z; sum2 += double(z)*double(z); ++n;
// //       }
// //     }
// //     if (n<3) { R(c(0), c(1)) = NaNf(); continue; }
// //     const double mean = sum / n;
// //     const double var  = std::max(0.0, (sum2 / n) - mean*mean);
// //     R(c(0), c(1)) = static_cast<float>(std::sqrt(var));
// //   }

// //   if (map.exists("roughness")) map["roughness"] = R;
// //   else map.add("roughness", R);
// // }

// // // --------------------------- STEP (m) ---------------------------
// // void addStep(grid_map::GridMap& map, double window_m)
// // {
// //   if (!map.exists("elevation")) return;
// //   const Matrix& Z = map["elevation"];
// //   Matrix S(Z.rows(), Z.cols());

// //   const int win = metersToCells(map, window_m);
// //   const int half = std::max(1, win/2);

// //   for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
// //     Index c(*it);
// //     const float zc = Z(c(0), c(1));
// //     if (!isFiniteF(zc)) { S(c(0), c(1)) = NaNf(); continue; }

// //     double sum=0.0; int n=0;
// //     for (int di=-half; di<=half; ++di) {
// //       for (int dj=-half; dj<=half; ++dj) {
// //         if (di==0 && dj==0) continue;
// //         Index p(c(0)+di, c(1)+dj);
// //         if (!insideIndex(map, p)) continue;
// //         const float z = Z(p(0), p(1));
// //         if (!isFiniteF(z)) continue;
// //         sum += z; ++n;
// //       }
// //     }
// //     if (n<3) { S(c(0), c(1)) = NaNf(); continue; }
// //     const double mean = sum / n;
// //     S(c(0), c(1)) = static_cast<float>(std::fabs(double(zc) - mean));
// //   }

// //   if (map.exists("step")) map["step"] = S;
// //   else map.add("step", S);
// // }

// // // --------------------------- OBSTÁCULOS (0/1) ---------------------------
// // void addObstacleBinaryFromGeom(grid_map::GridMap& map,
// //                                double slope_thresh_rad,
// //                                double step_thresh_m)
// // {
// //   if (!map.exists("slope") || !map.exists("step")) return;
// //   const Matrix& S = map["slope"];
// //   const Matrix& H = map["step"];
// //   Matrix O(S.rows(), S.cols());

// //   for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
// //     Index idx(*it);
// //     const float s = S(idx(0), idx(1));
// //     const float h = H(idx(0), idx(1));
// //     const bool s_bad = isFiniteF(s) && s >= slope_thresh_rad;
// //     const bool h_bad = isFiniteF(h) && h >= step_thresh_m;
// //     O(idx(0), idx(1)) = (s_bad || h_bad) ? 1.0f : 0.0f;
// //   }

// //   if (map.exists("obstacles")) map["obstacles"] = O;
// //   else map.add("obstacles", O);
// // }

// // // --------------------------- TRAVERSABILIDAD [0..1] ---------------------------
// // void addTraversabilityFromSlopeRough(grid_map::GridMap& map,
// //                                      double slope_max_rad,
// //                                      double rough_max_m,
// //                                      double w_slope,
// //                                      double w_rough)
// // {
// //   if (!map.exists("slope") || !map.exists("roughness")) return;
// //   const Matrix& S = map["slope"];
// //   const Matrix& R = map["roughness"];
// //   Matrix T(S.rows(), S.cols());

// //   const double ws = std::clamp(w_slope, 0.0, 1.0);
// //   const double wr = std::clamp(w_rough, 0.0, 1.0);
// //   const double wsum = std::max(1e-6, ws + wr);
// //   const double smax = std::max(1e-6, slope_max_rad);
// //   const double rmax = std::max(1e-6, rough_max_m);

// //   for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
// //     Index idx(*it);
// //     const float s = S(idx(0), idx(1));
// //     const float r = R(idx(0), idx(1));
// //     if (!isFiniteF(s) || !isFiniteF(r)) { T(idx(0), idx(1)) = NaNf(); continue; }

// //     const double ns = 1.0 - std::clamp(double(s)/smax, 0.0, 1.0);
// //     const double nr = 1.0 - std::clamp(double(r)/rmax, 0.0, 1.0);
// //     const double t  = (ws*ns + wr*nr) / wsum;

// //     T(idx(0), idx(1)) = static_cast<float>(std::clamp(t, 0.0, 1.0));
// //   }

// //   if (map.exists("trav")) map["trav"] = T;
// //   else map.add("trav", T);
// // }

// // // --------------------------- CVaR (riesgo) ---------------------------
// // void addCvarTraversability(grid_map::GridMap& map,
// //                            double alpha,
// //                            double window_m)
// // {
// //   if (!map.exists("trav")) return;
// //   const Matrix& TR = map["trav"];
// //   Matrix CV(TR.rows(), TR.cols());

// //   const int win  = metersToCells(map, window_m);
// //   const int half = std::max(1, win/2);
// //   const double a = std::clamp(alpha, 0.0, 0.999);

// //   for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
// //     Index c(*it);
// //     const float tc = TR(c(0), c(1));
// //     if (!isFiniteF(tc)) { CV(c(0), c(1)) = NaNf(); continue; }

// //     std::vector<double> risks;
// //     risks.reserve((2*half+1)*(2*half+1));

// //     for (int di=-half; di<=half; ++di) {
// //       for (int dj=-half; dj<=half; ++dj) {
// //         Index p(c(0)+di, c(1)+dj);
// //         if (!insideIndex(map, p)) continue;
// //         const float t = TR(p(0), p(1));
// //         if (!isFiniteF(t)) continue;
// //         risks.push_back(1.0 - double(t)); // riesgo = 1 - trav
// //       }
// //     }
// //     if (risks.size() < 5) { CV(c(0), c(1)) = NaNf(); continue; }

// //     std::sort(risks.begin(), risks.end()); // asc
// //     const size_t n = risks.size();
// //     const size_t q = std::min(n-1, static_cast<size_t>(std::ceil(a * (n - 1))));
// //     const double thresh = risks[q];

// //     double sumTail=0.0; size_t m=0;
// //     for (size_t k=q; k<n; ++k) { sumTail += risks[k]; ++m; }
// //     const double cvar_risk = (m>0) ? (sumTail / double(m)) : thresh;

// //     CV(c(0), c(1)) = static_cast<float>(std::clamp(cvar_risk, 0.0, 1.0));
// //   }

// //   // Escribimos riesgo (coincide con "cvar_risk" que usas en combineToFinalObstacles)
// //   if (map.exists("cvar_risk")) map["cvar_risk"] = CV;
// //   else map.add("cvar_risk", CV);
// // }

// // // --------------------------- NEGATIVE OBSTACLES (versión corta) ---------------------------
// // // Marca "negatives" = 1 si el centro es significativamente más bajo que el anillo alrededor.
// // void addNegativeObstacles(grid_map::GridMap& map,
// //                           double drop_thresh_m,
// //                           double ring_m)
// // {
// //   if (!map.exists("elevation")) return;
// //   const Matrix& Z = map["elevation"];
// //   Matrix N(Z.rows(), Z.cols());
// //   N.setZero();

// //   const int r = std::max(1, metersToCells(map, ring_m));
// //   const int nx = map.getSize()(0);
// //   const int ny = map.getSize()(1);

// //   for (int i=0; i<nx; ++i) {
// //     for (int j=0; j<ny; ++j) {
// //       const float zc = Z(i,j);
// //       if (!isFiniteF(zc)) { N(i,j) = 0.0f; continue; }

// //       // Estadística del anillo (entre radio r y 2r, aproximado en cuadrado)
// //       int cnt = 0;
// //       double sum = 0.0;
// //       for (int di=-2*r; di<=2*r; ++di) {
// //         for (int dj=-2*r; dj<=2*r; ++dj) {
// //           const int ad = std::abs(di) + std::abs(dj);
// //           if (ad < r || ad > 2*r) continue;               // “anillo” taxicab
// //           const int ii = i + di, jj = j + dj;
// //           if (ii < 0 || jj < 0 || ii >= nx || jj >= ny) continue;
// //           const float zn = Z(ii, jj);
// //           if (!isFiniteF(zn)) continue;
// //           sum += zn; ++cnt;
// //         }
// //       }
// //       if (cnt < 8) { N(i,j) = 0.0f; continue; }

// //       const double ring_mean = sum / cnt;
// //       const double drop = ring_mean - double(zc);
// //       N(i,j) = (drop >= drop_thresh_m) ? 1.0f : 0.0f;
// //     }
// //   }

// //   if (map.exists("negatives")) map["negatives"] = N;
// //   else map.add("negatives", N);
// // }

// // // --------------------------- NEGATIVE robusta (envoltorio) ---------------------------
// // void addNegativeObstaclesRobust(grid_map::GridMap& map,
// //                                 double drop_thresh_m,
// //                                 double ring_m,
// //                                 double /*min_valid_ratio*/,
// //                                 double /*slope_gate_rad*/,
// //                                 const std::string& /*elevation_layer*/,
// //                                 const std::string& /*slope_layer*/,
// //                                 const std::string& out_layer)
// // {
// //   // Reutiliza la versión corta que produce "negatives"
// //   addNegativeObstacles(map, drop_thresh_m, ring_m);

// //   if (!map.exists("negatives")) return;
// //   if (!map.exists(out_layer)) map.add(out_layer, 0.0f);
// //   map[out_layer] = map["negatives"];
// // }

// // // --------------------------- Combinar capas ---------------------------
// // void combineToFinalObstacles(grid_map::GridMap& map,
// //                              const std::string& obstacles_layer,
// //                              const std::string& negatives_layer,
// //                              const std::string& cvar_layer,
// //                              double cvar_tau,
// //                              const std::string& out_layer)
// // {
// //   const bool have_obst = map.exists(obstacles_layer);
// //   const bool have_neg  = map.exists(negatives_layer);
// //   const bool have_cvar = map.exists(cvar_layer);

// //   if (!map.exists(out_layer)) map.add(out_layer, 0.0f);
// //   auto& outM = map[out_layer];

// //   const int nx = map.getSize()(0);
// //   const int ny = map.getSize()(1);

// //   const grid_map::Matrix* obst = have_obst ? &map[obstacles_layer] : nullptr;
// //   const grid_map::Matrix* neg  = have_neg  ? &map[negatives_layer]  : nullptr;
// //   const grid_map::Matrix* cvar = have_cvar ? &map[cvar_layer]       : nullptr;

// //   outM.setZero(nx, ny);

// //   for (int i = 0; i < nx; ++i) {
// //     for (int j = 0; j < ny; ++j) {
// //       float v = 0.0f;
// //       if (obst) v = std::max(v, (*obst)(i,j));
// //       if (neg)  v = std::max(v, (*neg)(i,j));
// //       if (cvar) {
// //         const float gate = ((*cvar)(i,j) > static_cast<float>(cvar_tau)) ? 1.0f : 0.0f; // cvar = riesgo
// //         v = std::max(v, gate);
// //       }
// //       outM(i,j) = v;
// //     }
// //   }
// // }

// // } // namespace em::layers
// layer_tools.cpp
//
// Utilidades de capas para elevation_mapping.
// Nota: Implementaciones sencillas y robustas, cuidadosas con NaNs.
//
// Requiere: grid_map_core

#include <algorithm>
#include <cmath>
#include <numeric>
#include <string>
#include <vector>
#include <limits>

#include <grid_map_core/grid_map_core.hpp>

namespace em {
namespace layers {

// ---- helpers ----

static inline bool isFinite(float v) {
  return std::isfinite(v);
}

static inline int clampi(int v, int lo, int hi) {
  return std::max(lo, std::min(hi, v));
}

static void ensureLayer(grid_map::GridMap& map, const std::string& name) {
  if (!map.exists(name)) {
    map.add(name, std::numeric_limits<float>::quiet_NaN());
  }
}

static float readCell(const grid_map::Matrix& M, const grid_map::Index& I) {
  return M(I(0), I(1));
}

static void writeCell(grid_map::Matrix& M, const grid_map::Index& I, float v) {
  M(I(0), I(1)) = v;
}

static bool validIndex(const grid_map::GridMap& map, const grid_map::Index& I) {
  const auto size = map.getSize();
  return (I(0) >= 0 && I(1) >= 0 && I(0) < size(0) && I(1) < size(1));
}

static void neighborhoodIndicesSquare(const grid_map::GridMap& map,
                                      const grid_map::Index& center,
                                      int r,
                                      std::vector<grid_map::Index>& out) {
  out.clear();
  const auto size = map.getSize();
  const int i0 = clampi(center(0) - r, 0, size(0) - 1);
  const int i1 = clampi(center(0) + r, 0, size(0) - 1);
  const int j0 = clampi(center(1) - r, 0, size(1) - 1);
  const int j1 = clampi(center(1) + r, 0, size(1) - 1);
  out.reserve((i1 - i0 + 1) * (j1 - j0 + 1));
  for (int i = i0; i <= i1; ++i) {
    for (int j = j0; j <= j1; ++j) {
      out.emplace_back(grid_map::Index(i, j));
    }
  }
}

// ---- API ----

void addSlope(grid_map::GridMap& map, double /*cell*/) {
  const std::string in  = "elevation";
  const std::string out = "slope";
  if (!map.exists(in)) return;
  ensureLayer(map, out);

  auto& Mout = map[out];
  const auto& Min = map[in];
  const double res = map.getResolution();
  const auto size = map.getSize();

  for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
    const grid_map::Index I(*it);
    const int i = I(0), j = I(1);

    auto val_c = Min(i, j);
    if (!isFinite(val_c)) { Mout(i, j) = std::numeric_limits<float>::quiet_NaN(); continue; }

    // Vecinos para derivadas (centrales cuando se puede)
    const int im1 = clampi(i - 1, 0, size(0) - 1);
    const int ip1 = clampi(i + 1, 0, size(0) - 1);
    const int jm1 = clampi(j - 1, 0, size(1) - 1);
    const int jp1 = clampi(j + 1, 0, size(1) - 1);

    const float left  = Min(im1, j);
    const float right = Min(ip1, j);
    const float down  = Min(i, jm1);
    const float up    = Min(i, jp1);

    // Si vecinos NaN, degradar a forward/backward cuando sea posible
    double dzdx = 0.0, dzdy = 0.0;
    if (isFinite(left) && isFinite(right)) {
      dzdx = (right - left) / (2.0 * res);
    } else if (isFinite(right)) {
      dzdx = (right - val_c) / res;
    } else if (isFinite(left)) {
      dzdx = (val_c - left) / res;
    } else {
      Mout(i, j) = std::numeric_limits<float>::quiet_NaN();
      continue;
    }

    if (isFinite(down) && isFinite(up)) {
      dzdy = (up - down) / (2.0 * res);
    } else if (isFinite(up)) {
      dzdy = (up - val_c) / res;
    } else if (isFinite(down)) {
      dzdy = (val_c - down) / res;
    } else {
      Mout(i, j) = std::numeric_limits<float>::quiet_NaN();
      continue;
    }

    const double tan_theta = std::sqrt(dzdx * dzdx + dzdy * dzdy);
    Mout(i, j) = static_cast<float>(std::atan(tan_theta));  // rad
  }
}

void addRoughness(grid_map::GridMap& map, double window_m) {
  const std::string in  = "elevation";
  const std::string out = "rough";
  if (!map.exists(in)) return;
  ensureLayer(map, out);

  auto& Mout = map[out];
  const auto& Min = map[in];
  const double res = map.getResolution();
  const int r = std::max(1, static_cast<int>(std::round(window_m / res)));

  std::vector<grid_map::Index> neigh;
  for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
    const grid_map::Index I(*it);
    neighborhoodIndicesSquare(map, I, r, neigh);

    double sum = 0.0, sum2 = 0.0;
    int n = 0;
    for (const auto& N : neigh) {
      const float v = readCell(Min, N);
      if (!isFinite(v)) continue;
      sum += v; sum2 += (double)v * v; ++n;
    }
    if (n < 3) {
      writeCell(Mout, I, std::numeric_limits<float>::quiet_NaN());
      continue;
    }
    const double mean = sum / n;
    const double var  = std::max(0.0, (sum2 / n) - mean * mean);
    writeCell(Mout, I, static_cast<float>(std::sqrt(var))); // m (std)
  }
}

void addStep(grid_map::GridMap& map, double window_m) {
  const std::string in  = "elevation";
  const std::string out = "step";
  if (!map.exists(in)) return;
  ensureLayer(map, out);

  auto& Mout = map[out];
  const auto& Min = map[in];
  const double res = map.getResolution();
  const int r = std::max(1, static_cast<int>(std::round(window_m / res)));

  std::vector<grid_map::Index> neigh;
  for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
    const grid_map::Index I(*it);
    neighborhoodIndicesSquare(map, I, r, neigh);

    float vmin =  std::numeric_limits<float>::infinity();
    float vmax = -std::numeric_limits<float>::infinity();
    int n = 0;
    for (const auto& N : neigh) {
      const float v = readCell(Min, N);
      if (!isFinite(v)) continue;
      vmin = std::min(vmin, v);
      vmax = std::max(vmax, v);
      ++n;
    }
    if (n < 2) {
      writeCell(Mout, I, std::numeric_limits<float>::quiet_NaN());
      continue;
    }
    writeCell(Mout, I, (isFinite(vmin) && isFinite(vmax)) ? (vmax - vmin) : std::numeric_limits<float>::quiet_NaN());
  }
}

void addObstacleBinaryFromGeom(grid_map::GridMap& map,
                               double slope_thresh_rad,
                               double step_thresh_m) {
  ensureLayer(map, "obstacles");
  auto& Mobst = map["obstacles"];

  const bool haveSlope = map.exists("slope");
  const bool haveStep  = map.exists("step");
  if (!haveSlope && !haveStep) {
    // nada que combinar
    return;
  }

  const grid_map::Matrix* Ms = haveSlope ? &map["slope"] : nullptr;
  const grid_map::Matrix* Mt = haveStep  ? &map["step"]  : nullptr;

  for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
    const grid_map::Index I(*it);
    bool is_obst = false;

    if (Ms) {
      const float s = readCell(*Ms, I);
      if (isFinite(s) && s > static_cast<float>(slope_thresh_rad)) is_obst = true;
    }
    if (Mt) {
      const float st = readCell(*Mt, I);
      if (isFinite(st) && st > static_cast<float>(step_thresh_m)) is_obst = true;
    }

    writeCell(Mobst, I, is_obst ? 1.0f : 0.0f);
  }
}

// ---- IMPLEMENTACIÓN BÁSICA QUE FALTABA ----
// Detecta negativos comparando el centro contra la media de un anillo cuadrado de radio 'ring_m'.
// Escribe en "negatives" (0/1). Usa capa "elevation".
void addNegativeObstacles(grid_map::GridMap& map,
                          double drop_thresh_m,
                          double ring_m) {
  const std::string inElev = "elevation";
  const std::string outNeg = "negatives";

  if (!map.exists(inElev)) return;
  ensureLayer(map, outNeg);

  const auto& Me = map[inElev];
  auto& Mn = map[outNeg];

  const double res = map.getResolution();
  const int r = std::max(1, static_cast<int>(std::round(ring_m / res)));
  if (r < 1) return;

  std::vector<grid_map::Index> neigh;
  for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
    const grid_map::Index I(*it);
    const float c = readCell(Me, I);
    if (!isFinite(c)) { writeCell(Mn, I, std::numeric_limits<float>::quiet_NaN()); continue; }

    neighborhoodIndicesSquare(map, I, r, neigh);

    double sum = 0.0; int n = 0;
    for (const auto& N : neigh) {
      // evitar el propio centro
      if (N(0) == I(0) && N(1) == I(1)) continue;

      const float v = readCell(Me, N);
      if (!isFinite(v)) continue;
      sum += v;
      ++n;
    }

    if (n < 6) { writeCell(Mn, I, 0.0f); continue; } // poco contexto => conservador
    const double mean_ring = sum / std::max(1, n);
    const bool neg = (mean_ring - c) > drop_thresh_m;
    writeCell(Mn, I, neg ? 1.0f : 0.0f);
  }
}

// Variante robusta (parámetros extra y nombres de capas)
void addNegativeObstaclesRobust(grid_map::GridMap& map,
                                double drop_thresh_m,
                                double ring_m,
                                double min_valid_ratio,
                                double slope_gate_rad,
                                const std::string& elevation_layer,
                                const std::string& slope_layer,
                                const std::string& out_layer) {
  const std::string inElev = elevation_layer.empty() ? "elevation" : elevation_layer;
  const std::string inSlope= slope_layer.empty()     ? "slope"     : slope_layer;
  const std::string outNeg = out_layer.empty()       ? "negatives" : out_layer;

  if (!map.exists(inElev)) return;
  ensureLayer(map, outNeg);

  // Si no hay slope, intenta calcular algo razonable antes de filtrar
  if (!map.exists(inSlope)) {
    // cálculo local sencillo
    addSlope(map, map.getResolution());
  }

  const auto& Me = map[inElev];
  const auto& Ms = map[inSlope];
  auto& Mn = map[outNeg];

  const double res = map.getResolution();
  const int r = std::max(1, static_cast<int>(std::round(ring_m / res)));
  if (r < 1) return;

  std::vector<grid_map::Index> neigh;
  for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
    const grid_map::Index I(*it);
    const float c = readCell(Me, I);
    const float s0= readCell(Ms, I);

    if (!isFinite(c)) { writeCell(Mn, I, std::numeric_limits<float>::quiet_NaN()); continue; }
    if (isFinite(s0) && s0 > static_cast<float>(slope_gate_rad)) {
      // Puerta: si ya hay mucha pendiente local, no marcar negativo (probable borde/escalón)
      writeCell(Mn, I, 0.0f); continue;
    }

    neighborhoodIndicesSquare(map, I, r, neigh);

    double sum = 0.0; int n = 0, nvalid = 0;
    for (const auto& N : neigh) {
      // evitar el propio centro
      if (N(0) == I(0) && N(1) == I(1)) continue;

      const float v = readCell(Me, N);
      if (!isFinite(v)) continue;
      sum += v;
      ++n;
      ++nvalid;
    }


    const int total_ring = static_cast<int>(neigh.size()) - 1;
    const double ratio = total_ring > 0 ? static_cast<double>(nvalid) / total_ring : 0.0;
    if (ratio < min_valid_ratio) { writeCell(Mn, I, 0.0f); continue; }

    if (n < 6) { writeCell(Mn, I, 0.0f); continue; }
    const double mean_ring = sum / std::max(1, n);
    const bool neg = (mean_ring - c) > drop_thresh_m;
    writeCell(Mn, I, neg ? 1.0f : 0.0f);
  }
}

void addCvarTraversability(grid_map::GridMap& map,
                           double alpha,
                           double window_m /*, const std::string& out = "cvar_risk"*/) {
  const std::string in = "slope";
  const std::string out= "cvar_risk";
  if (!map.exists(in)) {
    // si no hay slope, créalo rápido
    addSlope(map, map.getResolution());
    if (!map.exists(in)) return;
  }
  ensureLayer(map, out);

  auto& Mrisk = map[out];
  const auto& Mslope = map[in];

  const double res = map.getResolution();
  const int r = std::max(1, static_cast<int>(std::round(window_m / res)));
  const double alpha_clamped = std::min(0.999, std::max(0.001, alpha));

  std::vector<grid_map::Index> neigh;
  std::vector<float> vals;
  for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
    const grid_map::Index I(*it);
    neighborhoodIndicesSquare(map, I, r, neigh);

    vals.clear();
    vals.reserve(neigh.size());
    for (const auto& N : neigh) {
      const float s = readCell(Mslope, N);
      if (isFinite(s)) vals.push_back(s);
    }
    if (vals.size() < 6) {
      writeCell(Mrisk, I, std::numeric_limits<float>::quiet_NaN());
      continue;
    }

    std::sort(vals.begin(), vals.end());
    const size_t k = static_cast<size_t>(std::floor(alpha_clamped * (vals.size() - 1)));
    const float VaR = vals[k];

    double tail_sum = 0.0; size_t tail_n = 0;
    for (size_t t = k; t < vals.size(); ++t) { tail_sum += vals[t]; ++tail_n; }
    const double cvar = tail_n ? (tail_sum / tail_n) : VaR;

    // normalización simple por pi/2 (pendiente máxima teórica)
    const double risk = std::min(1.0, std::max(0.0, cvar / (M_PI_2)));
    writeCell(Mrisk, I, static_cast<float>(risk));
  }
}

void combineToFinalObstacles(grid_map::GridMap& map,
                             const std::string& obstacles_layer,
                             const std::string& negatives_layer,
                             const std::string& cvar_layer,
                             double cvar_tau,
                             const std::string& out_layer) {
  ensureLayer(map, out_layer);
  auto& Mout = map[out_layer];

  const grid_map::Matrix* Mo = map.exists(obstacles_layer) ? &map[obstacles_layer] : nullptr;
  const grid_map::Matrix* Mn = map.exists(negatives_layer) ? &map[negatives_layer] : nullptr;
  const grid_map::Matrix* Mc = map.exists(cvar_layer)      ? &map[cvar_layer]      : nullptr;

  for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
    const grid_map::Index I(*it);
    bool occ = false;

    if (Mo) {
      const float v = readCell(*Mo, I);
      if (isFinite(v) && v >= 0.5f) occ = true;
    }
    if (Mn && !occ) {
      const float v = readCell(*Mn, I);
      if (isFinite(v) && v >= 0.5f) occ = true;
    }
    if (Mc && !occ) {
      const float v = readCell(*Mc, I);
      if (isFinite(v) && v >= static_cast<float>(cvar_tau)) occ = true;
    }

    writeCell(Mout, I, occ ? 1.0f : 0.0f);
  }
}

} // namespace layers
} // namespace em
