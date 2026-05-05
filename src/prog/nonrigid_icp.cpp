#include <fmt/format.h>
#include <spdlog/spdlog.h>

#include <cxxopts.hpp>
#include <iostream>

#include "src/lib/correspondences.hpp"
#include "src/lib/io_utils.hpp"
#include "src/lib/named_column_matrix.hpp"
#include "src/lib/optimization.hpp"
#include "src/lib/profiler.hpp"
#include "src/lib/pt_cloud.hpp"
#include "src/lib/timer.hpp"

struct CorrespondencesResults {
  int num{};
  double mean_dists_before_optimization{};
  double std_dists_before_optimization{};
  double mean_dists_after_optimization{};
  double std_dists_after_optimization{};
};

struct IterationResults {
  int it{};
  OptimizationResults optimization_results{};
  CorrespondencesResults correspondences_results{};
};

namespace {
ErrorMetric ParseErrorMetric(const std::string& s) {
  if (s == "point_to_plane") return ErrorMetric::PointToPlane;
  if (s == "point_to_point") return ErrorMetric::PointToPoint;
  throw std::runtime_error("Error metric \"" + s + "\" is not available!");
}
}  // namespace

struct Params {
  std::string fixed;
  std::string movable;
  std::string transform;
  double voxel_size;
  std::vector<double> grid_limits;
  uint32_t buffer_voxels;
  std::string matching_mode;
  ErrorMetric error_metric;
  uint32_t max_correspondences_per_voxel;
  double max_euclidean_distance;
  double sigma_mad_factor;
  uint32_t num_iterations;
  std::vector<double> weights;
  std::string debug_dir;
  bool suppress_logging;
  bool profiling;
};

Params ParseUserInputs(int argc, char** argv);

void ReportIterationResults(const IterationResults& iteration_results);

int main(int argc, char** argv) {
  try {
    Params params = ParseUserInputs(argc, argv);

    auto& profiler = Profiler::Instance();

    Timer timer;
    if (!params.suppress_logging) {
      std::cout << "Start of \"nonrigid-icp\"\n";
    }

    if (params.profiling) profiler.Start("A.01 Create point cloud objects");
    if (!params.suppress_logging) {
      std::cout << "Create point cloud objects\n";
    }
    const bool needs_normals = (params.error_metric == ErrorMetric::PointToPlane);
    auto X_fix = ImportFileToMatrix(params.fixed, needs_normals,
                                    params.matching_mode == "id" ? true : false);
    auto X_mov = ImportFileToMatrix(params.movable, needs_normals,
                                    params.matching_mode == "id" ? true : false);

    auto pc_fix{PtCloud(X_fix(Eigen::all, {X_fix.namedColIndex("x"), X_fix.namedColIndex("y"),
                                           X_fix.namedColIndex("z")}))};
    auto pc_mov{PtCloud(X_mov(Eigen::all, {X_fix.namedColIndex("x"), X_fix.namedColIndex("y"),
                                           X_fix.namedColIndex("z")}))};

    if (needs_normals) {
      pc_fix.SetNormals(X_fix.namedCol("nx"), X_fix.namedCol("ny"), X_fix.namedCol("nz"));
      pc_mov.SetNormals(X_mov.namedCol("nx"), X_mov.namedCol("ny"), X_mov.namedCol("nz"));
    }
    if (params.matching_mode == "id") {
      pc_fix.SetCorrespondenceId(X_fix.namedCol("correspondence_id"));
      pc_mov.SetCorrespondenceId(X_mov.namedCol("correspondence_id"));
    }
    if (!params.suppress_logging) {
      std::cout << fmt::format("  Fixed point cloud has {:d} points\n", pc_fix.NumPts());
      std::cout << fmt::format("  Movable point cloud has {:d} points\n", pc_mov.NumPts());
    }
    if (params.profiling) profiler.Stop("A.01 Create point cloud objects");

    if (params.profiling) profiler.Start("A.02 Initialization of translation grids");
    if (!params.suppress_logging) {
      std::cout << "Initialize x/y/z translation grids for movable point cloud\n";
    }
    pc_mov.InitializeTranslationGrids(params.voxel_size, params.buffer_voxels, params.grid_limits);
    pc_mov.InitMatricesForUpdateXt();
    if (!params.suppress_logging) {
      std::cout << "Each translation grid (including buffer voxels) has the properties:\n";
      std::cout << fmt::format(
          "  x_min/x_max/x_num_voxels = {:.3f}/{:.3f}/{:d}\n",
          pc_mov.x_translation_grid().grid_origin()(0),
          pc_mov.x_translation_grid().grid_origin()(0) +
              pc_mov.x_translation_grid().voxel_size() * pc_mov.x_translation_grid().x_num_voxels(),
          pc_mov.x_translation_grid().x_num_voxels());
      std::cout << fmt::format(
          "  y_min/y_max/y_num_voxels = {:.3f}/{:.3f}/{:d}\n",
          pc_mov.x_translation_grid().grid_origin()(1),
          pc_mov.x_translation_grid().grid_origin()(1) +
              pc_mov.x_translation_grid().voxel_size() * pc_mov.x_translation_grid().y_num_voxels(),
          pc_mov.x_translation_grid().y_num_voxels());
      std::cout << fmt::format(
          "  z_min/z_max/z_num_voxels = {:.3f}/{:.3f}/{:d}\n",
          pc_mov.x_translation_grid().grid_origin()(2),
          pc_mov.x_translation_grid().grid_origin()(2) +
              pc_mov.x_translation_grid().voxel_size() * pc_mov.x_translation_grid().z_num_voxels(),
          pc_mov.x_translation_grid().z_num_voxels());
      std::cout << fmt::format("  num_grid_vals = {:d}\n",
                               pc_mov.x_translation_grid().num_grid_vals());
    }
    if (params.profiling) profiler.Stop("A.02 Initialization of translation grids");

    if (params.profiling) profiler.Start("A.03 Selection of correspondences");
    if (!params.suppress_logging) {
      std::cout << "Selection of correspondences in fixed point cloud\n";
    }
    Correspondences correspondences{pc_fix, pc_mov};
    correspondences.SelectPointsByVoxelStratifiedSampling(params.max_correspondences_per_voxel);
    auto idx_pc_fix{correspondences.GetSelectedPoints()};
    if (!params.suppress_logging) {
      std::cout << fmt::format("Selected {:d} points in fixed point cloud\n",
                               correspondences.num());
    }
    if (params.profiling) profiler.Stop("A.03 Selection of correspondences");

    auto debug_mode = (params.debug_dir != "");

    if (!params.suppress_logging) {
      std::cout << "Start iterative point cloud matching\n";
    }
    IterationResults iteration_results{};
    for (uint32_t it = 0; it < params.num_iterations; it++) {
      iteration_results.it = it + 1;

      if (params.profiling) profiler.Start("A.04 Matching");
      correspondences.SetSelectedPoints(idx_pc_fix);
      if (params.matching_mode == "nn") {
        correspondences.MatchPointsByNearestNeighbor();
      } else if (params.matching_mode == "id") {
        correspondences.MatchPointsByCorrespondenceId();
      }
      correspondences.RejectMaxEuclideanDistanceCriteria(params.max_euclidean_distance);
      correspondences.RejectStdMadCriteria(params.error_metric, params.sigma_mad_factor);

      if (debug_mode) {
        char it_string[100];
        std::sprintf(it_string, "%03d", iteration_results.it);
        auto debug_file_name =
            params.debug_dir + "correspondences_it" + std::string(it_string) + ".poly";
        correspondences.ExportCorrespondences(debug_file_name);
      }

      const Dists& dists_before = correspondences.dists_t(params.error_metric);
      iteration_results.correspondences_results.num = correspondences.num();
      iteration_results.correspondences_results.mean_dists_before_optimization = dists_before.mean;
      iteration_results.correspondences_results.std_dists_before_optimization = dists_before.std;
      if (params.profiling) profiler.Stop("A.04 Matching");

      if (params.profiling) profiler.Start("A.05 Optimization");
      Optimization optimization{};
      iteration_results.optimization_results =
          Optimization::Solve(correspondences, params.weights, params.error_metric);
      if (params.profiling) profiler.Stop("A.05 Optimization");

      if (iteration_results.optimization_results.success) {
        const Dists& dists_after = correspondences.dists_t(params.error_metric);
        iteration_results.correspondences_results.mean_dists_after_optimization = dists_after.mean;
        iteration_results.correspondences_results.std_dists_after_optimization = dists_after.std;
        ReportIterationResults(iteration_results);
      } else {
        throw std::runtime_error("Optimization was not successful! " +
                                 iteration_results.optimization_results.error_message);
      }
    }

    if (params.profiling) profiler.Start("A.06 Export of translation grids");
    if (!params.suppress_logging) {
      std::cout << fmt::format("Export of estimated translation grids to \"{}\"\n",
                               params.transform);
    }
    pc_mov.ExportTranslationGrids(params.transform);
    if (params.profiling) profiler.Stop("A.06 Export of translation grids");

    if (!params.suppress_logging) {
      std::cout << fmt::format("Finished \"nonrigid-icp\" in {}!\n", timer);
    }

    if (params.profiling && !params.suppress_logging) profiler.PrintSummary();

  } catch (const std::exception& e) {
    std::cerr << "Caught exception: " << e.what() << std::endl;
    return 1;
  } catch (...) {
    std::cerr << "Caught unknown exception." << std::endl;
    return 1;
  }

  return 0;
}

Params ParseUserInputs(int argc, char** argv) {
  cxxopts::Options options("nonrigid-icp", "Grid based point cloud matching.");

  // clang-format off
  options.add_options()
    ("f,fixed",
    "Path to fixed point cloud",
    cxxopts::value<std::string>())
    ("m,movable",
    "Path to movable point cloud",
    cxxopts::value<std::string>())
    ("t,transform",
    "Path to generated transform file. This file contains the estimated translation grids for "
    "the movable point cloud. The executable \"nonrigid-icp-transform\" can be used to transform a "
    "point cloud with this transform file.",
    cxxopts::value<std::string>())
    ("v,voxel_size",
    "Voxel size of translation grids",
    cxxopts::value<double>()->default_value("1"))
    ("g,grid_limits",
    "Limits of translation grids to be defined as \"x_min,y_min,z_min,x_max,y_max,z_max\". Note "
    "that the extent of the grids in x,y,z must be an integer multiple of the voxel size. The "
    "grid limits are chosen automatically by passing \"0,0,0,0,0,0\".",
    cxxopts::value<std::vector<double>>()->default_value("0,0,0,0,0,0"))
    ("b,buffer_voxels",
    "Number of voxels to be used as buffer around the translation grids",
    cxxopts::value<uint32_t>()->default_value("2"))
    ("a,matching_mode",
    "Matching mode for correspondences. Available modes are \"nn\" (nearest neighbor) and \"id\" "
    "(correspondence_id).",
    cxxopts::value<std::string>()->default_value("nn"))
    ("k,error_metric",
    "Error metric for optimization. Available values are \"point_to_plane\" (requires normals on "
    "the fixed point cloud) and \"point_to_point\" (no normals required).",
    cxxopts::value<std::string>()->default_value("point_to_plane"))
    ("n,max_correspondences_per_voxel",
    "Maximum number of correspondences sampled per non-empty translation-grid voxel. Voxels "
    "with fewer fixed-cloud points than this value contribute all their points; otherwise this "
    "many points are randomly drawn from the voxel. Voxel-stratified sampling ensures every "
    "voxel containing fixed-cloud data is constrained by at least one correspondence.",
    cxxopts::value<uint32_t>()->default_value("20"))
    ("e,max_euclidean_distance",
    "Maximum euclidean distance between corresponding points",
    cxxopts::value<double>()->default_value("1"))
    ("r,sigma_mad_factor",
    "Factor for MAD-based rejection of correspondences. Correspondences whose distance "
    "deviates from the median by more than this factor times the MAD-derived standard "
    "deviation (1.4826*MAD) are rejected. Set to 0 to deactivate the rejection.",
    cxxopts::value<double>()->default_value("5"))
    ("i,num_iterations",
    "Number of iterations",
    cxxopts::value<uint32_t>()->default_value("5"))
    ("w,weights",
    "Weights of zero observations as list for \"f,fx/fy/fz,fxy/fxz/fyz,fxyz\"",
    cxxopts::value<std::vector<double>>()->default_value("1,1,1,1"))
    ("d,debug_dir",
    "Directory for debug output for correspondences.",
    cxxopts::value<std::string>()->default_value(""))
    ("s,suppress_logging",
    "Suppress log output",
    cxxopts::value<bool>()->default_value("false"))
    ("p,profiling",
    "Enable runtime profiling output (timing summary)",
    cxxopts::value<bool>()->default_value("false"))
    ("h,help",
    "Print usage");
  // clang-format on

  // Show help if no arguments are provided
  if (argc == 1) {
    std::cout << options.help() << std::endl;
    exit(0);
  }

  auto result = options.parse(argc, argv);

  if (result.count("help")) {
    std::cout << options.help() << std::endl;
    exit(0);
  }

  // Save to params
  Params params{};
  params.fixed = result["fixed"].as<std::string>();
  params.movable = result["movable"].as<std::string>();
  params.transform = result["transform"].as<std::string>();
  params.voxel_size = result["voxel_size"].as<double>();
  params.grid_limits = result["grid_limits"].as<std::vector<double>>();
  params.buffer_voxels = result["buffer_voxels"].as<uint32_t>();
  params.matching_mode = result["matching_mode"].as<std::string>();
  params.error_metric = ParseErrorMetric(result["error_metric"].as<std::string>());
  params.max_correspondences_per_voxel = result["max_correspondences_per_voxel"].as<uint32_t>();
  if (params.max_correspondences_per_voxel == 0) {
    throw std::runtime_error("max_correspondences_per_voxel must be > 0!");
  }
  params.max_euclidean_distance = result["max_euclidean_distance"].as<double>();
  params.sigma_mad_factor = result["sigma_mad_factor"].as<double>();
  if (params.sigma_mad_factor < 0.0) {
    throw std::runtime_error("sigma_mad_factor must be >= 0!");
  }
  params.num_iterations = result["num_iterations"].as<uint32_t>();
  params.weights = result["weights"].as<std::vector<double>>();
  if (params.weights.size() != 4) {
    throw std::runtime_error(
        "weights must have exactly 4 values (f,fx/fy/fz,fxy/fxz/fyz,fxyz)!");
  }
  params.debug_dir = result["debug_dir"].as<std::string>();
  params.suppress_logging = result["suppress_logging"].as<bool>();
  params.profiling = result["profiling"].as<bool>();

  if (params.matching_mode == "id") {
    params.num_iterations = 1;
    if (!params.suppress_logging) {
      std::cout << fmt::format("Set num_iterations to {:d} as matching mode \"{}\" was selected.\n",
                               params.num_iterations, params.matching_mode.c_str());
    }
  }

  if (params.matching_mode != "nn" && params.matching_mode != "id") {
    std::string error_string = "Matching mode \"" + params.matching_mode + "\" is not available!";
    throw std::runtime_error(error_string);
  }

  if (params.debug_dir != "") {
    // Add trailing slash if not present
    if (params.debug_dir.back() != '/') {
      params.debug_dir += '/';
    }

    // Check if path exists
    if (!std::filesystem::exists(params.debug_dir)) {
      std::string error_string = "Debug directory \"" + params.debug_dir + "\" does not exist!";
      throw std::runtime_error(error_string);
    }
  }

  return params;
}

void ReportIterationResults(const IterationResults& iteration_results) {
  if (iteration_results.it == 1) {
    spdlog::info("{:>4} {:>10} {:>10} {:>10} {:>10} {:>10} {:>10} {:>10}", "it", "num_corr",
                 "num_obs", "num_unkn", "mean(dist)", "mean(dist)", "std(dist)", "std(dist)");
    spdlog::info("{:37} {:>10} {:>10} {:>10} {:>10}", "", "before", "after", "before", "after");
  }
  spdlog::info("{:4d} {:10d} {:10d} {:10d} {:10.3f} {:10.3f} {:10.3f} {:10.3f}",
               iteration_results.it, iteration_results.correspondences_results.num,
               iteration_results.optimization_results.num_observations,
               iteration_results.optimization_results.num_unknowns,
               iteration_results.correspondences_results.mean_dists_before_optimization,
               iteration_results.correspondences_results.mean_dists_after_optimization,
               iteration_results.correspondences_results.std_dists_before_optimization,
               iteration_results.correspondences_results.std_dists_after_optimization);
}