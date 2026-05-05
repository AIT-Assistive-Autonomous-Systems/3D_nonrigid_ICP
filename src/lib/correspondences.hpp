#pragma once

#include <algorithm>
#include <fstream>
#include <random>

#include "pt_cloud.hpp"

enum class ErrorMetric {
  PointToPlane,
  PointToPoint,
};

struct Dists {
  Eigen::VectorXd dists{};
  double mean{NAN};
  double median{NAN};
  double std{NAN};
  double std_mad{NAN};
  void ComputeStats();
};

struct CorrespondencesPointsWithAttributes {
  int num{};
  Eigen::MatrixX3d pc_fix_X{};
  Eigen::VectorXd pc_fix_nx{};
  Eigen::VectorXd pc_fix_ny{};
  Eigen::VectorXd pc_fix_nz{};
  Eigen::MatrixX3d pc_mov_X{};
  Eigen::MatrixX3d pc_mov_Xt{};
};

class Correspondences {
 public:
  Correspondences(PtCloud& pc_fix, PtCloud& pc_mov);
  void SelectPointsByVoxelStratifiedSampling(const uint32_t& max_correspondences_per_voxel);
  void MatchPointsByNearestNeighbor();
  void MatchPointsByCorrespondenceId();
  void RejectMaxEuclideanDistanceCriteria(const double& max_euclidean_distance);
  void RejectStdMadCriteria(ErrorMetric error_metric, double sigma_mad_factor);
  CorrespondencesPointsWithAttributes GetCorrespondences();
  void ComputeDists();
  void SetSelectedPoints(std::vector<int> idx_pc_fix);
  void ExportCorrespondences(const std::string& debug_file_name);

  uint64_t num();
  // Getters
  PtCloud& pc_fix();
  PtCloud& pc_mov();
  const Dists& point_to_plane_dists();
  const Dists& point_to_plane_dists_t();
  const Dists& euclidean_dists();
  const Dists& euclidean_dists_t();
  const Dists& dists_t(ErrorMetric error_metric);
  std::vector<int> GetSelectedPoints();

 private:
  Eigen::MatrixXd GetSelectedPoints_();
  Eigen::MatrixXd GetSelectedCorrespondenceIds_();

  PtCloud& pc_fix_;
  PtCloud& pc_mov_;
  std::vector<int> idx_pc_fix_;
  std::vector<int> idx_pc_mov_;
  Dists point_to_plane_dists_;
  Dists point_to_plane_dists_t_;
  Dists euclidean_dists_;
  Dists euclidean_dists_t_;
};

Eigen::MatrixXi KnnSearch(const Eigen::MatrixXd& X, const Eigen::MatrixXd& X_query,
                          const int& k = 1);

template <typename T>
std::vector<T> KeepSubsetOfVector(const std::vector<T>& old_vector, const std::vector<bool>& keep);

double Median(const Eigen::VectorXd& v);

// Median of absolute differences (mad) with respect to the median
double MAD(const Eigen::VectorXd& v);

double Std(const Eigen::VectorXd& v);

template <typename T>
std::vector<T> Range(T start, T stop, T step = 1);