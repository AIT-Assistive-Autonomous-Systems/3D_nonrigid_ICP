#include "correspondences.hpp"

#include <nanoflann.hpp>
#include <random>
#include <tuple>
#include <unordered_map>

const int LEAF_SIZE{200};

namespace {
struct VoxelHash {
  size_t operator()(const std::tuple<int, int, int>& v) const noexcept {
    // boost::hash_combine pattern: 0x9e3779b9... is the golden-ratio constant
    // used to mix bits across the three voxel indices.
    size_t h = static_cast<size_t>(std::get<0>(v));
    h ^= static_cast<size_t>(std::get<1>(v)) + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
    h ^= static_cast<size_t>(std::get<2>(v)) + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
    return h;
  }
};
}  // namespace

Correspondences::Correspondences(PtCloud& pc_fix, PtCloud& pc_mov)
    : pc_fix_{pc_fix}, pc_mov_{pc_mov} {}

void Correspondences::SelectPointsByVoxelStratifiedSampling(
    const uint32_t& max_correspondences_per_voxel) {
  const auto& grid = pc_mov_.x_translation_grid();
  const double voxel_size = grid.voxel_size();
  const auto& origin = grid.grid_origin();
  const int nx = grid.x_num_voxels();
  const int ny = grid.y_num_voxels();
  const int nz = grid.z_num_voxels();

  std::unordered_map<std::tuple<int, int, int>, std::vector<int>, VoxelHash> bins;
  const auto& X = pc_fix_.X();
  for (long i = 0; i < pc_fix_.NumPts(); i++) {
    int vx = static_cast<int>(std::floor((X(i, 0) - origin(0)) / voxel_size));
    int vy = static_cast<int>(std::floor((X(i, 1) - origin(1)) / voxel_size));
    int vz = static_cast<int>(std::floor((X(i, 2) - origin(2)) / voxel_size));
    if (vx < 0 || vx >= nx || vy < 0 || vy >= ny || vz < 0 || vz >= nz) continue;
    bins[{vx, vy, vz}].push_back(static_cast<int>(i));
  }

  std::vector<int> selected;
  auto rng = std::default_random_engine{};
  for (auto& [voxel, points] : bins) {
    if (points.size() <= max_correspondences_per_voxel) {
      selected.insert(selected.end(), points.begin(), points.end());
    } else {
      std::shuffle(points.begin(), points.end(), rng);
      selected.insert(selected.end(), points.begin(),
                      points.begin() + max_correspondences_per_voxel);
    }
  }

  std::sort(selected.begin(), selected.end());
  idx_pc_fix_ = std::move(selected);

  if (idx_pc_fix_.empty()) {
    throw std::runtime_error(
        "Voxel-stratified sampling produced no correspondences: every fixed-cloud point lies "
        "outside the moving cloud's translation grid. Are the point clouds overlapping?");
  }
}

void Correspondences::MatchPointsByNearestNeighbor() {
  Eigen::MatrixXd pc_fix_X_sel{GetSelectedPoints_()};

  idx_pc_mov_ = std::vector<int>(num());

  Eigen::MatrixXi idx_nn(num(), 1);
  idx_nn = KnnSearch(pc_mov_.Xt(), pc_fix_X_sel, 1);
  for (int i = 0; i < idx_nn.rows(); i++) {
    idx_pc_mov_[i] = idx_nn(i, 0);
  }

  ComputeDists();
}

void Correspondences::MatchPointsByCorrespondenceId() {
  Eigen::MatrixXd pc_fix_X_sel{GetSelectedCorrespondenceIds_()};

  idx_pc_mov_ = std::vector<int>(num());

  Eigen::MatrixXi idx_nn(num(), 1);
  idx_nn = KnnSearch(pc_mov_.correspondence_id(), pc_fix_X_sel, 1);

  std::vector<bool> keep(num(), true);

  for (int i = 0; i < idx_nn.rows(); i++) {
    // Take only matches with same correspondence id
    auto nn{idx_nn(i, 0)};
    auto distance = abs(pc_mov_.correspondence_id()(nn) - pc_fix_X_sel(i, 0));
    if (distance == 0) {
      idx_pc_mov_[i] = idx_nn(i, 0);
    } else {
      keep[i] = false;
    }
  }

  idx_pc_fix_ = KeepSubsetOfVector(idx_pc_fix_, keep);
  idx_pc_mov_ = KeepSubsetOfVector(idx_pc_mov_, keep);

  if (num() == 0) {
    throw std::runtime_error(
        "No correspondences found while matching points by id! Please check the correspondence "
        "ids.");
  }

  ComputeDists();
}

void Correspondences::RejectMaxEuclideanDistanceCriteria(const double& max_euclidean_distance) {
  std::vector<bool> keep(num(), true);

  for (uint64_t i = 0; i < num(); i++) {
    if (euclidean_dists_t_.dists[i] > max_euclidean_distance) {
      keep[i] = false;
    }
  }

  idx_pc_fix_ = KeepSubsetOfVector(idx_pc_fix_, keep);
  idx_pc_mov_ = KeepSubsetOfVector(idx_pc_mov_, keep);

  ComputeDists();
}

void Correspondences::RejectStdMadCriteria(ErrorMetric error_metric, double sigma_mad_factor) {
  // Skip rejection when disabled. The cached Dists remain valid because
  // RejectMaxEuclideanDistanceCriteria (the prior step) ends with ComputeDists().
  if (sigma_mad_factor <= 0.0) return;

  std::vector<bool> keep(num(), true);

  const Dists& d = dists_t(error_metric);

  for (uint64_t i = 0; i < num(); i++) {
    if (abs(d.dists[i] - d.median) > sigma_mad_factor * d.std_mad) {
      keep[i] = false;
    }
  }

  idx_pc_fix_ = KeepSubsetOfVector(idx_pc_fix_, keep);
  idx_pc_mov_ = KeepSubsetOfVector(idx_pc_mov_, keep);

  ComputeDists();
}

CorrespondencesPointsWithAttributes Correspondences::GetCorrespondences() {
  int num_correspondences{static_cast<int>(idx_pc_fix_.size())};

  const bool has_normals = pc_fix_.has_normals();

  Eigen::MatrixX3d pc_fix_X(num_correspondences, 3);
  Eigen::VectorXd pc_fix_nx;
  Eigen::VectorXd pc_fix_ny;
  Eigen::VectorXd pc_fix_nz;
  if (has_normals) {
    pc_fix_nx.resize(num_correspondences);
    pc_fix_ny.resize(num_correspondences);
    pc_fix_nz.resize(num_correspondences);
  }
  Eigen::MatrixX3d pc_mov_X(num_correspondences, 3);
  Eigen::MatrixX3d pc_mov_Xt(num_correspondences, 3);

  for (int i = 0; i < num_correspondences; i++) {
    pc_fix_X(i, 0) = pc_fix_.X()(idx_pc_fix_[i], 0);
    pc_fix_X(i, 1) = pc_fix_.X()(idx_pc_fix_[i], 1);
    pc_fix_X(i, 2) = pc_fix_.X()(idx_pc_fix_[i], 2);

    if (has_normals) {
      pc_fix_nx(i) = pc_fix_.nx()(idx_pc_fix_[i]);
      pc_fix_ny(i) = pc_fix_.ny()(idx_pc_fix_[i]);
      pc_fix_nz(i) = pc_fix_.nz()(idx_pc_fix_[i]);
    }

    pc_mov_X(i, 0) = pc_mov_.X()(idx_pc_mov_[i], 0);
    pc_mov_X(i, 1) = pc_mov_.X()(idx_pc_mov_[i], 1);
    pc_mov_X(i, 2) = pc_mov_.X()(idx_pc_mov_[i], 2);

    pc_mov_Xt(i, 0) = pc_mov_.Xt()(idx_pc_mov_[i], 0);
    pc_mov_Xt(i, 1) = pc_mov_.Xt()(idx_pc_mov_[i], 1);
    pc_mov_Xt(i, 2) = pc_mov_.Xt()(idx_pc_mov_[i], 2);
  }

  CorrespondencesPointsWithAttributes X{};
  X.num = num_correspondences;
  X.pc_fix_X = pc_fix_X;
  X.pc_fix_nx = pc_fix_nx;
  X.pc_fix_ny = pc_fix_ny;
  X.pc_fix_nz = pc_fix_nz;
  X.pc_mov_X = pc_mov_X;
  X.pc_mov_Xt = pc_mov_Xt;

  return X;
}

template <typename T>
std::vector<T> KeepSubsetOfVector(const std::vector<T>& old_vector, const std::vector<bool>& keep) {
  size_t num_remaining = count(keep.begin(), keep.end(), true);
  std::vector<T> new_vector(num_remaining);
  int j{0};
  for (size_t i = 0; i < old_vector.size(); i++) {
    if (keep[i]) {
      new_vector[j] = old_vector[i];
      j++;
    }
  }
  return new_vector;
}

Eigen::MatrixXd Correspondences::GetSelectedPoints_() {
  Eigen::MatrixXd X_sel(idx_pc_fix_.size(), 3);
  for (size_t i = 0; i < idx_pc_fix_.size(); i++) {
    X_sel(i, 0) = pc_fix_.X()(idx_pc_fix_[i], 0);
    X_sel(i, 1) = pc_fix_.X()(idx_pc_fix_[i], 1);
    X_sel(i, 2) = pc_fix_.X()(idx_pc_fix_[i], 2);
  }
  return X_sel;
}

Eigen::MatrixXd Correspondences::GetSelectedCorrespondenceIds_() {
  Eigen::MatrixXd X_sel(idx_pc_fix_.size(), 1);
  for (size_t i = 0; i < idx_pc_fix_.size(); i++) {
    X_sel(i, 0) = pc_fix_.correspondence_id()(idx_pc_fix_[i]);
  }
  return X_sel;
}

void Correspondences::ComputeDists() {
  const bool has_normals = pc_fix_.has_normals();

  point_to_plane_dists_ = Dists{};
  point_to_plane_dists_t_ = Dists{};
  euclidean_dists_.dists = Eigen::VectorXd(idx_pc_fix_.size());
  euclidean_dists_t_.dists = Eigen::VectorXd(idx_pc_fix_.size());
  if (has_normals) {
    point_to_plane_dists_.dists = Eigen::VectorXd(idx_pc_fix_.size());
    point_to_plane_dists_t_.dists = Eigen::VectorXd(idx_pc_fix_.size());
  }

  auto X{GetCorrespondences()};

  if (X.num == 0) {
    throw std::runtime_error("Number of correspondences is zero!");
  }

  for (int i = 0; i < X.num; i++) {
    double dx{X.pc_mov_X(i, 0) - X.pc_fix_X(i, 0)};
    double dy{X.pc_mov_X(i, 1) - X.pc_fix_X(i, 1)};
    double dz{X.pc_mov_X(i, 2) - X.pc_fix_X(i, 2)};

    double dxt{X.pc_mov_Xt(i, 0) - X.pc_fix_X(i, 0)};
    double dyt{X.pc_mov_Xt(i, 1) - X.pc_fix_X(i, 1)};
    double dzt{X.pc_mov_Xt(i, 2) - X.pc_fix_X(i, 2)};

    euclidean_dists_.dists(i) = sqrt(pow(dx, 2) + pow(dy, 2) + pow(dz, 2));
    euclidean_dists_t_.dists(i) = sqrt(pow(dxt, 2) + pow(dyt, 2) + pow(dzt, 2));

    if (has_normals) {
      point_to_plane_dists_.dists(i) =
          dx * X.pc_fix_nx(i) + dy * X.pc_fix_ny(i) + dz * X.pc_fix_nz(i);
      point_to_plane_dists_t_.dists(i) =
          dxt * X.pc_fix_nx(i) + dyt * X.pc_fix_ny(i) + dzt * X.pc_fix_nz(i);
    }
  }

  if (has_normals) {
    point_to_plane_dists_.ComputeStats();
    point_to_plane_dists_t_.ComputeStats();
  }
  euclidean_dists_.ComputeStats();
  euclidean_dists_t_.ComputeStats();
}

void Dists::ComputeStats() {
  mean = dists.mean();
  median = Median(dists);
  std = Std(dists);
  std_mad = 1.4826 * MAD(dists);
}

Eigen::MatrixXi KnnSearch(const Eigen::MatrixXd& X, const Eigen::MatrixXd& X_query, const int& k) {
  // Create kd tree
  typedef nanoflann::KDTreeEigenMatrixAdaptor<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>>
      kd_tree;
  kd_tree mat_index(X.cols(), std::cref(X), LEAF_SIZE);

  // Iterate over all query points
  Eigen::MatrixXi mat_idx_nn(X_query.rows(), k);
  for (Eigen::Index i = 0; i < X_query.rows(); i++) {
    // Query point
    Eigen::VectorXd row = X_query.row(i);
    std::vector<double> qp{row.data(), row.data() + row.size()};

    // Search for nn of query point
    std::vector<size_t> idx_nn(k);
    std::vector<double> dists_nn(k);  // not used
    nanoflann::KNNResultSet<double> resultSet(k);
    resultSet.init(&idx_nn[0], &dists_nn[0]);
    mat_index.index_->findNeighbors(resultSet, &qp[0], nanoflann::SearchParameters(10));

    // Save indices of nn to matrix
    for (int j = 0; j < k; j++) {
      mat_idx_nn(i, j) = idx_nn[j];
    }
  }
  return mat_idx_nn;
}

double Median(const Eigen::VectorXd& v) {
  // VectorXd --> vector<double>
  std::vector<double> vv(v.size());
  for (int i = 1; i < v.size(); i++) {
    vv[i] = v[i];
  }

  // Median
  const auto median_it = vv.begin() + vv.size() / 2;
  nth_element(vv.begin(), median_it, vv.end());
  auto median = *median_it;

  return median;
}

double MAD(const Eigen::VectorXd& v) {
  auto med{Median(v)};
  Eigen::VectorXd dmed(v.size());
  for (int i = 1; i < v.size(); i++) {
    dmed[i] = abs(v[i] - med);
  }
  auto mad{Median(dmed)};
  return mad;
}

double Std(const Eigen::VectorXd& v) {
  double std{sqrt((v.array() - v.mean()).square().sum() / ((double)v.size() - 1))};
  return std;
}

template <typename T>
std::vector<T> Range(T start, T stop, T step) {
  std::vector<T> vals;
  vals.reserve(ceil((stop - start) / (step - 1)));
  for (T val = start; val < stop; val += step) vals.push_back(val);
  return vals;
}

uint64_t Correspondences::num() { return idx_pc_fix_.size(); }

PtCloud& Correspondences::pc_fix() { return pc_fix_; }
PtCloud& Correspondences::pc_mov() { return pc_mov_; }
const Dists& Correspondences::point_to_plane_dists() { return point_to_plane_dists_; }
const Dists& Correspondences::point_to_plane_dists_t() { return point_to_plane_dists_t_; }
const Dists& Correspondences::euclidean_dists() { return euclidean_dists_; }
const Dists& Correspondences::euclidean_dists_t() { return euclidean_dists_t_; }
const Dists& Correspondences::dists_t(ErrorMetric error_metric) {
  return (error_metric == ErrorMetric::PointToPlane) ? point_to_plane_dists_t_ : euclidean_dists_t_;
}
std::vector<int> Correspondences::GetSelectedPoints() { return idx_pc_fix_; }
void Correspondences::SetSelectedPoints(const std::vector<int> idx_pc_fix) {
  idx_pc_fix_ = idx_pc_fix;
}

void Correspondences::ExportCorrespondences(const std::string& filepath) {
  auto X{GetCorrespondences()};

  std::ofstream file(filepath);
  if (file.is_open()) {
    for (int i = 0; i < X.pc_fix_X.rows(); i++) {
      file << X.pc_fix_X(i, 0) << " " << X.pc_fix_X(i, 1) << " " << X.pc_fix_X(i, 2) << std::endl;
      file << X.pc_mov_Xt(i, 0) << " " << X.pc_mov_Xt(i, 1) << " " << X.pc_mov_Xt(i, 2) << std::endl
           << std::endl;
    }
    file.close();
  } else {
    std::string error_string = "Unable to open file \"" + filepath + "\".";
    throw std::runtime_error(error_string);
  }
}