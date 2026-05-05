#include "optimization.hpp"

#include <fmt/format.h>

namespace {
const char* ComputationInfoToString(Eigen::ComputationInfo info) {
  switch (info) {
    case Eigen::Success:
      return "Success";
    case Eigen::NumericalIssue:
      return "NumericalIssue";
    case Eigen::NoConvergence:
      return "NoConvergence";
    case Eigen::InvalidInput:
      return "InvalidInput";
  }
  return "Unknown";
}
}  // namespace

Optimization::Optimization() = default;

OptimizationResults Optimization::Solve(Correspondences& correspondences,
                                        const std::vector<double>& weights_zero_observations,
                                        ErrorMetric error_metric) {
  OptimizationResults optimization_results{};  // returned

  CorrespondencesPointsWithAttributes X{correspondences.GetCorrespondences()};

  auto J_pc_mov_x_triplets{correspondences.pc_mov().x_translation_grid().J(X.pc_mov_X)};
  auto J_pc_mov_y_triplets{correspondences.pc_mov().y_translation_grid().J(X.pc_mov_X)};
  auto J_pc_mov_z_triplets{correspondences.pc_mov().z_translation_grid().J(X.pc_mov_X)};

  int num_unknowns{correspondences.pc_mov().x_translation_grid().num_grid_vals() +
                   correspondences.pc_mov().y_translation_grid().num_grid_vals() +
                   correspondences.pc_mov().z_translation_grid().num_grid_vals()};

  auto J_direct_obs_triplets(Optimization::SparseIdentity(num_unknowns));

  int num_correspondence_obs{};
  int num_observations{};
  std::vector<Eigen::Triplet<double>> J_triplets;
  Eigen::VectorXd b0;
  Eigen::VectorXd p;

  if (error_metric == ErrorMetric::PointToPlane) {
    auto J_pc_mov_x_nx_triplets{
        Optimization::MultiplyWithComponentsOfNormalVectors(J_pc_mov_x_triplets, X.pc_fix_nx)};
    auto J_pc_mov_y_ny_triplets{
        Optimization::MultiplyWithComponentsOfNormalVectors(J_pc_mov_y_triplets, X.pc_fix_ny)};
    auto J_pc_mov_z_nz_triplets{
        Optimization::MultiplyWithComponentsOfNormalVectors(J_pc_mov_z_triplets, X.pc_fix_nz)};

    // J_pc_mov_*_triplets not needed past this point in the point-to-plane branch. Thus, we free
    // their memory here by swapping with empty vectors (a clear() would not free memory).
    std::vector<Eigen::Triplet<double>>().swap(J_pc_mov_x_triplets);
    std::vector<Eigen::Triplet<double>>().swap(J_pc_mov_y_triplets);
    std::vector<Eigen::Triplet<double>>().swap(J_pc_mov_z_triplets);

    num_correspondence_obs = X.num;
    num_observations = num_correspondence_obs + num_unknowns;

    J_triplets.reserve(J_pc_mov_x_nx_triplets.size() + J_pc_mov_y_ny_triplets.size() +
                       J_pc_mov_z_nz_triplets.size() + J_direct_obs_triplets.size());
    Optimization::AddSubblockTriplets(0, 0, J_pc_mov_x_nx_triplets, J_triplets);
    Optimization::AddSubblockTriplets(0, 0, J_pc_mov_y_ny_triplets, J_triplets);
    Optimization::AddSubblockTriplets(0, 0, J_pc_mov_z_nz_triplets, J_triplets);
    Optimization::AddSubblockTriplets(num_correspondence_obs, 0, J_direct_obs_triplets, J_triplets);

    p = Eigen::VectorXd(num_observations);
    p << Eigen::VectorXd::Ones(num_correspondence_obs),
        Optimization::BuildZeroObservationWeights(num_unknowns, weights_zero_observations);

    b0 = Eigen::VectorXd(num_observations);
    b0 << correspondences.point_to_plane_dists().dists, Eigen::VectorXd::Zero(num_unknowns);
  } else {
    num_correspondence_obs = 3 * X.num;
    num_observations = num_correspondence_obs + num_unknowns;

    J_triplets.reserve(J_pc_mov_x_triplets.size() + J_pc_mov_y_triplets.size() +
                       J_pc_mov_z_triplets.size() + J_direct_obs_triplets.size());
    // first_col is 0 for all three grid blocks: each grid's J() already emits
    // triplets with column indices offset by its first_idx_adj, so the x/y/z
    // blocks land in disjoint column ranges of the unknowns vector.
    Optimization::AddSubblockTriplets(0, 0, J_pc_mov_x_triplets, J_triplets);
    Optimization::AddSubblockTriplets(X.num, 0, J_pc_mov_y_triplets, J_triplets);
    Optimization::AddSubblockTriplets(2 * X.num, 0, J_pc_mov_z_triplets, J_triplets);
    Optimization::AddSubblockTriplets(num_correspondence_obs, 0, J_direct_obs_triplets, J_triplets);

    p = Eigen::VectorXd(num_observations);
    p << Eigen::VectorXd::Ones(num_correspondence_obs),
        Optimization::BuildZeroObservationWeights(num_unknowns, weights_zero_observations);

    b0 = Eigen::VectorXd(num_observations);
    b0.segment(0, X.num) = X.pc_mov_X.col(0) - X.pc_fix_X.col(0);
    b0.segment(X.num, X.num) = X.pc_mov_X.col(1) - X.pc_fix_X.col(1);
    b0.segment(2 * X.num, X.num) = X.pc_mov_X.col(2) - X.pc_fix_X.col(2);
    b0.segment(num_correspondence_obs, num_unknowns).setZero();
  }

  Eigen::SparseMatrix<double> J(num_observations, num_unknowns);
  J.setFromTriplets(J_triplets.begin(), J_triplets.end());

  auto P{p.asDiagonal()};

  Eigen::VectorXd b = Eigen::VectorXd::Zero(num_observations);
  auto l{b - b0};

  // Solve!
  Eigen::VectorXd xhat(num_unknowns);
  Eigen::BiCGSTAB<Eigen::SparseMatrix<double>> solver;
  solver.compute(J.transpose() * P * J);
  if (solver.info() != Eigen::Success) {
    optimization_results.success = false;
    optimization_results.error_message = fmt::format(
        "BiCGSTAB::compute failed with status \"{}\" (num_observations={}, num_unknowns={}).",
        ComputationInfoToString(solver.info()), num_observations, num_unknowns);
    return optimization_results;
  }
  xhat = solver.solve(J.transpose() * P * l);
  if (solver.info() != Eigen::Success) {
    optimization_results.success = false;
    optimization_results.error_message = fmt::format(
        "BiCGSTAB::solve failed with status \"{}\" after {} iterations (error={:.3e}, "
        "tolerance={:.3e}, max_iterations={}, num_observations={}, num_unknowns={}).",
        ComputationInfoToString(solver.info()), solver.iterations(), solver.error(),
        solver.tolerance(), solver.maxIterations(), num_observations, num_unknowns);
    return optimization_results;
  }
  optimization_results.success = true;

  // auto v{J * xhat - l};

  // Save estimated unknowns to translation grids
  correspondences.pc_mov().x_translation_grid().UpdateAllGridValsFromVector(xhat);
  correspondences.pc_mov().y_translation_grid().UpdateAllGridValsFromVector(xhat);
  correspondences.pc_mov().z_translation_grid().UpdateAllGridValsFromVector(xhat);
  correspondences.pc_mov().UpdateXt();
  correspondences.ComputeDists();

  optimization_results.num_observations = num_observations;
  optimization_results.num_unknowns = num_unknowns;

  return optimization_results;
}

std::vector<Eigen::Triplet<double>> Optimization::SparseIdentity(const int& n) {
  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(n);
  for (int i = 0; i < n; i++) {
    triplets.emplace_back(i, i, 1.0);
  }
  return triplets;
}

std::vector<Eigen::Triplet<double>> Optimization::MultiplyWithComponentsOfNormalVectors(
    const std::vector<Eigen::Triplet<double>>& triplets_in, const Eigen::VectorXd& n_component) {
  std::vector<Eigen::Triplet<double>> triplets_out;
  triplets_out.reserve(triplets_in.size());
  for (auto const& triplet : triplets_in) {
    int row{triplet.row()};
    int col{triplet.col()};
    double val{triplet.value() * n_component(triplet.row())};
    triplets_out.emplace_back(row, col, val);
  }

  return triplets_out;
}

void Optimization::AddSubblockTriplets(const int& first_row, const int& first_col,
                                       const std::vector<Eigen::Triplet<double>>& subblock_triplets,
                                       std::vector<Eigen::Triplet<double>>& triplets) {
  for (auto const& triplet : subblock_triplets) {
    int row{first_row + triplet.row()};
    int col{first_col + triplet.col()};
    double val{triplet.value()};
    triplets.emplace_back(row, col, val);
  }
}

Eigen::VectorXd Optimization::BuildZeroObservationWeights(int num_unknowns,
                                                          const std::vector<double>& weights) {
  // Each grid corner contributes 8 unknowns laid out as
  // [f, fx, fy, fz, fxy, fxz, fyz, fxyz]; this layout repeats over corners
  // and across the three (x/y/z) translation grids. Map each derivative class
  // to its weight: weights[0]=f, [1]=fx/fy/fz, [2]=fxy/fxz/fyz, [3]=fxyz.
  const double w_per_unknown[8] = {weights[0], weights[1], weights[1], weights[1],
                                   weights[2], weights[2], weights[2], weights[3]};
  Eigen::VectorXd p(num_unknowns);
  for (int i = 0; i < num_unknowns; i++) p(i) = w_per_unknown[i % 8];
  return p;
}
