#include "pt_cloud.hpp"

#include <fstream>
#include <iostream>

PtCloud::PtCloud(Eigen::MatrixXd X) : X_{X} {}

void PtCloud::SetNormals(Eigen::VectorXd nx, Eigen::VectorXd ny, Eigen::VectorXd nz) {
  nx_ = nx;
  ny_ = ny;
  nz_ = nz;
  has_normals_ = true;
}

void PtCloud::SetCorrespondenceId(Eigen::VectorXd correspondence_id) {
  correspondence_id_ = correspondence_id;
}

long PtCloud::NumPts() { return X_.rows(); }

void PtCloud::InitializeTranslationGrids(const double& voxel_size, const uint32_t& buffer_voxels,
                                         const std::vector<double>& grid_limits) {
  // Check if grid_limits elements are all zero
  bool grid_limits_are_not_set =
      std::all_of(grid_limits.begin(), grid_limits.end(), [](int i) { return i == 0; });

  std::vector<double> grid_limits_with_buffer(6);
  if (grid_limits_are_not_set) {
    grid_limits_with_buffer[0] = floor(x_min()) - buffer_voxels * voxel_size;
    grid_limits_with_buffer[1] = floor(y_min()) - buffer_voxels * voxel_size;
    grid_limits_with_buffer[2] = floor(z_min()) - buffer_voxels * voxel_size;
    grid_limits_with_buffer[3] =
        grid_limits_with_buffer[0] +
        ceil((x_max() - grid_limits_with_buffer[0]) / voxel_size) * voxel_size +
        buffer_voxels * voxel_size;
    grid_limits_with_buffer[4] =
        grid_limits_with_buffer[1] +
        ceil((y_max() - grid_limits_with_buffer[1]) / voxel_size) * voxel_size +
        buffer_voxels * voxel_size;
    grid_limits_with_buffer[5] =
        grid_limits_with_buffer[2] +
        ceil((z_max() - grid_limits_with_buffer[2]) / voxel_size) * voxel_size +
        buffer_voxels * voxel_size;
  } else {
    // ToDo Check if passed limits are valid
    grid_limits_with_buffer[0] = grid_limits[0] - buffer_voxels * voxel_size;
    grid_limits_with_buffer[1] = grid_limits[1] - buffer_voxels * voxel_size;
    grid_limits_with_buffer[2] = grid_limits[2] - buffer_voxels * voxel_size;
    grid_limits_with_buffer[3] = grid_limits[3] + buffer_voxels * voxel_size;
    grid_limits_with_buffer[4] = grid_limits[4] + buffer_voxels * voxel_size;
    grid_limits_with_buffer[5] = grid_limits[5] + buffer_voxels * voxel_size;
  }

  // ToDo Conversion double to int
  int x_num_voxels = (grid_limits_with_buffer[3] - grid_limits_with_buffer[0]) / voxel_size;
  int y_num_voxels = (grid_limits_with_buffer[4] - grid_limits_with_buffer[1]) / voxel_size;
  int z_num_voxels = (grid_limits_with_buffer[5] - grid_limits_with_buffer[2]) / voxel_size;
  Eigen::RowVector3d grid_origin{};
  grid_origin << grid_limits_with_buffer[0], grid_limits_with_buffer[1], grid_limits_with_buffer[2];

  int first_idx_adj{};
  first_idx_adj = 0;
  x_translation_grid_.Initialize(grid_origin, x_num_voxels, y_num_voxels, z_num_voxels, voxel_size,
                                 first_idx_adj);

  first_idx_adj = x_translation_grid_.num_grid_vals();
  y_translation_grid_.Initialize(grid_origin, x_num_voxels, y_num_voxels, z_num_voxels, voxel_size,
                                 first_idx_adj);

  first_idx_adj = x_translation_grid_.num_grid_vals() + y_translation_grid_.num_grid_vals();
  z_translation_grid_.Initialize(grid_origin, x_num_voxels, y_num_voxels, z_num_voxels, voxel_size,
                                 first_idx_adj);

  Xt_ = X_;
}

void PtCloud::ExportTranslationGrids(const std::string& filepath) {
  auto write_value = [](std::ofstream& file, const auto& value) {
    file.write(reinterpret_cast<const char*>(&value), sizeof(value));
  };

  // Open file
  std::ofstream file{filepath, std::ios::out | std::ios::binary};
  if (!file.is_open()) {
    std::cerr << "Cannot open file \"" << filepath << "\"!" << std::endl;
    exit(1);
  }

  // Write header
  HeaderInfo header_info;
  write_value(file, header_info.identifier);
  write_value(file, header_info.fileversion);
  write_value(file, x_translation_grid_.grid_origin()(0));
  write_value(file, x_translation_grid_.grid_origin()(1));
  write_value(file, x_translation_grid_.grid_origin()(2));
  write_value(file, x_translation_grid_.x_num_voxels());
  write_value(file, x_translation_grid_.y_num_voxels());
  write_value(file, x_translation_grid_.z_num_voxels());
  write_value(file, x_translation_grid_.voxel_size());
  file.seekp(header_info.length);

  // Write data
  for (int x_voxel_idx = 0; x_voxel_idx < x_translation_grid_.x_num_voxels() + 1; x_voxel_idx++)
    for (int y_voxel_idx = 0; y_voxel_idx < x_translation_grid_.y_num_voxels() + 1; y_voxel_idx++)
      for (int z_voxel_idx = 0; z_voxel_idx < x_translation_grid_.z_num_voxels() + 1;
           z_voxel_idx++) {
        // clang-format off
        write_value(file, x_translation_grid_.grid_vals()[x_voxel_idx][y_voxel_idx][z_voxel_idx].f);
        write_value(file, x_translation_grid_.grid_vals()[x_voxel_idx][y_voxel_idx][z_voxel_idx].fx);
        write_value(file, x_translation_grid_.grid_vals()[x_voxel_idx][y_voxel_idx][z_voxel_idx].fy);
        write_value(file, x_translation_grid_.grid_vals()[x_voxel_idx][y_voxel_idx][z_voxel_idx].fz);
        write_value(file, x_translation_grid_.grid_vals()[x_voxel_idx][y_voxel_idx][z_voxel_idx].fxy);
        write_value(file, x_translation_grid_.grid_vals()[x_voxel_idx][y_voxel_idx][z_voxel_idx].fxz);
        write_value(file, x_translation_grid_.grid_vals()[x_voxel_idx][y_voxel_idx][z_voxel_idx].fyz);
        write_value(file, x_translation_grid_.grid_vals()[x_voxel_idx][y_voxel_idx][z_voxel_idx].fxyz);

        write_value(file, y_translation_grid_.grid_vals()[x_voxel_idx][y_voxel_idx][z_voxel_idx].f);
        write_value(file, y_translation_grid_.grid_vals()[x_voxel_idx][y_voxel_idx][z_voxel_idx].fx);
        write_value(file, y_translation_grid_.grid_vals()[x_voxel_idx][y_voxel_idx][z_voxel_idx].fy);
        write_value(file, y_translation_grid_.grid_vals()[x_voxel_idx][y_voxel_idx][z_voxel_idx].fz);
        write_value(file, y_translation_grid_.grid_vals()[x_voxel_idx][y_voxel_idx][z_voxel_idx].fxy);
        write_value(file, y_translation_grid_.grid_vals()[x_voxel_idx][y_voxel_idx][z_voxel_idx].fxz);
        write_value(file, y_translation_grid_.grid_vals()[x_voxel_idx][y_voxel_idx][z_voxel_idx].fyz);
        write_value(file, y_translation_grid_.grid_vals()[x_voxel_idx][y_voxel_idx][z_voxel_idx].fxyz);

        write_value(file, z_translation_grid_.grid_vals()[x_voxel_idx][y_voxel_idx][z_voxel_idx].f);
        write_value(file, z_translation_grid_.grid_vals()[x_voxel_idx][y_voxel_idx][z_voxel_idx].fx);
        write_value(file, z_translation_grid_.grid_vals()[x_voxel_idx][y_voxel_idx][z_voxel_idx].fy);
        write_value(file, z_translation_grid_.grid_vals()[x_voxel_idx][y_voxel_idx][z_voxel_idx].fz);
        write_value(file, z_translation_grid_.grid_vals()[x_voxel_idx][y_voxel_idx][z_voxel_idx].fxy);
        write_value(file, z_translation_grid_.grid_vals()[x_voxel_idx][y_voxel_idx][z_voxel_idx].fxz);
        write_value(file, z_translation_grid_.grid_vals()[x_voxel_idx][y_voxel_idx][z_voxel_idx].fyz);
        write_value(file, z_translation_grid_.grid_vals()[x_voxel_idx][y_voxel_idx][z_voxel_idx].fxyz);
        // clang-format on
      }

  // Final check and close file
  if (!file.good()) {
    std::cerr << "Error occurred writing file \"" << filepath << "\"!" << std::endl;
    exit(1);
  }
  file.close();
}

void PtCloud::ImportTranslationGrids(const std::string& filepath) {
  auto read_value = [](std::ifstream& file, auto& var) {
    file.read(reinterpret_cast<char*>(&var), sizeof(var));
  };

  // Open file
  std::ifstream file{filepath, std::ios::in | std::ios::binary};
  if (!file.is_open()) {
    std::cerr << "Cannot open file \"" << filepath << "\"!" << std::endl;
    exit(1);
  }

  // Read header
  HeaderInfo header_info;
  Eigen::RowVector3d grid_origin(3);
  int x_num_voxels{};
  int y_num_voxels{};
  int z_num_voxels{};
  double voxel_size{};
  read_value(file, header_info.identifier);
  if (strcmp(header_info.identifier, "nricp") != 0) {  // check identifier
    std::cerr << "Header of \"" << filepath << "\" does not start with char \"nricp\"!"
              << std::endl;
    exit(1);
  }
  int current_fileversion{header_info.fileversion};  // save file version for check
  read_value(file, header_info.fileversion);
  if (header_info.fileversion != current_fileversion) {  // check file version
    std::cerr << "File version of \"" << filepath << "\" is \"" << header_info.fileversion
              << "\", but should be \"" << current_fileversion << "\"!" << std::endl;
    exit(1);
  }
  read_value(file, grid_origin(0));
  read_value(file, grid_origin(1));
  read_value(file, grid_origin(2));
  read_value(file, x_num_voxels);
  read_value(file, y_num_voxels);
  read_value(file, z_num_voxels);
  read_value(file, voxel_size);
  file.seekg(header_info.length);

  // Verify header
  HeaderInfo header_info_for_verification;
  if (strcmp(header_info.identifier, header_info_for_verification.identifier) != 0) {
    std::cerr << "Error importing \"" << filepath << "\"!" << std::endl;
    std::cerr << "Header identifier is \"" << header_info.identifier << "\" but must be \""
              << header_info_for_verification.identifier << "\"!" << std::endl;
  }
  if (header_info.fileversion != header_info_for_verification.fileversion) {
    std::cerr << "Error importing \"" << filepath << "\"!" << std::endl;
    std::cerr << "Header fileversion is \"" << header_info.fileversion << "\" but must be \""
              << header_info_for_verification.fileversion << "\"!" << std::endl;
  }

  // Initialize grids
  // ToDo Make first_idx_adj an optional argument
  x_translation_grid_.Initialize(grid_origin, x_num_voxels, y_num_voxels, z_num_voxels, voxel_size,
                                 0);
  y_translation_grid_.Initialize(grid_origin, x_num_voxels, y_num_voxels, z_num_voxels, voxel_size,
                                 0);
  z_translation_grid_.Initialize(grid_origin, x_num_voxels, y_num_voxels, z_num_voxels, voxel_size,
                                 0);

  GridVals grid_vals_new{};

  for (int x_voxel_idx = 0; x_voxel_idx < x_num_voxels + 1; x_voxel_idx++)
    for (int y_voxel_idx = 0; y_voxel_idx < y_num_voxels + 1; y_voxel_idx++)
      for (int z_voxel_idx = 0; z_voxel_idx < z_num_voxels + 1; z_voxel_idx++) {
        read_value(file, grid_vals_new.f);
        read_value(file, grid_vals_new.fx);
        read_value(file, grid_vals_new.fy);
        read_value(file, grid_vals_new.fz);
        read_value(file, grid_vals_new.fxy);
        read_value(file, grid_vals_new.fxz);
        read_value(file, grid_vals_new.fyz);
        read_value(file, grid_vals_new.fxyz);
        x_translation_grid().UpdateVoxelGridVals(x_voxel_idx, y_voxel_idx, z_voxel_idx,
                                                 grid_vals_new);

        read_value(file, grid_vals_new.f);
        read_value(file, grid_vals_new.fx);
        read_value(file, grid_vals_new.fy);
        read_value(file, grid_vals_new.fz);
        read_value(file, grid_vals_new.fxy);
        read_value(file, grid_vals_new.fxz);
        read_value(file, grid_vals_new.fyz);
        read_value(file, grid_vals_new.fxyz);
        y_translation_grid().UpdateVoxelGridVals(x_voxel_idx, y_voxel_idx, z_voxel_idx,
                                                 grid_vals_new);

        read_value(file, grid_vals_new.f);
        read_value(file, grid_vals_new.fx);
        read_value(file, grid_vals_new.fy);
        read_value(file, grid_vals_new.fz);
        read_value(file, grid_vals_new.fxy);
        read_value(file, grid_vals_new.fxz);
        read_value(file, grid_vals_new.fyz);
        read_value(file, grid_vals_new.fxyz);
        z_translation_grid().UpdateVoxelGridVals(x_voxel_idx, y_voxel_idx, z_voxel_idx,
                                                 grid_vals_new);
      }

  // Final check and close file
  if (!file.good()) {
    std::cerr << "Error occurred reading file \"" << filepath << "\"!" << std::endl;
    exit(1);
  }
  file.close();
}

void PtCloud::InitMatricesForUpdateXt() {
  auto [X_voxel_idx, Xn_voxel]{x_translation_grid_.GetGridReference(X_)};
  X_voxel_idx_ = X_voxel_idx;
  Xn_voxel_ = Xn_voxel;
  X_power_ = TranslationGrid::Compute_X_power(Xn_voxel);
}

void PtCloud::UpdateXt() {
  auto tx{x_translation_grid_.p(X_, X_power_, X_voxel_idx_)};
  auto ty{y_translation_grid_.p(X_, X_power_, X_voxel_idx_)};
  auto tz{z_translation_grid_.p(X_, X_power_, X_voxel_idx_)};

  Xt_ = Eigen::MatrixX3d(NumPts(), 3);
  Xt_ << X_.col(0) + tx, X_.col(1) + ty, X_.col(2) + tz;
}

const Eigen::MatrixXd& PtCloud::X() { return X_; }
const Eigen::MatrixXd& PtCloud::Xt() { return Xt_; }
const Eigen::VectorXd& PtCloud::nx() { return nx_; }
const Eigen::VectorXd& PtCloud::ny() { return ny_; }
const Eigen::VectorXd& PtCloud::nz() { return nz_; }
bool PtCloud::has_normals() const { return has_normals_; }
const Eigen::VectorXd& PtCloud::correspondence_id() { return correspondence_id_; }

TranslationGrid& PtCloud::x_translation_grid() { return x_translation_grid_; }
TranslationGrid& PtCloud::y_translation_grid() { return y_translation_grid_; }
TranslationGrid& PtCloud::z_translation_grid() { return z_translation_grid_; }
double PtCloud::x_min() { return X_.col(0).minCoeff(); }
double PtCloud::x_max() { return X_.col(0).maxCoeff(); }
double PtCloud::y_min() { return X_.col(1).minCoeff(); }
double PtCloud::y_max() { return X_.col(1).maxCoeff(); }
double PtCloud::z_min() { return X_.col(2).minCoeff(); }
double PtCloud::z_max() { return X_.col(2).maxCoeff(); }
