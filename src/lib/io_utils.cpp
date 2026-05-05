#include "io_utils.hpp"

NamedColumnMatrix<Eigen::MatrixXd> ImportFileToMatrix(const std::string& path,
                                                      const bool& with_normals,
                                                      const bool& with_correspondence_id) {
  // Extract extension
  std::string extension = std::filesystem::path(path).extension().string();

  // Check if pdal supports the file format
  pdal::StageFactory factory;

  // Create pdal reader type based on suffix
  std::string pdal_reader_type;
  if (extension == ".las" || extension == ".laz") {
    pdal_reader_type = "readers.las";
  } else if (extension == ".ply") {
    pdal_reader_type = "readers.ply";
  } else if (extension == ".xyz" || extension == ".txt") {
    pdal_reader_type = "readers.text";
  } else {
    std::string error_string;
    error_string += "File format '" + extension + "' is not supported.\n";
    throw std::runtime_error(error_string);
  }

  // Check if a reader type is available
  pdal::Stage* reader = factory.createStage(pdal_reader_type);

  // Initialize PDAL reader
  pdal::PointTable table;
  pdal::Options options;
  options.add("filename", path);
  reader->setOptions(options);
  reader->prepare(table);

  // Execute the reader to read the point cloud
  pdal::PointViewSet view_set = reader->execute(table);

  // Take first view from view set (contains the point cloud data)
  pdal::PointViewPtr view = *view_set.begin();

  if (view->empty()) throw std::runtime_error("Point cloud is empty!");

  // Return 6D eigen matrix
  return ExtractMatrix(view, with_normals, with_correspondence_id);
}

NamedColumnMatrix<Eigen::MatrixXd> ExtractMatrix(const pdal::PointViewPtr view,
                                                 const bool& with_normals,
                                                 const bool& with_correspondence_id) {
  if (!view->hasDim(pdal::Dimension::Id::X) || !view->hasDim(pdal::Dimension::Id::Y) ||
      !view->hasDim(pdal::Dimension::Id::Z)) {
    std::string error_string;
    error_string += "Point cloud does not have all fields required!\n";
    error_string += "Fields required: X, Y, Z\n";
    error_string += PointcloudFieldsToString(view);
    throw std::runtime_error(error_string);
  }

  if (with_normals &&
      (!view->hasDim(pdal::Dimension::Id::NormalX) || !view->hasDim(pdal::Dimension::Id::NormalY) ||
       !view->hasDim(pdal::Dimension::Id::NormalZ))) {
    std::string error_string;
    error_string += "Point cloud does not have all fields required!\n";
    error_string += "Fields required: NormalX, NormalY, NormalZ\n";
    error_string += PointcloudFieldsToString(view);
    throw std::runtime_error(error_string);
  }

  // Create id for custom field "correspondence_id"
  auto correspondence_id_dimension = view->table().layout()->findDim("CorrespondenceID");
  if (with_correspondence_id && (correspondence_id_dimension == pdal::Dimension::Id::Unknown)) {
    std::string error_string;
    error_string += "Point cloud does not have all fields required!\n";
    error_string += "Fields required: CorrespondenceID\n";
    error_string += PointcloudFieldsToString(view);
    throw std::runtime_error(error_string);
  }

  int matrix_cols = with_normals ? 6 : 3;
  matrix_cols += with_correspondence_id ? 1 : 0;
  std::vector<std::string> col_names;
  col_names.emplace_back("x");
  col_names.emplace_back("y");
  col_names.emplace_back("z");
  if (with_normals) {
    col_names.emplace_back("nx");
    col_names.emplace_back("ny");
    col_names.emplace_back("nz");
  }
  if (with_correspondence_id) {
    col_names.emplace_back("correspondence_id");
  }
  NamedColumnMatrix<Eigen::MatrixXd> x(Eigen::MatrixXd(view->size(), matrix_cols), col_names);

  // Get named column indices once
  Eigen::Index x_col, y_col, z_col, nx_col = -1, ny_col = -1, nz_col = -1,
                                    correspondence_id_col = -1;
  x_col = x.namedColIndex("x");
  y_col = x.namedColIndex("y");
  z_col = x.namedColIndex("z");
  if (with_normals) {
    nx_col = x.namedColIndex("nx");
    ny_col = x.namedColIndex("ny");
    nz_col = x.namedColIndex("nz");
  }
  if (with_correspondence_id) {
    correspondence_id_col = x.namedColIndex("correspondence_id");
  }

  for (pdal::PointId idx = 0; idx < view->size(); idx++) {
    x(idx, x_col) = view->getFieldAs<double>(pdal::Dimension::Id::X, idx);
    x(idx, y_col) = view->getFieldAs<double>(pdal::Dimension::Id::Y, idx);
    x(idx, z_col) = view->getFieldAs<double>(pdal::Dimension::Id::Z, idx);
    if (with_normals) {
      x(idx, nx_col) = view->getFieldAs<double>(pdal::Dimension::Id::NormalX, idx);
      x(idx, ny_col) = view->getFieldAs<double>(pdal::Dimension::Id::NormalY, idx);
      x(idx, nz_col) = view->getFieldAs<double>(pdal::Dimension::Id::NormalZ, idx);
    }
    if (with_correspondence_id) {
      x(idx, correspondence_id_col) = view->getFieldAs<int>(correspondence_id_dimension, idx);
    }
  }

  return x;
}

void UpdateTransformedPointcloud(const pdal::PointViewPtr view,
                                 const NamedColumnMatrix<Eigen::MatrixXd>& x_updated) {
  if (!view->hasDim(pdal::Dimension::Id::X) || !view->hasDim(pdal::Dimension::Id::Y) ||
      !view->hasDim(pdal::Dimension::Id::Z)) {
    std::string error_string;
    error_string += "Point cloud does not have all required fields!\n";
    error_string += "Fields required: X, Y, Z\n";
    error_string += PointcloudFieldsToString(view);
    throw std::runtime_error(error_string);
  }

  // Get named column indices once
  Eigen::Index x_col, y_col, z_col;
  x_col = x_updated.namedColIndex("x");
  y_col = x_updated.namedColIndex("y");
  z_col = x_updated.namedColIndex("z");
  for (pdal::PointId idx = 0; idx < view->size(); idx++) {
    view->setField(pdal::Dimension::Id::X, idx, x_updated(idx, x_col));
    view->setField(pdal::Dimension::Id::Y, idx, x_updated(idx, y_col));
    view->setField(pdal::Dimension::Id::Z, idx, x_updated(idx, z_col));
  }
}

std::string PointcloudFieldsToString(const pdal::PointViewPtr view) {
  std::stringstream string_stream;
  string_stream << "Fields in pointcloud: ";
  for (auto const& dim : view->dims()) {
    string_stream << view->dimName(dim);
    if (dim != view->dims().back()) {
      string_stream << ", ";
    }
  }
  string_stream << std::endl;
  return string_stream.str();
}

void SaveMatrixToFile(const NamedColumnMatrix<Eigen::MatrixXd>& x_updated,
                      const std::string& path_in, const std::string& path_out) {
  // Extract extension
  std::string extension_in = std::filesystem::path(path_in).extension().string();
  std::string extension_out = std::filesystem::path(path_out).extension().string();

  // Create pdal reader/writer type based on suffix
  std::string pdal_reader_type = CreatePDALReaderType(extension_in);
  std::string pdal_writer_type = CreatePDALWriterType(extension_out);

  // Check if input and output file formats are the same
  if (extension_in != extension_out) {
    std::string error_string;
    error_string += "Input and output file formats are not the same!\n";
    error_string += "Input file format: " + extension_in + "\n";
    error_string += "Output file format: " + extension_out + "\n";
    throw std::runtime_error(error_string);
  }

  // Create stage factory
  pdal::StageFactory factory;

  // Check if a reader type is available
  pdal::Stage* reader = factory.createStage(pdal_reader_type);

  // Initialize PDAL reader
  pdal::PointTable table;
  pdal::Options options;
  options.add("filename", path_in);
  reader->setOptions(options);
  reader->prepare(table);

  // Execute the reader to read the point cloud
  pdal::PointViewSet view_set = reader->execute(table);

  // Take first view from view set (contains the point cloud data)
  pdal::PointViewPtr view = *view_set.begin();
  if (view->empty()) throw std::runtime_error("Point cloud is empty!");

  // Update x,y,z fields of pointcloud with transformed values
  UpdateTransformedPointcloud(view, x_updated);

  // PDAL Writer
  pdal::Stage* writer = factory.createStage(pdal_writer_type);
  pdal::Options writer_options = CreatePDALWriterOptions(extension_out);
  writer_options.add("filename", path_out);
  writer->setOptions(writer_options);

  // Link the reader to the writer with a buffer reader
  pdal::BufferReader buffer_reader;
  buffer_reader.addView(view);
  writer->setInput(buffer_reader);

  // Prepare the writer
  writer->prepare(table);

  // Execute the writer to write the point cloud
  writer->execute(table);
}

std::string CreatePDALReaderType(const std::string& extension) {
  std::string pdal_reader_type;
  if (extension == ".las" || extension == ".laz") {
    pdal_reader_type = "readers.las";
  } else if (extension == ".ply") {
    pdal_reader_type = "readers.ply";
  } else if (extension == ".xyz" || extension == ".txt") {
    pdal_reader_type = "readers.text";
  } else {
    std::string error_string;
    error_string += "File format '" + extension + "' is not supported.\n";
    throw std::runtime_error(error_string);
  }
  return pdal_reader_type;
}

std::string CreatePDALWriterType(const std::string& extension) {
  std::string pdal_writer_type;
  if (extension == ".las" || extension == ".laz") {
    pdal_writer_type = "writers.las";
  } else if (extension == ".ply") {
    pdal_writer_type = "writers.ply";
  } else if (extension == ".xyz" || extension == ".txt") {
    pdal_writer_type = "writers.text";
  } else {
    std::string error_string;
    error_string += "File format '" + extension + "' is not supported.\n";
    throw std::runtime_error(error_string);
  }
  return pdal_writer_type;
}

pdal::Options CreatePDALWriterOptions(const std::string& extension) {
  pdal::Options pdal_writer_options;
  if (extension == ".las" || extension == ".laz") {
    pdal_writer_options.add("forward", "all");
    pdal_writer_options.add("extra_dims", "all");
  } else if (extension == ".ply") {
  } else if (extension == ".xyz" || extension == ".txt") {
    pdal_writer_options.add("precision", "4");
  } else {
    std::string error_string;
    error_string += "File format '" + extension + "' is not supported.\n";
    throw std::runtime_error(error_string);
  }
  return pdal_writer_options;
}