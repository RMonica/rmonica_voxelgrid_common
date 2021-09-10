#ifndef METADATA_H
#define METADATA_H

#include <ros/ros.h>

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <memory>
#include <string>
#include <fstream>
#include <sstream>

namespace VoxelgridMetadata
{

  struct Metadata
  {
    Eigen::Vector3f bbox_min = Eigen::Vector3f::Zero();
    Eigen::Vector3f bbox_max = Eigen::Vector3f::Zero();
    float voxel_size = 0.0;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  };
  typedef std::shared_ptr<Metadata> MetadataPtr;

  inline bool SaveMetadata(const std::string & filename, const Metadata & metadata)
  {
    std::ofstream ofile(filename);
    ofile << "BBOX_MIN " << metadata.bbox_min.transpose() << "\n";
    ofile << "BBOX_MAX " << metadata.bbox_max.transpose() << "\n";
    ofile << "VOXEL_SIZE " << metadata.voxel_size << "\n";
    return bool(ofile);
  }

  inline MetadataPtr LoadMetadata(const std::string & filename)
  {
    MetadataPtr result(new Metadata);

    std::ifstream ifile(filename.c_str());
    if (!ifile)
    {
      ROS_ERROR("VoxelgridMetadata: LoadMetadata: could not open file %s", filename.c_str());
      return MetadataPtr();
    }

    std::string line;
    while (std::getline(ifile, line))
    {
      if (line.empty())
        continue;

      std::istringstream istr(line);
      std::string field;
      istr >> field;
      if (field == "BBOX_MIN")
        istr >> result->bbox_min.x() >> result->bbox_min.y() >> result->bbox_min.z();
      else if (field == "BBOX_MAX")
        istr >> result->bbox_max.x() >> result->bbox_max.y() >> result->bbox_max.z();
      else if (field == "VOXEL_SIZE")
        istr >> result->voxel_size;
      else
      {
        ROS_WARN("VoxelgridMetadata: LoadMetadata: unknown entry %s", field.c_str());
      }

      if (!istr)
      {
        ROS_ERROR("VoxelgridMetadata: LoadMetadata: could not parse line %s.", line.c_str());
        return MetadataPtr();
      }
    }

    return result;
  }
  
}
  
#endif // METADATA_H
