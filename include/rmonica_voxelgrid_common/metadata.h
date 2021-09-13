/*
 * Copyright (c) 2021, Riccardo Monica
 *   RIMLab, Department of Engineering and Architecture, University of Parma, Italy
 *   http://www.rimlab.ce.unipr.it/
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted
 * provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions
 * and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of
 * conditions and the following disclaimer in the documentation and/or other materials provided with
 * the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

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
