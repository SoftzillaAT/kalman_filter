#ifndef TUW_GEOMETRY
#define TUW_GEOMETRY

#include <tuw_geometry/point2d.h>
#include <tuw_geometry/polar2d.h>
#include <tuw_geometry/pose2d.h>
#include <tuw_geometry/line2d.h>
#include <tuw_geometry/linesegment2d.h>
#include <tuw_geometry/figure.h>
#include <tuw_geometry/command.h>
#include <tuw_geometry/measurement_laser.h>
#include <string>
#include <map>

namespace tuw {
  enum DistributionType {
      NORMAL_DISTRIBUTION = 0,
      UNIFORM_DISTRIBUTION = 1,
      GRID_DISTRIBUTION = 2
  };
  static std::map<DistributionType, std::string> DistributionTypeName; 
  
  enum ResampleType {
      SIMPLE_RESAMPLE = 0,
      LOW_VARIANCE_RESAMPLE = 1
  };
  static std::map<ResampleType, std::string> ResampleTypeName; 
}

#endif // TUW_GEOMETRY
