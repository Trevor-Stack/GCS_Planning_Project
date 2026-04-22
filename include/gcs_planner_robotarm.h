#pragma once

#include "gcs_structs.h"
#include <memory>

class GcsPlannerRobotArm {
public:
    GcsPlannerRobotArm() = default;

    bool SolvePath(const GCSOptions& opts,
        const std::shared_ptr<MapData>& map);
};