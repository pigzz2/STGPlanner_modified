
#ifndef ROBO_PLANNER_WS_FRONTIER_H
#define ROBO_PLANNER_WS_FRONTIER_H

#include "point3d.h"

namespace utils {
    using Frontier = Point3D;

    using FrontierSet = Point3DSet;
    template<typename T>
    using FrontierMap = Point3DMap<T>;
    using FrontierQueue = Point3DQueue;
}


#endif