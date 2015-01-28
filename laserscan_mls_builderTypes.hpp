#ifndef LASERSCAN_MLS_BUILDER_TYPES_HPP
#define LASERSCAN_MLS_BUILDER_TYPES_HPP

#include <base/Eigen.hpp>

namespace laserscan_mls_builder {
    struct Box
    {
        base::Vector3d min;
        base::Vector3d max;

        bool include(Eigen::Vector3d const& p) const
        {
            return (min.x() <= p.x() && p.x() <= max.x()) &&
                (min.y() <= p.y() && p.y() <= max.y()) &&
                (min.z() <= p.z() && p.z() <= max.z());
        }
    };
}

#endif

