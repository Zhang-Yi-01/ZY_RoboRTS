/*
 * @Description: 不滤波
 */
#include "../../../include/models/cloud_filter/no_filter.hpp"
#include "glog/logging.h"

namespace robot_localization {
NoFilter::NoFilter() {
}

bool NoFilter::Filter(const CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& filtered_cloud_ptr) {
    filtered_cloud_ptr.reset(new CloudData::CLOUD(*input_cloud_ptr));
    return true;
}
} 