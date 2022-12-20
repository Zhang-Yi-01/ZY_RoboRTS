/*
 * @Description: 不滤波
 * @Author: lsc
 * @Date: 
 */
#ifndef LOCALIZATION_MODELS_CLOUD_FILTER_NO_FILTER_HPP_
#define LOCALIZATION_MODELS_CLOUD_FILTER_NO_FILTER_HPP_
#include "cloud_filter_interface.hpp"

namespace robot_localization {
class NoFilter: public CloudFilterInterface {
  public:
    NoFilter();

    bool Filter(const CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& filtered_cloud_ptr) override;
};
}
#endif