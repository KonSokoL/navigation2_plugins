# include "costmap2d_plugins/gradient_example.hpp"

#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

namespace nav2_gradient_costmap_example_plugin
{
    GradientLayer::GradientLayer()
    : last_min_x_(-std::numeric_limits<float>::max()),
      last_min_y_(-std::numeric_limits<float>::max()),
      last_max_x_(std::numeric_limits<float>::max()),
      last_max_y_(std::numeric_limits<float>::max())
    {
    }

    void GradientLayer::onInitialize()
    {
        auto node = node_.lock();
        declareParameter("enabled", rclcpp::ParameterValue(true));
        node->get_parameter(name_ + "." + "enabled", enabled_);

        need_recalculation_ = false;
        current_ = true;
    }

    void GradientLayer::updateBounds(
        double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, double * min_x,
        double * min_y, double * max_x, double * max_y)
    {
        if (need_recalculation_) {
            last_min_x_ = *min_x;
            last_min_y_ = *min_y;
            last_max_x_ = *max_x;
            last_max_y_ = *max_y;

            *min_x = -std::numeric_limits<float>::max();
            *min_y = -std::numeric_limits<float>::max();
            *max_x = std::numeric_limits<float>::max();
            *max_y = std::numeric_limits<float>::max();
            need_recalculation_ = false;
        } else {
            double tmp_min_x = last_max_x_;
            double tmp_min_y = last_max_y_;
            double tmp_max_x = last_max_x_;
            double tmp_max_y = last_max_y_;

            last_min_x_ = *min_x;
            last_min_y_ = *min_y;
            last_max_x_ = *max_x;
            last_max_y_ = *max_y;

            *min_x = std::min(tmp_min_x, *min_x);
            *min_y = std::min(tmp_min_y, *min_y);
            *max_x = std::max(tmp_max_x, *max_x);
            *max_y = std::max(tmp_max_y, *max_y);
        }
    }

    void GradientLayer::onFootprintChanged()
    {
        need_recalculation_ = true;

        RCLCPP_DEBUG(
            rclcpp::get_logger("nav2_costmap_2d"), 
            "GradientLayer::onFootprintChanged(): num footprint points: %lu", 
            layered_costmap_->getFootprint().size()
        );
    }

    void GradientLayer::updateCosts(
        nav2_costmap_2d::Costmap2D & master_grid, 
        int min_i, int min_j, int max_i, int max_j
    )
    {
        if (!enabled_){
            return;
        }

        unsigned char * master_array = master_grid.getCharMap();
        unsigned int size_x = master_grid.getSizeInCellsX(), size_y = master_grid.getSizeInCellsX();

        min_i = std::max(0, min_i);
        min_j = std::max(0, min_j);
        max_i = std::min(static_cast<int>(size_x), max_i);
        max_j = std::min(static_cast<int>(size_y), max_j);

        int gradient_index;
        for (int j = min_j; j < max_j; j++){
            gradient_index = 0;
            for (int i = min_i; i < max_i; i++){
                int index = master_grid.getIndex(i, j);

                unsigned char cost = (LETHAL_OBSTACLE - gradient_index*GRADIENT_FACTOR)%255;
                
                if (gradient_index <= GRADIENT_SIZE) {
                    gradient_index++;
                } else {
                    gradient_index = 0;
                }

                master_array[index] = cost;
            }
        }
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_gradient_costmap_example_plugin::GradientLayer, nav2_costmap_2d::Layer)