#include "tuw_self_localization/tuw_self_localization.h"
#include <opencv/cv.h>
#include <boost/concept_check.hpp>

using namespace tuw;

void SelfLocalization::onMouseMap ( int event, int x, int y, int flags, void* param ) {
    SelfLocalization *self_localization = ( SelfLocalization * ) param;
    static Pose2D pose;
    /**
     * @ToDo Sensor model
     * catch the mouse and reset the filter
     * use the event CV_EVENT_LBUTTONDOWN and CV_EVENT_LBUTTONUP to define a pose with orientation
     * reset the filter with ground truth on the middle button
     **/
    
    if (event == CV_EVENT_LBUTTONDOWN) {
        pose.set(x, y, 0);
        //ROS_INFO("Mouse Down: x:%f, y: %f", pose.x(), pose.y());
    }
    
    if (event == CV_EVENT_LBUTTONUP) {
        double theta = atan2((double)x - pose.x(), double(y) - pose.y()) - M_PI;
        if (theta < -M_PI)
            theta = 2 * M_PI + theta;
        pose.theta() = theta ;
        pose.position() = self_localization->figure_map_.m2w(pose.position());
        self_localization->pose_filter_->reinitialize(pose);
        //ROS_INFO("Mouse left up: Event: %d, x: %d, y: %d, flags: %d", event, x, y, flags);
    }

    if (event == CV_EVENT_RBUTTONUP) {
        pose.set(self_localization->pose_ground_truth_);
        self_localization->pose_filter_->reinitialize(pose);
        ROS_INFO("Reinitalize by mouse");
    }
}

    SelfLocalization::SelfLocalization ( const std::string &ns )
: loop_count_ ( 0 )
    , figure_map_ ( ns + ", Global View" ){
        measurement_laser_ = std::make_shared<tuw::MeasurementLaser>();   /// laser measurements
    }

void SelfLocalization::init() {
    figure_map_.init ( config_.map_pix_x, config_.map_pix_y,
            config_.map_min_x, config_.map_max_x,
            config_.map_min_y, config_.map_max_y,
            config_.map_rotation + M_PI,
            config_.map_grid_x, config_.map_grid_y, filename_map_image_ );

    figure_map_.setLabel("x=%4.2f","y=%4.2f");

    if ( config_.plot_data ) cv::namedWindow ( figure_map_.title(), 1 );
    if ( config_.plot_data ) cv::setMouseCallback ( figure_map_.title(), SelfLocalization::onMouseMap, this );
    if ( config_.plot_data) {
        cv::moveWindow ( figure_map_.title(), 20, 20 );
    }

    std::string filename_map;
    if ( pose_filter_->getType() == PoseFilter::PARTICLE_FILTER ) filename_map =  filename_map_image_;
    if ( pose_filter_->getType() == PoseFilter::KALMAN_FILTER ) filename_map =  filename_map_lines_;

    pose_filter_->loadMap ( config_.map_pix_x, config_.map_pix_y,
            config_.map_min_x, config_.map_max_x,
            config_.map_min_y, config_.map_max_y,
            config_.map_rotation + M_PI, filename_map );

}


void SelfLocalization::plot() {
    if ( config_.plot_data ) plotMap();
    cv::waitKey ( 10 );
}

void SelfLocalization::plotMap() {
    figure_map_.clear();
    char text[0xFF];

    cv::Matx33d M = pose_ground_truth_.tf();
    cv::Matx33d Tl = measurement_laser_->tf();
    cv::Matx33d Ml2m = M * Tl;

    for ( size_t i = 0; i < measurement_laser_->size(); i++ ) {
        /**
         * @ToDo MotionModel
         * plot the laser data into the map and use the transformation measurement_laser_->tf() as well!!
         **/
        MeasurementLaser::Beam &beam = measurement_laser_->operator[] ( i );
        figure_map_.symbol(Ml2m * beam.end_point, 0.01, Figure::red, 1);
    }
    sprintf ( text, "%5lu,  <%+4.2fm, %+4.2f>", loop_count_, mouse_on_map_.x(), mouse_on_map_.y() );
    cv::putText ( figure_map_.view(), text, cv::Point ( 20,20 ), cv::FONT_HERSHEY_PLAIN, 1, Figure::white,3, CV_AA );
    cv::putText ( figure_map_.view(), text, cv::Point ( 20,20 ), cv::FONT_HERSHEY_PLAIN, 1, Figure::black,1, CV_AA );

    /**
     * @ToDo Put your name into the figure
     **/ 
    cv::putText ( figure_map_.background(), "Dominik Streicher", 
            cv::Point (figure_map_.view().cols - 170, figure_map_.view().rows - 10),
            cv::FONT_HERSHEY_PLAIN, 1, Figure::black, 1, CV_AA);
    figure_map_.symbol ( odom_, 0.2, Figure::cyan, 1 );
    figure_map_.symbol ( pose_ground_truth_, 0.2, Figure::orange, 1 );
    pose_filter_->plotData ( figure_map_ );
    cv::imshow ( figure_map_.title(),figure_map_.view() );
    cv::waitKey ( 10 );
}

void SelfLocalization::localization () {
    if ( measurement_laser_->empty() ) return;
    pose_estimated_ = pose_filter_->localization ( cmd_, ( MeasurementPtr ) measurement_laser_ );
    loop_count_++;
}

