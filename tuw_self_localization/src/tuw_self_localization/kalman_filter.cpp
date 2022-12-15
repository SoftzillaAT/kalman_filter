#include "tuw_self_localization/kalman_filter.h"
#include <boost/lexical_cast.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <tuw_geometry/linesegment2d_detector.h>
#include <iostream>
using namespace tuw;
KalmanFilter::KalmanFilter()
    : PoseFilter ( KALMAN_FILTER )
    , figure_hspace_ ( "Hough Space" ) {
}

/**
 * computes the angle difference between two angles by taking into account the circular space
 * @param alpha0
 * @param angle1
 * @return difference
 **/

void KalmanFilter::init ( ) {
    pose_estimated_ = pose_init_;
    P = cv::Matx<double, 3, 3> ( config_.init_sigma_location, 0, 0,
                                 0, config_.init_sigma_location, 0,
                                 0, 0, config_.init_sigma_orientation );
    reset_ = false;


}

void KalmanFilter::detect_lines ( const MeasurementLaserConstPtr &z ) {

    LineSegment2DDetector linesegment_detector;
    linesegment_detector.config_.threshold_split_neighbor = config_.line_dection_split_neighbor;
    linesegment_detector.config_.threshold_split = config_.line_dection_split_threshold;
    linesegment_detector.config_.min_length = config_.line_dection_min_length;
    linesegment_detector.config_.min_points_per_line = config_.line_dection_min_points_per_line;
    linesegment_detector.config_.min_points_per_unit = config_.line_dection_min_points_per_unit;
    measurement_local_scanpoints_.resize ( z->size() );
    for ( size_t i = 0; i < z->size(); i++ ) {
        measurement_local_scanpoints_[i] = z->tf() * z->operator[] ( i ).end_point;
    }
    measurement_linesegments_.clear();
    linesegment_detector.start ( measurement_local_scanpoints_, measurement_linesegments_ );

    measurement_match_.resize ( measurement_linesegments_.size(), -1 );
}


void KalmanFilter::plotData ( Figure &figure_map ) {
    plotMap ( figure_map );
    if ( config_.plot_hough_space ) plotHoughSpace();
}

void KalmanFilter::plotMap ( Figure &figure_map ) {
    char text[0xFF];
    cv::Scalar color;

    /// Plot known line segments (map)
    for ( size_t i = 0; i < map_linesegments_.size(); i++ ) {
        color = Figure::orange;
        figure_map.line ( map_linesegments_[i].p0(), map_linesegments_[i].p1(), color, 1 );
        figure_map.putText ( boost::lexical_cast<std::string> ( i ),  map_linesegments_[i].pc(), cv::FONT_HERSHEY_PLAIN, 0.6, Figure::white,  3, CV_AA );
        figure_map.putText ( boost::lexical_cast<std::string> ( i ),  map_linesegments_[i].pc(), cv::FONT_HERSHEY_PLAIN, 0.6, color, 1, CV_AA );
    }

    cv::Matx33d M = pose_estimated_.tf();
    for ( size_t i = 0; i < measurement_linesegments_.size(); i++ ) {
        color = Figure::red;
	if ( measurement_match_[i] >= 0 ) color = Figure::green_dark;
        /**
        * @ToDo EKF
        * Draw the measurment mark matches
        **/
	
	Point2D p0 = M * measurement_linesegments_[i].p0();
	Point2D p1 = M * measurement_linesegments_[i].p1();
	Point2D pc = M * measurement_linesegments_[i].pc();

        figure_map.line ( p0, p1, color );
        figure_map.putText ( boost::lexical_cast<std::string> ( i ),  pc, cv::FONT_HERSHEY_PLAIN, 0.6, Figure::white,  3, CV_AA );
        figure_map.putText ( boost::lexical_cast<std::string> ( i ),  pc, cv::FONT_HERSHEY_PLAIN, 0.6, color, 1, CV_AA );
    }

    for ( size_t i = 0; i < measurement_match_.size(); i++ ) {
        if ( measurement_match_[i] >= 0 ) {
            /**
            * @ToDo EKF
            * Draw the match relation, it is up to you how you visualize the realtion
            **/
	    color = Figure::blue;
	    //ROS_INFO("FOUND MATCH: %d, %d", i, measurement_match_[i]);
	    Point2D p00 = measurement_linesegments_[i].p0();
	    Point2D p01 = map_linesegments_[measurement_match_[i]].p0();
	    figure_map.line ( M * p00, p01, color );
	    
	    Point2D p10 = measurement_linesegments_[i].p1();
	    Point2D p11 = map_linesegments_[measurement_match_[i]].p1();
	    figure_map.line ( M * p10, p11, color );
        }
    }

    sprintf ( text, "%4.3fsec", duration_last_update_.total_microseconds() /1000000. );
    cv::putText ( figure_map.view(), text, cv::Point ( figure_map.view().cols-100,20 ), cv::FONT_HERSHEY_PLAIN, 1, Figure::white,3, CV_AA );
    cv::putText ( figure_map.view(), text, cv::Point ( figure_map.view().cols-100,20 ), cv::FONT_HERSHEY_PLAIN, 1, Figure::black,1, CV_AA );


    cv::Matx<double,2,2> Mw2m ( figure_map.Mw2m() ( 0,0 ), figure_map.Mw2m() ( 0,1 ), figure_map.Mw2m() ( 1,0 ), figure_map.Mw2m() ( 1,1 ) );
    /**
    * @ToDo EKF
    * Compute and plot the pose coaraiance in x and y direction
    * take the pose covariance P and crate a 2x2 matrix out of the x,y components
    * transform the matrix into the plot E = Mw2m*P(0:1,0:1)*Mw2m'
    * use the opencv to compute eigenvalues and eigen vectors to compute the size and orienation of the ellipse
    **/
    cv::Matx<double, 2, 2> E ( P(0,0), P(0,1), P(1,0), P(1,1) );  /// must be changed
    E = Mw2m * E * Mw2m.t();
    cv::Mat_<double> eigval, eigvec;
    cv::eigen ( E, eigval, eigvec );
    eigval(0) = sqrt(eigval(0));
    eigval(1) = sqrt(eigval(1));
    //int a = (int) round(eigval.at<double>(0,0));
    //int b = (int) round(eigval.at<double>(0,1));
    double angle = atan2(eigvec.at<double>(0,1), eigvec.at<double>(0,0));
    //ROS_INFO("Eigenvalues: %f, %f", eigval.at<double>(0,0), eigval.at<double>(1,0));
    
    cv::RotatedRect ellipse ( ( figure_map.Mw2m() * pose_estimated_.position() ).cv(),cv::Size ( abs(eigval.at<double>(0,0)) , abs(eigval.at<double>(1,0))), angle*180/M_PI ); /// must be changed

    cv::ellipse ( figure_map.view(),ellipse, Figure::magenta, 1, CV_AA );
    ///Plot estimated pose
    figure_map.symbol ( pose_estimated_, 0.5, Figure::magenta, 1 );
}
void KalmanFilter::plotHoughSpace ( ) {
    if ( figure_hspace_.initialized() == false ) {
        figure_hspace_.setLabel ( "alpha=%4.2f","rho=%4.2f" );
        figure_hspace_.init ( config_.hough_space_pixel_alpha, config_.hough_space_pixel_rho,
                              -M_PI*1.1, +M_PI*1.1,
                              0, config_.hough_space_meter_rho,
                              M_PI,
                              1, M_PI/4 );

        if ( config_.plot_hough_space ) cv::namedWindow ( figure_hspace_.title(), 1 );
        if ( config_.plot_hough_space ) {
            cv::moveWindow ( figure_hspace_.title(), 640, 20 );
        }
    }
    figure_hspace_.clear();

    cv::Rect rectSpace ( 0,0, figure_hspace_.view().cols, figure_hspace_.view().rows );
    for ( unsigned int i = 0; i < measurement_local_scanpoints_.size(); i++ ) {
        Point2D p0 = measurement_local_scanpoints_[i];
        for ( double alpha = figure_hspace_.min_x(); alpha < figure_hspace_.max_x(); alpha +=1.0/figure_hspace_.scale_x() ) {
            /**
            * @ToDo EKF
            * draw a wave with angle = [-pi...pi], r = x*cos(angle) + y *sin(angle) for every laser point [x,y].
            * The function Line2D::toPolar() can be used to transform a line into polar coordinates
            **/
	    
	    double r = p0.x() * cos(alpha) + p0.y() * sin(alpha);
	    Point2D tmp = figure_hspace_.w2m(Point2D(alpha, r));
            cv::Point hspace = tmp.cv(); /// point in hspace
            if ( hspace.inside ( rectSpace ) ) {
	      figure_hspace_.view().at<cv::Vec3b> ( hspace ) -=  cv::Vec3b ( 50,10,10 );  // changes a pixel value in hspace
            }
        }
    }


    cv::Scalar color;
    /// Plot measurement prediction
    Tf2D tf = figure_hspace_.Mw2m();
    for ( size_t i = 0; i < predicted_linesegments_.size(); i++ ) {
        color = Figure::orange;
        /**
        * @ToDo EKF
        * the map prediction in the hough space as a circle or dot
        **/
	Polar2D polar = predicted_linesegments_[i].toPolar();
        figure_hspace_.circle ( polar, 3, color, 1 );
        figure_hspace_.putText ( boost::lexical_cast<std::string> ( i ),  polar, cv::FONT_HERSHEY_PLAIN, 0.6, Figure::white,  3, CV_AA );
        figure_hspace_.putText ( boost::lexical_cast<std::string> ( i ),  polar, cv::FONT_HERSHEY_PLAIN, 0.6, color, 1, CV_AA );
    }

    /// Plot measurement
    cv::RotatedRect ellipse;
    ellipse.angle  = 0;
    ellipse.size.width  = config_.data_association_line_alpha  * figure_hspace_.scale_x() * 2.0;
    ellipse.size.height = config_.data_association_line_rho * figure_hspace_.scale_y() * 2.0;
    for ( size_t i = 0; i < measurement_linesegments_.size(); i++ ) {
        Polar2D  polar = measurement_linesegments_[i].toPolar();
        color = Figure::blue_dark;
        /**
        * @ToDo EKF
        * Plot the measurement prediction KalmanFilter::predicted_linesegments_ with an ellipse
        * to show the data association threshold config_.data_association_line_alpha and config_.data_association_line_rho.
               **/
	Point2D p = tf * Point2D(polar.alpha(), polar.rho());
        ellipse.center = p.cv();
        cv::ellipse ( figure_hspace_.view(), ellipse, color, 1, CV_AA );
        figure_hspace_.putText ( boost::lexical_cast<std::string> ( i ),  polar, cv::FONT_HERSHEY_PLAIN, 0.6, Figure::white,  3, CV_AA );
        figure_hspace_.putText ( boost::lexical_cast<std::string> ( i ),  polar, cv::FONT_HERSHEY_PLAIN, 0.6, color, 1, CV_AA );
    }
    cv::imshow ( figure_hspace_.title(),figure_hspace_.view() );
}

void KalmanFilter::data_association ( ) {
    if ( config_.enable_data_association == false ) return;


    /// compute the predicted measurement
    predicted_linesegments_.resize ( map_linesegments_.size() );
    Tf2D M = pose_predicted_.tf().inv();
    for ( size_t i = 0; i < predicted_linesegments_.size(); i++ ) {
        /**
        * @ToDo EKF
        * compute the mesurment prediction
        * predicted_linesegments_[i].set ( .... )
        **/
	Point2D p0 = M * map_linesegments_[i].p0();
	Point2D p1 = M * map_linesegments_[i].p1();
	predicted_linesegments_[i].set(p0, p1);
    }

    /// Match line segments in polar coordinates which are near to the robot
    Tf2D Mp2h = figure_hspace_.Mw2m();
    for ( size_t i = 0; i < measurement_linesegments_.size(); i++ ) {
        Polar2D measurement = measurement_linesegments_[i].toPolar();
        float dMin = FLT_MAX;
        measurement_match_[i] = -1;
        for ( size_t j = 0; j < predicted_linesegments_.size(); j++ ) {
            Polar2D prediction = predicted_linesegments_[j].toPolar();
            /**
            * @ToDo EKF
            * find the best mesurment prediction idx j and store it in measurement_match_[i]
            **/
	    double diff_roh  = abs(prediction.rho() - measurement.rho());
	    double diff_alpha = abs(prediction.alpha() - measurement.alpha());
	    
	    if (diff_roh <= config_.data_association_line_rho && diff_alpha <= config_.data_association_line_alpha) {
	      
	      LineSegment2D line_pred = predicted_linesegments_[j];
	      LineSegment2D line_measure = measurement_linesegments_[i];
	      
	      // @ToDo: Calculate end points and take minimum distance. Also take normal distance which is already implemented
	      double dist1 = line_pred.distanceTo(line_measure.p0());
	      double dist2 = line_pred.distanceTo(line_measure.p1());
	      double d = dist1 + dist2;
	      // use the data_association_line_alpha and data_association_line_rho
	      if (d < dMin /*&& d < 1.5*/) {
		measurement_match_[i] = j;
		dMin = d;
	      }
	    }
        }
    }


}

void KalmanFilter::reinitialize ( const Pose2D &p ) {
    setPoseInit ( p );
    reset();
}


void KalmanFilter::loadMap ( int width_pixel, int height_pixel, double min_y, double max_y, double min_x, double max_x, double roation, const std::string &file ) {
    init();
    cv::FileStorage fs ( file, cv::FileStorage::READ );
    cv::Mat_<double> l;
    fs["line segments"] >> l;
    map_linesegments_.resize ( l.rows );
    for ( size_t i = 0; i < map_linesegments_.size(); i++ ) {
        map_linesegments_[i].set ( l ( i,0 ),l ( i,1 ), l ( i,2 ),l ( i,3 ) );
    }
}

Pose2D KalmanFilter::localization ( const Command &u, const MeasurementConstPtr &z ) {
    detect_lines ( ( const MeasurementLaserConstPtr& ) z );
    if ( updateTimestamp ( z->stamp() ) ) {
        if ( reset_ ) init();
        prediction ( u );
        data_association ( );
        correction ( );
    }
    return pose_estimated_;
}

void KalmanFilter::setConfig ( const void *config ) {
    config_ = * ( ( tuw_self_localization::KalmanFilterConfig* ) config );
}

void KalmanFilter::prediction ( const Command &u ) {
    x = pose_estimated_.state_vector();
    if ( config_.enable_prediction ) {

        /**
        * @ToDo EKF
        * compute KalmanFilter::xp and KalmanFilter::Pp as predicted pose and Covariance
        **/
        
	double v = u.v();
	double w = u.w();
	double theta = x[2];
	//ROS_INFO("Time: %f | %f", duration_last_update_.total_microseconds() /1000000., config_.forward_prediction_time);
	double dt = duration_last_update_.total_microseconds() /1000000.; //config_.forward_prediction_time;
	double r = v/w;
	
	
	// Calculate G
	double g02;
	double g12;
	
	if (fabs(w) > FLT_MIN) {
	  g02 = -r * cos(theta) + r * cos(theta + w * dt); 
	  g12 = -r * sin(theta) + r * sin(theta + w * dt) ;
	} else {
	  g02 = 0;
	  g12 = 0;
	}
	
	G = cv::Matx<double, 3, 3> ( 1, 0, g02,
				     0, 1, g12,
				     0, 0, 1   );
	
	// Calculate V
	double v00;
	double v01;
	double v10;
	double v11;
	
	if (fabs(w) > FLT_MIN) {
	  v00 = (-sin(theta) + sin(theta + w * dt)) / w;
	  v01 = (v * (sin(theta) - sin(theta + w * dt))) / w*w + (v * (cos(theta + w * dt) * dt) ) / w;
	  v10 = (cos(theta) - cos(theta + w * dt)) / w;
	  v11 = -(v * (cos(theta) - cos(theta + w * dt))) / (w*w) + (v * (sin(theta) + w * dt) * dt ) / w;
	} else {
	  v00 = cos(theta) * dt;
	  v01 = -v * sin(theta) * dt * dt / 2.0;
	  v10 = sin(theta) * dt;
	  v11 = -v * cos(theta) * dt * dt / 2.0;
	}
	
	V = cv::Matx<double, 3, 2> ( v00, v01,
				     v10, v11,
				     0,   dt  );
	
	
	// Calculate M
	double m00 = config_.alpha_1 * v * v + config_.alpha_2 * w * w;
	double m11 = config_.alpha_3 * v * v + config_.alpha_4 * w * w;
	
	M = cv::Matx<double, 2, 2> ( m00, 0,
				     0,   m11 ); 
	
	// Calculate R
	R = V * M * V.t();

	/// Covariance update
	Pp = G * P * G.t() + R;
	
	
	/// Pose prediction
	double dx;
	double dy;
	double dtheta;
	
	if (fabs(w) > FLT_MIN) {
	  dx = -r * sin(theta) + r * sin(theta + w * dt);
	  dy = r * cos(theta) - r * cos(theta + w * dt);
	  dtheta = w * dt;
	} else {
	  dx =  v * dt * cos(theta);
	  dy =  v * dt * sin(theta);
	  dtheta =  0;
	}
	
	xp = x + cv::Vec<double, 3> (dx, dy, dtheta);
        
	//ROS_INFO("Vehicle [%f, %f, %f]", Pp(0), Pp(1), Pp(2));
	
    } else {
        xp = x;
        Pp = P;
    }
    pose_predicted_ = xp;
}



void KalmanFilter::correction () {

    xc = pose_predicted_.state_vector();
    Pc = Pp;

    Q = Q.eye();

    /**
    * @ToDo EKF
    * Pose correction must update the KalmanFilter::xc and KalmanFilter::Pc which reprecents the corrected pose with covaraiance
    * have a look into Siegwart 2011 section 5.6.8.5 Case study: Kalman filter localization with line feature extraction
    **/
    Q = cv::Matx<double, 2,2> ( config_.sigma_alpha, 0, 0, config_.sigma_rho );
    for ( size_t idx_measurement = 0; idx_measurement < measurement_match_.size(); idx_measurement++ ) {
        int idx_map = measurement_match_[idx_measurement];
        if ( idx_map != -1 ) {
            /**
            * @ToDo EKF
            * Pose correction must update the KalmanFilter::xc and KalmanFilter::Pc which reprecents the corrected pose with covaraiance
            * have a look into Siegwart 2004 section Case study: Kalman filter localization with line feature extraction
            **/
	    //continue;
            /// first the prediciton and the measurment into polar space and compute the distance
            Tf2D M = Pose2D(xc).tf().inv();
            Polar2D w = map_linesegments_[idx_map].toPolar(); // in the slides with ^ symbol
	    Polar2D z = measurement_linesegments_[idx_measurement].toPolar();
	    
	    Polar2D z_ = Line2D(M*map_linesegments_[idx_map].p0(), M*map_linesegments_[idx_map].p1()).toPolar();
	    double rho = xc[0] * cos(w.alpha()) + xc[1] * sin(w.alpha());
	    
	    cv::Matx<double, 2, 1> z_p;
	    
	    if(w.rho() > rho) {
	      
	      H = cv::Matx<double, 2,3> (0, 			  0, 			    -1,
					 -cos(w.alpha()), -sin(w.alpha()), 0);
	      z_p =  cv::Matx<double, 2, 1> ( w.alpha() - xc(2), 
					      w.rho() - (xc(0) * cos( w.alpha() ) + xc(1) * sin(w.alpha()))
	      );
	    }
	    else {
	      H = cv::Matx<double, 2,3> (0, 			  0, 			    -1,
					 cos(w.alpha()), sin(w.alpha()), 0);
	      
	      z_p =  cv::Matx<double, 2, 1> ( w.alpha() + M_PI - xc(2), 
					      ( xc(0) * cos( w.alpha() ) + xc(1) * sin( w.alpha() ) ) - w.rho()
					    );
	      
	    }
	      
	      
	      double angle = angle_difference(z.alpha(), z_p(0));
	    angle_normalize(angle);
	    
	    
	    //cv::Matx<double, 2, 1> v (angle, measurement.rho() - map.rho()); /// Messurment error between predition (known data) and detetion --> Siegwart
	      
	    cv::Matx<double, 2, 1> v (angle_difference(z.alpha(), z_.alpha()), z.rho() - z_.rho());   
            
	    cv::Matx<double, 2,2> Si = H * Pc * H.t() + Q;
            // cv::Matx<double, 1,1> d_mahalanobis   // just for debugging reasons
            cv::Matx<double, 3,2> K = Pc * H.t() * Si.inv();
            cv::Matx<double, 3,1> dx = K * v;
            
	    cv::Matx<double, 3, 3> I = (1,0,0,
					0,1,0,
					0,0,1);
	    
	    ROS_INFO("Delta x:%f, y:%f", dx(0), dx(1));
	    xc += dx;
	    
	    Pc = Pc - K * Si * K.t();
            //Pc = (I - K * H) * Pc; 
        }
    }

    if ( config_.enable_correction ) {
        pose_estimated_ = xc;
        P = Pc;
    } else {
        P = Pp;
        pose_estimated_ =  pose_predicted_.state_vector();
    }
}
