#include "visualOdometry.h"
#include <algorithm>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "config.h"

namespace myslam
{
	VisualOdometry::VisualOdometry() :
		state_(INITIALIZING), ref_(nullptr), curr_(nullptr), map_(new Map), num_lost_(0), num_inliers_(0) 
	{
		num_of_features_	= Config::get<int>("ORBextractor.nFeatures");
		scale_factor_		= Config::get<double>("ORBextractor.scaleFactor");
		level_pyramid_		= Config::get<int>("ORBextractor.nLevels");
		match_ratio_		= Config::get<double>("match_ratio");
		max_num_lost_		= Config::get<float>("max_num_lost");
		min_inliers_		= Config::get<int>("min_inliers");
		key_frame_min_rot	= Config::get<double>("keyframe_rotation");
		key_frame_min_trans = Config::get<double>("keyframe_translation");
		orb_				= cv::ORB::create(num_of_features_, scale_factor_, level_pyramid_);
	}
	VisualOdometry::~VisualOdometry() {}

	bool VisualOdometry::addFrame(Frame::Ptr frame)
	{
		switch (state_)
		{
			case INITIALIZING:
			{
				state_ = OK;
				curr_ = ref_ = frame;
				map_->insertKeyFrame(frame);
				//extract keypoints and compute descriptors in first frame
				extractKeyPoints();
				computeDescriptors();
				setRef3DPoints();
				break;
			}
			case OK:
			{
			curr_ = frame;
			extractKeyPoints();
			computeDescriptors();
			featureMatching();
			poseEstimationPnP();
			if (checkEstimatedPose() == true) // a good estimation
			{
				curr_->T_c_w_ = T_c_r_estimated_*ref_->T_c_w_; //	T_c_w = T_c_r*T_r_w, world2current = reference2current * world2reference
				ref_ = curr_;
				setRef3DPoints();
				num_lost_ = 0;
				if (checkKeyFrame() == true) //is a key frame
					addKeyFrame();
			}
			else {
				num_lost_++;
				if (num_lost_ > max_num_lost_)
				{
					state_ = LOST;
					cout << "vo has lost" << endl;
				}				
				return false;
			}
			break;
			}
			case LOST:
			{
				cout << "vo has lost" << endl;
				break;
			}
		}
		return true;
	}

	void VisualOdometry::extractKeyPoints()
	{
		orb_->detect(curr_->color_, keypoints_curr_);
	}
	void VisualOdometry::computeDescriptors()
	{
		orb_->compute(curr_->color_, keypoints_curr_, descriptors_curr_);
	}

	void VisualOdometry::featureMatching() {
		//match desp_ref and desp_curr, use OpenCV's force match
		vector<cv::DMatch> matches;
		cv::BFMatcher matcher(cv::NORM_HAMMING);
		matcher.match(descriptors_ref_, descriptors_curr_, matches);
		//cout << "matches: " << matches.size() << endl;
		//select the best matches
		float min_dis = std::min_element(matches.begin(), matches.end(), [](const cv::DMatch& m1, const cv::DMatch& m2) { return m1.distance < m2.distance; })->distance;
		feature_matches_.clear();
		for (cv::DMatch& m : matches) {
			if (m.distance < max<float>(min_dis*match_ratio_, 20.0))
				feature_matches_.push_back(m);
		}

		//cv::Mat imgMatches;
		//cv::drawMatches(ref_->color_, keypoints_ref_, curr_->color_, keypoints_curr_, feature_matches_, imgMatches);
		//cv::imwrite("inliers.png", imgMatches);

		//cout << "good matches: " << feature_matches_.size() << endl;
	}

	void VisualOdometry::setRef3DPoints()
	{
		pts_3d_ref_.clear();
		descriptors_ref_ = Mat();
		//keypoints_ref_.clear();
		for (size_t i = 0; i < keypoints_curr_.size(); ++i)
		{
			double d = ref_->findDepth(keypoints_curr_[i]);
			if (d > 0)
			{
				//keypoints_ref_.push_back(keypoints_curr_[i]);
				Vector3d p_cam = ref_->camera_->pixel2camera(Vector2d(keypoints_curr_[i].pt.x, keypoints_curr_[i].pt.y), d);
				pts_3d_ref_.push_back(cv::Point3f(p_cam(0, 0), p_cam(1, 0), p_cam(2, 0)));
				descriptors_ref_.push_back(descriptors_curr_.row(i));
			}
		}
	}
	void VisualOdometry::poseEstimationPnP() {
		//construct the 3d 2d observations
		vector<cv::Point3f> pts3d;
		vector<cv::Point2f> pts2d;

		for (cv::DMatch m : feature_matches_)
		{
			pts3d.push_back(pts_3d_ref_[m.queryIdx]);
			pts2d.push_back(keypoints_curr_[m.trainIdx].pt);
		}
		Mat K = (cv::Mat_<double>(3, 3) <<
			ref_->camera_->fx_,					 0, ref_->camera_->cx_,
							 0, ref_->camera_->fy_, ref_->camera_->cy_,
							 0,					 0,					  1
			);

		Mat rvec, tvec, inliers;
		cv::solvePnPRansac(pts3d, pts2d, K, Mat(), rvec, tvec, false, 100, 4.0, 0.99, inliers);
		num_inliers_ = inliers.rows;
		//cout << "pnp inliers: " << num_inliers_<< endl;
		T_c_r_estimated_ = SE3(
			SO3(rvec.at<double>(0, 0), rvec.at<double>(1, 0), rvec.at<double>(2, 0)),
			Vector3d(tvec.at<double>(0, 0), tvec.at<double>(1, 0), tvec.at<double>(2, 0)));
	}

	bool VisualOdometry::checkEstimatedPose()
	{
		if (num_inliers_ < min_inliers_)
		{
			cout << "reject because inlier is too small: " << num_inliers_ << endl;
			return false;
		}
		//if the motion is too large, it is probably wrong
		Sophus::Vector6d d = T_c_r_estimated_.log();
		if (d.norm() > 5.0){
			cout << "reject because motion is too large: " << d.norm() << endl;
			return false;
		}
		return true;
	}

	bool VisualOdometry::checkKeyFrame()
	{
		Sophus::Vector6d d = T_c_r_estimated_.log();
		Vector3d trans = d.head<3>();
		Vector3d rot = d.tail<3>();
		if (rot.norm() > key_frame_min_rot || trans.norm() > key_frame_min_trans)	return true;
		return false;
	}

	void VisualOdometry::addKeyFrame()
	{
		cout << "adding a key-frame" << endl;
		map_->insertKeyFrame(curr_);
	}
}