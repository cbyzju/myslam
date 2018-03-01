#ifndef FRAME_H_
#define FRAME_H_

#include "common.h"
#include "camera.h"

namespace myslam 
{
	class Frame
	{
	public:
		typedef shared_ptr<Frame> Ptr;
		unsigned long	id_;			//id of this frame
		double			time_stamp_;	//when it is recorded
		SE3				T_c_w_;			//transform from world to camera
		Camera::Ptr		camera_;		//Pinhole RGBD camera model
		Mat				color_, depth_;	//color and depth image

	public:
		Frame();
		Frame(long id, double time_stamp = 0, SE3 T_c_w = SE3(), Camera::Ptr camera = nullptr, Mat color = Mat(), Mat depth = Mat());
		static Frame::Ptr createFrame();			//factory function
		double findDepth(const cv::KeyPoint& kp);	//find the depth in depth map
		Vector3d getCamCenter() const;				//get camera center
		bool isInFrame(const Vector3d& pt_world);	//check if a point is in this frame
	};
}
#endif