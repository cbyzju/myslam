#ifndef MAPPOINT_H_
#define MAPPOINT_H_

#include "common.h"

namespace myslam
{
	class MapPoint 
	{
		public:
			typedef shared_ptr<MapPoint> Ptr;
			unsigned long id_;
			Vector3d pos_;		//position in world
			Vector3d norm_;		//Normal of viewing direction
			Mat descriptor_;	//Descriptor for matching
			int observed_times_;//being 
			int correct_times_;
			MapPoint();
			MapPoint(long id, Vector3d position, Vector3d norm);
			
			static MapPoint::Ptr createMapPoint();//factory function
	};
}

#endif;