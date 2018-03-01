#ifndef MAP_H_
#define MAP_H_

#include "common.h"
#include "mappoint.h"
#include "frame.h"

namespace myslam
{
	class Map {
	public:
		typedef shared_ptr<Map> Ptr;
		unordered_map<unsigned long, MapPoint::Ptr> map_points_;	//all
		unordered_map<unsigned long, Frame::Ptr> keyframe_;

		Map() {}
		void insertKeyFrame(Frame::Ptr frame);
		void insertMapPoint(MapPoint::Ptr map_point);
	};

}
#endif
