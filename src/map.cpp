#include "map.h"

namespace myslam
{
	void Map::insertKeyFrame(Frame::Ptr frame)
	{
		cout << "Key frame size = " << keyframe_.size() << endl;
		if (keyframe_.find(frame->id_) == keyframe_.end())
			keyframe_.insert(make_pair(frame->id_, frame));
		else
			keyframe_[frame->id_] = frame;
	}

	void Map::insertMapPoint(MapPoint::Ptr map_point)
	{
		if (map_points_.find(map_point->id_) == map_points_.end())
			map_points_.insert(make_pair(map_point->id_, map_point));
		else
			map_points_[map_point->id_] = map_point;
	}
}
