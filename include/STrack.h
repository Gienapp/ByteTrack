#pragma once

#include <opencv2/opencv.hpp>
#include "FixedQueue.h"
#include "kalmanFilter.h"
namespace ByteTrack{

enum TrackState { New = 0, Tracked, Lost, Removed };

class STrack
{
public:
	STrack(std::vector<float> tlwh_, float score_);
	STrack(std::vector<float> tlwh_, float score_, std::vector<int> label_ids_, std::vector<float> confidences_, float objectness_, int time_stamp_);
	//STrack(std::vector<float> tlwh_, float score_, int state_id, std::string readable_state, float state_confidence, int pictogram_id, float picto_confidence, std::string readable_pictogram, float objectness);

	~STrack();

	std::vector<float> static tlbr_to_tlwh(std::vector<float> &tlbr);
	void static multi_predict(std::vector<STrack*> &stracks, byte_kalman::KalmanFilter &kalman_filter);
	void static_tlwh();
	void static_tlbr();
	std::vector<float> tlwh_to_xyah(std::vector<float> tlwh_tmp);
	std::vector<float> to_xyah();
	void mark_lost();
	void mark_removed();
	int next_id();
	int end_frame();
	
	void activate(byte_kalman::KalmanFilter &kalman_filter_, int frame_id_);
	void re_activate(STrack &new_track, int frame_id_, bool new_id = false);
	void update(STrack &new_track, int frame_id_);

public:
	bool is_activated;
	int track_id;
	int state;

	// additional information from the detections that need to be carried along to have them at the end
	std::vector<int> label_ids;
	std::vector<float> confidences;
	float objectness;

	std::vector<ByteTrack::FixedQueue<int, 10>> label_queues;
	int track_start_time_stamp;
	int track_duration;

	std::vector<float> _tlwh;
	std::vector<float> tlwh;
	std::vector<float> tlbr;
	int frame_id;
	int tracklet_len;
	int start_frame;

	KAL_MEAN mean;
	KAL_COVA covariance;
	float score;

private:
	byte_kalman::KalmanFilter kalman_filter;
};
}
