#pragma once

#include "STrack.h"

namespace ByteTrack{
struct Object
{
    cv::Rect_<float> rect;
    float prob;
	// additional informations from the detections that need to be carried along
	std::vector<int> label_ids;
	std::vector<float> confidences;
	float objectness;
	int time_stamp;

    // int label;
    // std::string readable_state;
    // float state_confidence;
    // int pictogram_id;
    // float picto_confidence;
    // std::string readable_pictogram;
    // float objectness;

};

class BYTETracker
{
public:
	BYTETracker(int track_buffer = 30);
	BYTETracker(float track_thresh, float high_thresh, float match_thresh, int track_buffer = 30);
	~BYTETracker();

	std::vector<STrack> update(const std::vector<Object>& objects);
	cv::Scalar get_color(int idx);

private:
	std::vector<STrack*> joint_stracks(std::vector<STrack*> &tlista, std::vector<STrack> &tlistb);
	std::vector<STrack> joint_stracks(std::vector<STrack> &tlista, std::vector<STrack> &tlistb);

	std::vector<STrack> sub_stracks(std::vector<STrack> &tlista, std::vector<STrack> &tlistb);
	void remove_duplicate_stracks(std::vector<STrack> &resa, std::vector<STrack> &resb, std::vector<STrack> &stracksa, std::vector<STrack> &stracksb);

	void linear_assignment(std::vector<std::vector<float> > &cost_matrix, int cost_matrix_size, int cost_matrix_size_size, float thresh,
		std::vector<std::vector<int> > &matches, std::vector<int> &unmatched_a, std::vector<int> &unmatched_b);
	std::vector<std::vector<float> > iou_distance(std::vector<STrack*> &atracks, std::vector<STrack> &btracks, int &dist_size, int &dist_size_size);
	std::vector<std::vector<float> > iou_distance(std::vector<STrack> &atracks, std::vector<STrack> &btracks);
	std::vector<std::vector<float> > ious(std::vector<std::vector<float> > &atlbrs, std::vector<std::vector<float> > &btlbrs);

	double lapjv(const std::vector<std::vector<float> > &cost, std::vector<int> &rowsol, std::vector<int> &colsol, 
		bool extend_cost = false, float cost_limit = static_cast<float>(LONG_MAX), bool return_cost = true);

private:

	float m_track_thresh;
	float m_high_thresh;
	float m_match_thresh;
	int m_frame_id;
	int m_max_time_lost;

	std::vector<STrack> m_tracked_stracks;
	std::vector<STrack> m_lost_stracks;
	std::vector<STrack> m_removed_stracks;
	byte_kalman::KalmanFilter m_kalman_filter;
};
}
