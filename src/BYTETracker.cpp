#include "BYTETracker.h"
#include <fstream>

namespace ByteTrack{
BYTETracker::BYTETracker(int track_buffer)
{
	m_track_thresh = 0.9f;
	m_high_thresh = 0.975f;
	m_match_thresh = 0.7f;

	m_frame_id = 0;
	m_max_time_lost = track_buffer;
	std::cout << "Init ByteTrack!" << std::endl;
}

BYTETracker::BYTETracker(float track_thresh, float high_thresh, float match_thresh, int track_buffer)
{
	m_track_thresh = track_thresh;
	m_high_thresh = high_thresh;
	m_match_thresh = match_thresh;

	m_frame_id = 0;
	m_max_time_lost = track_buffer;
	std::cout << "Init ByteTrack!" << std::endl;
}

BYTETracker::~BYTETracker()
{
}

std::vector<STrack> BYTETracker::update(const std::vector<Object>& objects)
{

	////////////////// Step 1: Get detections //////////////////
	this->m_frame_id++;
	std::vector<STrack> activated_stracks;
	std::vector<STrack> refind_stracks;
	std::vector<STrack> removed_stracks;
	std::vector<STrack> lost_stracks;
	std::vector<STrack> detections;
	std::vector<STrack> detections_low;

	std::vector<STrack> detections_cp;
	std::vector<STrack> tracked_stracks_swap;
	std::vector<STrack> resa, resb;
	std::vector<STrack> output_stracks;

	std::vector<STrack*> unconfirmed;
	std::vector<STrack*> tracked_stracks;
	std::vector<STrack*> strack_pool;
	std::vector<STrack*> r_tracked_stracks;

	if (objects.size() > 0)
	{
		for (std::size_t i = 0; i < objects.size(); i++)
		{
			std::vector<float> tlbr_;
			tlbr_.resize(4);
			tlbr_[0] = objects[i].rect.x;
			tlbr_[1] = objects[i].rect.y;
			tlbr_[2] = objects[i].rect.x + objects[i].rect.width;
			tlbr_[3] = objects[i].rect.y + objects[i].rect.height;

			float score = objects[i].prob;

			STrack strack(STrack::tlbr_to_tlwh(tlbr_), score, objects[i].label_ids, objects[i].confidences, objects[i].objectness);

		if (score >= m_track_thresh)
			{
				detections.push_back(strack);
			}
			else
			{
				detections_low.push_back(strack);
			}
			
		}
	}

	// Add newly detected tracklets to tracked_stracks
	for (std::size_t i = 0; i < this->m_tracked_stracks.size(); i++)
	{
		if (!this->m_tracked_stracks[i].is_activated)
			unconfirmed.push_back(&this->m_tracked_stracks[i]);
		else
			tracked_stracks.push_back(&this->m_tracked_stracks[i]);
	}

	////////////////// Step 2: First association, with IoU //////////////////
	strack_pool = joint_stracks(tracked_stracks, this->m_lost_stracks);
	STrack::multi_predict(strack_pool, this->m_kalman_filter);

	std::vector<std::vector<float> > dists;
	int dist_size = 0, dist_size_size = 0;
	dists = iou_distance(strack_pool, detections, dist_size, dist_size_size);

	std::vector<std::vector<int> > matches;
	std::vector<int> u_track, u_detection;
	linear_assignment(dists, dist_size, dist_size_size, m_match_thresh, matches, u_track, u_detection);

	for (std::size_t i = 0; i < matches.size(); i++)
	{
		STrack *track = strack_pool[static_cast<size_t>(matches[i][0])];
		STrack *det = &detections[static_cast<size_t>(matches[i][1])];
		if (track->state == TrackState::Tracked)
		{
			track->update(*det, this->m_frame_id);
			activated_stracks.push_back(*track);
		}
		else
		{
			track->re_activate(*det, this->m_frame_id, false);
			refind_stracks.push_back(*track);
		}
	}

	////////////////// Step 3: Second association, using low score dets //////////////////
	for (std::size_t i = 0; i < u_detection.size(); i++)
	{
		detections_cp.push_back(detections[static_cast<size_t>(u_detection[i])]);
	}
	detections.clear();
	detections.assign(detections_low.begin(), detections_low.end());
	
	for (std::size_t i = 0; i < u_track.size(); i++)
	{
		if (strack_pool[static_cast<size_t>(u_track[i])]->state == TrackState::Tracked)
		{
			r_tracked_stracks.push_back(strack_pool[static_cast<size_t>(u_track[i])]);
		}
	}

	dists.clear();
	dists = iou_distance(r_tracked_stracks, detections, dist_size, dist_size_size);

	matches.clear();
	u_track.clear();
	u_detection.clear();
	linear_assignment(dists, dist_size, dist_size_size, 0.5, matches, u_track, u_detection);

	for (std::size_t i = 0; i < matches.size(); i++)
	{
		STrack *track = r_tracked_stracks[static_cast<size_t>(matches[i][0])];
		STrack *det = &detections[static_cast<size_t>(matches[i][1])];
		if (track->state == TrackState::Tracked)
		{
			track->update(*det, this->m_frame_id);
			activated_stracks.push_back(*track);
		}
		else
		{
			track->re_activate(*det, this->m_frame_id, false);
			refind_stracks.push_back(*track);
		}
	}

	for (std::size_t i = 0; i < u_track.size(); i++)
	{
		STrack *track = r_tracked_stracks[static_cast<size_t>(u_track[i])];
		if (track->state != TrackState::Lost)
		{
			track->mark_lost();
			lost_stracks.push_back(*track);
		}
	}

	// Deal with unconfirmed tracks, usually tracks with only one beginning frame
	detections.clear();
	detections.assign(detections_cp.begin(), detections_cp.end());

	dists.clear();
	dists = iou_distance(unconfirmed, detections, dist_size, dist_size_size);

	matches.clear();
	std::vector<int> u_unconfirmed;
	u_detection.clear();
	linear_assignment(dists, dist_size, dist_size_size, 0.7f, matches, u_unconfirmed, u_detection);

	for (std::size_t i = 0; i < matches.size(); i++)
	{
		unconfirmed[static_cast<size_t>(matches[i][0])]->update(detections[static_cast<size_t>(matches[i][1])], this->m_frame_id);
		activated_stracks.push_back(*unconfirmed[static_cast<size_t>(matches[i][0])]);
	}

	for (std::size_t i = 0; i < u_unconfirmed.size(); i++)
	{
		STrack *track = unconfirmed[static_cast<size_t>(u_unconfirmed[i])];
		track->mark_removed();
		removed_stracks.push_back(*track);
	}

	////////////////// Step 4: Init new stracks //////////////////
	for (std::size_t i = 0; i < u_detection.size(); i++)
	{
		STrack *track = &detections[static_cast<size_t>(u_detection[i])];
		if (track->score < this->m_high_thresh)
			continue;
		track->activate(this->m_kalman_filter, this->m_frame_id);
		activated_stracks.push_back(*track);
	}

	////////////////// Step 5: Update state //////////////////
	for (std::size_t i = 0; i < this->m_lost_stracks.size(); i++)
	{
		if (this->m_frame_id - this->m_lost_stracks[i].end_frame() > this->m_max_time_lost)
		{
			this->m_lost_stracks[i].mark_removed();
			removed_stracks.push_back(this->m_lost_stracks[i]);
		}
	}
	
	for (std::size_t i = 0; i < this->m_tracked_stracks.size(); i++)
	{
		if (this->m_tracked_stracks[i].state == TrackState::Tracked)
		{
			tracked_stracks_swap.push_back(this->m_tracked_stracks[i]);
		}
	}
	this->m_tracked_stracks.clear();
	this->m_tracked_stracks.assign(tracked_stracks_swap.begin(), tracked_stracks_swap.end());

	this->m_tracked_stracks = joint_stracks(this->m_tracked_stracks, activated_stracks);
	this->m_tracked_stracks = joint_stracks(this->m_tracked_stracks, refind_stracks);

	//std::cout << activated_stracks.size() << std::endl;

	this->m_lost_stracks = sub_stracks(this->m_lost_stracks, this->m_tracked_stracks);
	for (std::size_t i = 0; i < lost_stracks.size(); i++)
	{
		this->m_lost_stracks.push_back(lost_stracks[i]);
	}

	this->m_lost_stracks = sub_stracks(this->m_lost_stracks, this->m_removed_stracks);
	for (std::size_t i = 0; i < removed_stracks.size(); i++)
	{
		this->m_removed_stracks.push_back(removed_stracks[i]);
	}
	
	remove_duplicate_stracks(resa, resb, this->m_tracked_stracks, this->m_lost_stracks);

	this->m_tracked_stracks.clear();
	this->m_tracked_stracks.assign(resa.begin(), resa.end());
	this->m_lost_stracks.clear();
	this->m_lost_stracks.assign(resb.begin(), resb.end());
	
	for (std::size_t i = 0; i < this->m_tracked_stracks.size(); i++)
	{
		if (this->m_tracked_stracks[i].is_activated)
		{
			output_stracks.push_back(this->m_tracked_stracks[i]);
		}
	}
	return output_stracks;
}
}
