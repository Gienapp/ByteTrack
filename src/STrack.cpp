#include "STrack.h"
namespace ByteTrack{
STrack::STrack(std::vector<float> tlwh_, float score_)
{
	_tlwh.resize(4);
	_tlwh.assign(tlwh_.begin(), tlwh_.end());

	is_activated = false;
	track_id = 0;
	state = TrackState::New;
	
	tlwh.resize(4);
	tlbr.resize(4);

	static_tlwh();
	static_tlbr();
	frame_id = 0;
	tracklet_len = 0;
	this->score = score_;
	start_frame = 0;
}

STrack::STrack(std::vector<float> tlwh_, float score_, std::vector<int> label_ids_, std::vector<float> confidences_, float objectness_, int time_stamp_)
{
	_tlwh.resize(4);
	_tlwh.assign(tlwh_.begin(), tlwh_.end());

	is_activated = false;
	track_id = 0;
	state = TrackState::New;
	
	tlwh.resize(4);
	tlbr.resize(4);

	static_tlwh();
	static_tlbr();
	frame_id = 0;
	tracklet_len = 0;
	this->score = score_;
	this->label_ids = label_ids_;
	this->confidences = confidences_;
	for(auto label_id : label_ids_){
		ByteTrack::FixedQueue<int, 10> label_queue;
		label_queue.push(label_id);
		this->label_queues.push_back(label_queue);
	}
	this->objectness = objectness_;
	this->track_start_time_stamp = time_stamp_;
	
	start_frame = 0;
}

STrack::~STrack()
{
}

void STrack::activate(byte_kalman::KalmanFilter &kalman_filter_, int frame_id_)
{
	this->kalman_filter = kalman_filter_;
	this->track_id = this->next_id();

	std::vector<float> _tlwh_tmp(4);
	_tlwh_tmp[0] = this->_tlwh[0];
	_tlwh_tmp[1] = this->_tlwh[1];
	_tlwh_tmp[2] = this->_tlwh[2];
	_tlwh_tmp[3] = this->_tlwh[3];
	std::vector<float> xyah = tlwh_to_xyah(_tlwh_tmp);
	DETECTBOX xyah_box;
	xyah_box[0] = xyah[0];
	xyah_box[1] = xyah[1];
	xyah_box[2] = xyah[2];
	xyah_box[3] = xyah[3];
	auto mc = this->kalman_filter.initiate(xyah_box);
	this->mean = mc.first;
	this->covariance = mc.second;

	static_tlwh();
	static_tlbr();

	this->tracklet_len = 0;
	this->state = TrackState::Tracked;
	if (frame_id_ == 1)
	{
		this->is_activated = true;
	}
	//this->is_activated = true;
	this->frame_id = frame_id_;
	this->start_frame = frame_id_;
}

void STrack::re_activate(STrack &new_track, int frame_id_, bool new_id)
{
	std::vector<float> xyah = tlwh_to_xyah(new_track.tlwh);
	DETECTBOX xyah_box;
	xyah_box[0] = xyah[0];
	xyah_box[1] = xyah[1];
	xyah_box[2] = xyah[2];
	xyah_box[3] = xyah[3];
	auto mc = this->kalman_filter.update(this->mean, this->covariance, xyah_box);
	this->mean = mc.first;
	this->covariance = mc.second;

	static_tlwh();
	static_tlbr();

	this->tracklet_len = 0;
	this->state = TrackState::Tracked;
	this->is_activated = true;
	this->frame_id = frame_id_;
	this->score = new_track.score;
	if (new_id)
		this->track_id = next_id();

	for(std::vector<int>::size_type i=0; i<this->label_ids.size(); i++){
		this->label_queues[i].push(new_track.label_ids[i]);
		this->label_ids[i] = this->label_queues[i].most_frequent_element();
		if(this->label_ids[i] == new_track.label_ids[i]){
			this->track_duration = new_track.track_start_time_stamp - this->track_start_time_stamp;
		} else{
			this->track_start_time_stamp = new_track.track_start_time_stamp;
			this->track_duration = 0;
		}
	}
}

void STrack::update(STrack &new_track, int frame_id_)
{
	this->frame_id = frame_id_;
	this->tracklet_len++;

	std::vector<float> xyah = tlwh_to_xyah(new_track.tlwh);
	DETECTBOX xyah_box;
	xyah_box[0] = xyah[0];
	xyah_box[1] = xyah[1];
	xyah_box[2] = xyah[2];
	xyah_box[3] = xyah[3];

	auto mc = this->kalman_filter.update(this->mean, this->covariance, xyah_box);
	this->mean = mc.first;
	this->covariance = mc.second;

	static_tlwh();
	static_tlbr();

	this->state = TrackState::Tracked;
	this->is_activated = true;

	this->score = new_track.score;

	for(std::vector<int>::size_type i=0; i<this->label_ids.size(); i++){
		this->label_queues[i].push(new_track.label_ids[i]);
		this->label_ids[i] = this->label_queues[i].most_frequent_element();
		if(this->label_ids[i] == new_track.label_ids[i]){
			this->track_duration = new_track.track_start_time_stamp - this->track_start_time_stamp;
		} else{
			this->track_start_time_stamp = new_track.track_start_time_stamp;
			this->track_duration = 0;
		}
	}
}

void STrack::static_tlwh()
{
	if (this->state == TrackState::New)
	{
		tlwh[0] = _tlwh[0];
		tlwh[1] = _tlwh[1];
		tlwh[2] = _tlwh[2];
		tlwh[3] = _tlwh[3];
		return;
	}

	tlwh[0] = mean[0];
	tlwh[1] = mean[1];
	tlwh[2] = mean[2];
	tlwh[3] = mean[3];

	tlwh[2] *= tlwh[3];
	tlwh[0] -= tlwh[2] / 2;
	tlwh[1] -= tlwh[3] / 2;
}

void STrack::static_tlbr()
{
	tlbr.clear();
	tlbr.assign(tlwh.begin(), tlwh.end());
	tlbr[2] += tlbr[0];
	tlbr[3] += tlbr[1];
}

std::vector<float> STrack::tlwh_to_xyah(std::vector<float> tlwh_tmp)
{
	std::vector<float> tlwh_output = tlwh_tmp;
	tlwh_output[0] += tlwh_output[2] / 2;
	tlwh_output[1] += tlwh_output[3] / 2;
	tlwh_output[2] /= tlwh_output[3];
	return tlwh_output;
}

std::vector<float> STrack::to_xyah()
{
	return tlwh_to_xyah(tlwh);
}

std::vector<float> STrack::tlbr_to_tlwh(std::vector<float> &tlbr)
{
	tlbr[2] -= tlbr[0];
	tlbr[3] -= tlbr[1];
	return tlbr;
}

void STrack::mark_lost()
{
	state = TrackState::Lost;
}

void STrack::mark_removed()
{
	state = TrackState::Removed;
}

int STrack::next_id()
{
	static int _count = 0;
	_count++;
	return _count;
}

int STrack::end_frame()
{
	return this->frame_id;
}

void STrack::multi_predict(std::vector<STrack*> &stracks, byte_kalman::KalmanFilter &kalman_filter)
{
	for (std::vector<ByteTrack::STrack*>::size_type i = 0; i < stracks.size(); i++)
	{
		if (stracks[i]->state != TrackState::Tracked)
		{
			stracks[i]->mean[7] = 0;
		}
		kalman_filter.predict(stracks[i]->mean, stracks[i]->covariance);
		stracks[i]->static_tlwh();
		stracks[i]->static_tlbr();
	}
}
}
