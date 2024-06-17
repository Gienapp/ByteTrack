#include "BYTETracker.h"
#include "lapjv.h"
namespace ByteTrack{
std::vector<STrack*> BYTETracker::joint_stracks(std::vector<STrack*> &tlista, std::vector<STrack> &tlistb)
{
	std::map<int, int> exists;
	std::vector<STrack*> res;
	for (std::size_t i = 0; i < tlista.size(); i++)
	{
		exists.insert(std::pair<int, int>(tlista[i]->track_id, 1));
		res.push_back(tlista[i]);
	}
	for (std::size_t i = 0; i < tlistb.size(); i++)
	{
		int tid = tlistb[i].track_id;
		if (!exists[tid] || exists.count(tid) == 0)
		{
			exists[tid] = 1;
			res.push_back(&tlistb[i]);
		}
	}
	return res;
}

std::vector<STrack> BYTETracker::joint_stracks(std::vector<STrack> &tlista, std::vector<STrack> &tlistb)
{
	std::map<int, int> exists;
	std::vector<STrack> res;
	for (std::size_t i = 0; i < tlista.size(); i++)
	{
		exists.insert(std::pair<int, int>(tlista[i].track_id, 1));
		res.push_back(tlista[i]);
	}
	for (std::size_t i = 0; i < tlistb.size(); i++)
	{
		int tid = tlistb[i].track_id;
		if (!exists[tid] || exists.count(tid) == 0)
		{
			exists[tid] = 1;
			res.push_back(tlistb[i]);
		}
	}
	return res;
}

std::vector<STrack> BYTETracker::sub_stracks(std::vector<STrack> &tlista, std::vector<STrack> &tlistb)
{
	std::map<int, STrack> stracks;
	for (std::size_t i = 0; i < tlista.size(); i++)
	{
		stracks.insert(std::pair<int, STrack>(tlista[i].track_id, tlista[i]));
	}
	for (std::size_t i = 0; i < tlistb.size(); i++)
	{
		int tid = tlistb[i].track_id;
		if (stracks.count(tid) != 0)
		{
			stracks.erase(tid);
		}
	}

	std::vector<STrack> res;
	std::map<int, STrack>::iterator  it;
	for (it = stracks.begin(); it != stracks.end(); ++it)
	{
		res.push_back(it->second);
	}

	return res;
}

void BYTETracker::remove_duplicate_stracks(std::vector<STrack> &resa, std::vector<STrack> &resb, std::vector<STrack> &stracksa, std::vector<STrack> &stracksb)
{
	std::vector<std::vector<float> > pdist = iou_distance(stracksa, stracksb);
	std::vector<std::pair<int, int> > pairs;
	for (std::size_t i = 0; i < pdist.size(); i++)
	{
		for (std::size_t j = 0; j < pdist[i].size(); j++)
		{
			if (pdist[i][j] < 0.15f)
			{
				pairs.push_back(std::pair<int, int>(i, j));
			}
		}
	}

	std::vector<int> dupa, dupb;
	for (std::size_t i = 0; i < pairs.size(); i++)
	{
		int timep = stracksa[static_cast<std::size_t>(pairs[i].first)].frame_id - stracksa[static_cast<std::size_t>(pairs[i].first)].start_frame;
		int timeq = stracksb[static_cast<std::size_t>(pairs[i].second)].frame_id - stracksb[static_cast<std::size_t>(pairs[i].second)].start_frame;
		if (timep > timeq)
			dupb.push_back(pairs[i].second);
		else
			dupa.push_back(pairs[i].first);
	}

	for (std::size_t i = 0; i < stracksa.size(); i++)
	{
		std::vector<int>::iterator iter = find(dupa.begin(), dupa.end(), i);
		if (iter == dupa.end())
		{
			resa.push_back(stracksa[i]);
		}
	}

	for (std::size_t i = 0; i < stracksb.size(); i++)
	{
		std::vector<int>::iterator iter = find(dupb.begin(), dupb.end(), i);
		if (iter == dupb.end())
		{
			resb.push_back(stracksb[i]);
		}
	}
}

void BYTETracker::linear_assignment(std::vector<std::vector<float> > &cost_matrix, int cost_matrix_size, int cost_matrix_size_size, float thresh,
	std::vector<std::vector<int> > &matches, std::vector<int> &unmatched_a, std::vector<int> &unmatched_b)
{
	if (cost_matrix.size() == 0)
	{
		for (int i = 0; i < cost_matrix_size; i++)
		{
			unmatched_a.push_back(i);
		}
		for (int i = 0; i < cost_matrix_size_size; i++)
		{
			unmatched_b.push_back(i);
		}
		return;
	}

	std::vector<int> rowsol; std::vector<int> colsol;
	float c = static_cast<float>(lapjv(cost_matrix, rowsol, colsol, true, thresh));
	// c has to be used such that there is no warning for it :D 
	c = c + 1;
	for (std::size_t i = 0; i < rowsol.size(); i++)
	{
		if (rowsol[i] >= 0)
		{
			std::vector<int> match;
			match.push_back(static_cast<int>(i));
			match.push_back(rowsol[i]);
			matches.push_back(match);
		}
		else
		{
			unmatched_a.push_back(static_cast<int>(i));
		}
	}

	for (std::size_t i = 0; i < colsol.size(); i++)
	{
		if (colsol[i] < 0)
		{
			unmatched_b.push_back(static_cast<int>(i));
		}
	}
}

std::vector<std::vector<float> > BYTETracker::ious(std::vector<std::vector<float> > &atlbrs, std::vector<std::vector<float> > &btlbrs)
{
	std::vector<std::vector<float> > ious;
	if (atlbrs.size()*btlbrs.size() == 0)
		return ious;

	ious.resize(atlbrs.size());
	for (std::size_t i = 0; i < ious.size(); i++)
	{
		ious[i].resize(btlbrs.size());
	}

	//bbox_ious
	for (std::size_t k = 0; k < btlbrs.size(); k++)
	{
		std::vector<float> ious_tmp;
		float box_area = (btlbrs[k][2] - btlbrs[k][0] + 1)*(btlbrs[k][3] - btlbrs[k][1] + 1);
		for (std::size_t n = 0; n < atlbrs.size(); n++)
		{
			float iw = std::min(atlbrs[n][2], btlbrs[k][2]) - std::max(atlbrs[n][0], btlbrs[k][0]) + 1;
			if (iw > 0)
			{
				float ih = std::min(atlbrs[n][3], btlbrs[k][3]) - std::max(atlbrs[n][1], btlbrs[k][1]) + 1;
				if(ih > 0)
				{
					float ua = (atlbrs[n][2] - atlbrs[n][0] + 1)*(atlbrs[n][3] - atlbrs[n][1] + 1) + box_area - iw * ih;
					ious[n][k] = iw * ih / ua;
				}
				else
				{
					ious[n][k] = 0.0;
				}
			}
			else
			{
				ious[n][k] = 0.0;
			}
		}
	}

	return ious;
}

std::vector<std::vector<float> > BYTETracker::iou_distance(std::vector<STrack*> &atracks, std::vector<STrack> &btracks, int &dist_size, int &dist_size_size)
{
	std::vector<std::vector<float> > cost_matrix;
	if (atracks.size() * btracks.size() == 0)
	{
		dist_size = static_cast<int>(atracks.size());
		dist_size_size = static_cast<int>(btracks.size());
		return cost_matrix;
	}
	std::vector<std::vector<float> > atlbrs, btlbrs;
	for (std::size_t i = 0; i < atracks.size(); i++)
	{
		atlbrs.push_back(atracks[i]->tlbr);
	}
	for (std::size_t i = 0; i < btracks.size(); i++)
	{
		btlbrs.push_back(btracks[i].tlbr);
	}

	dist_size = static_cast<int>(atracks.size());
	dist_size_size = static_cast<int>(btracks.size());

	std::vector<std::vector<float> > _ious = ious(atlbrs, btlbrs);
	
	for (std::size_t i = 0; i < _ious.size();i++)
	{
		std::vector<float> _iou;
		for (std::size_t j = 0; j < _ious[i].size(); j++)
		{
			_iou.push_back(1 - _ious[i][j]);
		}
		cost_matrix.push_back(_iou);
	}

	return cost_matrix;
}

std::vector<std::vector<float> > BYTETracker::iou_distance(std::vector<STrack> &atracks, std::vector<STrack> &btracks)
{
	std::vector<std::vector<float> > atlbrs, btlbrs;
	for (std::size_t i = 0; i < atracks.size(); i++)
	{
		atlbrs.push_back(atracks[i].tlbr);
	}
	for (std::size_t i = 0; i < btracks.size(); i++)
	{
		btlbrs.push_back(btracks[i].tlbr);
	}

	std::vector<std::vector<float> > _ious = ious(atlbrs, btlbrs);
	std::vector<std::vector<float> > cost_matrix;
	for (std::size_t i = 0; i < _ious.size(); i++)
	{
		std::vector<float> _iou;
		for (std::size_t j = 0; j < _ious[i].size(); j++)
		{
			_iou.push_back(1 - _ious[i][j]);
		}
		cost_matrix.push_back(_iou);
	}

	return cost_matrix;
}

double BYTETracker::lapjv(const std::vector<std::vector<float> > &cost, std::vector<int> &rowsol, std::vector<int> &colsol,
	bool extend_cost, float cost_limit, bool return_cost)
{
	std::vector<std::vector<float> > cost_c;
	cost_c.assign(cost.begin(), cost.end());

	std::vector<std::vector<float> > cost_c_extended;

	std::size_t n_rows = cost.size();
	std::size_t n_cols = cost[0].size();
	rowsol.resize(n_rows);
	colsol.resize(n_cols);

	std::size_t n = 0;
	if (n_rows == n_cols)
	{
		n = n_rows;
	}
	else
	{
		if (!extend_cost)
		{
			std::cout << "set extend_cost=True" << std::endl;
			system("pause");
			exit(0);
		}
	}
		
	if (extend_cost || cost_limit < static_cast<float>(LONG_MAX))
	{
		n = n_rows + n_cols;
		cost_c_extended.resize(n);
		for (std::size_t i = 0; i < cost_c_extended.size(); i++)
			cost_c_extended[i].resize(n);

		if (cost_limit < static_cast<float>(LONG_MAX))
		{
			for (std::size_t i = 0; i < cost_c_extended.size(); i++)
			{
				for (std::size_t j = 0; j < cost_c_extended[i].size(); j++)
				{
					cost_c_extended[i][j] = cost_limit / 2.0f;
				}
			}
		}
		else
		{
			float cost_max = -1;
			for (std::size_t i = 0; i < cost_c.size(); i++)
			{
				for (std::size_t j = 0; j < cost_c[i].size(); j++)
				{
					if (cost_c[i][j] > cost_max)
						cost_max = cost_c[i][j];
				}
			}
			for (std::size_t i = 0; i < cost_c_extended.size(); i++)
			{
				for (std::size_t j = 0; j < cost_c_extended[i].size(); j++)
				{
					cost_c_extended[i][j] = cost_max + 1;
				}
			}
		}

		for (std::size_t i = n_rows; i < cost_c_extended.size(); i++)
		{
			for (std::size_t j = n_cols; j < cost_c_extended[i].size(); j++)
			{
				cost_c_extended[i][j] = 0;
			}
		}
		for (std::size_t i = 0; i < n_rows; i++)
		{
			for (std::size_t j = 0; j < n_cols; j++)
			{
				cost_c_extended[i][j] = cost_c[i][j];
			}
		}

		cost_c.clear();
		cost_c.assign(cost_c_extended.begin(), cost_c_extended.end());
	}

	double **cost_ptr;
	cost_ptr = new double *[sizeof(double *) * n];
	for (std::size_t i = 0; i < n; i++)
		cost_ptr[i] = new double[sizeof(double) * n];

	for (std::size_t i = 0; i < n; i++)
	{
		for (std::size_t j = 0; j < n; j++)
		{
			cost_ptr[i][j] = cost_c[i][j];
		}
	}

	int* x_c = new int[sizeof(int) * n];
	int *y_c = new int[sizeof(int) * n];

	int ret = lapjv_internal(static_cast<ByteTrack::uint_t>(n), cost_ptr, x_c, y_c);
	if (ret != 0)
	{
		std::cout << "Calculate Wrong!" << std::endl;
		system("pause");
		exit(0);
	}

	double opt = 0.0;

	if (n != n_rows)
	{
		for (std::size_t i = 0; i < n; i++)
		{
			if (x_c[i] >= static_cast<int>(n_cols))
				x_c[i] = -1;
			if (y_c[i] >= static_cast<int>(n_rows))
				y_c[i] = -1;
		}
		for (std::size_t i = 0; i < n_rows; i++)
		{
			rowsol[i] = x_c[i];
		}
		for (std::size_t i = 0; i < n_cols; i++)
		{
			colsol[i] = y_c[i];
		}

		if (return_cost)
		{
			for (std::size_t i = 0; i < rowsol.size(); i++)
			{
				if (rowsol[i] != -1)
				{
					//cout << i << "\t" << rowsol[i] << "\t" << cost_ptr[i][rowsol[i]] << endl;
					opt += cost_ptr[i][rowsol[i]];
				}
			}
		}
	}
	else if (return_cost)
	{
		for (std::size_t i = 0; i < rowsol.size(); i++)
		{
			opt += cost_ptr[i][rowsol[i]];
		}
	}

	for (std::size_t i = 0; i < n; i++)
	{
		delete[]cost_ptr[i];
	}
	delete[]cost_ptr;
	delete[]x_c;
	delete[]y_c;

	return opt;
}

cv::Scalar BYTETracker::get_color(int idx)
{
	idx += 3;
	return cv::Scalar(37 * idx % 255, 17 * idx % 255, 29 * idx % 255);
}
}