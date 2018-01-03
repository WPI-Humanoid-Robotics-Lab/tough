/**
 ********************************************************************************************************
 * @file    utils.h
 * @brief   utils.h class is 
 * @details Used to ...
 ********************************************************************************************************
 */

#ifndef UTILS_H_
#define UTILS_H_

#include <string>
#include <map>
#include <queue>
#include <tough_perception_common/gnuplot-iostream.h>

class TimePlot
{
	std::queue<int> 					frame_no_;
	std::vector<std::queue<float> >		data_;
	std::map<std::string, int>				name2id_;
	int									window_size_;
public:
	TimePlot(int window_size)
	{
		window_size_=window_size;
	}
	void add(std::string str)
	{
		name2id_.insert(std::pair<std::string,int>(str,data_.size()));
		data_.push_back(std::queue<float>());
	}
	void updateValue(int frame_no,std::string name,float value)
	{
		if(frame_no_.empty())
		{
			frame_no_.push(frame_no);
			for(size_t i=0;i<data_.size();i++)
			{
				data_[i].push(0.0f);
			}
		}
		else if(frame_no!=frame_no_.back())
		{
			frame_no_.push(frame_no);
			for(size_t i=0;i<data_.size();i++)
			{
				data_[i].push(0.0f);
			}
		}
		if(data_[name2id_[name]].back()!=value)
		{
			data_[name2id_[name]].back()=value;
		}
		if((int)frame_no_.size()>window_size_)
		{
			frame_no_.pop();
			for(size_t i=0;i<data_.size();i++)
			{
				data_[i].pop();
			}
		}
	}
	void setup(Gnuplot *plot)
	{
		*plot<<"reset\n";
		*plot<<"set yr [0:35]\n";
	}
	void plot(Gnuplot *plot)
	{
		if((int)frame_no_.size()!=window_size_)
			return;
		*plot<<"set terminal wxt 1 noraise\n";
		setup(plot);
		*plot<<"plot ";
		int ctr=0;
		std::vector<std::pair<int,float> > linePlot(frame_no_.size());
		for(std::map<std::string, int>::iterator i=name2id_.begin();i!=name2id_.end();i++)
		{
			ctr++;
			std::queue<float> q=data_[i->second];
			std::queue<int> f=frame_no_;
			linePlot.clear();
			while(!f.empty())
			{
				linePlot.push_back(std::pair<int,float>(f.front(),q.front()));
				q.pop();
				f.pop();
			}
			if(ctr==(int)name2id_.size())
				 *plot<<plot->binFile1d(linePlot, "record")<<" title '"<<i->first<<"' "<<"with lines "<<std::endl;
			else
				 *plot<<plot->binFile1d(linePlot, "record")<<" title '"<<i->first<<"' with lines, ";
		 }
		*plot<<"refresh\n";
	}
	~TimePlot()
	{

	}
};





#endif /* UTILS_H_ */
