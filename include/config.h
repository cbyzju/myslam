#ifndef CONFIG_H_
#define CONFIG_H_

#include "common.h"

namespace myslam 
{
	class Config
	{
	private:
		static shared_ptr<Config> config_;
		cv::FileStorage file_;
		Config() {}	//private constructor makes a singleton

	public:
		~Config();
		static void setParameterFile(const std::string& filename); //set a new config file
		template<typename T>
		static T get(const std::string& key)
		{
			return T(Config::config_->file_[key]);
		}

	};
}
#endif