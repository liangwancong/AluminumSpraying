#include "logger.h"
#include <cstdlib>
#include <ctime>

#include <iostream>
#include <fstream>
#include <io.h>
#include <direct.h>
#include "Win32\OniPlatformWin32.h"

namespace logger {
	std::ofstream Logger::m_error_log_file;
	std::ofstream Logger::m_info_log_file;
	std::ofstream Logger::m_warn_log_file;

	void initLogger(const std::string&info_log_filename,
		const std::string&warn_log_filename,
		const std::string&error_log_filename) {
		//if (fileExist_logger((char*)info_log_filename.c_str())!=0) {
		//	logger::initLogger(logs_path, logs_path, logs_path);
		//}
		if (!Logger::m_info_log_file.is_open()) {
			Logger::m_info_log_file.open(info_log_filename.c_str(), std::ios::app);
			Logger::m_warn_log_file.open(warn_log_filename.c_str(), std::ios::app);
			Logger::m_error_log_file.open(error_log_filename.c_str(), std::ios::app);
		}

	}

	//std::ostream& Logger::getStream(log_rank_t log_rank) {
	//	return (INFO == log_rank) ?
	//		(m_info_log_file.is_open() ? m_info_log_file : std::cout) :
	//		(WARNING == log_rank ?
	//		(m_warn_log_file.is_open() ? m_warn_log_file : std::cerr) :
	//			(m_error_log_file.is_open() ? m_error_log_file : std::cerr));
	//}

	std::string getRunPath_logger(){
		TCHAR szLongPathName[_MAX_PATH];
		GetModuleFileName(NULL, szLongPathName, _MAX_PATH);
		std::string runPath(szLongPathName);
		int pos = runPath.find_last_of('\\');
		runPath = runPath.substr(0, pos);
		return runPath;
	}
	int fileExist_logger(char *file) {
		std::fstream File;
		File.open(file, std::ios::in);
		if (!File)
		{
			return -1;
		}
		return 0;
	}

	

	//fileutil::getFilesName(dirString, fileNames);
			//for (int i = 0; i < fileNames.size(); i++) {

	std::ostream& Logger::getStream(log_rank_t log_rank) {
		time_t c_time;
		c_time = time(NULL); //获取日历时间  
		struct tm* t = localtime(&c_time);
		std::string logs_path = "";
		if (t->tm_mon + 1 < 10) {
			logs_path = saveLoggerPath + std::to_string(t->tm_year + 1900) +"0"+ std::to_string(t->tm_mon + 1) + std::to_string(t->tm_mday) + ".txt";
		}
		else {
			logs_path = saveLoggerPath + std::to_string(t->tm_year + 1900) + std::to_string(t->tm_mon + 1) + std::to_string(t->tm_mday) + ".txt";
		}
	
		//if (fileExist_logger((char*)logs_path.c_str())!=0) {
		//	logger::initLogger(logs_path, logs_path, logs_path);
		//}
		//m_info_log_file.close();
		//m_info_log_file.close();
		//m_info_log_file.close();
		logger::initLogger(logs_path, logs_path, logs_path);

		return (INFO == log_rank|| WARNING == log_rank|| ERRORL == log_rank) ?
			(m_info_log_file.is_open() ? m_info_log_file : std::cout) :
			(WARNING == log_rank ?
			(m_warn_log_file.is_open() ? m_warn_log_file : std::cerr) :
				(m_error_log_file.is_open() ? m_error_log_file : std::cerr));
	}

	std::ostream& Logger::start(log_rank_t log_rank,
		const int long line,
		const std::string&function) {
		time_t tm;
		time(&tm);
		char time_string[128];
		time_t c_time;
		c_time = time(NULL); //获取日历时间  
		struct tm* t = localtime(&c_time);
		std::string log_type = " [None] ";
		if (log_rank == INFO) {
			log_type = " [INFO] ";
		}
		else if (log_rank == WARNING) {
			log_type = " [WARNING] ";
		}
		else if (log_rank == ERRORL) {
			log_type = " [ERROR] ";
		}

		//return getStream(log_rank) << t->tm_year + 1900<< t->tm_mon + 1<< t->tm_mday<< t->tm_hour<< t->tm_min<< t->tm_sec<<":"
		//	<< log_type
		//	<< "function (" << function << ")"
		//	<< "line " << line
		//	<< std::flush;
		
		std::string head = std::to_string(t->tm_year + 1900) + std::to_string(t->tm_mon + 1) + std::to_string(t->tm_mday) + std::to_string(t->tm_hour) + std::to_string(t->tm_min) + std::to_string(t->tm_sec) + ":"
			+ log_type
			+ "function (" + function + ")"
			+ "line " + std::to_string(line)
			+ "--->";
		return getStream(log_rank).write(head.c_str(), head.length());
	}

	Logger::~Logger() {
		getStream(m_log_rank) << std::endl << std::flush;

		if (FATAL == m_log_rank) {
			m_info_log_file.close();
			m_info_log_file.close();
			m_info_log_file.close();
			abort();
		}
	}
}
