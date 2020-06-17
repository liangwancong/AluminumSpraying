#pragma once
#include <string>
#include <vector>

namespace fileutil {

#ifdef DEBUGC
#define savePath getRunPath() + "\\aluminum\\"
#define saveCloudPath savePath + "cloud\\"
#define saveGroupPath savePath + "group\\"
#define saveModelPath savePath + "trainModel\\"
#define saveScanPath getRunPath() + "\\scan\\"
#define saveModelImgPath savePath + "trainImage\\"
#endif 


	/******************************************************************************************
	Function:       saveObjectToFile
	Description:    存储对象到文件夹
	Input:          _Filename:文件夹路径 _Object:对象 _Count:对象大小
	*******************************************************************************************/
	void saveObjectToFile(const char *_Filename, char *_Object,
		long long _Count);

	/******************************************************************************************
	Function:       readObjectFromFile
	Description:    从文件中读取对象
	Input:          _Filename:文件夹路径 _Object:对象 _Count:对象大小
	Output:         _Object:对象
	*******************************************************************************************/
	void readObjectFromFile(const char *_Filename, char *_Object,
		long long _Count);

	/******************************************************************************************
	Function:       getFilesName
	Description:    获取文件列表
	Input:          folder_path:文件夹路径
	Output:         files:用于保存文件名称的数组
	*******************************************************************************************/
	void getFilesName(std::string folder_path, std::vector<std::string>& files, std::string format = "");
	void getFiles(std::string path, std::vector<std::string>& files);
	/******************************************************************************************
	Function:       copyFile
	Description:    复制文件
	Input:          SourceFile:原文件路径
	Output:         NewFile:新文件的路径
	*******************************************************************************************/
	int copyFile(const char *SourceFile, const char *NewFile);
	/******************************************************************************************
	Function:       getRunPath
	Description:    获取执行文件路径
	Return:		执行文件路径
	*******************************************************************************************/
	std::string getRunPath();
	/******************************************************************************************
	Function:       createFolder
	Description:    创建文件夹
	Input:         文件夹名称
	*******************************************************************************************/
	void createFolder(std::string folder);
	std::string  getFileName(std::string file_path);
	std::string  getFilePath(std::string file_path);
	int fileExist(char *file);
}
