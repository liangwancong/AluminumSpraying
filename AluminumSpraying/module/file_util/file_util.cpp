#include "file_util.h"

#include <iostream>
#include <fstream>
#include <io.h>
#include <direct.h>

#include "Win32\OniPlatformWin32.h"


namespace fileutil {
	using namespace std;

	/******************************************************************************************
	Function:       saveObjectToFile
	Description:    存储对象到文件夹
	Input:          _Filename:文件夹路径 _Object:对象 _Count:对象大小
	*******************************************************************************************/
	void saveObjectToFile(const char *_Filename, char *_Object,
		long long _Count) {
		try
		{
			ofstream outfile(_Filename, ios::binary);
			outfile.write(_Object, sizeof(_Count));
			outfile.close();
		}
		catch (const std::exception&)
		{

		}
	}

	/******************************************************************************************
	Function:       readObjectFromFile
	Description:    从文件中读取对象
	Input:          _Filename:文件夹路径 _Object:对象 _Count:对象大小
	Output:         _Object:对象
	*******************************************************************************************/
	void readObjectFromFile(const char *_Filename, char *_Object,
		long long _Count) {
		try
		{
			ifstream infile(_Filename, ios::binary);
			infile.read(_Object, _Count);
			infile.close();
		}
		catch (const std::exception&)
		{

		}
	}

	/******************************************************************************************
	Function:       getFilesName
	Description:    获取文件列表
	Input:          folder_path:文件夹路径
	Output:         files:用于保存文件名称的数组
	*******************************************************************************************/
	void getFilesName(std::string folder_path, std::vector<std::string>& files, std::string format)
	{

		//文件句柄
		//intptr_t hFile = 0;//Win10
		intptr_t hFile = 0;
		//文件信息  
		struct _finddata_t fileinfo;
		std::string p;
		try
		{
			if ((hFile = _findfirst(p.assign(folder_path).append("\\*").c_str(), &fileinfo)) != -1)
			{
				do
				{
					//如果是目录,迭代之  
					//如果不是,加入列表  
					if ((fileinfo.attrib &  _A_SUBDIR))
					{
						if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
							getFilesName(p.assign(folder_path).append("\\").append(fileinfo.name), files);
					}
					else
					{

						files.push_back(fileinfo.name);
					}
				} while (_findnext(hFile, &fileinfo) == 0);

				_findclose(hFile);
			}
		}
		catch (std::exception e)
		{
		}
	}
	void getFiles(std::string path, std::vector<std::string>& files)
	{
		//文件句柄  
		//long hFile = 0;  //win7
		intptr_t hFile = 0;   //win10
		//文件信息  
		struct _finddata_t fileinfo;
		string p;
		if ((hFile = _findfirst(p.assign(path).append("\\*").c_str(), &fileinfo)) != -1)
			// "\\*"是指读取文件夹下的所有类型的文件，若想读取特定类型的文件，以png为例，则用“\\*.png”
		{
			do
			{
				//如果是目录,迭代之  
				//如果不是,加入列表  
				if ((fileinfo.attrib &  _A_SUBDIR))
				{
					if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
						getFiles(p.assign(path).append("\\").append(fileinfo.name), files);
				}
				else
				{
					files.push_back(path + "\\" + fileinfo.name);
				}
			} while (_findnext(hFile, &fileinfo) == 0);
			_findclose(hFile);
		}
	}
	/******************************************************************************************
	Function:       getFilesPath
	Description:    获取文件列表
	Input:          folder_path:文件夹路径
	Output:         files:用于保存文件路径的数组
	*******************************************************************************************/
	void getFilesPath(std::string folder_path, std::vector<std::string>& files)
	{

		//文件句柄
		//intptr_t hFile = 0;//Win10
		long hFile = 0;
		//文件信息  
		struct _finddata_t fileinfo;
		std::string p;
		try
		{
			if ((hFile = _findfirst(p.assign(folder_path).append("\\*").c_str(), &fileinfo)) != -1)
			{
				do
				{
					//如果是目录,迭代之  
					//如果不是,加入列表  
					if ((fileinfo.attrib &  _A_SUBDIR))
					{
						if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
							getFilesPath(p.assign(folder_path).append("\\").append(fileinfo.name), files);
					}
					else
					{

						files.push_back(p.assign(folder_path).append("\\").append(fileinfo.name));
					}
				} while (_findnext(hFile, &fileinfo) == 0);

				_findclose(hFile);
			}
		}
		catch (std::exception e)
		{
		}
	}

	int copyFile(const char *SourceFile, const char *NewFile)
	{
		if (strcmp(SourceFile, NewFile)==0)
		{
			return 1;
		}
		std::ifstream in;

		std::ofstream out;

		try

		{

			in.open(SourceFile, std::ios::binary);//打开源文件

			if (in.fail())//打开源文件失败

			{

				std::cout << "Error 1: Fail to open the source file." << std::endl;

				in.close();

				out.close();

				return 0;

			}

			out.open(NewFile, std::ios::binary);//创建目标文件

			if (out.fail())//创建文件失败

			{

				std::cout << "Error 2: Fail to create the new file." << std::endl;

				out.close();

				in.close();

				return 0;

			}

			else//复制文件

			{

				out << in.rdbuf();

				out.close();

				in.close();

				return 1;

			}

		}

		catch (std::exception e)

		{

		}

	}
	/******************************************************************************************
	Function:       getRunPath
	Description:    获取执行文件路径
	Return:		执行文件路径
	*******************************************************************************************/
	std::string getRunPath(){
		TCHAR szLongPathName[_MAX_PATH];
		GetModuleFileName(NULL, szLongPathName, _MAX_PATH);
		std::string runPath(szLongPathName);
		int pos = runPath.find_last_of('\\');
		runPath = runPath.substr(0, pos);
		return runPath;
	}

	/******************************************************************************************
	Function:       createFolder
	Description:    创建文件夹
	Input:         文件夹名称
	*******************************************************************************************/
	void createFolder(std::string folder) {
		std::string prefix = folder;
		char *sub = strtok((char*)prefix.c_str(), "\\");

		std::string sub_folder;
		while (sub != NULL) {
			std::string substr(sub);
			sub_folder += substr+ "\\";
			sub = strtok(NULL, "\\");
			//std::cout << "************" << sub_folder << std::endl;
			if (access(sub_folder.c_str(), 0) == -1)	//如果文件夹不存在
			{
				mkdir(sub_folder.c_str());
			}
		}
	}

	std::string  getFileName(std::string file_path) {
		int pos = file_path.find_last_of("\\");
		string res = file_path.substr(pos + 1, file_path.length());
		return res;
	}
	std::string  getFilePath(std::string file_path) {
		int pos = file_path.find_last_of("\\");
		string res = file_path.substr(0, pos + 1);
		return res;
	}
	int fileExist(char *file) {
		std::fstream File;
		File.open(file, std::ios::in);
		if (!File)
		{
			return -1;
		}
		return 0;
	}
}
