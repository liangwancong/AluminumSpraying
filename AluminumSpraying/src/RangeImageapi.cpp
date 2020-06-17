#include "RangeImage.h"
#include "windows.h"
#include "parameters.h"
//#include <iostream>

using namespace AluminumParameters;
int WINAPI DllMain(
	HINSTANCE hinstDLL,  // handle to DLL module
	DWORD fdwReason,     // reason for calling function
	LPVOID lpReserved)  // reserved
{
	// Perform actions based on the reason for calling.
	switch (fdwReason)
	{
	case DLL_PROCESS_ATTACH:
		// Initialize once for each new process.
		// Return FALSE to fail DLL load.
		break;

	case DLL_THREAD_ATTACH:
		// Do thread-specific initialization.
		break;

	case DLL_THREAD_DETACH:
		// Do thread-specific cleanup.
		break;

	case DLL_PROCESS_DETACH:
		// Perform any necessary cleanup.
		break;
	}
	return TRUE;  // Successful DLL_PROCESS_ATTACH.
}

/*
char* output_result_json:输出json
char * cloud_front_file:前一个点云文件
char* cloud_file:当前点云文件
char* match_name:强制匹配模板, 
double cloud_length:点云总长度 
double stepDistanceThresh:滤波step,根据杆子间距设置
double poleWidthThresh:杆子宽度
int poleSizeThresh :杆子点云数量
double poleOffesetThresh:杆子滤波偏移量(poleOffesetThresh-杆子宽度+poleOffesetThresh)
bool need_only_use_mid:使用最适工件做模板
double minAngleThresh:直线最小角度
double minIntervalThresh :直线最小间隔
double minDistanceThresh:最小直线距离
double planeWidthThresh:平面左右偏移量
double novangle:直线角度为多大,视为没有平面
double minPointDistanceThresh :最小点之间距离
double leftNoRPThresh:切割点云偏移量(leftNoRPThresh-点云中间点+leftNoRPThresh),供点云拍平面使用
bool skeletonImage:是否需要骨架化
*/

extern "C" _declspec(dllexport) int __cdecl AluminumPointCloudResolve_api(char* output_result_json, char * cloud_front_file, char* cloud_file, char* match_name = "", double cloud_length = CloudLength, double stepDistanceThresh = StepDistanceThresh, double poleWidthThresh = PoleWidthThresh, int poleSizeThresh = PoleSizeThresh, double poleOffesetThresh = PoleOffesetThresh, bool need_only_use_mid = NeedOnlyUseMid, double minAngleThresh = MinAngleThresh, double minIntervalThresh = MinIntervalThresh, double minDistanceThresh = MinDistanceThresh, double planeWidthThresh = PlaneWidthThresh, double novangle = Novangle, double minPointDistanceThresh = MinPointDistanceThresh, double leftNoRPThresh = LeftNoRPThresh, bool skeletonImage = SkeletonImage, int cloud_pos = CloudPos, float head_offset = HeadOffset, float mid_offset = MidOffset, float tail_offset = TailOffset) {
	return AluminumPointCloudResolve(output_result_json,  cloud_front_file,  cloud_file,  match_name,  cloud_length,  stepDistanceThresh ,  poleWidthThresh,  poleSizeThresh,  poleOffesetThresh,  need_only_use_mid ,  minAngleThresh,  minIntervalThresh,  minDistanceThresh,  planeWidthThresh ,  novangle,  minPointDistanceThresh,  leftNoRPThresh ,  skeletonImage , cloud_pos, head_offset, mid_offset, tail_offset);
}

extern "C" _declspec(dllexport) int __cdecl SaveDistanceAngleMatchModelWithImageFilePath_api(const char* file_path,const char* out_path="", double minAngleThresh = MinAngleThresh, double minIntervalThresh = MinIntervalThresh, double minDistanceThresh = MinDistanceThresh, double planeWidthThresh = PlaneWidthThresh, double novangle = Novangle, double minPointDistanceThresh = MinPointDistanceThresh, double leftNoRPThresh = LeftNoRPThresh, bool skeletonImage = SkeletonImage, int cloud_pos = CloudPos, float head_offset = HeadOffset, float mid_offset = MidOffset, float tail_offset = TailOffset) {
	return SaveDistanceAngleMatchModelWithImageFilePath(file_path, out_path, minAngleThresh, minIntervalThresh, minDistanceThresh, planeWidthThresh, novangle, minPointDistanceThresh, leftNoRPThresh, skeletonImage, cloud_pos, head_offset, mid_offset, tail_offset);
}

extern "C" _declspec(dllexport) int __cdecl AluminumImageResolve_api(char* output_result_json, char* image_file, double minAngleThresh = MinAngleThresh, double minIntervalThresh = MinIntervalThresh, double minDistanceThresh = MinDistanceThresh, double planeWidthThresh = PlaneWidthThresh, double novangle = Novangle, double minPointDistanceThresh = MinPointDistanceThresh, double leftNoRPThresh = LeftNoRPThresh, bool skeletonImage = SkeletonImage, int cloud_pos = CloudPos, float head_offset = HeadOffset, float mid_offset = MidOffset, float tail_offset = TailOffset) {
	return AluminumImageResolve(output_result_json, image_file,  minAngleThresh, minIntervalThresh, minDistanceThresh, planeWidthThresh, novangle, minPointDistanceThresh, leftNoRPThresh, skeletonImage, cloud_pos, head_offset, mid_offset, tail_offset);
}
extern "C" _declspec(dllexport) int __cdecl SaveModelapi(char* cloud_file, char* out_name = "", double cloud_length = CloudLength, double stepDistanceThresh = StepDistanceThresh, double poleWidthThresh = PoleWidthThresh, int poleSizeThresh = PoleSizeThresh, double poleOffesetThresh = PoleOffesetThresh) {
	return SaveModel(cloud_file, out_name, cloud_length, stepDistanceThresh, poleWidthThresh, poleSizeThresh, poleOffesetThresh);
}