#include "AluminumSpraying.h"
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


extern "C" _declspec(dllexport) int __cdecl GetMatchMsg_api(char* output_result_json, char * cloud_front_file, char * cloud_file, char* model_name, double cloud_length = CloudLength, int cloud_pos = CloudPos, float head_offset = HeadOffset, float mid_offset = MidOffset, float tail_offset = TailOffset) {
	return getMatchMsg(output_result_json, cloud_front_file, cloud_file, model_name, cloud_length, cloud_pos, head_offset, mid_offset, tail_offset);
}
