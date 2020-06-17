#include "windows.h"
#include "CRMath.h"

using namespace CRMath;
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
xyzrxryrz1:��һ�����λ����̬
xyzrxryrz2:�ڶ������λ����̬
angle_spling:˦ǹ�Ƕ�
rxryrz_out:��̬
����-1:ִ�д���
����0:ִ����ȷ
*/
extern "C" _declspec(dllexport) int __cdecl getSplingRxRyRzapi(double* xyzrxryrz1, double*xyzrxryrz2, double angle_spling, double &Rx, double &Ry, double &Rz) {
	return getSplingRxRyRz(xyzrxryrz1, xyzrxryrz2, angle_spling, Rx, Ry, Rz);
}


