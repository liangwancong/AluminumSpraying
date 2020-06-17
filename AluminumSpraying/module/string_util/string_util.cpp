#include "string_util.h"

#ifdef _WIN32
#include<Windows.h>
#endif

namespace stringutil {
#ifdef _WIN32
	std::string TCHAR2STRING(TCHAR* str)
	{
		std::string strstr;
		try
		{
			int iLen = WideCharToMultiByte(CP_ACP, 0, (LPCWCH)str, -1, NULL, 0, NULL, NULL);

			char* chRtn = new char[iLen * sizeof(char)];

			WideCharToMultiByte(CP_ACP, 0, (LPCWCH)str, -1, chRtn, iLen, NULL, NULL);

			strstr = chRtn;
		}
		catch (std::exception e)
		{
		}

		return strstr;
	}
#endif
}