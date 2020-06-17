#pragma once
#include <iostream>
#include <vector>
#include <tchar.h>

namespace stringutil {
	template <typename ...Args>

	inline std::string format_string(const char* format, Args... args) {
		constexpr size_t oldlen = BUFSIZ;
		char buffer[oldlen];  // 默认栈上的缓冲区

		size_t newlen = snprintf(&buffer[0], oldlen, format, args...);
		newlen++;  // 算上终止符'\0'

		if (newlen > oldlen) {  // 默认缓冲区不够大，从堆上分配
			std::vector<char> newbuffer(newlen);
			snprintf(newbuffer.data(), newlen, format, args...);
			return std::string(newbuffer.data());
		}

		return buffer;
	}

#ifdef _WIN32
	/******************************************************************************************
	Function:        TCHAR2STRING
	Description:     TCHAR转string
	Input:           str:待转化的TCHAR*类型字符串
	Return:          转化后的string类型字符串
	*******************************************************************************************/
	std::string TCHAR2STRING(TCHAR* str);
#endif
}