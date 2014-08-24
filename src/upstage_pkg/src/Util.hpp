#ifndef SE306P1_UPSTAGE_UTIL_HPP_DEFINED
#define SE306P1_UPSTAGE_UTIL_HPP_DEFINED

#include "Debug.hpp"
#include <cstdio>
#include <string>
#include <vector>

namespace ups
{
	long getFileSize(std::FILE *f);
	std::vector<char> readAll(std::FILE *f);
	bool canOpen(const std::string &fname, const char *openMode);
	bool canRead(const std::string &fname);
	bool canWrite(const std::string &fname);
	std::string getFileExtension(const std::string &fileName);
	
	bool isWhitespace(char testChar);
	bool isDigit(char testChar);
	
	// Converts the string str to the given number type,
	// setting strPtr to point to the first position after the number, if it's not null.
	template<typename t>
	t s2num(const char *str, const char **strPtr);
	
	// Different version of the above, returning a bool indicating success
	template<typename t>
	bool s2num(t &output, const char *str, const char **strPtr);
	
	// Similar to above, but doesn't return the new position upon success
	template<typename t>
	bool s2num(t &output, const char *str);
	
	
	inline long getFileSize(std::FILE *f)
	{
		// Remember where we are
		long currentPos = ftell(f);
		
		// Go to the end, and get the position of the end
		std::fseek(f, 0, SEEK_END);
		long fileSize = std::ftell(f);
		
		// Restore the original position and return
		std::fseek(f, currentPos, SEEK_SET);
		return fileSize;
	}
	
	inline std::vector<char> readAll(std::FILE *f)
	{
		long fileSize = getFileSize(f);
		char *buffer = new char[fileSize];
		if (std::fread(buffer, 1, fileSize, f) != static_cast<std::size_t>(fileSize))
		{
			// Something funny went on
			UPS_LOG("May not have correctly loaded a file. Sorry, I can't be more specific.");
		}
		
		std::vector<char> returnedValue(buffer, buffer + fileSize);
		
		// Don't create a memory leak
		delete[] buffer;
		
		return returnedValue;
	}
	
	inline bool canOpen(const std::string &fname, const char *openMode)
	{
		FILE *f = fopen(fname.c_str(), openMode);
		
		if (f)
		{	fclose(f);
			return true;
		}
		return false;
	}
	
	inline bool canRead(const std::string &fname)
	{
		return canOpen(fname, "rb");
	}
	
	inline bool canWrite(const std::string &fname)
	{
		return canOpen(fname, "wb");
	}
	
	inline std::string getFileExtension(const std::string &fileName)
	{
		std::size_t dotPos = fileName.rfind(".");
		
		if (dotPos == std::string::npos || dotPos + 1 >= fileName.length())
		{
			return std::string();
		}
		
		return fileName.substr(dotPos + 1);
	}
	
	inline bool isWhitespace(char testChar)
	{
		return (testChar == ' ' || testChar == '\t' || testChar == '\n' || testChar == '\r');
	}
	
	inline bool isDigit(char testChar)
	{
		return testChar >= '0' && testChar <= '9';
	}
	
	template<>
	inline float s2num<float>(const char *str, const char **strPtr)
	{
		char c;
		float returnValue = 0.f;
		
		// Skip over whitespace
		while (isWhitespace(c = *str))
		{
			++str;
		}
		
		// Read in values before the decimal point (if any)
		while (isDigit(c = *str))
		{
			returnValue = returnValue * 10.f + (c - '0');
			++str;
		}
		
		// Escape if we're not at the decimal point, otherwise, step over the decimal point
		if (c != '.') return 0.f;
		++str;
		
		// Read in values after the decimal point
		for (float value = 1.f; isDigit(c = *str); ++str)
		{
			returnValue += (c - '0') * (value *= 0.1f);
		}
		
		// Update the position, and return.
		if (strPtr)
		{
			*strPtr = str;
		}
		
		return returnValue;
	}
	
	template<>
	inline double s2num<double>(const char *str, const char **strPtr)
	{
		char c;
		double returnValue = 0.0;
		
		// Skip over whitespace
		while (isWhitespace(c = *str))
		{
			++str;
		}
		
		// Read in values before the decimal point (if any)
		while (isDigit(c = *str))
		{
			returnValue = returnValue * 10.0 + (c - '0');
			++str;
		}
		
		// Escape if we're not at the decimal point, otherwise, step over the decimal point
		if (c != '.') return 0.0;
		++str;
		
		// Read in values after the decimal point
		for (float value = 1.0; isDigit(c = *str); ++str)
		{
			returnValue += (c - '0') * (value *= 0.1);
		}
		
		// Update the position, and return.
		if (strPtr)
		{
			*strPtr = str;
		}
		
		return returnValue;
	}
	
	template<typename t>
	inline bool s2num(t &output, const char *str, const char **strPtr)
	{
		output = s2num<t>(str, strPtr);
		return (str != *strPtr);
	}
	
	template<typename t>
	inline bool s2num(t &output, const char *str)
	{
		const char *strCpy = str;
		return s2num(output, str, &strCpy);
	}
}

#endif // #ifndef SE306P1_UPSTAGE_UTIL_HPP_DEFINED