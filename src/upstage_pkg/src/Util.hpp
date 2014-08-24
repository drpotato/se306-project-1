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
}

#endif // #ifndef SE306P1_UPSTAGE_UTIL_HPP_DEFINED