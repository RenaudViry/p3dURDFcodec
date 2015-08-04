#ifndef LOGGER_HPP
#define LOGGER_HPP

#include <iostream>
#include <string>
#include <vector>

static int LOGGER_LEVEL = 4; // DEBUG

typedef enum {
	FATAL_LVL = 0,
	ERROR_LVL, //1
	WARNING_LVL, //2
	INFO_LVL, //3
	DEBUG_LVL, //4
	NB_LVL //5
} LOGGER_LEVELS;
const std::vector<std::string> LOGGER_LEVELS_NAME = {"FATAL", "ERROR", "WARNING", "INFO", "DEBUG"};

inline void logger (int level, std::string message)
{
	if (level <= LOGGER_LEVEL)
		std::cout << LOGGER_LEVELS_NAME[level] << ": " << message << std::endl;
}

#endif /* LOGGER_HPP */