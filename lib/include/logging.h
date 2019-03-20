#pragma once

#include <iostream>

/// INFO stream
#define LOG_INFO std::cout << "[INFO] "

/// ERROR stream
#define LOG_ERROR std::cerr << "[ERROR] "

/// End of line, without stream flushing
#define EOL "\n"

/// End of line, with stream flushing
#define EOL_FLUSH std::endl
