// A hack so that I can access private members
#define private public
#define protected public

// Bring in my package's API, which is what I'm testing
#include "../src/GraphSearch.h"

// Bring in gtest
#include <gtest/gtest.h>

