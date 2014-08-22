#ifndef SE306P1_UPSTAGE_DEBUG_HPP_DEFINED
#define SE306P1_UPSTAGE_DEBUG_HPP_DEFINED

#include <cstdlib>
#include <cstdio>

#define ANSI_ESC_FAIL "\x1b[38;5;255;48;5;9m"
#define ANSI_ESC_RSET "\x1b[39;49m"

#define UPS_ASSERT(exp,msg) if (!(exp)) \
{\
	std::fprintf(stderr, "\n\n" ANSI_ESC_FAIL "###ASSERTION FAILED###\nIn: %s:%d\nAssert( %s )\n###\n%s\n###" ANSI_ESC_RSET "\n", \
	__FILE__, __LINE__, #exp, msg);\
	exit(1);\
}

#define UPS_ASSERTF(exp,msg, ...) if (!(exp)) \
{\
	std::fprintf(stderr, "\n\n" ANSI_ESC_FAIL "###ASSERTION FAILED###\nIn: %s:%d\nAssert( %s )\n###\n" msg "\n###" ANSI_ESC_RSET "\n", \
	__FILE__, __LINE__, #exp, __VA_ARGS__);\
	exit(1);\
}

#define UPS_LOG(msg) std::printf("LOG | %s:%d | %s\n", __FILE__, __LINE__, msg);

#define UPS_LOGF(msg, ...) std::printf("LOG | %s:%d | " msg "\n", __FILE__, __LINE__, __VA_ARGS__);

#endif // #ifndef SE306P1_UPSTAGE_DEBUG_HPP_DEFINED