#include "headers/config.h"

#ifdef LOGGING
    #include "stdio.h"

    #define LOG(msg) do{ \
        printf("Function: %s. Line: %d. Msg: %s", __func__, __LINE__, msg); \
        /*fflush(stdout);*/ \
        }while(0)

    #define PRINT(msg) do{ \
        printf("%s", msg); \
        /*fflush(stdout);*/ \
        }while(0)

    #define PRINTNUM(string, num) do{ \
        printf(string, num); \
        /*fflush(stdout);*/ \
        }while(0)
#else
    #define LOG(msg) ((void)0)
    #define PRINT(msg) ((void)0)
    #define PRINTNUM(string, num) ((void)0)
#endif
