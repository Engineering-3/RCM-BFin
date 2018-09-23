/* picoc main program - this varies depending on your operating system and
 * how you're using picoc */
 
/* include only picoc.h here - should be able to use it with only the external interfaces, no internals from interpreter.h */
#include "picoc.h"

/* platform-dependent code for running programs is in this file */

#if defined(UNIX_HOST) || defined(WIN32)
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#define PICOC_STACK_SIZE (128*1024)              /* space for the the stack */

int main(int argc, char **argv)
{
    int ParamCount = 1;
    int DontRunMain = FALSE;
    int StackSize = getenv("STACKSIZE") ? atoi(getenv("STACKSIZE")) : PICOC_STACK_SIZE;
    
    if (argc < 2)
    {
        printf("Format: picoc <csource1.c>... [- <arg1>...]    : run a program (calls main() to start it)\n"
               "        picoc -s <csource1.c>... [- <arg1>...] : script mode - runs the program without calling main()\n"
               "        picoc -i                               : interactive mode\n");
        exit(1);
    }
    
    PicocInitialise(StackSize);
    
    if (strcmp(argv[ParamCount], "-s") == 0 || strcmp(argv[ParamCount], "-m") == 0)
    {
        DontRunMain = TRUE;
        PicocIncludeAllSystemHeaders();
        ParamCount++;
    }
        
    if (argc > ParamCount && strcmp(argv[ParamCount], "-i") == 0)
    {
        PicocIncludeAllSystemHeaders();
        PicocParseInteractive(TRUE);
    }
    else
    {
        if (PicocPlatformSetExitPoint())
        {
            PicocCleanup();
            return PicocExitValue;
        }
        
        for (; ParamCount < argc && strcmp(argv[ParamCount], "-") != 0; ParamCount++)
            PicocPlatformScanFile(argv[ParamCount]);
        
        if (!DontRunMain)
            PicocCallMain(argc - ParamCount, &argv[ParamCount]);
    }
    
    PicocCleanup();
    return PicocExitValue;
}
#else
    #ifdef SURVEYOR_HOST
    #define HEAP_SIZE C_HEAPSIZE
    #include <setjmp.h>
    #include "../rcm-bfin.h"
    #include <stdlib.h>
    #include <stdio.h>
    #include <string.h>

// Global to hold TRUE/FALSE value - if we should allow running RCM-Bfin firmware commands while PicoC is running
unsigned char gAllowBackgroundMode;

int picoc(char *SourceStr, unsigned char AllowBackgroundMode)
{   
    char *pos;
    gAllowBackgroundMode = AllowBackgroundMode;

    // For some reason we turn all 0x1A's into 0x20's  (space) before running. Not sure how 0x1As can get . . .
    if (SourceStr)
    {
        for (pos = SourceStr; *pos != 0; pos++)
        {
            if (*pos == 0x1a)
            {
                *pos = 0x20;
            }
        }
    }
    
    // Fix for RCM-Bfin issue #4 :
    // If user tries to execute PicoC program, and first character in SourceStr is not printable ASCII, then
    // there is no PicoC program to run, and we will just exit.
    if (SourceStr)
    {
      if (*SourceStr < 0x20 || *SourceStr > 0x7E)
      {
        printf("Error: Attempted to run PicoC with no PicoC program loaded.\r\n");
        printf("Leaving PicoC\r\n");
        PicocCleanup();
        return PicocExitValue;
      }
    }

    PicocInitialise(HEAP_SIZE);

    PicocExitBuf[40] = 0;
    PicocPlatformSetExitPoint();
    if (PicocExitBuf[40]) {
        printf("Leaving PicoC\r\n");
        PicocCleanup();
        return PicocExitValue;
    }

    if (SourceStr)
        PicocParse("nofile", SourceStr, strlen(SourceStr), TRUE, TRUE, FALSE, TRUE);

    PicocParseInteractive();
    PicocCleanup();
    
    return PicocExitValue;
}
#endif
#endif
