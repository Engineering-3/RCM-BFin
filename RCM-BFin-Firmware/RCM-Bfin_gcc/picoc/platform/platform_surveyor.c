#include "../interpreter.h"
#include "../picoc.h"

/* mark where to end the program for platforms which require this */
int PicocExitBuf[41];

void PlatformInit()
{
}

/* deallocate any storage */
void PlatformCleanup()
{
}

/* get a line of interactive input */
char *PlatformGetLine(char *Buf, int MaxLen, const char *Prompt)
{
    unsigned int ix;
    char ch;
    
    printf(Prompt);
    fflush(NULL);

    ix = 0;
    
    // If the first character is \n or \r, eat it
    ch = getch();
    if (ch == '\n' || ch == '\r')
    {
        // And get the next character
        ch = getch();
    }
    
    while (ix < MaxLen) 
    {
        // ESC character or ctrl-c (to avoid problem with TeraTerm) - exit
        if (ch == 0x1B)
        { 
            printf("##Leaving PicoC\r\n");
            return NULL;
        }
        else if (ch == 0x03) 
        { 
            printf("##Leaving PicoC\r\n");
            return NULL;
        }
        // Backspace character has to be handled special
        else if (ch == 0x08)
        {
            // Remove the latest character from our buffer
            if (ix > 0)
            {
                // Send a space and then backspace again
                putchar(' ');
                putchar(0x08);
                ix--;
                Buf[ix] = 0x00;
            }
        }
        else if (ch == '\r')
        {
            Buf[ix] = ch;  // if CR, send newline character followed by null
            ix++;
            Buf[ix] = 0;
            
            return Buf;
        }
        else
        {
            // Handle the normal storage of the byte
            Buf[ix] = ch;
            ix++;
        }
        ch = getch();
    }
    return NULL;
}

/* write a character to the console */
void PlatformPutc(unsigned char OutCh, union OutputStreamInfo *Stream)
{
	/// TODO: I don't think we want this, because we want to allow arbitrary data
	/// to be printed out to the PC by printf(), right? So don't add \r!
	//if (OutCh == '\n')
    //    putchar('\r');
    
	// If the user has the printf() buffered stream feature turned on, then store
	// this byte in that buffer rather than sending directly to the PC.
	if (PicoCStreamBufferEnabled)
	{
		if (PicoCStreamBufferLength < 0xFFF0)
		{
			PicoCStreamBuffer[PicoCStreamBufferIn] = OutCh;
			PicoCStreamBufferIn++;
			if (PicoCStreamBufferIn > 0xFFFF)
			{
				PicoCStreamBufferIn = 0;
			}
			PicoCStreamBufferLength++;
		}
		// If we're full, then just drop the byte
	}
	else
	{
		putchar(OutCh);
	}
}

/* read a character */
int PlatformGetCharacter()
{
    return getch();
}

/* exit the program */
void PlatformExit(int RetVal)
{
    PicocExitValue = RetVal;
    PicocExitBuf[40] = 1;
    longjmp(PicocExitBuf, 1);
}

