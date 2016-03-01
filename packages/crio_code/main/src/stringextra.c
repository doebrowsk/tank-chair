#include "stringextra.h"
#include <ctype.h>
#include <stdio.h>
#include <string.h>
#include "harlielog.h"

// @TODO check if the chars should be unsigned chars
char StringExtraReturnStr[RETURN_STRING_LENGTH];

// Converts the string to only use lowercase
char * strToLower(char * str)
{
    int i = 0;
    int strLength = strlen(str);
    if(strLength > RETURN_STRING_LENGTH)
    {
        LOG.ERR("The string length sent to strToLower was larger than : %i",RETURN_STRING_LENGTH);
    }

    for(i = 0;i < strLength; i++)
    {
        StringExtraReturnStr[i] = tolower(str[i]);
    }

    StringExtraReturnStr[i+1] = '\0';
    return & StringExtraReturnStr[0];
}
