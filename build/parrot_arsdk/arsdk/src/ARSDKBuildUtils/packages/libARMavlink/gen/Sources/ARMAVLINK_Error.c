/*
    Copyright (C) 2014 Parrot SA

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in
      the documentation and/or other materials provided with the 
      distribution.
    * Neither the name of Parrot nor the names
      of its contributors may be used to endorse or promote products
      derived from this software without specific prior written
      permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
    OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED 
    AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
    OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
    OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
    SUCH DAMAGE.
*/
/*
 * GENERATED FILE
 *  Do not modify this file, it will be erased during the next configure run 
 */

/**
 * @file ARMAVLINK_Error.c
 * @brief ToString function for eARMAVLINK_ERROR enum
 */

#include <libARMavlink/ARMAVLINK_Error.h>

const char* ARMAVLINK_Error_ToString (eARMAVLINK_ERROR error)
{
    switch (error)
    {
    case ARMAVLINK_OK:
        return "No error";
        break;
    case ARMAVLINK_ERROR:
        return "Unknown generic error";
        break;
    case ARMAVLINK_ERROR_ALLOC:
        return "Memory allocation error";
        break;
    case ARMAVLINK_ERROR_BAD_PARAMETER:
        return "Bad parameters";
        break;
    case ARMAVLINK_ERROR_MANAGER:
        return "Unknown ARMAVLINK_Manager error";
        break;
    case ARMAVLINK_ERROR_FILE_GENERATOR:
        return "Unknown ARMAVLINK_FileGenerator error";
        break;
    case ARMAVLINK_ERROR_LIST_UTILS:
        return "Unknown ARMAVLINK_ListUtils error";
        break;
    case ARMAVLINK_ERROR_MISSION_ITEM_UTILS:
        return "Unknown ARMAVLINK_MissionItemUtils error";
        break;
    case ARMAVLINK_ERROR_MISSION_ITEM_UTILS_NOT_LINKED_COMMAND:
        return "Command not linked with Mavlink commands";
        break;
    case ARMAVLINK_ERROR_FILE_PARSER:
        return "Unknown ARMAVLINK_FileParser error";
        break;
    case ARMAVLINK_ERROR_FILE_PARSER_FILE_NOT_FOUND:
        return "File to parse not found";
        break;
    case ARMAVLINK_ERROR_FILE_PARSER_WORD_NOT_EXPTECTED:
        return "A word was not expected during parsing";
        break;
    default:
        break;
    }
    return "Unknown value";
}
