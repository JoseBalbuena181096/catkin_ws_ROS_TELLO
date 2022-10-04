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
/**
 * @file ARSAL_Ftw.h
 * @brief libARSAL Ftw header file.
 * @date 19/12/2013
 * @author david.flattin.ext@parrot.com
 **/

#ifndef _ARSAL_FTW_PRIVATE_H_
#define _ARSAL_FTW_PRIVATE_H_

#ifdef HAVE_FTW_H
//The ftw.h will provide all defined values
#else
/**
 * Custom incomplete ftw/nftw implementation for system without support
 * (Android as of ndk r7, r8, r9)
 */

/**
 * Internal result values get from ARSAL_NftwCallback return analyse 
 */
typedef enum 
{
    ARSAL_FTW_FAIL = 0,
    ARSAL_FTW_OK,
} eARSAL_FTW_RESULT;

/**
 * @brief ftw-like function legacy support implementation, Recursively descends the directory hierarchy
 * @param dirpath The directory to descend
 * @param cb The callback recursively on each element of run through the directories
 * @param nopenfd The maximum number of directories depth
 * @retval On success, returns 0. Otherwise, it returns -1, or callack user value
 * @see ftw standard documentation
 */
int ARSAL_Ftw_internal(const char *dirPath, ARSAL_FtwCallback cb, int nopenfd);

 /**
 * @brief nftw-like function legacy support implementation, recursively descends the directory hierarchy
 * @param dirpath The directory to descend
 * @param cb The callback recursively on each element of run through the directories
 * @param nopenfd The maximum number of directories depth
 * @param flags The flag of the type of tree explore
 * @param currentLevel The current depth level, should be 0
 * @param currentBase The base depth level, should be 0
 * @retval On success, returns 0. Otherwise, it returns -1, or callack user value
 * @see nftw standard documentation
 */
int ARSAL_Nftw_internal(const char *dirPath, ARSAL_NftwCallback cb, int nopenfd, eARSAL_FTW_FLAG flags, int currentLevel, int currentBase);

#endif /* HAVE_FTW_H */
#endif /* _ARSAL_FTW_PRIVATE_H_ */

