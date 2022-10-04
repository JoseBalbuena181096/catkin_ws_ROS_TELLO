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
 * @file ARSAL_JNI.h
 * @brief libARSAL JNI header file.
 * @date 03/06/2014
 * @author david.flattin.ext@parrot.com
 **/

#ifndef _ARSAL_JNI_H_
#define _ARSAL_JNI_H_

#ifndef JNI_OK
#define JNI_OK      0
#endif
#ifndef JNI_FAILED
#define JNI_FAILED  -1
#endif

extern JavaVM* ARSAL_JNI_Manager_VM;


/**
 * @brief Throw a new ARSALException
 * @param env The java env
 * @param nativeError The error
 * @retval void
 * @see ARSAL_JNI_Manager_NewARSALException
 */
void ARSAL_JNI_Manager_ThrowARSALException(JNIEnv *env, eARSAL_ERROR nativeError);

/**
 * @brief Create a new ARSALException
 * @param env The java env
 * @param nativeError The error
 * @retval the new ARSALException
 * @see ARSAL_JNI_Manager_ThrowARSALException
 */
jobject ARSAL_JNI_Manager_NewARSALException(JNIEnv *env, eARSAL_ERROR nativeError);

/**
 * @brief Get the ARSALException JNI class
 * @param env The java env
 * @retval JNI_TRUE if Success, else JNI_FALSE
 * @see ARSAL_JNI_Manager_FreeARSALExceptionJNI
 */
int ARSAL_JNI_Manager_NewARSALExceptionJNI(JNIEnv *env);

/**
 * @brief Free the ARSALException JNI class
 * @param env The java env
 * @retval void
 * @see ARSAL_JNI_Manager_NewARSALExceptionJNI
 */
void ARSAL_JNI_Manager_FreeARSALExceptionJNI(JNIEnv *env);

/**
 * @brief Create a new ARSAL_ERROR_ENUM
 * @param env The java env
 * @param nativeError The error
 * @retval the new ARSAL_ERROR_ENUM
 * @see ARSAL_JNI_Manager_NewERROR_ENUM_JNI
 */
jobject ARSAL_JNI_Manager_NewERROR_ENUM(JNIEnv *env, eARSAL_ERROR nativeError);

/**
 * @brief Get the ARSAL_ERROR_ENUM JNI class
 * @param env The java env
 * @retval JNI_TRUE if Success, else JNI_FALSE
 * @see ARSAL_JNI_Manager_FreeERROR_ENUM_JNI
 */
int ARSAL_JNI_Manager_NewERROR_ENUM_JNI(JNIEnv *env);

/**
 * @brief Free the ARSAL_ERROR_ENUM JNI class
 * @param env The java env
 * @retval void
 * @see ARSAL_JNI_Manager_NewERROR_ENUM_JNI
 */
void ARSAL_JNI_Manager_FreeERROR_ENUM_JNI(JNIEnv *env);


#endif /* _ARSAL_JNI_H_ */
