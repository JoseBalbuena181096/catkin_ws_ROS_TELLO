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
 * @file ARNETWORK_IOBufferParam.c
 * @brief prameters used to set the parameters of a new IOBuffer
 * @date 28/09/2012
 * @author maxime.maitre@parrot.com
 **/

/*****************************************
 *
 *             include file :
 *
 ******************************************/

#include <stdlib.h>

#include <libARSAL/ARSAL_Print.h>

#include <libARNetwork/ARNETWORK_Error.h>
#include <libARNetwork/ARNETWORK_IOBufferParam.h>

/*****************************************
 *
 *             define :
 *
 ******************************************/

#define ARNETWORK_IOBUFFER_PARAM_TAG "ARNETWORK_IOBufferParam"

#define ARNETWORK_IOBUFFER_ID_DEFAULT -1
#define ARNETWORK_IOBUFFER_ID_MIN 10
#define ARNETWORK_IOBUFFER_ID_MAX 127
#define ARNETWORK_IOBUFFER_DATA_TYPE_DEFAULT ARNETWORKAL_FRAME_TYPE_UNINITIALIZED
#define ARNETWORK_IOBUFFER_SENDING_WAIT_TIME_DEFAULT 1
#define ARNETWORK_IOBUFFER_ACK_TIMEOUT_MS_DEFAULT ARNETWORK_IOBUFFERPARAM_INFINITE_NUMBER
#define ARNETWORK_IOBUFFER_NUMBER_OF_RETRY_DEFAULT ARNETWORK_IOBUFFERPARAM_INFINITE_NUMBER
#define ARNETWORK_IOBUFFER_NUMBER_OF_CELL_DEFAULT 0
#define ARNETWORK_IOBUFFER_MAX_SIZE_OF_DATA_COPY_DEFAULT 0
#define ARNETWORK_IOBUFFER_OVERWRITING_DEFAULT 0

/*****************************************
 *
 *             implementation :
 *
 ******************************************/

eARNETWORK_ERROR ARNETWORK_IOBufferParam_DefaultInit (ARNETWORK_IOBufferParam_t *IOBufferParam)
{
    /** -- initialization of the IOBufferParam with default parameters -- */

    /** local declarations */
    eARNETWORK_ERROR error = ARNETWORK_OK;

    if (IOBufferParam != NULL)
    {
        IOBufferParam->ID = ARNETWORK_IOBUFFER_ID_DEFAULT;
        IOBufferParam->dataType = ARNETWORK_IOBUFFER_DATA_TYPE_DEFAULT;
        IOBufferParam->sendingWaitTimeMs = ARNETWORK_IOBUFFER_SENDING_WAIT_TIME_DEFAULT;
        IOBufferParam->ackTimeoutMs = ARNETWORK_IOBUFFER_ACK_TIMEOUT_MS_DEFAULT;
        IOBufferParam->numberOfRetry = ARNETWORK_IOBUFFER_NUMBER_OF_RETRY_DEFAULT;

        IOBufferParam->numberOfCell = ARNETWORK_IOBUFFER_NUMBER_OF_CELL_DEFAULT;
        IOBufferParam->dataCopyMaxSize = ARNETWORK_IOBUFFER_MAX_SIZE_OF_DATA_COPY_DEFAULT;
        IOBufferParam->isOverwriting = ARNETWORK_IOBUFFER_OVERWRITING_DEFAULT;
    }
    else
    {
        error = ARNETWORK_ERROR_BAD_PARAMETER;
    }

    return error;
}

int ARNETWORK_IOBufferParam_Check (const ARNETWORK_IOBufferParam_t *IOBufferParam)
{
    /** -- check the values of the IOBufferParam -- */

    /** local declarations */
    int ok = 0;

    /** check the parameters values */
    if ((IOBufferParam != NULL) &&
        (IOBufferParam->ID >= ARNETWORK_IOBUFFER_ID_MIN) &&
        (IOBufferParam->ID <= ARNETWORK_IOBUFFER_ID_MAX) &&
        (IOBufferParam->dataType != ARNETWORKAL_FRAME_TYPE_UNINITIALIZED) &&
        (IOBufferParam->sendingWaitTimeMs >= 0) &&
        (IOBufferParam->ackTimeoutMs >= -1) &&
        (IOBufferParam->numberOfRetry >= -1))
    {
        ok = 1;
    }
    else
    {
        if (IOBufferParam != NULL)
        {
            ARSAL_PRINT (ARSAL_PRINT_ERROR, ARNETWORK_IOBUFFER_PARAM_TAG, "Parameters for new IOBuffer are not correct. \n\
values expected: \n\
    - %d <= ID <= %d (value set: %d)\n\
    - dataType != %d (value set: %d)\n\
    - sendingWaitTimeMs >= 0 (value set: %d)\n\
    - ackTimeoutMs > 0 or -1 if not used (value set: %d)\n\
    - numberOfRetry > 0 or -1 if not used  (value set: %d)\n\
    - numberOfCell > 0 (value set: %d)\n\
    - dataCopyMaxSize >= 0 (value set: %d)\n\
    - isOverwriting = 0 or 1 (value set: %d)",
                     ARNETWORK_IOBUFFER_ID_MIN, ARNETWORK_IOBUFFER_ID_MAX, IOBufferParam->ID,
                     ARNETWORKAL_FRAME_TYPE_UNINITIALIZED, IOBufferParam->dataType,
                     IOBufferParam->sendingWaitTimeMs,
                     IOBufferParam->ackTimeoutMs,
                     IOBufferParam->numberOfRetry,
                     IOBufferParam->numberOfCell,
                     IOBufferParam->dataCopyMaxSize,
                     IOBufferParam->isOverwriting);
        }
        else
        {
            ARSAL_PRINT (ARSAL_PRINT_ERROR, ARNETWORK_IOBUFFER_PARAM_TAG, "Parameters for new IOBuffer are NULL");
        }
    }

    return ok;
}
