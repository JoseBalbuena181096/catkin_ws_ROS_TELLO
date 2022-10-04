/**
 * @file arstream2_h264_parser.c
 * @brief Parrot Streaming Library - H.264 Parser
 * @date 08/04/2015
 * @author aurelien.barre@parrot.com
 */

#include <stdio.h>
#include <string.h>
#include <arpa/inet.h>

#include <libARStream2/arstream2_h264_sei.h>
#include <libARSAL/ARSAL_Print.h>


#define ARSTREAM2_H264_SEI_TAG "ARSTREAM2_H264Sei"


static eARSTREAM2_ERROR ARSTREAM2_H264Sei_SerializeParrotStreamingV1(const ARSTREAM2_H264Sei_ParrotStreamingV1_t *streaming, const uint16_t *sliceMbCount, void* pBuf, unsigned int bufSize, unsigned int *size)
{
    uint8_t* pbBuf = (uint8_t*)pBuf;
    uint16_t* pwBuf;
    int i;
    unsigned int _size;

    if (!pBuf)
    {
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }

    _size = sizeof(ARSTREAM2_H264Sei_ParrotStreamingV1_t) + streaming->sliceCount * sizeof(uint16_t);
    if (bufSize < _size)
    {
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }

    *(pbBuf++) = streaming->indexInGop;
    *(pbBuf++) = streaming->sliceCount;

    pwBuf = (uint16_t*)pbBuf;
    for (i = 0; i < streaming->sliceCount; i++)
    {
        *(pwBuf++) = htons(sliceMbCount[i]);
    }

    if (size)
    {
        *size = _size;
    }

    return ARSTREAM2_OK;
}


static eARSTREAM2_ERROR ARSTREAM2_H264Sei_DeserializeParrotStreamingV1(const void* pBuf, unsigned int bufSize, ARSTREAM2_H264Sei_ParrotStreamingV1_t *streaming, uint16_t *sliceMbCount)
{
    const uint8_t* pbBuf = (const uint8_t*)pBuf;
    const uint16_t* pwBuf;
    int i;

    if (!pBuf)
    {
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }

    if (bufSize < sizeof(ARSTREAM2_H264Sei_ParrotStreamingV1_t))
    {
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }

    streaming->indexInGop = *(pbBuf++);
    streaming->sliceCount = *(pbBuf++);

    if (streaming->sliceCount > ARSTREAM2_H264_SEI_PARROT_STREAMING_MAX_SLICE_COUNT)
    {
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }
    if (bufSize < sizeof(ARSTREAM2_H264Sei_ParrotStreamingV1_t) + streaming->sliceCount * sizeof(uint16_t))
    {
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }

    pwBuf = (const uint16_t*)pbBuf;
    for (i = 0; i < streaming->sliceCount; i++)
    {
        sliceMbCount[i] = ntohs(*(pwBuf++));
    }

    return ARSTREAM2_OK;
}


eARSTREAM2_ERROR ARSTREAM2_H264Sei_SerializeUserDataParrotStreamingV1(const ARSTREAM2_H264Sei_ParrotStreamingV1_t *streaming, const uint16_t *sliceMbCount, void* pBuf, unsigned int bufSize, unsigned int *size)
{
    eARSTREAM2_ERROR ret = ARSTREAM2_OK;
    uint8_t* pbBuf = (uint8_t*)pBuf;
    uint32_t* pdwBuf = (uint32_t*)pBuf;
    unsigned int _size = 0, outSize = 0;

    if (!pBuf)
    {
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }

    if (bufSize < sizeof(ARSTREAM2_H264Sei_UserDataParrotStreamingV1_t) + streaming->sliceCount * sizeof(uint16_t))
    {
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }

    *(pdwBuf++) = htonl(ARSTREAM2_H264_SEI_PARROT_STREAMING_V1_UUID_0);
    _size += 4;
    *(pdwBuf++) = htonl(ARSTREAM2_H264_SEI_PARROT_STREAMING_V1_UUID_1);
    _size += 4;
    *(pdwBuf++) = htonl(ARSTREAM2_H264_SEI_PARROT_STREAMING_V1_UUID_2);
    _size += 4;
    *(pdwBuf++) = htonl(ARSTREAM2_H264_SEI_PARROT_STREAMING_V1_UUID_3);
    _size += 4;

    pbBuf = (uint8_t*)pdwBuf;
    bufSize -= _size;
    ret = ARSTREAM2_H264Sei_SerializeParrotStreamingV1(streaming, sliceMbCount, (void*)pbBuf, bufSize, &outSize);
    _size += outSize;

    if (size)
    {
        *size = _size;
    }

    return ret;
}


eARSTREAM2_ERROR ARSTREAM2_H264Sei_DeserializeUserDataParrotStreamingV1(const void* pBuf, unsigned int bufSize, ARSTREAM2_H264Sei_ParrotStreamingV1_t *streaming, uint16_t *sliceMbCount)
{
    eARSTREAM2_ERROR ret = ARSTREAM2_OK;
    const uint8_t* pbBuf = (uint8_t*)pBuf;
    const uint32_t* pdwBuf = (uint32_t*)pBuf;
    uint32_t uuid0, uuid1, uuid2, uuid3;

    if (!pBuf)
    {
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }

    if (bufSize < sizeof(ARSTREAM2_H264Sei_UserDataParrotStreamingV1_t))
    {
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }

    uuid0 = ntohl(*(pdwBuf++));
    bufSize -=4;
    uuid1 = ntohl(*(pdwBuf++));
    bufSize -=4;
    uuid2 = ntohl(*(pdwBuf++));
    bufSize -=4;
    uuid3 = ntohl(*(pdwBuf++));
    bufSize -=4;
    if ((uuid0 != ARSTREAM2_H264_SEI_PARROT_STREAMING_V1_UUID_0) || (uuid1 != ARSTREAM2_H264_SEI_PARROT_STREAMING_V1_UUID_1) 
            || (uuid2 != ARSTREAM2_H264_SEI_PARROT_STREAMING_V1_UUID_2) || (uuid3 != ARSTREAM2_H264_SEI_PARROT_STREAMING_V1_UUID_3))
    {
        return ARSTREAM2_ERROR_NOT_FOUND;
    }

    pbBuf = (uint8_t*)pdwBuf;
    ret = ARSTREAM2_H264Sei_DeserializeParrotStreamingV1((const void*)pbBuf, bufSize, streaming, sliceMbCount);

    return ret;
}


int ARSTREAM2_H264Sei_IsUserDataParrotStreamingV1(const void* pBuf, unsigned int bufSize)
{
    uint32_t uuid0, uuid1, uuid2, uuid3;

    if (!pBuf)
    {
        return -1;
    }

    if (bufSize < 16)
    {
        return -1;
    }

    uuid0 = ntohl(*((uint32_t*)pBuf));
    uuid1 = ntohl(*((uint32_t*)pBuf + 1));
    uuid2 = ntohl(*((uint32_t*)pBuf + 2));
    uuid3 = ntohl(*((uint32_t*)pBuf + 3));

    if ((uuid0 == ARSTREAM2_H264_SEI_PARROT_STREAMING_V1_UUID_0) && (uuid1 == ARSTREAM2_H264_SEI_PARROT_STREAMING_V1_UUID_1) 
            && (uuid2 == ARSTREAM2_H264_SEI_PARROT_STREAMING_V1_UUID_2) && (uuid3 == ARSTREAM2_H264_SEI_PARROT_STREAMING_V1_UUID_3))
    {
        return 1;
    }

    return 0;
}


static eARSTREAM2_ERROR ARSTREAM2_H264Sei_SerializeParrotStreamingV2(const ARSTREAM2_H264Sei_ParrotStreamingV2_t *streaming, void* pBuf, unsigned int bufSize, unsigned int *size)
{
    uint16_t* pwBuf = (uint16_t*)pBuf;
    unsigned int _size;

    if (!pBuf)
    {
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }

    _size = sizeof(ARSTREAM2_H264Sei_ParrotStreamingV2_t);
    if (bufSize < _size)
    {
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }

    *(pwBuf++) = htons(streaming->frameSliceCount);
    *(pwBuf++) = htons(streaming->sliceMbCount);

    if (size)
    {
        *size = _size;
    }

    return ARSTREAM2_OK;
}


static eARSTREAM2_ERROR ARSTREAM2_H264Sei_DeserializeParrotStreamingV2(const void* pBuf, unsigned int bufSize, ARSTREAM2_H264Sei_ParrotStreamingV2_t *streaming)
{
    const uint16_t* pwBuf = (const uint16_t*)pBuf;

    if (!pBuf)
    {
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }

    if (bufSize < sizeof(ARSTREAM2_H264Sei_ParrotStreamingV2_t))
    {
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }

    streaming->frameSliceCount = ntohs(*(pwBuf++));
    streaming->sliceMbCount = ntohs(*(pwBuf++));

    return ARSTREAM2_OK;
}


eARSTREAM2_ERROR ARSTREAM2_H264Sei_SerializeUserDataParrotStreamingV2(const ARSTREAM2_H264Sei_ParrotStreamingV2_t *streaming, void* pBuf, unsigned int bufSize, unsigned int *size)
{
    eARSTREAM2_ERROR ret = ARSTREAM2_OK;
    uint8_t* pbBuf = (uint8_t*)pBuf;
    uint32_t* pdwBuf = (uint32_t*)pBuf;
    unsigned int _size = 0, outSize = 0;

    if (!pBuf)
    {
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }

    if (bufSize < sizeof(ARSTREAM2_H264Sei_UserDataParrotStreamingV2_t))
    {
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }

    *(pdwBuf++) = htonl(ARSTREAM2_H264_SEI_PARROT_STREAMING_V2_UUID_0);
    _size += 4;
    *(pdwBuf++) = htonl(ARSTREAM2_H264_SEI_PARROT_STREAMING_V2_UUID_1);
    _size += 4;
    *(pdwBuf++) = htonl(ARSTREAM2_H264_SEI_PARROT_STREAMING_V2_UUID_2);
    _size += 4;
    *(pdwBuf++) = htonl(ARSTREAM2_H264_SEI_PARROT_STREAMING_V2_UUID_3);
    _size += 4;

    pbBuf = (uint8_t*)pdwBuf;
    bufSize -= _size;
    ret = ARSTREAM2_H264Sei_SerializeParrotStreamingV2(streaming, (void*)pbBuf, bufSize, &outSize);
    _size += outSize;

    if (size)
    {
        *size = _size;
    }

    return ret;
}


eARSTREAM2_ERROR ARSTREAM2_H264Sei_DeserializeUserDataParrotStreamingV2(const void* pBuf, unsigned int bufSize, ARSTREAM2_H264Sei_ParrotStreamingV2_t *streaming)
{
    eARSTREAM2_ERROR ret = ARSTREAM2_OK;
    const uint8_t* pbBuf = (uint8_t*)pBuf;
    const uint32_t* pdwBuf = (uint32_t*)pBuf;
    uint32_t uuid0, uuid1, uuid2, uuid3;

    if (!pBuf)
    {
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }

    if (bufSize < sizeof(ARSTREAM2_H264Sei_UserDataParrotStreamingV2_t))
    {
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }

    uuid0 = ntohl(*(pdwBuf++));
    bufSize -=4;
    uuid1 = ntohl(*(pdwBuf++));
    bufSize -=4;
    uuid2 = ntohl(*(pdwBuf++));
    bufSize -=4;
    uuid3 = ntohl(*(pdwBuf++));
    bufSize -=4;
    if ((uuid0 != ARSTREAM2_H264_SEI_PARROT_STREAMING_V2_UUID_0) || (uuid1 != ARSTREAM2_H264_SEI_PARROT_STREAMING_V2_UUID_1)
            || (uuid2 != ARSTREAM2_H264_SEI_PARROT_STREAMING_V2_UUID_2) || (uuid3 != ARSTREAM2_H264_SEI_PARROT_STREAMING_V2_UUID_3))
    {
        return ARSTREAM2_ERROR_NOT_FOUND;
    }

    pbBuf = (uint8_t*)pdwBuf;
    ret = ARSTREAM2_H264Sei_DeserializeParrotStreamingV2((const void*)pbBuf, bufSize, streaming);

    return ret;
}


int ARSTREAM2_H264Sei_IsUserDataParrotStreamingV2(const void* pBuf, unsigned int bufSize)
{
    uint32_t uuid0, uuid1, uuid2, uuid3;

    if (!pBuf)
    {
        return -1;
    }

    if (bufSize < 16)
    {
        return -1;
    }

    uuid0 = ntohl(*((uint32_t*)pBuf));
    uuid1 = ntohl(*((uint32_t*)pBuf + 1));
    uuid2 = ntohl(*((uint32_t*)pBuf + 2));
    uuid3 = ntohl(*((uint32_t*)pBuf + 3));

    if ((uuid0 == ARSTREAM2_H264_SEI_PARROT_STREAMING_V2_UUID_0) && (uuid1 == ARSTREAM2_H264_SEI_PARROT_STREAMING_V2_UUID_1)
            && (uuid2 == ARSTREAM2_H264_SEI_PARROT_STREAMING_V2_UUID_2) && (uuid3 == ARSTREAM2_H264_SEI_PARROT_STREAMING_V2_UUID_3))
    {
        return 1;
    }

    return 0;
}
