/**
 * @file arstream2_h264_parser.c
 * @brief Parrot Streaming Library - H.264 Parser
 * @date 08/04/2015
 * @author aurelien.barre@parrot.com
 */

#ifndef _FILE_OFFSET_BITS
#define _FILE_OFFSET_BITS 64
#endif

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <arpa/inet.h>

#include <libARSAL/ARSAL_Print.h>

#include <libARStream2/arstream2_h264_parser.h>
#include "arstream2_h264.h"


#define ARSTREAM2_H264_PARSER_TAG "ARSTREAM2_H264Parser"

#define ARSTREAM2_H264_PARSER_MAX_USER_DATA_SEI_COUNT (16)
#define log2(x) (log(x) / log(2)) //TODO


typedef struct ARSTREAM2_H264Parser_s
{
    ARSTREAM2_H264Parser_Config_t config;
    
    // NALU buffer
    uint8_t* pNaluBuf;
    uint8_t* pNaluBufCur;
    unsigned int naluBufSize;
    int naluBufManaged;
    unsigned int naluSize;      // in bytes
    unsigned int remNaluSize;   // in bytes
    
    // Bitstream cache
    uint32_t cache;
    int cacheLength;   // in bits
    int oldZeroCount;

    // SPS/PPS context
    ARSTREAM2_H264_SpsContext_t spsContext;
    int spsSync;
    ARSTREAM2_H264_PpsContext_t ppsContext;
    int ppsSync;

    // Slice context
    ARSTREAM2_H264_SliceContext_t sliceContext;

    // User data SEI
    uint8_t* pUserDataBuf[ARSTREAM2_H264_PARSER_MAX_USER_DATA_SEI_COUNT];
    int userDataBufSize[ARSTREAM2_H264_PARSER_MAX_USER_DATA_SEI_COUNT];
    int userDataSize[ARSTREAM2_H264_PARSER_MAX_USER_DATA_SEI_COUNT];
    unsigned int userDataCount;

    // Recovery point SEI
    ARSTREAM2_H264Parser_RecoveryPointSei_t recoveryPoint;
    int hasRecoveryPoint;

} ARSTREAM2_H264Parser_t;


typedef int (*ARSTREAM2_H264Parser_ParseNaluType_func)(ARSTREAM2_H264Parser_t* parser);

static int ARSTREAM2_H264Parser_ParseSps(ARSTREAM2_H264Parser_t* parser);
static int ARSTREAM2_H264Parser_ParsePps(ARSTREAM2_H264Parser_t* parser);
static int ARSTREAM2_H264Parser_ParseSei(ARSTREAM2_H264Parser_t* parser);
static int ARSTREAM2_H264Parser_ParseAud(ARSTREAM2_H264Parser_t* parser);
static int ARSTREAM2_H264Parser_ParseFillerData(ARSTREAM2_H264Parser_t* parser);
static int ARSTREAM2_H264Parser_ParseSlice(ARSTREAM2_H264Parser_t* parser);


static ARSTREAM2_H264Parser_ParseNaluType_func ARSTREAM2_H264Parser_ParseNaluType[] = 
{
    NULL,
    ARSTREAM2_H264Parser_ParseSlice,
    NULL,
    NULL,
    NULL,
    ARSTREAM2_H264Parser_ParseSlice,
    ARSTREAM2_H264Parser_ParseSei,
    ARSTREAM2_H264Parser_ParseSps,
    ARSTREAM2_H264Parser_ParsePps,
    ARSTREAM2_H264Parser_ParseAud,
    NULL,
    NULL,
    ARSTREAM2_H264Parser_ParseFillerData,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL
};


static char *ARSTREAM2_H264Parser_naluTypeStr[] =
{
    "undefined",
    "coded slice",
    "coded slice data partition A",
    "coded slice data partition B",
    "coded slice data partition C",
    "coded slice IDR",
    "SEI",
    "SPS",
    "PPS",
    "access unit delimiter",
    "end of sequence",
    "end of stream",
    "filler data",
    "SPS extension",
    "prefix NALU",
    "subset SPS",

    "reserved",
    "reserved",
    "reserved",

    "coded slice auxiliary coded picture",
    "coded slice extension",
    "coded slice extension - depth view",
    
    "reserved",
    "reserved",

    "unspecified",
    "unspecified",
    "unspecified",
    "unspecified",
    "unspecified",
    "unspecified",
    "unspecified",
    "unspecified"
};


static char *ARSTREAM2_H264Parser_sliceTypeStr[] =
{
    "P",
    "B",
    "I",
    "SP",
    "SI",
    "P (all)",
    "B (all)",
    "I (all)",
    "SP (all)",
    "SI (all)"
};


static int ARSTREAM2_H264Parser_picStructToNumClockTS[16] =
{ 1, 1, 1, 2, 2, 3, 3, 2, 3, 0, 0, 0, 0, 0, 0, 0};


static inline int bitstreamByteAlign(ARSTREAM2_H264Parser_t* _parser)
{
    int _align = 0;

    if (_parser->cacheLength & 7)
    {
        _align = _parser->cacheLength & 7;
        _parser->cache <<= _align;
        _parser->cacheLength -= _align;
    }

    return _align;
}


static inline int readBits(ARSTREAM2_H264Parser_t* _parser, unsigned int _numBits, uint32_t *_value, int _emulationPrevention)
{
    unsigned int _count, _remBits = _numBits;
    uint32_t _val = 0, _bitMask;
    uint8_t _read8;

    while (_parser->cacheLength < (int)_remBits)
    {
        // Not enough bits in the cache

        if (_parser->remNaluSize == 0)
        {
            // No more bytes to read
            return -1;
        }

        // Get the cache remaining bits
        _remBits -= _parser->cacheLength;
        _val = (_parser->cache >> (32 - _parser->cacheLength)) << _remBits;

        // Read at most 4 bytes in the buffer
        _parser->cache = 0;
        _parser->cacheLength = 0;
        _count = 0;
        while ((_parser->remNaluSize) && (_count < 4))
        {
            _read8 = *(_parser->pNaluBufCur++);
            _parser->remNaluSize--;
            _parser->cache |= (_read8 << (24 - _parser->cacheLength));
            _parser->cacheLength += 8;
            _count++;
        }

        // Emulation prevention
        if (_emulationPrevention)
        {
            int _zeroCount = _parser->oldZeroCount;
            int _bitPos = 24; //32 - _parser->cacheLength;
            uint8_t _byteVal;
            uint32_t _cacheLeft, _cacheRight;

            while (_bitPos >= 32 - _parser->cacheLength)
            {
                _byteVal = ((_parser->cache >> _bitPos) & 0xFF);
                if ((_zeroCount == 2) && (_byteVal == 0x03))
                {
                    // Remove the 0x03 byte
                    _cacheLeft = (_bitPos < 24) ? (_parser->cache >> (_bitPos + 8)) << (_bitPos + 8) : 0;
                    _cacheRight = (_bitPos > 0) ? (_parser->cache << (32 - _bitPos)) >> (32 - _bitPos - 8) : 0;
                    _parser->cache = _cacheLeft | _cacheRight;
                    _parser->cacheLength -= 8;
                    _zeroCount = 0;
                }
                else if (_byteVal == 0x00)
                {
                    _zeroCount++;
                    _bitPos -= 8;
                }
                else
                {
                    _zeroCount = 0;
                    _bitPos -= 8;
                }
            }
            _parser->oldZeroCount = _zeroCount;
        }
    }

    // Get the bits from the cache and shift
    _val |= _parser->cache >> (32 - _remBits);
    _parser->cache <<= _remBits;
    _parser->cacheLength -= _remBits;

    _bitMask = (uint32_t)-1 >> (32 - _numBits);
    if (_value) *_value = _val & _bitMask;
    return _numBits;
}


static inline int peekBits(ARSTREAM2_H264Parser_t* _parser, unsigned int _numBits, uint32_t *_value, int _emulationPrevention)
{
    unsigned int _count, _remBits = _numBits;
    uint32_t _val = 0, _bitMask;
    uint8_t _read8;
    int _cacheLength = _parser->cacheLength;
    int _remNaluSize = _parser->remNaluSize;
    int _oldZeroCount = _parser->oldZeroCount;
    int _offset = 0;
    uint32_t _cache = _parser->cache;

    while (_cacheLength < (int)_remBits)
    {
        // Not enough bits in the cache

        if (_remNaluSize == 0)
        {
            // No more bytes to read
            return -1;
        }

        // Get the cache remaining bits
        _remBits -= _cacheLength;
        _val = (_cache >> (32 - _cacheLength)) << _remBits;

        // Read at most 4 bytes in the buffer
        _cache = 0;
        _cacheLength = 0;
        _count = 0;
        while ((_remNaluSize) && (_count < 4))
        {
            _read8 = *(_parser->pNaluBufCur + _offset);
            _offset++;
            _remNaluSize--;
            _cache |= (_read8 << (24 - _cacheLength));
            _cacheLength += 8;
            _count++;
        }

        // Emulation prevention
        if (_emulationPrevention)
        {
            int _zeroCount = _oldZeroCount;
            int _bitPos = 24; //32 - _cacheLength;
            uint8_t _byteVal;
            uint32_t _cacheLeft, _cacheRight;

            while (_bitPos >= 32 - _cacheLength)
            {
                _byteVal = ((_cache >> _bitPos) & 0xFF);
                if ((_zeroCount == 2) && (_byteVal == 0x03))
                {
                    // Remove the 0x03 byte
                    _cacheLeft = (_bitPos < 24) ? (_cache >> (_bitPos + 8)) << (_bitPos + 8) : 0;
                    _cacheRight = (_bitPos > 0) ? (_cache << (32 - _bitPos)) >> (32 - _bitPos - 8) : 0;
                    _cache = _cacheLeft | _cacheRight;
                    _cacheLength -= 8;
                    _zeroCount = 0;
                }
                else if (_byteVal == 0x00)
                {
                    _zeroCount++;
                    _bitPos -= 8;
                }
                else
                {
                    _zeroCount = 0;
                    _bitPos -= 8;
                }
            }
            _oldZeroCount = _zeroCount;
        }
    }

    // Get the bits from the cache and shift
    _val |= _cache >> (32 - _remBits);
    _cache <<= _remBits;
    _cacheLength -= _remBits;

    _bitMask = (uint32_t)-1 >> (32 - _numBits);
    if (_value) *_value = _val & _bitMask;
    return _numBits;
}


static inline int readBits_expGolomb_ue(ARSTREAM2_H264Parser_t* _parser, uint32_t *_value, int _emulationPrevention)
{
    int _ret, _leadingZeroBits = -1;
    uint32_t _b, _val;

    for (_b = 0; !_b; _leadingZeroBits++)
    {
        _ret = readBits(_parser, 1, &_b, _emulationPrevention);
        if (_ret != 1) return -1;
    }

    _b = 0;
    if (_leadingZeroBits)
    {
        _ret = readBits(_parser, _leadingZeroBits, &_b, _emulationPrevention);
        if (_ret < 0) return -1;
    }

    _val = (1 << _leadingZeroBits) - 1 + _b;
    *_value = _val;
    return _leadingZeroBits * 2 + 1;
}


static inline int readBits_expGolomb_se(ARSTREAM2_H264Parser_t* _parser, int32_t *_value, int _emulationPrevention)
{
    int _ret, _leadingZeroBits = -1;
    uint32_t _b, _val;

    for (_b = 0; !_b; _leadingZeroBits++)
    {
        _ret = readBits(_parser, 1, &_b, _emulationPrevention);
        if (_ret != 1) return -1;
    }

    _b = 0;
    if (_leadingZeroBits)
    {
        _ret = readBits(_parser, _leadingZeroBits, &_b, _emulationPrevention);
        if (_ret < 0) return -1;
    }

    _val = (1 << _leadingZeroBits) - 1 + _b;
    *_value = (_val & 1) ? (((int32_t)_val + 1) / 2) : (-((int32_t)_val + 1) / 2);
    return _leadingZeroBits * 2 + 1;
}


static inline int skipBytes(ARSTREAM2_H264Parser_t* _parser, int _byteCount)
{
    int _ret, _i, _readBits = 0;
    uint32_t _val;

    for (_i = 0; _i < _byteCount; _i++)
    {
        _ret = readBits(_parser, 8, &_val, 1);
        if (_ret < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
            return _ret;
        }
        _readBits += _ret;
    }

    return _readBits / 8;
}


static inline int moreRbspData(ARSTREAM2_H264Parser_t* _parser)
{
    int _ret, _retval = 0;
    uint32_t _val;

    if ((_parser->cacheLength == 0) && (_parser->remNaluSize == 0))
    {
        // No more bits available
        _retval = 0;
    }
    else if (_parser->cacheLength + _parser->remNaluSize * 8 > 8)
    {
        // More than 1 byte remaining
        _retval = 1;
    }
    else
    {
        // 8 bits max remaining
        int _remaining = _parser->cacheLength + _parser->remNaluSize * 8;
        int _i = 1;

        _ret = peekBits(_parser, _remaining, &_val, 1);
        if (_ret < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
            return 0;
        }
        while (((_val & 1) != 1) && (_i < _remaining))
        {
            _val >>= 1;
            _i++;
        }
        if (((_val & 1) == 1) && (_i != _remaining))
        {
            // Not only the RBSP trailing bits remain
            _retval = 1;
        }
        else if (((_val & 1) == 1) && (_i == _remaining))
        {
            // Not only the RBSP trailing bits remain
            _retval = 0;
        }
        else
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed on moreRbspData()");
        }
    }
    
    return _retval;
}


static int ARSTREAM2_H264Parser_ParseScalingList(ARSTREAM2_H264Parser_t* parser, int sizeOfScalingList)
{
    int ret, j;
    int _readBits = 0;
    int32_t val_se = 0;
    int lastScale = 8;
    int nextScale = 8;
    
    // scaling_list
    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "---- scaling_list()");
    
    for (j = 0; j < sizeOfScalingList; j++)
    {
        if (nextScale != 0)
        {
            // delta_scale
            ret = readBits_expGolomb_se(parser, &val_se, 1);
            if (ret < 0)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
                return ret;
            }
            _readBits += ret;
            if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ delta_scale = %d", val_se);
            nextScale = (lastScale + val_se + 256 ) & 0xFF;
        }
        lastScale = (nextScale == 0) ? lastScale : nextScale;
    }

    return _readBits;
}


static int ARSTREAM2_H264Parser_ParseHrdParams(ARSTREAM2_H264Parser_t* parser)
{
    int ret = 0;
    uint32_t val = 0;
    int _readBits = 0;
    unsigned int i;

    // hrd_parameters
    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ hrd_parameters()");

    // cpb_cnt_minus1
    ret = readBits_expGolomb_ue(parser, &val, 1);
    if (ret < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
        return ret;
    }
    _readBits += ret;
    parser->spsContext.cpb_cnt_minus1 = val;
    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "-------- cpb_cnt_minus1 = %d", val);

    // bit_rate_scale
    ret = readBits(parser, 4, &val, 1);
    if (ret < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
        return ret;
    }
    _readBits += ret;
    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "-------- bit_rate_scale = %d", val);

    // cpb_size_scale
    ret = readBits(parser, 4, &val, 1);
    if (ret < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
        return ret;
    }
    _readBits += ret;
    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "-------- cpb_size_scale = %d", val);

    for (i = 0; i <= parser->spsContext.cpb_cnt_minus1; i++)
    {
        // bit_rate_value_minus1
        ret = readBits_expGolomb_ue(parser, &val, 1);
        if (ret < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
            return ret;
        }
        _readBits += ret;
        if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "-------- bit_rate_value_minus1[%d] = %d", i, val);

        // cpb_size_value_minus1
        ret = readBits_expGolomb_ue(parser, &val, 1);
        if (ret < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
            return ret;
        }
        _readBits += ret;
        if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "-------- cpb_size_value_minus1[%d] = %d", i, val);

        // cbr_flag
        ret = readBits(parser, 1, &val, 1);
        if (ret < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
            return ret;
        }
        _readBits += ret;
        if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "-------- cbr_flag[%d] = %d", i, val);
    }

    // initial_cpb_removal_delay_length_minus1
    ret = readBits(parser, 5, &val, 1);
    if (ret < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
        return ret;
    }
    _readBits += ret;
    parser->spsContext.initial_cpb_removal_delay_length_minus1 = val;
    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "-------- initial_cpb_removal_delay_length_minus1 = %d", val);

    // cpb_removal_delay_length_minus1
    ret = readBits(parser, 5, &val, 1);
    if (ret < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
        return ret;
    }
    _readBits += ret;
    parser->spsContext.cpb_removal_delay_length_minus1 = val;
    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "-------- cpb_removal_delay_length_minus1 = %d", val);

    // dpb_output_delay_length_minus1
    ret = readBits(parser, 5, &val, 1);
    if (ret < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
        return ret;
    }
    _readBits += ret;
    parser->spsContext.dpb_output_delay_length_minus1 = val;
    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "-------- dpb_output_delay_length_minus1 = %d", val);

    // time_offset_length
    ret = readBits(parser, 5, &val, 1);
    if (ret < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
        return ret;
    }
    _readBits += ret;
    parser->spsContext.time_offset_length = val;
    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "-------- time_offset_length = %d", val);

    return _readBits;
}


static int ARSTREAM2_H264Parser_ParseVui(ARSTREAM2_H264Parser_t* parser)
{
    int ret = 0;
    uint32_t val = 0;
    int _readBits = 0;

    // vui_parameters
    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "---- vui_parameters()");

    // aspect_ratio_info_present_flag
    ret = readBits(parser, 1, &val, 1);
    if (ret < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
        return ret;
    }
    _readBits += ret;
    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ aspect_ratio_info_present_flag = %d", val);

    if (val)
    {
        // aspect_ratio_idc
        ret = readBits(parser, 8, &val, 1);
        if (ret < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
            return ret;
        }
        _readBits += ret;
        if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ aspect_ratio_idc = %d", val);

        if (val == 255)
        {
            // sar_width
            ret = readBits(parser, 16, &val, 1);
            if (ret < 0)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
                return ret;
            }
            _readBits += ret;
            if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ sar_width = %d", val);

            // sar_height
            ret = readBits(parser, 16, &val, 1);
            if (ret < 0)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
                return ret;
            }
            _readBits += ret;
            if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ sar_height = %d", val);
        }
    }

    // overscan_info_present_flag
    ret = readBits(parser, 1, &val, 1);
    if (ret < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
        return ret;
    }
    _readBits += ret;
    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ overscan_info_present_flag = %d", val);

    if (val)
    {
        // overscan_appropriate_flag
        ret = readBits(parser, 1, &val, 1);
        if (ret < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
            return ret;
        }
        _readBits += ret;
        if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ overscan_appropriate_flag = %d", val);
    }

    // video_signal_type_present_flag
    ret = readBits(parser, 1, &val, 1);
    if (ret < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
        return ret;
    }
    _readBits += ret;
    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ video_signal_type_present_flag = %d", val);

    if (val)
    {
        // video_format
        ret = readBits(parser, 3, &val, 1);
        if (ret < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
            return ret;
        }
        _readBits += ret;
        if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ video_format = %d", val);

        // video_full_range_flag
        ret = readBits(parser, 1, &val, 1);
        if (ret < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
            return ret;
        }
        _readBits += ret;
        if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ video_full_range_flag = %d", val);

        // colour_description_present_flag
        ret = readBits(parser, 1, &val, 1);
        if (ret < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
            return ret;
        }
        _readBits += ret;
        if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ colour_description_present_flag = %d", val);
        
        if (val)
        {
            // colour_primaries
            ret = readBits(parser, 8, &val, 1);
            if (ret < 0)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
                return ret;
            }
            _readBits += ret;
            if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ colour_primaries = %d", val);

            // transfer_characteristics
            ret = readBits(parser, 8, &val, 1);
            if (ret < 0)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
                return ret;
            }
            _readBits += ret;
            if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ transfer_characteristics = %d", val);

            // matrix_coefficients
            ret = readBits(parser, 8, &val, 1);
            if (ret < 0)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
                return ret;
            }
            _readBits += ret;
            if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ matrix_coefficients = %d", val);

        }
    }

    // chroma_loc_info_present_flag
    ret = readBits(parser, 1, &val, 1);
    if (ret < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
        return ret;
    }
    _readBits += ret;
    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ chroma_loc_info_present_flag = %d", val);
    
    if (val)
    {
        // chroma_sample_loc_type_top_field
        ret = readBits_expGolomb_ue(parser, &val, 1);
        if (ret < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
            return ret;
        }
        _readBits += ret;
        if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ chroma_sample_loc_type_top_field = %d", val);

        // chroma_sample_loc_type_bottom_field
        ret = readBits_expGolomb_ue(parser, &val, 1);
        if (ret < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
            return ret;
        }
        _readBits += ret;
        if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ chroma_sample_loc_type_bottom_field = %d", val);
    }

    // timing_info_present_flag
    ret = readBits(parser, 1, &val, 1);
    if (ret < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
        return ret;
    }
    _readBits += ret;
    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ timing_info_present_flag = %d", val);
    
    if (val)
    {
        // num_units_in_tick
        ret = readBits(parser, 32, &val, 1);
        if (ret < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
            return ret;
        }
        _readBits += ret;
        parser->spsContext.num_units_in_tick = val;
        if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ num_units_in_tick = %d", val);
        
        // time_scale
        ret = readBits(parser, 32, &val, 1);
        if (ret < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
            return ret;
        }
        _readBits += ret;
        parser->spsContext.time_scale = val;
        if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ time_scale = %d", val);
        
        // fixed_frame_rate_flag
        ret = readBits(parser, 1, &val, 1);
        if (ret < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
            return ret;
        }
        _readBits += ret;
        if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ fixed_frame_rate_flag = %d", val);
    }

    // nal_hrd_parameters_present_flag
    ret = readBits(parser, 1, &val, 1);
    if (ret < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
        return ret;
    }
    _readBits += ret;
    parser->spsContext.nal_hrd_parameters_present_flag = val;
    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ nal_hrd_parameters_present_flag = %d", val);
    
    if (parser->spsContext.nal_hrd_parameters_present_flag)
    {
        // hrd_parameters
        ret = ARSTREAM2_H264Parser_ParseHrdParams(parser);
        if (ret < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "ARSTREAM2_H264Parser_ParseHrdParams() failed (%d)", ret);
            return ret;
        }
        _readBits += ret;
    }

    // vcl_hrd_parameters_present_flag
    ret = readBits(parser, 1, &val, 1);
    if (ret < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
        return ret;
    }
    _readBits += ret;
    parser->spsContext.vcl_hrd_parameters_present_flag = val;
    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ vcl_hrd_parameters_present_flag = %d", val);
    
    if (parser->spsContext.vcl_hrd_parameters_present_flag)
    {
        // hrd_parameters
        ret = ARSTREAM2_H264Parser_ParseHrdParams(parser);
        if (ret < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "ARSTREAM2_H264Parser_ParseHrdParams() failed (%d)", ret);
            return ret;
        }
        _readBits += ret;
    }

    if (parser->spsContext.nal_hrd_parameters_present_flag || parser->spsContext.vcl_hrd_parameters_present_flag)
    {
        // low_delay_hrd_flag
        ret = readBits(parser, 1, &val, 1);
        if (ret < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
            return ret;
        }
        _readBits += ret;
        if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ low_delay_hrd_flag = %d", val);
    }

    // pic_struct_present_flag
    ret = readBits(parser, 1, &val, 1);
    if (ret < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
        return ret;
    }
    _readBits += ret;
    parser->spsContext.pic_struct_present_flag = val;
    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ pic_struct_present_flag = %d", val);
    
    // bitstream_restriction_flag
    ret = readBits(parser, 1, &val, 1);
    if (ret < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
        return ret;
    }
    _readBits += ret;
    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ bitstream_restriction_flag = %d", val);
    
    if (val)
    {
        // motion_vectors_over_pic_boundaries_flag
        ret = readBits(parser, 1, &val, 1);
        if (ret < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
            return ret;
        }
        _readBits += ret;
        if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ motion_vectors_over_pic_boundaries_flag = %d", val);

        // max_bytes_per_pic_denom
        ret = readBits_expGolomb_ue(parser, &val, 1);
        if (ret < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
            return ret;
        }
        _readBits += ret;
        if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ max_bytes_per_pic_denom = %d", val);

        // max_bits_per_mb_denom
        ret = readBits_expGolomb_ue(parser, &val, 1);
        if (ret < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
            return ret;
        }
        _readBits += ret;
        if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ max_bits_per_mb_denom = %d", val);

        // log2_max_mv_length_horizontal
        ret = readBits_expGolomb_ue(parser, &val, 1);
        if (ret < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
            return ret;
        }
        _readBits += ret;
        if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ log2_max_mv_length_horizontal = %d", val);

        // log2_max_mv_length_vertical
        ret = readBits_expGolomb_ue(parser, &val, 1);
        if (ret < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
            return ret;
        }
        _readBits += ret;
        if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ log2_max_mv_length_vertical = %d", val);

        // max_num_reorder_frames
        ret = readBits_expGolomb_ue(parser, &val, 1);
        if (ret < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
            return ret;
        }
        _readBits += ret;
        if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ max_num_reorder_frames = %d", val);

        // max_dec_frame_buffering
        ret = readBits_expGolomb_ue(parser, &val, 1);
        if (ret < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
            return ret;
        }
        _readBits += ret;
        if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ max_dec_frame_buffering = %d", val);
    }

    return _readBits;
}


static int ARSTREAM2_H264Parser_ParseSps(ARSTREAM2_H264Parser_t* parser)
{
    int ret = 0;
    uint32_t val = 0;
    int32_t val_se = 0;
    int readBytes = 0, _readBits = 0;
    int i, profile_idc, num_ref_frames_in_pic_order_cnt_cycle, width, height;

    // seq_parameter_set_rbsp
    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "-- seq_parameter_set_rbsp()");
    
    // seq_parameter_set_data

    ret = readBits(parser, 24, &val, 1);
    if (ret < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
        return ret;
    }
    readBytes += 3;
    profile_idc = val >> 16;
    
    // profile_idc
    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "---- profile_idc = %d", profile_idc);
    
    // constraint_set0_flag
    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "---- constraint_set0_flag = %d", (val >> 15) & 1);
    
    // constraint_set1_flag
    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "---- constraint_set1_flag = %d", (val >> 14) & 1);
    
    // constraint_set2_flag
    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "---- constraint_set2_flag = %d", (val >> 13) & 1);
    
    // constraint_set3_flag
    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "---- constraint_set3_flag = %d", (val >> 12) & 1);
    
    // constraint_set4_flag
    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "---- constraint_set4_flag = %d", (val >> 11) & 1);
    
    // constraint_set5_flag
    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "---- constraint_set5_flag = %d", (val >> 10) & 1);
    
    // reserved_zero_2bits
    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "---- reserved_zero_2bits = %d%d", (val >> 9) & 1, (val >> 8) & 1);
    
    // level_idc
    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "---- level_idc = %d", val & 0xFF);

    // seq_parameter_set_id
    ret = readBits_expGolomb_ue(parser, &val, 1);
    if (ret < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
        return ret;
    }
    _readBits += ret;
    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "---- seq_parameter_set_id = %d", val);

    if (profile_idc == 100 || profile_idc == 110 || profile_idc == 122 || profile_idc == 244 
            || profile_idc == 44 || profile_idc == 83 || profile_idc == 86 
            || profile_idc == 118 || profile_idc == 128 || profile_idc == 138)
    {
        // chroma_format_idc
        ret = readBits_expGolomb_ue(parser, &val, 1);
        if (ret < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
            return ret;
        }
        _readBits += ret;
        parser->spsContext.chroma_format_idc = val;
        if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "---- chroma_format_idc = %d", val);

        if (val == 3)
        {
            // separate_colour_plane_flag
            ret = readBits(parser, 1, &val, 1);
            if (ret < 0)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
                return ret;
            }
            _readBits += ret;
            parser->spsContext.separate_colour_plane_flag = val;
            if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "---- separate_colour_plane_flag = %d", val);
        }

        // bit_depth_luma_minus8
        ret = readBits_expGolomb_ue(parser, &val, 1);
        if (ret < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
            return ret;
        }
        _readBits += ret;
        if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "---- bit_depth_luma_minus8 = %d", val);

        // bit_depth_chroma_minus8
        ret = readBits_expGolomb_ue(parser, &val, 1);
        if (ret < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
            return ret;
        }
        _readBits += ret;
        if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "---- bit_depth_chroma_minus8 = %d", val);

        // qpprime_y_zero_transform_bypass_flag
        ret = readBits(parser, 1, &val, 1);
        if (ret < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
            return ret;
        }
        _readBits += ret;
        if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "---- qpprime_y_zero_transform_bypass_flag = %d", val);

        // seq_scaling_matrix_present_flag
        ret = readBits(parser, 1, &val, 1);
        if (ret < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
            return ret;
        }
        _readBits += ret;
        if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "---- seq_scaling_matrix_present_flag = %d", val);

        if (val)
        {
            for (i = 0; i < ((parser->spsContext.chroma_format_idc != 3) ? 8 : 12); i++)
            {
                // seq_scaling_list_present_flag
                ret = readBits(parser, 1, &val, 1);
                if (ret < 0)
                {
                    ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
                    return ret;
                }
                _readBits += ret;
                if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "---- seq_scaling_list_present_flag[%d] = %d", i, val);

                if (val)
                {
                    ret = ARSTREAM2_H264Parser_ParseScalingList(parser, (i < 6) ? 16 : 64);
                    if (ret < 0)
                    {
                        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "ARSTREAM2_H264Parser_ParseScalingList() failed (%d)", ret);
                        return ret;
                    }
                    _readBits += ret;
                }
            }
        }
    }

    // log2_max_frame_num_minus4
    ret = readBits_expGolomb_ue(parser, &val, 1);
    if (ret < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
        return ret;
    }
    _readBits += ret;
    parser->spsContext.log2_max_frame_num_minus4 = val;
    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "---- log2_max_frame_num_minus4 = %d", val);

    // pic_order_cnt_type
    ret = readBits_expGolomb_ue(parser, &val, 1);
    if (ret < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
        return ret;
    }
    _readBits += ret;
    parser->spsContext.pic_order_cnt_type = val;
    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "---- pic_order_cnt_type = %d", val);

    if (parser->spsContext.pic_order_cnt_type == 0)
    {
        // log2_max_pic_order_cnt_lsb_minus4
        ret = readBits_expGolomb_ue(parser, &val, 1);
        if (ret < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
            return ret;
        }
        _readBits += ret;
        parser->spsContext.log2_max_pic_order_cnt_lsb_minus4 = val;
        if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "---- log2_max_pic_order_cnt_lsb_minus4 = %d", val);
    }
    else if (parser->spsContext.pic_order_cnt_type == 1)
    {
        // delta_pic_order_always_zero_flag
        ret = readBits(parser, 1, &val, 1);
        if (ret < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
            return ret;
        }
        _readBits += ret;
        parser->spsContext.delta_pic_order_always_zero_flag = val;
        if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "---- delta_pic_order_always_zero_flag = %d", val);

        // offset_for_non_ref_pic
        ret = readBits_expGolomb_se(parser, &val_se, 1);
        if (ret < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
            return ret;
        }
        _readBits += ret;
        if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "---- offset_for_non_ref_pic = %d", val_se);

        // offset_for_top_to_bottom_field
        ret = readBits_expGolomb_se(parser, &val_se, 1);
        if (ret < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
            return ret;
        }
        _readBits += ret;
        if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "---- offset_for_top_to_bottom_field = %d", val_se);

        // num_ref_frames_in_pic_order_cnt_cycle
        ret = readBits_expGolomb_ue(parser, &val, 1);
        if (ret < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
            return ret;
        }
        _readBits += ret;
        num_ref_frames_in_pic_order_cnt_cycle = val;
        if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "---- num_ref_frames_in_pic_order_cnt_cycle = %d", num_ref_frames_in_pic_order_cnt_cycle);

        for (i = 0; i < num_ref_frames_in_pic_order_cnt_cycle; i++)
        {
            // offset_for_ref_frame
            ret = readBits_expGolomb_se(parser, &val_se, 1);
            if (ret < 0)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
                return ret;
            }
            _readBits += ret;
            if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "---- offset_for_ref_frame[%d] = %d", i, val_se);
        }
    }

    // max_num_ref_frames
    ret = readBits_expGolomb_ue(parser, &val, 1);
    if (ret < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
        return ret;
    }
    _readBits += ret;
    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "---- max_num_ref_frames = %d", val);

    // gaps_in_frame_num_value_allowed_flag
    ret = readBits(parser, 1, &val, 1);
    if (ret < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
        return ret;
    }
    _readBits += ret;
    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "---- gaps_in_frame_num_value_allowed_flag = %d", val);

    // pic_width_in_mbs_minus1
    ret = readBits_expGolomb_ue(parser, &val, 1);
    if (ret < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
        return ret;
    }
    _readBits += ret;
    parser->spsContext.pic_width_in_mbs_minus1 = val;
    width = (parser->spsContext.pic_width_in_mbs_minus1 + 1) * 16;
    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "---- pic_width_in_mbs_minus1 = %d (width = %d pixels)", val, width);

    // pic_height_in_map_units_minus1
    ret = readBits_expGolomb_ue(parser, &val, 1);
    if (ret < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
        return ret;
    }
    _readBits += ret;
    parser->spsContext.pic_height_in_map_units_minus1 = val;
    height = (parser->spsContext.pic_height_in_map_units_minus1 + 1) * 16;
    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "---- pic_height_in_map_units_minus1 = %d", val);

    // frame_mbs_only_flag
    ret = readBits(parser, 1, &val, 1);
    if (ret < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
        return ret;
    }
    _readBits += ret;
    parser->spsContext.frame_mbs_only_flag = val;
    if (!parser->spsContext.frame_mbs_only_flag) height *= 2;
    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "---- frame_mbs_only_flag = %d (height = %d pixels)", val, height);

    if (!val)
    {
        // mb_adaptive_frame_field_flag
        ret = readBits(parser, 1, &val, 1);
        if (ret < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
            return ret;
        }
        _readBits += ret;
        if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "---- mb_adaptive_frame_field_flag = %d", val);
    }

    // direct_8x8_inference_flag
    ret = readBits(parser, 1, &val, 1);
    if (ret < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
        return ret;
    }
    _readBits += ret;
    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "---- direct_8x8_inference_flag = %d", val);

    // frame_cropping_flag
    ret = readBits(parser, 1, &val, 1);
    if (ret < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
        return ret;
    }
    _readBits += ret;
    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "---- frame_cropping_flag = %d", val);

    if (val)
    {
        // frame_crop_left_offset
        ret = readBits_expGolomb_ue(parser, &val, 1);
        if (ret < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
            return ret;
        }
        _readBits += ret;
        if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "---- frame_crop_left_offset = %d", val);

        // frame_crop_right_offset
        ret = readBits_expGolomb_ue(parser, &val, 1);
        if (ret < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
            return ret;
        }
        _readBits += ret;
        if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "---- frame_crop_right_offset = %d", val);

        // frame_crop_top_offset
        ret = readBits_expGolomb_ue(parser, &val, 1);
        if (ret < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
            return ret;
        }
        _readBits += ret;
        if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "---- frame_crop_top_offset = %d", val);

        // frame_crop_bottom_offset
        ret = readBits_expGolomb_ue(parser, &val, 1);
        if (ret < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
            return ret;
        }
        _readBits += ret;
        if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "---- frame_crop_bottom_offset = %d", val);
    }

    // vui_parameters_present_flag
    ret = readBits(parser, 1, &val, 1);
    if (ret < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
        return ret;
    }
    _readBits += ret;
    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "---- vui_parameters_present_flag = %d", val);

    if (val)
    {
        // vui_parameters
        ret = ARSTREAM2_H264Parser_ParseVui(parser);
        if (ret < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "ARSTREAM2_H264Parser_ParseVui() failed (%d)", ret);
            return ret;
        }
        _readBits += ret;
    }

    // rbsp_trailing_bits
    ret = readBits(parser, 1, &val, 1);
    if (ret < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
        return ret;
    }
    _readBits += ret;

    ret = bitstreamByteAlign(parser);
    if (ret < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to align the bitstream (%d)", ret);
        return ret;
    }
    _readBits += ret;
    readBytes += _readBits / 8;

    parser->spsSync = 1;
    return readBytes;
}


static int ARSTREAM2_H264Parser_ParsePps(ARSTREAM2_H264Parser_t* parser)
{
    int ret = 0;
    uint32_t val = 0;
    int32_t val_se = 0;
    int readBytes = 0, _readBits = 0;
    unsigned int i, len, pic_size_in_map_units_minus1, transform_8x8_mode_flag;

    // pic_parameter_set_rbsp
    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "-- pic_parameter_set_rbsp()");

    // pic_parameter_set_id
    ret = readBits_expGolomb_ue(parser, &val, 1);
    if (ret < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
        return ret;
    }
    _readBits += ret;
    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "---- pic_parameter_set_id = %d", val);

    // seq_parameter_set_id
    ret = readBits_expGolomb_ue(parser, &val, 1);
    if (ret < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
        return ret;
    }
    _readBits += ret;
    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "---- seq_parameter_set_id = %d", val);

    // entropy_coding_mode_flag
    ret = readBits(parser, 1, &val, 1);
    if (ret < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
        return ret;
    }
    _readBits += ret;
    parser->ppsContext.entropy_coding_mode_flag = val;
    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "---- entropy_coding_mode_flag = %d", val);

    // bottom_field_pic_order_in_frame_present_flag
    ret = readBits(parser, 1, &val, 1);
    if (ret < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
        return ret;
    }
    _readBits += ret;
    parser->ppsContext.bottom_field_pic_order_in_frame_present_flag = val;
    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "---- bottom_field_pic_order_in_frame_present_flag = %d", val);

    // num_slice_groups_minus1
    ret = readBits_expGolomb_ue(parser, &val, 1);
    if (ret < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
        return ret;
    }
    _readBits += ret;
    parser->ppsContext.num_slice_groups_minus1 = val;
    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "---- num_slice_groups_minus1 = %d", val);

    if (parser->ppsContext.num_slice_groups_minus1 > 0)
    {
        // slice_group_map_type
        ret = readBits_expGolomb_ue(parser, &val, 1);
        if (ret < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
            return ret;
        }
        _readBits += ret;
        parser->ppsContext.slice_group_map_type = val;
        if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "---- slice_group_map_type = %d", val);
        
        if (parser->ppsContext.slice_group_map_type == 0)
        {
            for (i = 0; i <= parser->ppsContext.num_slice_groups_minus1; i++)
            {
                // run_length_minus1[i]
                ret = readBits_expGolomb_ue(parser, &val, 1);
                if (ret < 0)
                {
                    ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
                    return ret;
                }
                _readBits += ret;
                if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "---- run_length_minus1[%d] = %d", i, val);
            }
        }
        else if (parser->ppsContext.slice_group_map_type == 2)
        {
            for (i = 0; i < parser->ppsContext.num_slice_groups_minus1; i++)
            {
                // top_left[i]
                ret = readBits_expGolomb_ue(parser, &val, 1);
                if (ret < 0)
                {
                    ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
                    return ret;
                }
                _readBits += ret;
                if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "---- top_left[%d] = %d", i, val);

                // bottom_right[i]
                ret = readBits_expGolomb_ue(parser, &val, 1);
                if (ret < 0)
                {
                    ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
                    return ret;
                }
                _readBits += ret;
                if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "---- bottom_right[%d] = %d", i, val);
            }
        }
        else if ((parser->ppsContext.slice_group_map_type == 3) || (parser->ppsContext.slice_group_map_type == 4) || (parser->ppsContext.slice_group_map_type == 5))
        {
            // slice_group_change_direction_flag
            ret = readBits(parser, 1, &val, 1);
            if (ret < 0)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
                return ret;
            }
            _readBits += ret;
            if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "---- slice_group_change_direction_flag = %d", val);

            // slice_group_change_rate_minus1
            ret = readBits_expGolomb_ue(parser, &val, 1);
            if (ret < 0)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
                return ret;
            }
            _readBits += ret;
            parser->ppsContext.slice_group_change_rate_minus1 = val;
            if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "---- slice_group_change_rate_minus1 = %d", val);
        }
        else if (parser->ppsContext.slice_group_map_type == 6)
        {
            // pic_size_in_map_units_minus1
            ret = readBits_expGolomb_ue(parser, &val, 1);
            if (ret < 0)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
                return ret;
            }
            _readBits += ret;
            pic_size_in_map_units_minus1 = val;
            if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "---- pic_size_in_map_units_minus1 = %d", val);

            for (i = 0; i <= pic_size_in_map_units_minus1; i++)
            {
                // slice_group_id[i]
                len = (int)ceil(log2(parser->ppsContext.num_slice_groups_minus1 + 1));
                ret = readBits(parser, len, &val, 1);
                if (ret < 0)
                {
                    ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
                    return ret;
                }
                _readBits += ret;
                if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "---- slice_group_id[%d] = %d", i, val);
            }
        }
    }

    // num_ref_idx_l0_default_active_minus1
    ret = readBits_expGolomb_ue(parser, &val, 1);
    if (ret < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
        return ret;
    }
    _readBits += ret;
    parser->ppsContext.num_ref_idx_l0_default_active_minus1 = val;
    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "---- num_ref_idx_l0_default_active_minus1 = %d", val);

    // num_ref_idx_l1_default_active_minus1
    ret = readBits_expGolomb_ue(parser, &val, 1);
    if (ret < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
        return ret;
    }
    _readBits += ret;
    parser->ppsContext.num_ref_idx_l1_default_active_minus1 = val;
    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "---- num_ref_idx_l1_default_active_minus1 = %d", val);

    // weighted_pred_flag
    ret = readBits(parser, 1, &val, 1);
    if (ret < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
        return ret;
    }
    _readBits += ret;
    parser->ppsContext.weighted_pred_flag = val;
    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "---- weighted_pred_flag = %d", val);

    // weighted_bipred_idc
    ret = readBits(parser, 2, &val, 1);
    if (ret < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
        return ret;
    }
    _readBits += ret;
    parser->ppsContext.weighted_bipred_idc = val;
    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "---- weighted_bipred_idc = %d", val);

    // pic_init_qp_minus26
    ret = readBits_expGolomb_se(parser, &val_se, 1);
    if (ret < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
        return ret;
    }
    _readBits += ret;
    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "---- pic_init_qp_minus26 = %d", val_se);

    // pic_init_qs_minus26
    ret = readBits_expGolomb_se(parser, &val_se, 1);
    if (ret < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
        return ret;
    }
    _readBits += ret;
    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "---- pic_init_qs_minus26 = %d", val_se);

    // chroma_qp_index_offset
    ret = readBits_expGolomb_se(parser, &val_se, 1);
    if (ret < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
        return ret;
    }
    _readBits += ret;
    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "---- chroma_qp_index_offset = %d", val_se);

    // deblocking_filter_control_present_flag
    ret = readBits(parser, 1, &val, 1);
    if (ret < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
        return ret;
    }
    _readBits += ret;
    parser->ppsContext.deblocking_filter_control_present_flag = val;
    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "---- deblocking_filter_control_present_flag = %d", val);

    // constrained_intra_pred_flag
    ret = readBits(parser, 1, &val, 1);
    if (ret < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
        return ret;
    }
    _readBits += ret;
    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "---- constrained_intra_pred_flag = %d", val);

    // redundant_pic_cnt_present_flag
    ret = readBits(parser, 1, &val, 1);
    if (ret < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
        return ret;
    }
    _readBits += ret;
    parser->ppsContext.redundant_pic_cnt_present_flag = val;
    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "---- redundant_pic_cnt_present_flag = %d", val);


    if (moreRbspData(parser))
    {
        // transform_8x8_mode_flag
        ret = readBits(parser, 1, &val, 1);
        if (ret < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
            return ret;
        }
        _readBits += ret;
        transform_8x8_mode_flag = val;
        if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "---- transform_8x8_mode_flag = %d", val);

        // pic_scaling_matrix_present_flag
        ret = readBits(parser, 1, &val, 1);
        if (ret < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
            return ret;
        }
        _readBits += ret;
        if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "---- pic_scaling_matrix_present_flag = %d", val);

        if (val)
        {
            for (i = 0; i < 6 + ((parser->spsContext.chroma_format_idc != 3) ? 2 : 6) * transform_8x8_mode_flag; i++)
            {
                // pic_scaling_list_present_flag[i]
                ret = readBits(parser, 1, &val, 1);
                if (ret < 0)
                {
                    ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
                    return ret;
                }
                _readBits += ret;
                if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "---- pic_scaling_list_present_flag[%d] = %d", i, val);

                if (val)
                {
                    ret = ARSTREAM2_H264Parser_ParseScalingList(parser, (i < 6) ? 16 : 64);
                    if (ret < 0)
                    {
                        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "ARSTREAM2_H264Parser_ParseScalingList() failed (%d)", ret);
                        return ret;
                    }
                    _readBits += ret;
                }
            }
        }

        // second_chroma_qp_index_offset
        ret = readBits_expGolomb_se(parser, &val_se, 1);
        if (ret < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
            return ret;
        }
        _readBits += ret;
        if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "---- second_chroma_qp_index_offset = %d", val_se);
    }


    // rbsp_trailing_bits
    ret = readBits(parser, 1, &val, 1);
    if (ret < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
        return ret;
    }
    _readBits += ret;

    ret = bitstreamByteAlign(parser);
    if (ret < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to align the bitstream (%d)", ret);
        return ret;
    }
    _readBits += ret;
    readBytes += _readBits / 8;

    parser->ppsSync = 1;
    return readBytes;
}


static int ARSTREAM2_H264Parser_ParseSeiPayload_userDataUnregistered(ARSTREAM2_H264Parser_t* parser, int payloadSize)
{
    int ret = 0, i;
    uint32_t val = 0;
    int _readBits = 0;
    uint32_t uuid1, uuid2, uuid3, uuid4;

    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "---- SEI: user_data_unregistered");

    if (payloadSize < 16)
    {
        if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "---- wrong size for user_data_unregistered (%d < 16)", payloadSize);
        return _readBits;
    }
    
    // uuid_iso_iec_11578
    ret = readBits(parser, 32, &val, 1);
    if (ret < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
        return ret;
    }
    _readBits += ret;
    uuid1 = val;
    ret = readBits(parser, 32, &val, 1);
    if (ret < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
        return ret;
    }
    _readBits += ret;
    uuid2 = val;
    ret = readBits(parser, 32, &val, 1);
    if (ret < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
        return ret;
    }
    _readBits += ret;
    uuid3 = val;
    ret = readBits(parser, 32, &val, 1);
    if (ret < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
        return ret;
    }
    _readBits += ret;
    uuid4 = val;
    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ uuid_iso_iec_11578 = %08x-%04x-%04x-%08x-%08x", uuid1, uuid2 >> 16, uuid2 & 0xFFFF, uuid3, uuid4);

    if ((parser->config.extractUserDataSei) && (parser->userDataCount < ARSTREAM2_H264_PARSER_MAX_USER_DATA_SEI_COUNT))
    {
        if ((!parser->pUserDataBuf[parser->userDataCount]) || (parser->userDataBufSize[parser->userDataCount] < payloadSize))
        {
            parser->pUserDataBuf[parser->userDataCount] = (uint8_t*)realloc(parser->pUserDataBuf[parser->userDataCount], payloadSize);
            if (!parser->pUserDataBuf[parser->userDataCount])
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Allocation failed (size %d)", payloadSize);
                return -1;
            }
            parser->userDataBufSize[parser->userDataCount] = payloadSize;
        }

        parser->pUserDataBuf[parser->userDataCount][0] = (uint8_t)((uuid1 >> 24) & 0xFF);
        parser->pUserDataBuf[parser->userDataCount][1] = (uint8_t)((uuid1 >> 16) & 0xFF);
        parser->pUserDataBuf[parser->userDataCount][2] = (uint8_t)((uuid1 >> 8) & 0xFF);
        parser->pUserDataBuf[parser->userDataCount][3] = (uint8_t)((uuid1 >> 0) & 0xFF);
        parser->pUserDataBuf[parser->userDataCount][4] = (uint8_t)((uuid2 >> 24) & 0xFF);
        parser->pUserDataBuf[parser->userDataCount][5] = (uint8_t)((uuid2 >> 16) & 0xFF);
        parser->pUserDataBuf[parser->userDataCount][6] = (uint8_t)((uuid2 >> 8) & 0xFF);
        parser->pUserDataBuf[parser->userDataCount][7] = (uint8_t)((uuid2 >> 0) & 0xFF);
        parser->pUserDataBuf[parser->userDataCount][8] = (uint8_t)((uuid3 >> 24) & 0xFF);
        parser->pUserDataBuf[parser->userDataCount][9] = (uint8_t)((uuid3 >> 16) & 0xFF);
        parser->pUserDataBuf[parser->userDataCount][10] = (uint8_t)((uuid3 >> 8) & 0xFF);
        parser->pUserDataBuf[parser->userDataCount][11] = (uint8_t)((uuid3 >> 0) & 0xFF);
        parser->pUserDataBuf[parser->userDataCount][12] = (uint8_t)((uuid4 >> 24) & 0xFF);
        parser->pUserDataBuf[parser->userDataCount][13] = (uint8_t)((uuid4 >> 16) & 0xFF);
        parser->pUserDataBuf[parser->userDataCount][14] = (uint8_t)((uuid4 >> 8) & 0xFF);
        parser->pUserDataBuf[parser->userDataCount][15] = (uint8_t)((uuid4 >> 0) & 0xFF);
        for (i = 16; i < payloadSize; i++)
        {
            ret = readBits(parser, 8, &val, 1);
            if (ret < 0)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
                return ret;
            }
            _readBits += ret;
            parser->pUserDataBuf[parser->userDataCount][i] = (uint8_t)val;
        }
        parser->userDataSize[parser->userDataCount] = payloadSize;
        parser->userDataCount++;
    }
    else
    {
        ret = skipBytes(parser, payloadSize - 16);
        if (ret < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "skipBytes() failed (%d)", ret);
            return ret;
        }
        _readBits += ret * 8;
    }

    return _readBits;
}


static int ARSTREAM2_H264Parser_ParseSeiPayload_recoveryPoint(ARSTREAM2_H264Parser_t* parser, int payloadSize)
{
    int ret = 0;
    uint32_t val = 0;
    int _readBits = 0;

    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "---- SEI: recovery_point");

    // recovery_frame_count
    ret = readBits_expGolomb_ue(parser, &val, 1);
    if (ret < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
        return ret;
    }
    _readBits += ret;
    parser->recoveryPoint.recoveryFrameCnt = val;
    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ recovery_frame_count = %d", val);

    // exact_match_flag
    ret = readBits(parser, 1, &val, 1);
    if (ret < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
        return ret;
    }
    _readBits += ret;
    parser->recoveryPoint.exactMatchFlag = val;
    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ exact_match_flag = %d", val);

    // broken_link_flag
    ret = readBits(parser, 1, &val, 1);
    if (ret < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
        return ret;
    }
    _readBits += ret;
    parser->recoveryPoint.brokenLinkFlag = val;
    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ broken_link_flag = %d", val);

    // changing_slice_group_idc
    ret = readBits(parser, 2, &val, 1);
    if (ret < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
        return ret;
    }
    _readBits += ret;
    parser->recoveryPoint.changingSliceGroupIdc = val;
    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ changing_slice_group_idc = %d", val);

    parser->hasRecoveryPoint = 1;

    return _readBits;
}


static int ARSTREAM2_H264Parser_ParseSeiPayload_bufferingPeriod(ARSTREAM2_H264Parser_t* parser, int payloadSize)
{
    int ret = 0;
    uint32_t val = 0;
    int _readBits = 0;
    unsigned int i;

    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "---- SEI: buffering_period");

    // seq_parameter_set_id
    ret = readBits_expGolomb_ue(parser, &val, 1);
    if (ret < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
        return ret;
    }
    _readBits += ret;
    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ seq_parameter_set_id = %d", val);

    if (parser->spsContext.nal_hrd_parameters_present_flag)
    {
        for (i = 0; i <= parser->spsContext.cpb_cnt_minus1; i++)
        {
            // initial_cpb_removal_delay[i]
            ret = readBits(parser, parser->spsContext.initial_cpb_removal_delay_length_minus1 + 1, &val, 1);
            if (ret < 0)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
                return ret;
            }
            _readBits += ret;
            if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ initial_cpb_removal_delay[%d] = %d", i, val);

            // initial_cpb_removal_delay_offset[i]
            ret = readBits(parser, parser->spsContext.initial_cpb_removal_delay_length_minus1 + 1, &val, 1);
            if (ret < 0)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
                return ret;
            }
            _readBits += ret;
            if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ initial_cpb_removal_delay_offset[%d] = %d", i, val);
        }
    }
    
    if (parser->spsContext.vcl_hrd_parameters_present_flag)
    {
        for (i = 0; i <= parser->spsContext.cpb_cnt_minus1; i++)
        {
            // initial_cpb_removal_delay[i]
            ret = readBits(parser, parser->spsContext.initial_cpb_removal_delay_length_minus1 + 1, &val, 1);
            if (ret < 0)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
                return ret;
            }
            _readBits += ret;
            if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ initial_cpb_removal_delay[%d] = %d", i, val);

            // initial_cpb_removal_delay_offset[i]
            ret = readBits(parser, parser->spsContext.initial_cpb_removal_delay_length_minus1 + 1, &val, 1);
            if (ret < 0)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
                return ret;
            }
            _readBits += ret;
            if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ initial_cpb_removal_delay_offset[%d] = %d", i, val);
        }
    }

    return _readBits;
}


static int ARSTREAM2_H264Parser_ParseSeiPayload_picTiming(ARSTREAM2_H264Parser_t* parser, int payloadSize)
{
    int ret = 0;
    uint32_t val = 0;
    int _readBits = 0;
    int i, full_timestamp_flag, pic_struct, nuit_field_based_flag = 0;
    uint64_t clockTimestamp = 0;
    int hH = 0, mM = 0, sS = 0, nFrames = 0, tOffset = 0;

    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "---- SEI: pic_timing");

    if (parser->spsContext.nal_hrd_parameters_present_flag || parser->spsContext.vcl_hrd_parameters_present_flag)
    {
        // cpb_removal_delay
        ret = readBits(parser, parser->spsContext.cpb_removal_delay_length_minus1 + 1, &val, 1);
        if (ret < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
            return ret;
        }
        _readBits += ret;
        if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ cpb_removal_delay = %d", val);

        // dpb_output_delay
        ret = readBits(parser, parser->spsContext.dpb_output_delay_length_minus1 + 1, &val, 1);
        if (ret < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
            return ret;
        }
        _readBits += ret;
        if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ dpb_output_delay = %d", val);
    }

    if (parser->spsContext.pic_struct_present_flag)
    {
        // pic_struct
        ret = readBits(parser, 4, &val, 1);
        if (ret < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
            return ret;
        }
        _readBits += ret;
        pic_struct = val;
        if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ pic_struct = %d", val);
        
        for (i = 0; i < ARSTREAM2_H264Parser_picStructToNumClockTS[pic_struct]; i++)
        {
            // clock_timestamp_flag[i]
            ret = readBits(parser, 1, &val, 1);
            if (ret < 0)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
                return ret;
            }
            _readBits += ret;
            if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ clock_timestamp_flag[%d] = %d", i, val);
            
            if (val)
            {
                // ct_type
                ret = readBits(parser, 2, &val, 1);
                if (ret < 0)
                {
                    ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
                    return ret;
                }
                _readBits += ret;
                if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ ct_type = %d", val);

                // nuit_field_based_flag
                ret = readBits(parser, 1, &val, 1);
                if (ret < 0)
                {
                    ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
                    return ret;
                }
                _readBits += ret;
                nuit_field_based_flag = val;
                if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ nuit_field_based_flag = %d", val);

                // counting_type
                ret = readBits(parser, 5, &val, 1);
                if (ret < 0)
                {
                    ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
                    return ret;
                }
                _readBits += ret;
                if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ counting_type = %d", val);

                // full_timestamp_flag
                ret = readBits(parser, 1, &val, 1);
                if (ret < 0)
                {
                    ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
                    return ret;
                }
                _readBits += ret;
                full_timestamp_flag = val;
                if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ full_timestamp_flag = %d", val);

                // discontinuity_flag
                ret = readBits(parser, 1, &val, 1);
                if (ret < 0)
                {
                    ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
                    return ret;
                }
                _readBits += ret;
                if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ discontinuity_flag = %d", val);

                // cnt_dropped_flag
                ret = readBits(parser, 1, &val, 1);
                if (ret < 0)
                {
                    ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
                    return ret;
                }
                _readBits += ret;
                if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ cnt_dropped_flag = %d", val);

                // n_frames
                ret = readBits(parser, 8, &val, 1);
                if (ret < 0)
                {
                    ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
                    return ret;
                }
                _readBits += ret;
                nFrames = val;
                if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ n_frames = %d", val);
                
                if (full_timestamp_flag)
                {
                    // seconds_value
                    ret = readBits(parser, 6, &val, 1);
                    if (ret < 0)
                    {
                        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
                        return ret;
                    }
                    _readBits += ret;
                    sS = val;
                    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ seconds_value = %d", val);

                    // minutes_value
                    ret = readBits(parser, 6, &val, 1);
                    if (ret < 0)
                    {
                        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
                        return ret;
                    }
                    _readBits += ret;
                    mM = val;
                    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ minutes_value = %d", val);

                    // hours_value
                    ret = readBits(parser, 5, &val, 1);
                    if (ret < 0)
                    {
                        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
                        return ret;
                    }
                    _readBits += ret;
                    hH = val;
                    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ hours_value = %d", val);
                }
                else
                {
                    // seconds_flag
                    ret = readBits(parser, 1, &val, 1);
                    if (ret < 0)
                    {
                        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
                        return ret;
                    }
                    _readBits += ret;
                    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ seconds_flag = %d", val);

                    if (val)
                    {
                        // seconds_value
                        ret = readBits(parser, 6, &val, 1);
                        if (ret < 0)
                        {
                            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
                            return ret;
                        }
                        _readBits += ret;
                        sS = val;
                        if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ seconds_value = %d", val);

                        // minutes_flag
                        ret = readBits(parser, 1, &val, 1);
                        if (ret < 0)
                        {
                            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
                            return ret;
                        }
                        _readBits += ret;
                        if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ minutes_flag = %d", val);

                        if (val)
                        {
                            // minutes_value
                            ret = readBits(parser, 6, &val, 1);
                            if (ret < 0)
                            {
                                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
                                return ret;
                            }
                            _readBits += ret;
                            mM = val;
                            if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ minutes_value = %d", val);

                            // hours_flag
                            ret = readBits(parser, 1, &val, 1);
                            if (ret < 0)
                            {
                                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
                                return ret;
                            }
                            _readBits += ret;
                            if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ hours_flag = %d", val);

                            if (val)
                            {
                                // hours_value
                                ret = readBits(parser, 5, &val, 1);
                                if (ret < 0)
                                {
                                    ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
                                    return ret;
                                }
                                _readBits += ret;
                                hH = val;
                                if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ hours_value = %d", val);
                            }
                        }
                    }
                }

                if (parser->spsContext.time_offset_length)
                {
                    // time_offset
                    ret = readBits(parser, parser->spsContext.time_offset_length, &val, 1); //TODO: signed value
                    if (ret < 0)
                    {
                        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
                        return ret;
                    }
                    _readBits += ret;
                    tOffset = val;
                    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ time_offset = %d", val);
                }
            }
            clockTimestamp = (((uint64_t)hH * 60 + (uint64_t)mM) * 60 + (uint64_t)sS) * parser->spsContext.time_scale
                             + (uint64_t)nFrames * (parser->spsContext.num_units_in_tick * (1 + nuit_field_based_flag)) + (uint64_t)tOffset;
            if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ clockTimestamp = %" PRIu64, clockTimestamp);
        }
    }

    return _readBits;
}


static int ARSTREAM2_H264Parser_ParseSei(ARSTREAM2_H264Parser_t* parser)
{
    int ret = 0;
    uint32_t val = 0;
    int readBytes = 0, _readBits = 0, _readBits2;
    int payloadType, payloadSize;

    parser->hasRecoveryPoint = 0;

    // sei_rbsp
    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "-- sei_rbsp()");
    
    parser->userDataCount = 0;
    memset(parser->userDataSize, 0, sizeof(parser->userDataSize));
    
    do
    {
        // sei_message
        
        payloadType = 0;
        payloadSize = 0;
        _readBits2 = 0;
        
        // last_payload_type_byte
        do
        {
            ret = readBits(parser, 8, &val, 1);
            if (ret < 0)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
                return ret;
            }
            _readBits += ret;
            payloadType += val;
        }
        while (val == 0xFF);
        if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "---- last_payload_type_byte = %d (payloadType = %d)", val, payloadType);

        // last_payload_size_byte
        do
        {
            ret = readBits(parser, 8, &val, 1);
            if (ret < 0)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
                return ret;
            }
            _readBits += ret;
            payloadSize += val;
        }
        while (val == 0xFF);
        if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "---- last_payload_size_byte = %d (payloadSize = %d)", val, payloadSize);
        
        // sei_payload
        switch(payloadType)
        {
            case ARSTREAM2_H264_SEI_PAYLOAD_TYPE_BUFFERING_PERIOD:
                ret = ARSTREAM2_H264Parser_ParseSeiPayload_bufferingPeriod(parser, payloadSize);
                if (ret < 0)
                {
                    ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "ARSTREAM2_H264Parser_ParseSeiPayload_bufferingPeriod() failed (%d)", ret);
                    return ret;
                }
                _readBits2 += ret;
                break;
            case ARSTREAM2_H264_SEI_PAYLOAD_TYPE_PIC_TIMING:
                ret = ARSTREAM2_H264Parser_ParseSeiPayload_picTiming(parser, payloadSize);
                if (ret < 0)
                {
                    ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "ARSTREAM2_H264Parser_ParseSeiPayload_picTiming() failed (%d)", ret);
                    return ret;
                }
                _readBits2 += ret;
                break;
            case ARSTREAM2_H264_SEI_PAYLOAD_TYPE_RECOVERY_POINT:
                ret = ARSTREAM2_H264Parser_ParseSeiPayload_recoveryPoint(parser, payloadSize);
                if (ret < 0)
                {
                    ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "ARSTREAM2_H264Parser_ParseSeiPayload_recoveryPoint() failed (%d)", ret);
                    return ret;
                }
                _readBits2 += ret;
                break;
            case ARSTREAM2_H264_SEI_PAYLOAD_TYPE_USER_DATA_UNREGISTERED:
                ret = ARSTREAM2_H264Parser_ParseSeiPayload_userDataUnregistered(parser, payloadSize);
                if (ret < 0)
                {
                    ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "ARSTREAM2_H264Parser_ParseSeiPayload_userDataUnregistered() failed (%d)", ret);
                    return ret;
                }
                _readBits2 += ret;
                break;
            default:
                if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "---- unsupported payload type (skipping)");
                ret = skipBytes(parser, payloadSize);
                if (ret < 0)
                {
                    ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "skipBytes() failed (%d)", ret);
                    return ret;
                }
                _readBits2 += ret * 8;
                break;
        }

        // If not byte-aligned, read '1' bit and then align to byte
        if (parser->cacheLength & 7)
        {
            ret = readBits(parser, 1, &val, 1);
            if (ret < 0)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
                return ret;
            }
            _readBits2 += ret;
        }

        ret = bitstreamByteAlign(parser);
        if (ret < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to align the bitstream (%d)", ret);
            return ret;
        }
        _readBits2 += ret;

        if (_readBits2 != payloadSize * 8)
        {
            if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "---- warning: read bits != payload size (%d != %d)", _readBits2, payloadSize * 8);
        }
        if (_readBits2 < payloadSize * 8)
        {
            // Skip what we should have read
            ret = skipBytes(parser, payloadSize - (_readBits2 / 8));
            if (ret < 0)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "skipBytes() failed (%d)", ret);
                return ret;
            }
            _readBits2 += ret * 8;
        }
        
        _readBits += _readBits2;
    }
    while (moreRbspData(parser));

    // rbsp_trailing_bits
    ret = readBits(parser, 1, &val, 1);
    if (ret < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
        return ret;
    }
    _readBits += ret;

    ret = bitstreamByteAlign(parser);
    if (ret < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to align the bitstream (%d)", ret);
        return ret;
    }
    _readBits += ret;
    readBytes += _readBits / 8;

    return readBytes;
}


static int ARSTREAM2_H264Parser_ParseAud(ARSTREAM2_H264Parser_t* parser)
{
    int ret = 0;
    uint32_t val = 0;
    int readBytes = 0, _readBits = 0;

    // access_unit_delimiter_rbsp
    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "-- access_unit_delimiter_rbsp()");

    // primary_pic_type
    ret = readBits(parser, 3, &val, 1);
    if (ret < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
        return ret;
    }
    _readBits += ret;
    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "---- primary_pic_type = %d", val);

    // rbsp_trailing_bits
    ret = readBits(parser, 1, &val, 1);
    if (ret < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
        return ret;
    }
    _readBits += ret;

    ret = bitstreamByteAlign(parser);
    if (ret < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to align the bitstream (%d)", ret);
        return ret;
    }
    _readBits += ret;
    readBytes += _readBits / 8;

    return readBytes;
}


static int ARSTREAM2_H264Parser_ParseFillerData(ARSTREAM2_H264Parser_t* parser)
{
    int readBytes = 0;

    // filler_data_rbsp
    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "-- filler_data_rbsp()");

    //TODO

    return readBytes;
}


static int ARSTREAM2_H264Parser_ParseRefPicListModification(ARSTREAM2_H264Parser_t* parser)
{
    int ret = 0;
    unsigned int i;
    uint32_t val = 0;
    int _readBits = 0;
    int modification_of_pic_nums_idc;

    // ref_pic_list_modification
    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ ref_pic_list_modification()");

    if ((parser->sliceContext.sliceTypeMod5 != ARSTREAM2_H264_SLICE_TYPE_I) && (parser->sliceContext.sliceTypeMod5 != ARSTREAM2_H264_SLICE_TYPE_SI))
    {
        // ref_pic_list_modification_flag_l0
        ret = readBits(parser, 1, &val, 1);
        if (ret < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
            return ret;
        }
        _readBits += ret;
        parser->sliceContext.ref_pic_list_modification_flag_l0 = val;
        if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "-------- ref_pic_list_modification_flag_l0 = %d", val);
        
        if (val)
        {
            i = 0;
            do
            {
                // modification_of_pic_nums_idc
                ret = readBits_expGolomb_ue(parser, &val, 1);
                if (ret < 0)
                {
                    ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
                    return ret;
                }
                _readBits += ret;
                modification_of_pic_nums_idc = val;
                if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "-------- modification_of_pic_nums_idc = %d", val);

                if ((modification_of_pic_nums_idc == 0) || (modification_of_pic_nums_idc == 1))
                {
                    // abs_diff_pic_num_minus1
                    ret = readBits_expGolomb_ue(parser, &val, 1);
                    if (ret < 0)
                    {
                        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
                        return ret;
                    }
                    _readBits += ret;
                    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "-------- abs_diff_pic_num_minus1 = %d", val);
                }
                else if (modification_of_pic_nums_idc == 2)
                {
                    // long_term_pic_num
                    ret = readBits_expGolomb_ue(parser, &val, 1);
                    if (ret < 0)
                    {
                        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
                        return ret;
                    }
                    _readBits += ret;
                    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "-------- long_term_pic_num = %d", val);
                }
                i++;
            }
            while ((modification_of_pic_nums_idc != 3) && (i < parser->sliceContext.num_ref_idx_l0_active_minus1 + 1));
        }
    }

    if (parser->sliceContext.sliceTypeMod5 == ARSTREAM2_H264_SLICE_TYPE_B)
    {
        // ref_pic_list_modification_flag_l1
        ret = readBits(parser, 1, &val, 1);
        if (ret < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
            return ret;
        }
        _readBits += ret;
        parser->sliceContext.ref_pic_list_modification_flag_l1 = val;
        if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "-------- ref_pic_list_modification_flag_l1 = %d", val);
        
        if (val)
        {
            do
            {
                i = 0;
                // modification_of_pic_nums_idc
                ret = readBits_expGolomb_ue(parser, &val, 1);
                if (ret < 0)
                {
                    ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
                    return ret;
                }
                _readBits += ret;
                modification_of_pic_nums_idc = val;
                if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "-------- modification_of_pic_nums_idc = %d", val);

                if ((modification_of_pic_nums_idc == 0) || (modification_of_pic_nums_idc == 1))
                {
                    // abs_diff_pic_num_minus1
                    ret = readBits_expGolomb_ue(parser, &val, 1);
                    if (ret < 0)
                    {
                        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
                        return ret;
                    }
                    _readBits += ret;
                    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "-------- abs_diff_pic_num_minus1 = %d", val);
                }
                else if (modification_of_pic_nums_idc == 2)
                {
                    // long_term_pic_num
                    ret = readBits_expGolomb_ue(parser, &val, 1);
                    if (ret < 0)
                    {
                        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
                        return ret;
                    }
                    _readBits += ret;
                    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "-------- long_term_pic_num = %d", val);
                }
            }
            while ((modification_of_pic_nums_idc != 3) && (i < parser->sliceContext.num_ref_idx_l1_active_minus1 + 1));
        }
    }

    return _readBits;
}


static int ARSTREAM2_H264Parser_ParsePredWeightTable(ARSTREAM2_H264Parser_t* parser)
{
    int ret = 0;
    uint32_t val = 0;
    int _readBits = 0;

    // pred_weight_table
    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ pred_weight_table()");

    // luma_log2_weight_denom
    ret = readBits_expGolomb_ue(parser, &val, 1);
    if (ret < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
        return ret;
    }
    _readBits += ret;
    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "-------- luma_log2_weight_denom = %d", val);

    //TODO
    return -1;
}


static int ARSTREAM2_H264Parser_ParseDecRefPicMarking(ARSTREAM2_H264Parser_t* parser)
{
    int ret = 0;
    uint32_t val = 0;
    int _readBits = 0;

    // dec_ref_pic_marking
    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ dec_ref_pic_marking()");

    if (parser->sliceContext.idrPicFlag)
    {
        // no_output_of_prior_pics_flag
        ret = readBits(parser, 1, &val, 1);
        if (ret < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
            return ret;
        }
        _readBits += ret;
        parser->sliceContext.no_output_of_prior_pics_flag = val;
        if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "-------- no_output_of_prior_pics_flag = %d", val);

        // long_term_reference_flag
        ret = readBits(parser, 1, &val, 1);
        if (ret < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
            return ret;
        }
        _readBits += ret;
        parser->sliceContext.long_term_reference_flag = val;
        if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "-------- long_term_reference_flag = %d", val);
    }
    else
    {
        // adaptive_ref_pic_marking_mode_flag
        ret = readBits(parser, 1, &val, 1);
        if (ret < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
            return ret;
        }
        _readBits += ret;
        parser->sliceContext.adaptive_ref_pic_marking_mode_flag = val;
        if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "-------- adaptive_ref_pic_marking_mode_flag = %d", val);
        
        if (val)
        {
            int memory_management_control_operation = 0;
            
            do
            {
                // memory_management_control_operation
                ret = readBits_expGolomb_ue(parser, &val, 1);
                if (ret < 0)
                {
                    ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
                    return ret;
                }
                _readBits += ret;
                memory_management_control_operation = val;
                if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "-------- memory_management_control_operation = %d", val);
                
                if ((memory_management_control_operation == 1) || (memory_management_control_operation == 3))
                {
                    // difference_of_pic_nums_minus1
                    ret = readBits_expGolomb_ue(parser, &val, 1);
                    if (ret < 0)
                    {
                        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
                        return ret;
                    }
                    _readBits += ret;
                    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "-------- difference_of_pic_nums_minus1 = %d", val);
                }

                if (memory_management_control_operation == 2)
                {
                    // long_term_pic_num
                    ret = readBits_expGolomb_ue(parser, &val, 1);
                    if (ret < 0)
                    {
                        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
                        return ret;
                    }
                    _readBits += ret;
                    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "-------- long_term_pic_num = %d", val);
                }

                if ((memory_management_control_operation == 3) || (memory_management_control_operation == 6))
                {
                    // long_term_frame_idx
                    ret = readBits_expGolomb_ue(parser, &val, 1);
                    if (ret < 0)
                    {
                        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
                        return ret;
                    }
                    _readBits += ret;
                    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "-------- long_term_frame_idx = %d", val);
                }

                if (memory_management_control_operation == 4)
                {
                    // max_long_term_frame_idx_plus1
                    ret = readBits_expGolomb_ue(parser, &val, 1);
                    if (ret < 0)
                    {
                        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
                        return ret;
                    }
                    _readBits += ret;
                    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "-------- max_long_term_frame_idx_plus1 = %d", val);
                }
            }
            while (memory_management_control_operation != 0);
        }
    }

    return _readBits;
}


static int ARSTREAM2_H264Parser_ParseSlice(ARSTREAM2_H264Parser_t* parser)
{
    int ret = 0;
    uint32_t val = 0;
    int32_t val_se = 0;
    int readBytes = 0, _readBits = 0;

    if ((!parser->spsSync) || (!parser->ppsSync))
    {
        return readBytes;
    }

    // slice_layer_without_partitioning_rbsp
    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "-- slice_layer_without_partitioning_rbsp()");

    // slice_header
    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "---- slice_header()");

    // first_mb_in_slice
    ret = readBits_expGolomb_ue(parser, &val, 1);
    if (ret < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
        return ret;
    }
    _readBits += ret;
    parser->sliceContext.first_mb_in_slice = val;
    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ first_mb_in_slice = %d", val);

    // slice_type
    ret = readBits_expGolomb_ue(parser, &val, 1);
    if (ret < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
        return ret;
    }
    _readBits += ret;
    parser->sliceContext.slice_type = val;
    parser->sliceContext.sliceTypeMod5 = val % 5;
    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ slice_type = %d (%s)", val, (val <= 9) ? ARSTREAM2_H264Parser_sliceTypeStr[val] : "(invalid)");

    // pic_parameter_set_id
    ret = readBits_expGolomb_ue(parser, &val, 1);
    if (ret < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
        return ret;
    }
    _readBits += ret;
    parser->sliceContext.pic_parameter_set_id = val;
    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ pic_parameter_set_id = %d", val);

    if (parser->spsContext.separate_colour_plane_flag == 1)
    {
        // colour_plane_id
        ret = readBits(parser, 2, &val, 1);
        if (ret < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
            return ret;
        }
        _readBits += ret;
        parser->sliceContext.colour_plane_id = val;
        if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ colour_plane_id = %d", val);
    }

    // frame_num
    ret = readBits(parser, parser->spsContext.log2_max_frame_num_minus4 + 4, &val, 1);
    if (ret < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
        return ret;
    }
    _readBits += ret;
    parser->sliceContext.frame_num = val;
    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ frame_num = %d", val);

    if (!parser->spsContext.frame_mbs_only_flag)
    {
        // field_pic_flag
        ret = readBits(parser, 1, &val, 1);
        if (ret < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
            return ret;
        }
        _readBits += ret;
        parser->sliceContext.field_pic_flag = val;
        if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ field_pic_flag = %d", val);

        if (parser->sliceContext.field_pic_flag)
        {
            // bottom_field_flag
            ret = readBits(parser, 1, &val, 1);
            if (ret < 0)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
                return ret;
            }
            _readBits += ret;
            parser->sliceContext.bottom_field_flag = val;
            if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ bottom_field_flag = %d", val);
        }
    }

    if (parser->sliceContext.idrPicFlag)
    {
        // idr_pic_id
        ret = readBits_expGolomb_ue(parser, &val, 1);
        if (ret < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
            return ret;
        }
        _readBits += ret;
        parser->sliceContext.idr_pic_id = val;
        if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ idr_pic_id = %d", val);
    }

    if (parser->spsContext.pic_order_cnt_type == 0)
    {
        // pic_order_cnt_lsb
        ret = readBits(parser, parser->spsContext.log2_max_pic_order_cnt_lsb_minus4 + 4, &val, 1);
        if (ret < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
            return ret;
        }
        _readBits += ret;
        parser->sliceContext.pic_order_cnt_lsb = val;
        if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ pic_order_cnt_lsb = %d", val);

        if ((parser->ppsContext.bottom_field_pic_order_in_frame_present_flag) && (!parser->sliceContext.field_pic_flag))
        {
            // delta_pic_order_cnt_bottom
            ret = readBits_expGolomb_se(parser, &val_se, 1);
            if (ret < 0)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
                return ret;
            }
            _readBits += ret;
            parser->sliceContext.delta_pic_order_cnt_bottom = val_se;
            if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ delta_pic_order_cnt_bottom = %d", val_se);
        }
    }

    if ((parser->spsContext.pic_order_cnt_type == 1) && (!parser->spsContext.delta_pic_order_always_zero_flag))
    {
        // delta_pic_order_cnt[0]
        ret = readBits_expGolomb_se(parser, &val_se, 1);
        if (ret < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
            return ret;
        }
        _readBits += ret;
        parser->sliceContext.delta_pic_order_cnt_0 = val_se;
        if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ delta_pic_order_cnt[0] = %d", val_se);

        if ((parser->ppsContext.bottom_field_pic_order_in_frame_present_flag) && (!parser->sliceContext.field_pic_flag))
        {
            // delta_pic_order_cnt[1]
            ret = readBits_expGolomb_se(parser, &val_se, 1);
            if (ret < 0)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
                return ret;
            }
            _readBits += ret;
            parser->sliceContext.delta_pic_order_cnt_1 = val_se;
            if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ delta_pic_order_cnt[1] = %d", val_se);
        }
    }

    if (parser->ppsContext.redundant_pic_cnt_present_flag)
    {
        // redundant_pic_cnt
        ret = readBits_expGolomb_ue(parser, &val, 1);
        if (ret < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
            return ret;
        }
        _readBits += ret;
        parser->sliceContext.redundant_pic_cnt = val;
        if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ redundant_pic_cnt = %d", val);
    }
    
    if (parser->sliceContext.sliceTypeMod5 == ARSTREAM2_H264_SLICE_TYPE_B)
    {
        // direct_spatial_mv_pred_flag
        ret = readBits(parser, 1, &val, 1);
        if (ret < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
            return ret;
        }
        _readBits += ret;
        parser->sliceContext.direct_spatial_mv_pred_flag = val;
        if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ direct_spatial_mv_pred_flag = %d", val);
    }
    
    if ((parser->sliceContext.sliceTypeMod5 == ARSTREAM2_H264_SLICE_TYPE_P) || (parser->sliceContext.sliceTypeMod5 == ARSTREAM2_H264_SLICE_TYPE_SP) || (parser->sliceContext.sliceTypeMod5 == ARSTREAM2_H264_SLICE_TYPE_B))
    {
        // num_ref_idx_active_override_flag
        ret = readBits(parser, 1, &val, 1);
        if (ret < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
            return ret;
        }
        parser->sliceContext.num_ref_idx_active_override_flag = val;
        _readBits += ret;
        if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ num_ref_idx_active_override_flag = %d", val);
        
        if (val)
        {
            // num_ref_idx_l0_active_minus1
            ret = readBits_expGolomb_ue(parser, &val, 1);
            if (ret < 0)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
                return ret;
            }
            _readBits += ret;
            parser->sliceContext.num_ref_idx_l0_active_minus1 = val;
            if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ num_ref_idx_l0_active_minus1 = %d", val);

            if (parser->sliceContext.sliceTypeMod5 == ARSTREAM2_H264_SLICE_TYPE_B)
            {
                // num_ref_idx_l1_active_minus1
                ret = readBits_expGolomb_ue(parser, &val, 1);
                if (ret < 0)
                {
                    ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
                    return ret;
                }
                _readBits += ret;
                parser->sliceContext.num_ref_idx_l1_active_minus1 = val;
                if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ num_ref_idx_l1_active_minus1 = %d", val);
            }
        }
    }
    
    if ((parser->sliceContext.nal_unit_type == 20) || (parser->sliceContext.nal_unit_type == 21))
    {    
        // ref_pic_list_mvc_modification()
        //TODO
        return -1;
    }
    else
    {
        // ref_pic_list_modification()
        ret = ARSTREAM2_H264Parser_ParseRefPicListModification(parser);
        if (ret < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "ARSTREAM2_H264Parser_ParseRefPicListModification() failed (%d)", ret);
            return ret;
        }
        _readBits += ret;
    }
    
    
    if ((parser->ppsContext.weighted_pred_flag && ((parser->sliceContext.sliceTypeMod5 == ARSTREAM2_H264_SLICE_TYPE_P) || (parser->sliceContext.sliceTypeMod5 == ARSTREAM2_H264_SLICE_TYPE_SP))) 
            || ((parser->ppsContext.weighted_bipred_idc == 1) && (parser->sliceContext.sliceTypeMod5 == ARSTREAM2_H264_SLICE_TYPE_B)))
    {
        // pred_weight_table()
        ret = ARSTREAM2_H264Parser_ParsePredWeightTable(parser);
        if (ret < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "ARSTREAM2_H264Parser_ParsePredWeightTable() failed (%d)", ret);
            return ret;
        }
        _readBits += ret;
    }
    
    if (parser->sliceContext.nal_ref_idc != 0)
    {
        // dec_ref_pic_marking()
        ret = ARSTREAM2_H264Parser_ParseDecRefPicMarking(parser);
        if (ret < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "ARSTREAM2_H264Parser_ParseDecRefPicMarking() failed (%d)", ret);
            return ret;
        }
        _readBits += ret;
    }
    
    if ((parser->ppsContext.entropy_coding_mode_flag) && (parser->sliceContext.sliceTypeMod5 != ARSTREAM2_H264_SLICE_TYPE_I) && (parser->sliceContext.sliceTypeMod5 != ARSTREAM2_H264_SLICE_TYPE_SI))
    {
        // cabac_init_idc
        ret = readBits_expGolomb_ue(parser, &val, 1);
        if (ret < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
            return ret;
        }
        _readBits += ret;
        parser->sliceContext.cabac_init_idc = val;
        if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ cabac_init_idc = %d", val);
    }
    
    // slice_qp_delta
    ret = readBits_expGolomb_se(parser, &val_se, 1);
    if (ret < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
        return ret;
    }
    _readBits += ret;
    parser->sliceContext.slice_qp_delta = val_se;
    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ slice_qp_delta = %d", val_se);

    if ((parser->sliceContext.sliceTypeMod5 == ARSTREAM2_H264_SLICE_TYPE_SP) || (parser->sliceContext.sliceTypeMod5 == ARSTREAM2_H264_SLICE_TYPE_SI))
    {
        if (parser->sliceContext.sliceTypeMod5 == ARSTREAM2_H264_SLICE_TYPE_SP)
        {
            // sp_for_switch_flag
            ret = readBits(parser, 1, &val, 1);
            if (ret < 0)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
                return ret;
            }
            _readBits += ret;
            parser->sliceContext.sp_for_switch_flag = val;
            if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ sp_for_switch_flag = %d", val);
        }

        // slice_qs_delta
        ret = readBits_expGolomb_se(parser, &val_se, 1);
        if (ret < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
            return ret;
        }
        _readBits += ret;
        parser->sliceContext.slice_qs_delta = val_se;
        if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ slice_qs_delta = %d", val_se);
    }
    
    if (parser->ppsContext.deblocking_filter_control_present_flag)
    {
        // disable_deblocking_filter_idc
        ret = readBits_expGolomb_ue(parser, &val, 1);
        if (ret < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
            return ret;
        }
        _readBits += ret;
        parser->sliceContext.disable_deblocking_filter_idc = val;
        if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ disable_deblocking_filter_idc = %d", val);
        
        if (val != 1)
        {
            // slice_alpha_c0_offset_div2
            ret = readBits_expGolomb_se(parser, &val_se, 1);
            if (ret < 0)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
                return ret;
            }
            _readBits += ret;
            parser->sliceContext.slice_alpha_c0_offset_div2 = val_se;
            if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ slice_alpha_c0_offset_div2 = %d", val_se);

            // slice_beta_offset_div2
            ret = readBits_expGolomb_se(parser, &val_se, 1);
            if (ret < 0)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
                return ret;
            }
            _readBits += ret;
            parser->sliceContext.slice_beta_offset_div2 = val_se;
            if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ slice_beta_offset_div2 = %d", val_se);
        }
    }
    
    if ((parser->ppsContext.num_slice_groups_minus1 > 0) && (parser->ppsContext.slice_group_map_type >= 3) && (parser->ppsContext.slice_group_map_type <= 5))
    {
        int picSizeInMapUnits, n;

        picSizeInMapUnits = (parser->spsContext.pic_width_in_mbs_minus1 + 1) * (parser->spsContext.pic_height_in_map_units_minus1 + 1);
        n = ceil(log2((picSizeInMapUnits / (parser->ppsContext.slice_group_change_rate_minus1 + 1)) + 1));

        // slice_group_change_cycle
        ret = readBits(parser, n, &val, 1);
        if (ret < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
            return ret;
        }
        _readBits += ret;
        parser->sliceContext.slice_group_change_cycle = val;
        if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "------ slice_group_change_cycle = %d", val);
    }

    parser->sliceContext.sliceHeaderLengthInBits = _readBits;

    // rbsp_slice_trailing_bits

    // rbsp_trailing_bits
    ret = readBits(parser, 1, &val, 1);
    if (ret < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
        return ret;
    }
    _readBits += ret;

    ret = bitstreamByteAlign(parser);
    if (ret < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to align the bitstream (%d)", ret);
        return ret;
    }
    _readBits += ret;
    readBytes += _readBits / 8;

    //TODO: cabac_zero_word

    return readBytes;
}


eARSTREAM2_ERROR ARSTREAM2_H264Parser_ParseNalu(ARSTREAM2_H264Parser_Handle parserHandle, unsigned int* readBytes)
{
    ARSTREAM2_H264Parser_t* parser = (ARSTREAM2_H264Parser_t*)parserHandle;
    uint32_t val;
    int ret;
    int _readBytes = 0;
    int forbidden_zero_bit;

    if (!parserHandle)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Invalid handle");
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }

    memset(&parser->sliceContext, 0, sizeof(ARSTREAM2_H264_SliceContext_t));

    ret = readBits(parser, 8, &val, 0);
    if (ret != 8)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read from the bitstream");
        return ARSTREAM2_ERROR_INVALID_STATE;
    }
    _readBytes++;

    forbidden_zero_bit = (val >> 7) & 0x1;
    parser->sliceContext.nal_ref_idc = (val >> 5) & 0x3;
    parser->sliceContext.nal_unit_type = val & 0x1F;

    if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "-- NALU found: nal_ref_idc=%d, nal_unit_type=%d (%s)", parser->sliceContext.nal_ref_idc, parser->sliceContext.nal_unit_type, ARSTREAM2_H264Parser_naluTypeStr[parser->sliceContext.nal_unit_type]);

    parser->sliceContext.idrPicFlag = (parser->sliceContext.nal_unit_type == 5) ? 1 : 0;
    
    if (ARSTREAM2_H264Parser_ParseNaluType[parser->sliceContext.nal_unit_type])
    {
        ret = ARSTREAM2_H264Parser_ParseNaluType[parser->sliceContext.nal_unit_type](parser);
        if (ret < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "ARSTREAM2_H264Parser_ParseNaluType[%d]() failed (%d)", parser->sliceContext.nal_unit_type, ret);
            return ARSTREAM2_ERROR_INVALID_STATE;
        }
        _readBytes += ret;
    }
    
    if (forbidden_zero_bit != 0)
    {
        if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "   Warning: forbidden_zero_bit is not 0!");
    }

    if (readBytes) *readBytes = (unsigned int)_readBytes;

    return ARSTREAM2_OK;
}


static int ARSTREAM2_H264Parser_StartcodeMatch_file(ARSTREAM2_H264Parser_t* parser, FILE* fp, off_t fileSize, off_t *startcodePosition)
{
    int ret;
    off_t initPos, pos, end, i = 0;
    uint32_t val, shiftVal;

    pos = ftello(fp);
    if (pos < 0)
    {
        return -1;
    }
    initPos = pos;
    end = fileSize;

    if (pos + 4 > end) return -2;

    ret = fread(&val, 4, 1, fp);
    if (ret != 1) return -1;
    shiftVal = val = ntohl(val);

    while ((shiftVal != ARSTREAM2_H264_BYTE_STREAM_NALU_START_CODE) && (pos < end))
    {
        if ((!i) && (pos < end - 4))
        {
            i = 4;

            ret = fread(&val, 4, 1, fp);
            if (ret != 1)
            {
                return -1;
            }
            val = ntohl(val);
        }

        shiftVal <<= 8;
        shiftVal |= (val >> 24) & 0xFF;
        val <<= 8;

        pos++;
        i--;
    }

    if (shiftVal == ARSTREAM2_H264_BYTE_STREAM_NALU_START_CODE)
    {
        pos += 4;
        ret = fseeko(fp, pos, SEEK_SET);
        if (ret != 0) return -1;
        ret = 0;
        if (startcodePosition) *startcodePosition = pos - 4;
    }
    else
    {
        ret = fseeko(fp, initPos, SEEK_SET);
        if (ret != 0) return -1;
        ret = -2;
    }

    return ret;
}



eARSTREAM2_ERROR ARSTREAM2_H264Parser_ReadNextNalu_file(ARSTREAM2_H264Parser_Handle parserHandle, FILE* fp, unsigned long long fileSize, unsigned int *naluSize)
{
    ARSTREAM2_H264Parser_t* parser = (ARSTREAM2_H264Parser_t*)parserHandle;
    int ret = 0;
    off_t naluStart, naluEnd, startcodePosition = 0;
    unsigned int _naluSize = 0;

    if (!parserHandle)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Invalid handle");
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }
    
    // Search for next NALU start code
    ret = ARSTREAM2_H264Parser_StartcodeMatch_file(parser, fp, fileSize, &startcodePosition);
    if (ret >= 0)
    {
        // Start code found
        naluStart = startcodePosition + 4;
        if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "Start code at 0x%08X", (uint32_t)(startcodePosition));

        // Search for NALU end (next NALU start code or end of file)
        ret = ARSTREAM2_H264Parser_StartcodeMatch_file(parser, fp, fileSize, &startcodePosition);
        if (ret >= 0)
        {
            // Start code found
            naluEnd = startcodePosition;
        }
        else if (ret == -2)
        {
            // No start code found
            naluEnd = fileSize;
        }
        else //if (ret < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "ARSTREAM2_H264Parser_StartcodeMatch_file() failed (%d)", ret);
            return ARSTREAM2_ERROR_INVALID_STATE;
        }
        
        _naluSize = (unsigned int)(naluEnd - naluStart);
        if (_naluSize > 0)
        {
            ret = fseeko(fp, naluStart, SEEK_SET);
            if (ret != 0)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to seek in file");
                return ARSTREAM2_ERROR_INVALID_STATE;
            }
            
            parser->naluBufManaged = 1;
            if (_naluSize > parser->naluBufSize)
            {
                parser->naluBufSize = _naluSize;
                parser->pNaluBuf = (uint8_t*)realloc(parser->pNaluBuf, parser->naluBufSize);
                if (!parser->pNaluBuf)
                {
                    ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Reallocation failed (size %d)", parser->naluBufSize);
                    return ARSTREAM2_ERROR_ALLOC;
                }
            }
            
            ret = fread(parser->pNaluBuf, _naluSize, 1, fp);
            if (ret != 1)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Failed to read %d bytes in file", _naluSize);
                return ARSTREAM2_ERROR_INVALID_STATE;
            }
            parser->naluSize = _naluSize;
            parser->remNaluSize = _naluSize;
            parser->pNaluBufCur = parser->pNaluBuf;

            // Reset the cache
            parser->cache = 0;
            parser->cacheLength = 0;
            parser->oldZeroCount = 0; // NB: this value is wrong when emulation prevention is in use (inside NAL Units)
        }
        else
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Invalid NALU size");
            return ARSTREAM2_ERROR_INVALID_STATE;
        }
    }
    else if (ret == -2)
    {
        // No start code found
        if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "No start code found");
        return ARSTREAM2_ERROR_NOT_FOUND;
    }
    else //if (ret < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "ARSTREAM2_H264Parser_StartcodeMatch_file() failed (%d)", ret);
        return ARSTREAM2_ERROR_INVALID_STATE;
    }

    if (naluSize) *naluSize = _naluSize;
    return ARSTREAM2_OK;
}


static int ARSTREAM2_H264Parser_StartcodeMatch_buffer(ARSTREAM2_H264Parser_t* parser, uint8_t* pBuf, unsigned int bufSize)
{
    int ret, pos, end;
    uint32_t shiftVal = 0;
    uint8_t* ptr = pBuf;

    if (bufSize < 4) return -2;

    pos = 0;
    end = bufSize;

    do
    {
        shiftVal <<= 8;
        shiftVal |= (*ptr++) & 0xFF;
        pos++;
    }
    while (((shiftVal != ARSTREAM2_H264_BYTE_STREAM_NALU_START_CODE) && (pos < end)) || (pos < 4));

    if (shiftVal == ARSTREAM2_H264_BYTE_STREAM_NALU_START_CODE)
    {
        ret = pos;
    }
    else
    {
        ret = -2;
    }

    return ret;
}


eARSTREAM2_ERROR ARSTREAM2_H264Parser_ReadNextNalu_buffer(ARSTREAM2_H264Parser_Handle parserHandle, void* pBuf, unsigned int bufSize, unsigned int* naluStartPos, unsigned int* nextStartCodePos)
{
    ARSTREAM2_H264Parser_t* parser = (ARSTREAM2_H264Parser_t*)parserHandle;
    int ret = 0;
    unsigned int naluStart, naluEnd, naluSize;

    if (!parserHandle)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Invalid handle");
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }
    
    if (parser->naluBufManaged)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Invalid state");
        return ARSTREAM2_ERROR_INVALID_STATE;
    }

    if (naluStartPos) *naluStartPos = 0;
    if (nextStartCodePos) *nextStartCodePos = 0;

    // Search for next NALU start code
    ret = ARSTREAM2_H264Parser_StartcodeMatch_buffer(parser, (uint8_t*)pBuf, bufSize);
    if (ret >= 0)
    {
        // Start code found
        naluStart = ret;
        if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "Start code at 0x%08X", (uint32_t)(naluStart - 4));

        // Search for NALU end (next NALU start code or end of file)
        ret = ARSTREAM2_H264Parser_StartcodeMatch_buffer(parser, (uint8_t*)pBuf + naluStart, bufSize - naluStart);
        if (ret >= 0)
        {
            // Start code found
            naluEnd = naluStart + ret - 4;
            if (nextStartCodePos) *nextStartCodePos = naluEnd;
        }
        else if (ret == -2)
        {
            // No start code found
            naluEnd = bufSize;
        }
        else //if (ret < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "ARSTREAM2_H264Parser_StartcodeMatch_buffer() failed (%d)", ret);
            return ARSTREAM2_ERROR_INVALID_STATE;
        }

        naluSize = naluEnd - naluStart;
        if (naluSize > 0)
        {
            parser->naluSize = parser->remNaluSize = parser->naluBufSize = naluSize;
            parser->pNaluBufCur = parser->pNaluBuf = (uint8_t*)pBuf + naluStart;

            // Reset the cache
            parser->cache = 0;
            parser->cacheLength = 0;
            parser->oldZeroCount = 0; // NB: this value is wrong when emulation prevention is in use (inside NAL Units)
        }
        else
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Invalid NALU size");
            return ARSTREAM2_ERROR_INVALID_STATE;
        }

        if (naluStartPos) *naluStartPos = naluStart;
        return ARSTREAM2_OK;
    }
    else if (ret == -2)
    {
        // No start code found
        if (parser->config.printLogs) ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_PARSER_TAG, "No start code found");

        parser->naluSize = parser->remNaluSize = parser->naluBufSize = bufSize;
        parser->pNaluBufCur = parser->pNaluBuf = (uint8_t*)pBuf;

        // Reset the cache
        parser->cache = 0;
        parser->cacheLength = 0;
        parser->oldZeroCount = 0; // NB: this value is wrong when emulation prevention is in use (inside NAL Units)

        return ARSTREAM2_ERROR_NOT_FOUND;
    }
    else //if (ret < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "ARSTREAM2_H264Parser_StartcodeMatch_buffer() failed (%d)", ret);
        return ARSTREAM2_ERROR_INVALID_STATE;
    }

    return ARSTREAM2_ERROR_NOT_FOUND;
}


eARSTREAM2_ERROR ARSTREAM2_H264Parser_SetupNalu_buffer(ARSTREAM2_H264Parser_Handle parserHandle, void* pNaluBuf, unsigned int naluSize)
{
    ARSTREAM2_H264Parser_t* parser = (ARSTREAM2_H264Parser_t*)parserHandle;
    eARSTREAM2_ERROR ret = ARSTREAM2_OK;

    if (!parserHandle)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Invalid handle");
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }

    if (parser->naluBufManaged)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Invalid state");
        return ARSTREAM2_ERROR_INVALID_STATE;
    }

    parser->naluSize = parser->remNaluSize = parser->naluBufSize = naluSize;
    parser->pNaluBufCur = parser->pNaluBuf = (uint8_t*)pNaluBuf;

    // Reset the cache
    parser->cache = 0;
    parser->cacheLength = 0;
    parser->oldZeroCount = 0; // NB: this value is wrong when emulation prevention is in use (inside NAL Units)

    return ret;
}


uint8_t ARSTREAM2_H264Parser_GetLastNaluType(ARSTREAM2_H264Parser_Handle parserHandle)
{
    ARSTREAM2_H264Parser_t* parser = (ARSTREAM2_H264Parser_t*)parserHandle;

    if (!parserHandle)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Invalid handle");
        return 0;
    }

    return (uint8_t)parser->sliceContext.nal_unit_type;
}


eARSTREAM2_ERROR ARSTREAM2_H264Parser_GetSliceInfo(ARSTREAM2_H264Parser_Handle parserHandle, ARSTREAM2_H264Parser_SliceInfo_t* sliceInfo)
{
    ARSTREAM2_H264Parser_t* parser = (ARSTREAM2_H264Parser_t*)parserHandle;
    eARSTREAM2_ERROR ret = ARSTREAM2_OK;

    if (!parserHandle)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Invalid handle");
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }

    if (!sliceInfo)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Invalid pointer");
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }

    if ((parser->sliceContext.nal_unit_type != ARSTREAM2_H264_NALU_TYPE_SLICE) && (parser->sliceContext.nal_unit_type != ARSTREAM2_H264_NALU_TYPE_SLICE_IDR))
    {
        return ARSTREAM2_ERROR_NOT_FOUND;
    }

    sliceInfo->idrPicFlag = parser->sliceContext.idrPicFlag;
    sliceInfo->nal_ref_idc = parser->sliceContext.nal_ref_idc;
    sliceInfo->nal_unit_type = parser->sliceContext.nal_unit_type;
    sliceInfo->first_mb_in_slice = parser->sliceContext.first_mb_in_slice;
    sliceInfo->slice_type = parser->sliceContext.slice_type;
    sliceInfo->sliceTypeMod5 = parser->sliceContext.sliceTypeMod5;
    sliceInfo->frame_num = parser->sliceContext.frame_num;
    sliceInfo->idr_pic_id = parser->sliceContext.idr_pic_id;
    sliceInfo->slice_qp_delta = parser->sliceContext.slice_qp_delta;
    sliceInfo->disable_deblocking_filter_idc = parser->sliceContext.disable_deblocking_filter_idc;

    return ret;
}


int ARSTREAM2_H264Parser_GetUserDataSeiCount(ARSTREAM2_H264Parser_Handle parserHandle)
{
    ARSTREAM2_H264Parser_t* parser = (ARSTREAM2_H264Parser_t*)parserHandle;

    if (!parserHandle)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Invalid handle");
        return -1;
    }

    if (parser->config.extractUserDataSei)
    {
        return (int)parser->userDataCount;
    }
    else
    {
        return 0;
    }
}


int ARSTREAM2_H264Parser_GetRecoveryPointSei(ARSTREAM2_H264Parser_Handle parserHandle, ARSTREAM2_H264Parser_RecoveryPointSei_t *recoveryPoint)
{
    ARSTREAM2_H264Parser_t* parser = (ARSTREAM2_H264Parser_t*)parserHandle;

    if (!parserHandle)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Invalid handle");
        return -1;
    }

    if (!recoveryPoint)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Invalid pointer");
        return -1;
    }

    if (parser->hasRecoveryPoint)
    {
        memcpy(recoveryPoint, &parser->recoveryPoint, sizeof(ARSTREAM2_H264Parser_RecoveryPointSei_t));
        return 1;
    }
    else
    {
        return 0;
    }
}


eARSTREAM2_ERROR ARSTREAM2_H264Parser_GetUserDataSei(ARSTREAM2_H264Parser_Handle parserHandle, unsigned int index, void** pBuf, unsigned int* bufSize)
{
    ARSTREAM2_H264Parser_t* parser = (ARSTREAM2_H264Parser_t*)parserHandle;
    eARSTREAM2_ERROR ret = ARSTREAM2_OK;

    if (!parserHandle)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Invalid handle");
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }

    if ((!parser->userDataCount) || (index >= parser->userDataCount))
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Invalid index");
        return ARSTREAM2_ERROR_NOT_FOUND;
    }
    
    if ((parser->config.extractUserDataSei) && (parser->pUserDataBuf[index]) && (parser->userDataSize[index]))
    {
        if (bufSize) *bufSize = parser->userDataSize[index];
        if (pBuf) *pBuf = (void*)parser->pUserDataBuf[index];
    }
    else
    {
        if (bufSize) *bufSize = 0;
        if (pBuf) *pBuf = NULL;
    }

    return ret;
}


eARSTREAM2_ERROR ARSTREAM2_H264Parser_GetSpsPpsContext(ARSTREAM2_H264Parser_Handle parserHandle, void **spsContext, void **ppsContext)
{
    ARSTREAM2_H264Parser_t* parser = (ARSTREAM2_H264Parser_t*)parserHandle;
    eARSTREAM2_ERROR ret = ARSTREAM2_OK;

    if (!parserHandle)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Invalid handle");
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }
    if ((!spsContext) || (!ppsContext))
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Invalid pointer");
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }

    if ((!parser->spsSync) || (!parser->ppsSync))
    {
        return ARSTREAM2_ERROR_WAITING_FOR_SYNC;
    }

    *spsContext = &parser->spsContext;
    *ppsContext = &parser->ppsContext;

    return ret;
}


eARSTREAM2_ERROR ARSTREAM2_H264Parser_GetSliceContext(ARSTREAM2_H264Parser_Handle parserHandle, void **sliceContext)
{
    ARSTREAM2_H264Parser_t* parser = (ARSTREAM2_H264Parser_t*)parserHandle;
    eARSTREAM2_ERROR ret = ARSTREAM2_OK;

    if (!parserHandle)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Invalid handle");
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }
    if (!sliceContext)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Invalid pointer");
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }

    if ((parser->sliceContext.nal_unit_type != ARSTREAM2_H264_NALU_TYPE_SLICE) && (parser->sliceContext.nal_unit_type != ARSTREAM2_H264_NALU_TYPE_SLICE_IDR))
    {
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }

    *sliceContext = &parser->sliceContext;

    return ret;
}


eARSTREAM2_ERROR ARSTREAM2_H264Parser_Init(ARSTREAM2_H264Parser_Handle* parserHandle, ARSTREAM2_H264Parser_Config_t* config)
{
    ARSTREAM2_H264Parser_t* parser;

    if (!parserHandle)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Invalid pointer for handle");
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }

    parser = (ARSTREAM2_H264Parser_t*)malloc(sizeof(*parser));
    if (!parser)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_PARSER_TAG, "Allocation failed (size %zu)", sizeof(*parser));
        return ARSTREAM2_ERROR_ALLOC;
    }
    memset(parser, 0, sizeof(*parser));

    if (config)
    {
        memcpy(&parser->config, config, sizeof(parser->config));
    }

    parser->cache = 0;
    parser->cacheLength = 0;

    parser->naluBufSize = 0;
    parser->pNaluBuf = NULL;

    parser->spsContext.time_offset_length = 24;

    *parserHandle = (ARSTREAM2_H264Parser_Handle*)parser;

    return ARSTREAM2_OK;
}


eARSTREAM2_ERROR ARSTREAM2_H264Parser_Free(ARSTREAM2_H264Parser_Handle parserHandle)
{
    ARSTREAM2_H264Parser_t* parser = (ARSTREAM2_H264Parser_t*)parserHandle;
    int i;

    if (!parserHandle)
    {
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }

    if ((parser->pNaluBuf) && (parser->naluBufManaged))
    {
        free(parser->pNaluBuf);
    }

    for (i = 0; i < ARSTREAM2_H264_PARSER_MAX_USER_DATA_SEI_COUNT; i++)
    {
        free(parser->pUserDataBuf[i]);
    }

    free(parser);

    return ARSTREAM2_OK;
}

