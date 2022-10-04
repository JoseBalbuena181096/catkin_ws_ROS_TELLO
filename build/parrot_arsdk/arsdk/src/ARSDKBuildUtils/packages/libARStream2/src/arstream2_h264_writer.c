/**
 * @file arstream2_h264_writer.c
 * @brief Parrot Streaming Library - H.264 Writer
 * @date 08/04/2015
 * @author aurelien.barre@parrot.com
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <arpa/inet.h>

#include <libARStream2/arstream2_h264_writer.h>
#include "arstream2_h264.h"

#include <libARSAL/ARSAL_Print.h>

#define ARSTREAM2_H264_WRITER_TAG "ARSTREAM2_H264Writer"


#define log2(x) (log(x) / log(2)) //TODO


typedef struct ARSTREAM2_H264Writer_s
{
    ARSTREAM2_H264Writer_Config_t config;

    // NALU buffer
    uint8_t* pNaluBuf;
    unsigned int naluBufSize;
    unsigned int naluSize;      // in bytes

    // Bitstream cache
    uint32_t cache;
    int cacheLength;   // in bits
    int oldZeroCount;

    // Context
    ARSTREAM2_H264_SpsContext_t spsContext;
    ARSTREAM2_H264_PpsContext_t ppsContext;
    int isSpsPpsContextValid;
    ARSTREAM2_H264_SliceContext_t sliceContext;

} ARSTREAM2_H264Writer_t;


static inline int writeBits(ARSTREAM2_H264Writer_t* _writer, unsigned int _numBits, uint32_t _value, int _emulationPrevention)
{
    int _cacheBits, _remBits, _i;
    uint8_t _write8;
    uint32_t _bitMask;

    _remBits = (int)_numBits;
    _cacheBits = 32 - _writer->cacheLength;

    if (_remBits >= _cacheBits)
    {
        // Write _cacheBits bits to the cache
        _writer->cache |= (_value >> (_remBits - _cacheBits));

        // The cache is full; write 4 bytes in the buffer
        for (_i = 0; _i < 4; _i++)
        {
            _write8 = (_writer->cache >> 24) & 0xFF;

            // Emulation prevention
            if (_emulationPrevention)
            {
                if ((_writer->oldZeroCount == 2) && (_write8 <= 3))
                {
                    // 0x000000 or 0x000001 or 0x000002 or 0x000003 => insert 0x03
                    if (_writer->naluSize < _writer->naluBufSize)
                    {
                        *(_writer->pNaluBuf++) = 0x03;
                        _writer->naluSize++;
                    }
                    else
                    {
                        return -1;
                    }
                    _writer->oldZeroCount = 0;
                }
                if (_write8 == 0)
                {
                    _writer->oldZeroCount++;
                }
                else
                {
                    _writer->oldZeroCount = 0;
                }
            }

            if (_writer->naluSize < _writer->naluBufSize)
            {
                *(_writer->pNaluBuf++) = _write8;
                _writer->naluSize++;
            }
            else
            {
                return -1;
            }
            _writer->cache <<= 8;
        }
        _remBits -= _cacheBits;
        _bitMask = (uint32_t)-1 >> (32 - _remBits);
        _value &= _bitMask;

        // Reset the chache
        _writer->cache = 0;
        _writer->cacheLength = 0;
        _cacheBits = 32;
    }

    if (_remBits > 0)
    {
        _writer->cache |= (_value << (_cacheBits - _remBits));
        _writer->cacheLength += _remBits;
    }

    return _numBits;
}


static inline int bitstreamByteAlign(ARSTREAM2_H264Writer_t* _writer, int _emulationPrevention)
{
    int _bitsWritten = 0, _i;
    uint8_t _write8;

    if (_writer->cacheLength & 7)
    {
        _bitsWritten = writeBits(_writer, (8 - (_writer->cacheLength & 7)), 0, _emulationPrevention);
    }

    if (_writer->cacheLength)
    {
        // Write the cache bytes in the buffer
        for (_i = 0; _i < _writer->cacheLength / 8; _i++)
        {
            _write8 = (_writer->cache >> 24) & 0xFF;

            // Emulation prevention
            if (_emulationPrevention)
            {
                if ((_writer->oldZeroCount == 2) && (_write8 <= 3))
                {
                    // 0x000000 or 0x000001 or 0x000002 or 0x000003 => insert 0x03
                    if (_writer->naluSize < _writer->naluBufSize)
                    {
                        *(_writer->pNaluBuf++) = 0x03;
                        _writer->naluSize++;
                    }
                    else
                    {
                        return -1;
                    }
                    _writer->oldZeroCount = 0;
                }
                else if (_write8 == 0)
                {
                    _writer->oldZeroCount++;
                }
                else
                {
                    _writer->oldZeroCount = 0;
                }
            }

            if (_writer->naluSize < _writer->naluBufSize)
            {
                *(_writer->pNaluBuf++) = _write8;
                _writer->naluSize++;
            }
            else
            {
                return -1;
            }
            _writer->cache <<= 8;
        }

        // Reset the chache
        _writer->cache = 0;
        _writer->cacheLength = 0;
    }

    return _bitsWritten;
}


static inline int writeBits_expGolomb_code(ARSTREAM2_H264Writer_t* _writer, uint32_t _value, int _emulationPrevention)
{
    int _ret, _leadingZeroBits, _halfLength, _bitsWritten = 0;
    uint32_t _val;

    if (_value == 0)
    {
        return -1;
    }
    else
    {
        // Count leading zeros in _value
        for (_val = _value, _leadingZeroBits = 0; _val; _val <<= 1)
        {
            if (_val & 0x80000000)
            {
                break;
            }
            _leadingZeroBits++;
        }

        _halfLength = 31 - _leadingZeroBits;

        // Prefix
        _ret = writeBits(_writer, _halfLength, 0, _emulationPrevention);
        if (_ret != _halfLength) return -41;
        _bitsWritten += _ret;

        // Suffix
        _ret = writeBits(_writer, _halfLength + 1, _value, _emulationPrevention);
        if (_ret != _halfLength + 1) return -42;
        _bitsWritten += _ret;
    }

    return _halfLength * 2 + 1;
}


static inline int writeBits_expGolomb_ue(ARSTREAM2_H264Writer_t* _writer, uint32_t _value, int _emulationPrevention)
{
    if (_value == 0)
    {
        return writeBits(_writer, 1, 1, _emulationPrevention);
    }
    else
    {
        return writeBits_expGolomb_code(_writer, _value + 1, _emulationPrevention);
    }
}


static inline int writeBits_expGolomb_se(ARSTREAM2_H264Writer_t* _writer, int32_t _value, int _emulationPrevention)
{
    if (_value == 0)
    {
        return writeBits(_writer, 1, 1, _emulationPrevention);
    }
    else if (_value < 0)
    {
        return writeBits_expGolomb_code(_writer, (uint32_t)(-_value * 2 + 1), _emulationPrevention);
    }
    else
    {
        return writeBits_expGolomb_code(_writer, (uint32_t)(_value * 2), _emulationPrevention);
    }
}


static int ARSTREAM2_H264Writer_WriteSeiPayload_pictureTiming(ARSTREAM2_H264Writer_t* writer, ARSTREAM2_H264Writer_PictureTimingSei_t *pictureTiming)
{
    int ret = 0;
    int _bitsWritten = 0;
    unsigned int payloadType, payloadSize, payloadSizeBits;

    if (!writer->isSpsPpsContextValid)
    {
        return 0;
    }

    if (pictureTiming->picStruct != 0)
    {
        // Note: pic_struct != 0 is not supported
        return 0;
    }

    payloadType = ARSTREAM2_H264_SEI_PAYLOAD_TYPE_PIC_TIMING;

    // compute the payload size
    payloadSizeBits = 0;
    if ((writer->spsContext.nal_hrd_parameters_present_flag) || (writer->spsContext.vcl_hrd_parameters_present_flag))
    {
        // cpb_removal_delay
        payloadSizeBits += writer->spsContext.cpb_removal_delay_length_minus1 + 1;
        // dpb_output_delay
        payloadSizeBits += writer->spsContext.dpb_output_delay_length_minus1 + 1;
    }
    if (writer->spsContext.pic_struct_present_flag)
    {
        // pic_struct
        payloadSizeBits += 4;
        // Only NumClockTS == 1 is supported
        //for (i = 0; i < NumClockTS; i++)
        {
            // clock_timestamp_flag[i]
            payloadSizeBits += 1;
            //if (clock_timestamp_flag[i])
            {
                // ct_type
                payloadSizeBits += 2;
                // nuit_field_based_flag
                payloadSizeBits += 1;
                // counting_type
                payloadSizeBits += 5;
                // full_timestamp_flag
                payloadSizeBits += 1;
                // discontinuity_flag
                payloadSizeBits += 1;
                // cnt_dropped_flag
                payloadSizeBits += 1;
                // n_frames
                payloadSizeBits += 8;
                if (pictureTiming->fullTimestampFlag)
                {
                    // seconds_value
                    payloadSizeBits += 6;
                    // minutes_value
                    payloadSizeBits += 6;
                    // hours_value
                    payloadSizeBits += 5;
                }
                else
                {
                    // seconds_flag
                    payloadSizeBits += 1;
                    if (pictureTiming->secondsFlag)
                    {
                        // seconds_value
                        payloadSizeBits += 6;
                        // minutes_flag
                        payloadSizeBits += 1;
                        if (pictureTiming->minutesFlag)
                        {
                            // minutes_value
                            payloadSizeBits += 6;
                            // hours_flag
                            payloadSizeBits += 1;
                            if (pictureTiming->hoursFlag)
                            {
                                // hours_value
                                payloadSizeBits += 5;
                            }
                        }
                    }
                }
                if (writer->spsContext.time_offset_length > 0)
                {
                    // time_offset
                    payloadSizeBits +=  writer->spsContext.time_offset_length;
                }
            }
        }
    }
    // If not byte-aligned, write '1' bit and then align to byte
    if (payloadSizeBits & 7)
    {
        payloadSizeBits++;
    }
    payloadSize = (payloadSizeBits + 7) / 8;

    /*while (payloadType > 255) // logically dead code
    {
        // ff_byte
        ret = writeBits(writer, 8, 0xFF, 1);
        if (ret < 0)
        {
            return -1;
        }
        _bitsWritten += ret;
        payloadType -= 255;
    }*/
    // last_payload_type_byte
    ret = writeBits(writer, 8, payloadType, 1);
    if (ret < 0)
    {
        return -1;
    }
    _bitsWritten += ret;

    while (payloadSize > 255)
    {
        // ff_byte
        ret = writeBits(writer, 8, 0xFF, 1);
        if (ret < 0)
        {
            return -1;
        }
        _bitsWritten += ret;
        payloadSize -= 255;
    }
    // last_payload_type_byte
    ret = writeBits(writer, 8, payloadSize, 1);
    if (ret < 0)
    {
        return -1;
    }
    _bitsWritten += ret;

    if ((writer->spsContext.nal_hrd_parameters_present_flag) || (writer->spsContext.vcl_hrd_parameters_present_flag))
    {
        // cpb_removal_delay
        ret = writeBits(writer, writer->spsContext.cpb_removal_delay_length_minus1 + 1, pictureTiming->cpbRemovalDelay, 1);
        if (ret < 0)
        {
            return -1;
        }
        _bitsWritten += ret;

        // dpb_output_delay
        ret = writeBits(writer, writer->spsContext.dpb_output_delay_length_minus1 + 1, pictureTiming->dpbOutputDelay, 1);
        if (ret < 0)
        {
            return -1;
        }
        _bitsWritten += ret;
    }

    if (writer->spsContext.pic_struct_present_flag)
    {
        // pic_struct
        ret = writeBits(writer, 4, pictureTiming->picStruct, 1);
        if (ret < 0)
        {
            return -1;
        }
        _bitsWritten += ret;

        // Only NumClockTS == 1 is supported
        //for (i = 0; i < NumClockTS; i++)
        {
            // clock_timestamp_flag[i]
            ret = writeBits(writer, 1, 1, 1);
            if (ret < 0)
            {
                return -1;
            }
            _bitsWritten += ret;

            //if (clock_timestamp_flag[i])
            {
                // ct_type
                ret = writeBits(writer, 2, pictureTiming->ctType, 1);
                if (ret < 0)
                {
                    return -1;
                }
                _bitsWritten += ret;

                // nuit_field_based_flag
                ret = writeBits(writer, 1, pictureTiming->nuitFieldBasedFlag, 1);
                if (ret < 0)
                {
                    return -1;
                }
                _bitsWritten += ret;

                // counting_type
                ret = writeBits(writer, 5, pictureTiming->countingType, 1);
                if (ret < 0)
                {
                    return -1;
                }
                _bitsWritten += ret;

                // full_timestamp_flag
                ret = writeBits(writer, 1, pictureTiming->fullTimestampFlag, 1);
                if (ret < 0)
                {
                    return -1;
                }
                _bitsWritten += ret;

                // discontinuity_flag
                ret = writeBits(writer, 1, pictureTiming->discontinuityFlag, 1);
                if (ret < 0)
                {
                    return -1;
                }
                _bitsWritten += ret;

                // cnt_dropped_flag
                ret = writeBits(writer, 1, pictureTiming->cntDroppedFlag, 1);
                if (ret < 0)
                {
                    return -1;
                }
                _bitsWritten += ret;

                // n_frames
                ret = writeBits(writer, 8, pictureTiming->nFrames, 1);
                if (ret < 0)
                {
                    return -1;
                }
                _bitsWritten += ret;

                if (pictureTiming->fullTimestampFlag)
                {
                    // seconds_value
                    ret = writeBits(writer, 6, pictureTiming->secondsValue, 1);
                    if (ret < 0)
                    {
                        return -1;
                    }
                    _bitsWritten += ret;

                    // minutes_value
                    ret = writeBits(writer, 6, pictureTiming->minutesValue, 1);
                    if (ret < 0)
                    {
                        return -1;
                    }
                    _bitsWritten += ret;

                    // hours_value
                    ret = writeBits(writer, 5, pictureTiming->hoursValue, 1);
                    if (ret < 0)
                    {
                        return -1;
                    }
                    _bitsWritten += ret;
                }
                else
                {
                    // seconds_flag
                    ret = writeBits(writer, 1, pictureTiming->secondsFlag, 1);
                    if (ret < 0)
                    {
                        return -1;
                    }
                    _bitsWritten += ret;

                    if (pictureTiming->secondsFlag)
                    {
                        // seconds_value
                        ret = writeBits(writer, 6, pictureTiming->secondsValue, 1);
                        if (ret < 0)
                        {
                            return -1;
                        }
                        _bitsWritten += ret;

                        // minutes_flag
                        ret = writeBits(writer, 1, pictureTiming->minutesFlag, 1);
                        if (ret < 0)
                        {
                            return -1;
                        }
                        _bitsWritten += ret;

                        if (pictureTiming->minutesFlag)
                        {
                            // minutes_value
                            ret = writeBits(writer, 6, pictureTiming->minutesValue, 1);
                            if (ret < 0)
                            {
                                return -1;
                            }
                            _bitsWritten += ret;

                            // hours_flag
                            ret = writeBits(writer, 1, pictureTiming->hoursFlag, 1);
                            if (ret < 0)
                            {
                                return -1;
                            }
                            _bitsWritten += ret;

                            if (pictureTiming->hoursFlag)
                            {
                                // hours_value
                                ret = writeBits(writer, 5, pictureTiming->hoursValue, 1);
                                if (ret < 0)
                                {
                                    return -1;
                                }
                                _bitsWritten += ret;
                            }
                        }
                    }
                }
                if (writer->spsContext.time_offset_length > 0)
                {
                    // time_offset
                    ret = writeBits(writer, writer->spsContext.time_offset_length, (uint32_t)pictureTiming->timeOffset, 1);
                    if (ret < 0)
                    {
                        return -1;
                    }
                    _bitsWritten += ret;
                }
            }
        }
    }

    // If not byte-aligned, write '1' bit and then align to byte
    if (writer->cacheLength & 7)
    {
        ret = writeBits(writer, 1, 1, 1);
        if (ret < 0)
        {
            return -1;
        }
        _bitsWritten += ret;
    }

    ret = bitstreamByteAlign(writer, 1);
    if (ret < 0)
    {
        return -1;
    }
    _bitsWritten += ret;

    return _bitsWritten;
}


static int ARSTREAM2_H264Writer_WriteSeiPayload_recoveryPoint(ARSTREAM2_H264Writer_t* writer, ARSTREAM2_H264Writer_RecoveryPointSei_t *recoveryPoint)
{
    int ret = 0;
    int _bitsWritten = 0;
    unsigned int payloadType, payloadSize, payloadSizeBits;
    int recoveryFrameCntSize = 0;

    if (recoveryPoint->recoveryFrameCnt == 0)
    {
        recoveryFrameCntSize = 1;
    }
    else
    {
        unsigned int _val, _leadingZeroBits, _halfLength;
        // Count leading zeros in _value
        for (_val = recoveryPoint->recoveryFrameCnt, _leadingZeroBits = 0; _val; _val <<= 1)
        {
            if (_val & 0x80000000)
            {
                break;
            }
            _leadingZeroBits++;
        }

        _halfLength = 31 - _leadingZeroBits;

        recoveryFrameCntSize = _halfLength * 2 + 1;
    }

    payloadType = ARSTREAM2_H264_SEI_PAYLOAD_TYPE_RECOVERY_POINT;
    payloadSizeBits = recoveryFrameCntSize + 1 + 1 + 2;
    // If not byte-aligned, write '1' bit and then align to byte
    if (payloadSizeBits & 7)
    {
        payloadSizeBits++;
    }
    payloadSize = (payloadSizeBits + 7) / 8;

    /*while (payloadType > 255) // logically dead code
    {
        // ff_byte
        ret = writeBits(writer, 8, 0xFF, 1);
        if (ret < 0)
        {
            return -1;
        }
        _bitsWritten += ret;
        payloadType -= 255;
    }*/
    // last_payload_type_byte
    ret = writeBits(writer, 8, payloadType, 1);
    if (ret < 0)
    {
        return -1;
    }
    _bitsWritten += ret;

    /*while (payloadSize > 255) // logically dead code
    {
        // ff_byte
        ret = writeBits(writer, 8, 0xFF, 1);
        if (ret < 0)
        {
            return -1;
        }
        _bitsWritten += ret;
        payloadSize -= 255;
    }*/
    // last_payload_type_byte
    ret = writeBits(writer, 8, payloadSize, 1);
    if (ret < 0)
    {
        return -1;
    }
    _bitsWritten += ret;

    // recovery_frame_cnt
    ret = writeBits_expGolomb_ue(writer, recoveryPoint->recoveryFrameCnt, 1);
    if (ret < 0)
    {
        return -1;
    }
    _bitsWritten += ret;

    // exact_match_flag
    ret = writeBits(writer, 1, recoveryPoint->exactMatchFlag, 1);
    if (ret < 0)
    {
        return -1;
    }
    _bitsWritten += ret;

    // broken_link_flag
    ret = writeBits(writer, 1, recoveryPoint->brokenLinkFlag, 1);
    if (ret < 0)
    {
        return -1;
    }
    _bitsWritten += ret;

    // changing_slice_group_idc
    ret = writeBits(writer, 2, recoveryPoint->changingSliceGroupIdc, 1);
    if (ret < 0)
    {
        return -1;
    }
    _bitsWritten += ret;

    // If not byte-aligned, write '1' bit and then align to byte
    if (writer->cacheLength & 7)
    {
        ret = writeBits(writer, 1, 1, 1);
        if (ret < 0)
        {
            return -1;
        }
        _bitsWritten += ret;
    }

    ret = bitstreamByteAlign(writer, 1);
    if (ret < 0)
    {
        return -1;
    }
    _bitsWritten += ret;

    return _bitsWritten;
}


static int ARSTREAM2_H264Writer_WriteSeiPayload_userDataUnregistered(ARSTREAM2_H264Writer_t* writer, const uint8_t *pbPayload, unsigned int payloadSize)
{
    int ret = 0;
    unsigned int i;
    int _bitsWritten = 0;
    unsigned int payloadType;
    unsigned int payloadSize2 = payloadSize;

    payloadType = ARSTREAM2_H264_SEI_PAYLOAD_TYPE_USER_DATA_UNREGISTERED;

    /* while (payloadType > 255) // logically dead code
    {
        // ff_byte
        ret = writeBits(writer, 8, 0xFF, 1);
        if (ret < 0)
        {
            return -1;
        }
        _bitsWritten += ret;
        payloadType -= 255;
    }*/
    // last_payload_type_byte
    ret = writeBits(writer, 8, payloadType, 1);
    if (ret < 0)
    {
        return -1;
    }
    _bitsWritten += ret;

    while (payloadSize2 > 255)
    {
        // ff_byte
        ret = writeBits(writer, 8, 0xFF, 1);
        if (ret < 0)
        {
            return -1;
        }
        _bitsWritten += ret;
        payloadSize2 -= 255;
    }
    // last_payload_type_byte
    ret = writeBits(writer, 8, payloadSize2, 1);
    if (ret < 0)
    {
        return -1;
    }
    _bitsWritten += ret;

    // user_data_unregistered
    for (i = 0; i < payloadSize; i++)
    {
        ret = writeBits(writer, 8, (uint32_t)(*pbPayload++), 1);
        if (ret < 0)
        {
            return ret;
        }
        _bitsWritten += ret;
    }

    /* If not byte-aligned, write '1' bit and then align to byte...
     * ... but it's useless here as user data SEI are always byte-aligned.
     * So we do nothing more.
     */

    ret = bitstreamByteAlign(writer, 1);
    if (ret < 0)
    {
        return -1;
    }
    _bitsWritten += ret;

    return _bitsWritten;
}


int ARSTREAM2_H264Writer_WriteSeiNalu(ARSTREAM2_H264Writer_Handle writerHandle, ARSTREAM2_H264Writer_PictureTimingSei_t *pictureTiming,
                                      ARSTREAM2_H264Writer_RecoveryPointSei_t *recoveryPoint, unsigned int userDataUnregisteredCount,
                                      const uint8_t *pbUserDataUnregistered[], unsigned int userDataUnregisteredSize[],
                                      uint8_t *pbOutputBuf, unsigned int outputBufSize, unsigned int *outputSize)
{
    ARSTREAM2_H264Writer_t *writer = (ARSTREAM2_H264Writer_t*)writerHandle;
    int ret = 0, bitsWritten = 0;
    unsigned int i;

    if ((!writerHandle) || (!pbOutputBuf) || (outputBufSize == 0) || (!outputSize))
    {
        return -1;
    }

    writer->pNaluBuf = pbOutputBuf;
    writer->naluBufSize = outputBufSize;
    writer->naluSize = 0;

    // Reset the bitstream cache
    writer->cache = 0;
    writer->cacheLength = 0;
    writer->oldZeroCount = 0;

    // NALU start code
    if (writer->config.naluPrefix)
    {
        ret = writeBits(writer, 32, ARSTREAM2_H264_BYTE_STREAM_NALU_START_CODE, 0);
        if (ret < 0)
        {
            return -1;
        }
        bitsWritten += ret;
    }

    // forbidden_zero_bit = 0
    // nal_ref_idc = 0
    // nal_unit_type = 6
    ret = writeBits(writer, 8, ARSTREAM2_H264_NALU_TYPE_SEI, 0);
    if (ret < 0)
    {
        return -1;
    }
    bitsWritten += ret;

    if (pictureTiming)
    {
        // picture_timing
        ret = ARSTREAM2_H264Writer_WriteSeiPayload_pictureTiming(writer, pictureTiming);
        if (ret < 0)
        {
            return -1;
        }
        bitsWritten += ret;
    }

    if (recoveryPoint)
    {
        // recovery_point
        ret = ARSTREAM2_H264Writer_WriteSeiPayload_recoveryPoint(writer, recoveryPoint);
        if (ret < 0)
        {
            return -1;
        }
        bitsWritten += ret;
    }

    for (i = 0; i < userDataUnregisteredCount; i++)
    {
        if ((pbUserDataUnregistered[i]) && (userDataUnregisteredSize[i] >= 16))
        {
            // user_data_unregistered
            ret = ARSTREAM2_H264Writer_WriteSeiPayload_userDataUnregistered(writer, pbUserDataUnregistered[i], userDataUnregisteredSize[i]);
            if (ret < 0)
            {
                return -1;
            }
            bitsWritten += ret;
        }
    }

    // rbsp_trailing_bits
    ret = writeBits(writer, 1, 1, 1);
    if (ret < 0)
    {
        return -1;
    }
    bitsWritten += ret;

    ret = bitstreamByteAlign(writer, 1);
    if (ret < 0)
    {
        return -1;
    }
    bitsWritten += ret;

    *outputSize = writer->naluSize;

    return 0;
}


static int ARSTREAM2_H264Writer_WriteRefPicListModification(ARSTREAM2_H264Writer_t* writer, ARSTREAM2_H264_SliceContext_t *slice, ARSTREAM2_H264_SpsContext_t *sps, ARSTREAM2_H264_PpsContext_t *pps)
{
    int ret = 0;
    int bitsWritten = 0;

    if ((slice->sliceTypeMod5 != ARSTREAM2_H264_SLICE_TYPE_I) && (slice->sliceTypeMod5 != ARSTREAM2_H264_SLICE_TYPE_SI))
    {
        // ref_pic_list_modification_flag_l0
        ret = writeBits(writer, 1, slice->ref_pic_list_modification_flag_l0, 1);
        if (ret < 0)
        {
            return -1;
        }
        bitsWritten += ret;

        if (slice->ref_pic_list_modification_flag_l0)
        {
            // UNSUPPORTED
            return -1;
        }
    }

    if (slice->sliceTypeMod5 == ARSTREAM2_H264_SLICE_TYPE_B)
    {
        // ref_pic_list_modification_flag_l1
        ret = writeBits(writer, 1, slice->ref_pic_list_modification_flag_l1, 1);
        if (ret < 0)
        {
            return -1;
        }
        bitsWritten += ret;

        if (slice->ref_pic_list_modification_flag_l1)
        {
            // UNSUPPORTED
            return -1;
        }
    }

    return bitsWritten;
}


static int ARSTREAM2_H264Writer_WriteDecRefPicMarking(ARSTREAM2_H264Writer_t* writer, ARSTREAM2_H264_SliceContext_t *slice, ARSTREAM2_H264_SpsContext_t *sps, ARSTREAM2_H264_PpsContext_t *pps)
{
    int ret = 0;
    int bitsWritten = 0;

    if (slice->idrPicFlag)
    {
        // no_output_of_prior_pics_flag
        ret = writeBits(writer, 1, slice->no_output_of_prior_pics_flag, 1);
        if (ret < 0)
        {
            return -1;
        }
        bitsWritten += ret;

        // long_term_reference_flag
        ret = writeBits(writer, 1, slice->long_term_reference_flag, 1);
        if (ret < 0)
        {
            return -1;
        }
        bitsWritten += ret;
    }
    else
    {
        // adaptive_ref_pic_marking_mode_flag
        ret = writeBits(writer, 1, slice->adaptive_ref_pic_marking_mode_flag, 1);
        if (ret < 0)
        {
            return -1;
        }
        bitsWritten += ret;

        if (slice->adaptive_ref_pic_marking_mode_flag)
        {
            // UNSUPPORTED
            return -1;
        }
    }

    return bitsWritten;
}


static int ARSTREAM2_H264Writer_WriteSliceHeader(ARSTREAM2_H264Writer_t* writer, ARSTREAM2_H264_SliceContext_t *slice, ARSTREAM2_H264_SpsContext_t *sps, ARSTREAM2_H264_PpsContext_t *pps)
{
    int ret = 0;
    int bitsWritten = 0;

    // first_mb_in_slice
    ret = writeBits_expGolomb_ue(writer, slice->first_mb_in_slice, 1);
    if (ret < 0)
    {
        return -1;
    }
    bitsWritten += ret;

    // slice_type
    ret = writeBits_expGolomb_ue(writer, slice->slice_type, 1);
    if (ret < 0)
    {
        return -1;
    }
    bitsWritten += ret;

    // pic_parameter_set_id
    ret = writeBits_expGolomb_ue(writer, slice->pic_parameter_set_id, 1);
    if (ret < 0)
    {
        return -1;
    }
    bitsWritten += ret;

    if (sps->separate_colour_plane_flag == 1)
    {
        // colour_plane_id
        ret = writeBits(writer, 2, slice->colour_plane_id, 1);
        if (ret < 0)
        {
            return -1;
        }
        bitsWritten += ret;
    }

    // frame_num
    ret = writeBits(writer, sps->log2_max_frame_num_minus4 + 4, slice->frame_num, 1);
    if (ret < 0)
    {
        return -1;
    }
    bitsWritten += ret;

    if (!sps->frame_mbs_only_flag)
    {
        // field_pic_flag
        ret = writeBits(writer, 1, slice->field_pic_flag, 1);
        if (ret < 0)
        {
            return -1;
        }
        bitsWritten += ret;

        if (slice->field_pic_flag)
        {
            // bottom_field_flag
            ret = writeBits(writer, 1, slice->bottom_field_flag, 1);
            if (ret < 0)
            {
                return -1;
            }
            bitsWritten += ret;
        }
    }

    if (slice->idrPicFlag)
    {
        // idr_pic_id
        ret = writeBits_expGolomb_ue(writer, slice->idr_pic_id, 1);
        if (ret < 0)
        {
            return -1;
        }
        bitsWritten += ret;
    }

    if (sps->pic_order_cnt_type == 0)
    {
        // pic_order_cnt_lsb
        ret = writeBits(writer, sps->log2_max_pic_order_cnt_lsb_minus4 + 4, slice->pic_order_cnt_lsb, 1);
        if (ret < 0)
        {
            return -1;
        }
        bitsWritten += ret;

        if ((pps->bottom_field_pic_order_in_frame_present_flag) && (!slice->field_pic_flag))
        {
            // delta_pic_order_cnt_bottom
            ret = writeBits_expGolomb_se(writer, slice->delta_pic_order_cnt_bottom, 1);
            if (ret < 0)
            {
                return -1;
            }
            bitsWritten += ret;
        }
    }

    if ((sps->pic_order_cnt_type == 1) && (!sps->delta_pic_order_always_zero_flag))
    {
        // delta_pic_order_cnt[0]
        ret = writeBits_expGolomb_se(writer, slice->delta_pic_order_cnt_0, 1);
        if (ret < 0)
        {
            return -1;
        }
        bitsWritten += ret;

        if ((pps->bottom_field_pic_order_in_frame_present_flag) && (!slice->field_pic_flag))
        {
            // delta_pic_order_cnt[1]
            ret = writeBits_expGolomb_se(writer, slice->delta_pic_order_cnt_1, 1);
            if (ret < 0)
            {
                return -1;
            }
            bitsWritten += ret;
        }
    }

    if (pps->redundant_pic_cnt_present_flag)
    {
        // redundant_pic_cnt
        ret = writeBits_expGolomb_ue(writer, slice->redundant_pic_cnt, 1);
        if (ret < 0)
        {
            return -1;
        }
        bitsWritten += ret;
    }

    if (slice->sliceTypeMod5 == ARSTREAM2_H264_SLICE_TYPE_B)
    {
        // direct_spatial_mv_pred_flag
        ret = writeBits(writer, 1, slice->direct_spatial_mv_pred_flag, 1);
        if (ret < 0)
        {
            return -1;
        }
        bitsWritten += ret;
    }

    if ((slice->sliceTypeMod5 == ARSTREAM2_H264_SLICE_TYPE_P) || (slice->sliceTypeMod5 == ARSTREAM2_H264_SLICE_TYPE_SP) || (slice->sliceTypeMod5 == ARSTREAM2_H264_SLICE_TYPE_B))
    {
        // num_ref_idx_active_override_flag
        ret = writeBits(writer, 1, slice->num_ref_idx_active_override_flag, 1);
        if (ret < 0)
        {
            return -1;
        }
        bitsWritten += ret;

        if (slice->num_ref_idx_active_override_flag)
        {
            // num_ref_idx_l0_active_minus1
            ret = writeBits_expGolomb_ue(writer, slice->num_ref_idx_l0_active_minus1, 1);
            if (ret < 0)
            {
                return -1;
            }
            bitsWritten += ret;

            if (slice->sliceTypeMod5 == ARSTREAM2_H264_SLICE_TYPE_B)
            {
                // num_ref_idx_l1_active_minus1
                ret = writeBits_expGolomb_ue(writer, slice->num_ref_idx_l1_active_minus1, 1);
                if (ret < 0)
                {
                    return -1;
                }
                bitsWritten += ret;
            }
        }
    }

    if ((slice->nal_unit_type == 20) || (slice->nal_unit_type == 21))
    {
        // ref_pic_list_mvc_modification()
        // UNSUPPORTED
        return -1;
    }
    else
    {
        // ref_pic_list_modification()
        ret = ARSTREAM2_H264Writer_WriteRefPicListModification(writer, slice, sps, pps);
        if (ret < 0)
        {
            return -1;
        }
        bitsWritten += ret;
    }

    if ((pps->weighted_pred_flag && ((slice->sliceTypeMod5 == ARSTREAM2_H264_SLICE_TYPE_P) || (slice->sliceTypeMod5 == ARSTREAM2_H264_SLICE_TYPE_SP))) 
            || ((pps->weighted_bipred_idc == 1) && (slice->sliceTypeMod5 == ARSTREAM2_H264_SLICE_TYPE_B)))
    {
        // pred_weight_table()
        // UNSUPPORTED
        return -1;
    }

    if (slice->nal_ref_idc != 0)
    {
        // dec_ref_pic_marking()
        ret = ARSTREAM2_H264Writer_WriteDecRefPicMarking(writer, slice, sps, pps);
        if (ret < 0)
        {
            return -1;
        }
        bitsWritten += ret;
    }

    if ((pps->entropy_coding_mode_flag) && (slice->sliceTypeMod5 != ARSTREAM2_H264_SLICE_TYPE_I) && (slice->sliceTypeMod5 != ARSTREAM2_H264_SLICE_TYPE_SI))
    {
        // cabac_init_idc
        ret = writeBits_expGolomb_ue(writer, slice->cabac_init_idc, 1);
        if (ret < 0)
        {
            return -1;
        }
        bitsWritten += ret;
    }

    // slice_qp_delta
    ret = writeBits_expGolomb_se(writer, slice->slice_qp_delta, 1);
    if (ret < 0)
    {
        return -1;
    }
    bitsWritten += ret;

    if ((slice->sliceTypeMod5 == ARSTREAM2_H264_SLICE_TYPE_SP) || (slice->sliceTypeMod5 == ARSTREAM2_H264_SLICE_TYPE_SI))
    {
        if (slice->sliceTypeMod5 == ARSTREAM2_H264_SLICE_TYPE_SP)
        {
            // sp_for_switch_flag
            ret = writeBits(writer, 1, slice->sp_for_switch_flag, 1);
            if (ret < 0)
            {
                return -1;
            }
            bitsWritten += ret;
        }

        // slice_qs_delta
        ret = writeBits_expGolomb_se(writer, slice->slice_qs_delta, 1);
        if (ret < 0)
        {
            return -1;
        }
        bitsWritten += ret;
    }

    if (pps->deblocking_filter_control_present_flag)
    {
        // disable_deblocking_filter_idc
        ret = writeBits_expGolomb_ue(writer, slice->disable_deblocking_filter_idc, 1);
        if (ret < 0)
        {
            return -1;
        }
        bitsWritten += ret;

        if (slice->disable_deblocking_filter_idc != 1)
        {
            // slice_alpha_c0_offset_div2
            ret = writeBits_expGolomb_se(writer, slice->slice_alpha_c0_offset_div2, 1);
            if (ret < 0)
            {
                return -1;
            }
            bitsWritten += ret;

            // slice_beta_offset_div2
            ret = writeBits_expGolomb_se(writer, slice->slice_beta_offset_div2, 1);
            if (ret < 0)
            {
                return -1;
            }
            bitsWritten += ret;
        }
    }

    if ((pps->num_slice_groups_minus1 > 0) && (pps->slice_group_map_type >= 3) && (pps->slice_group_map_type <= 5))
    {
        int picSizeInMapUnits, n;

        picSizeInMapUnits = (sps->pic_width_in_mbs_minus1 + 1) * (sps->pic_height_in_map_units_minus1 + 1);
        n = ceil(log2((picSizeInMapUnits / (pps->slice_group_change_rate_minus1 + 1)) + 1));

        // slice_group_change_cycle
        ret = writeBits(writer, n, slice->slice_group_change_cycle, 1);
        if (ret < 0)
        {
            return -1;
        }
        bitsWritten += ret;
    }

    return bitsWritten;
}


static int ARSTREAM2_H264Writer_WriteSkippedPSliceData(ARSTREAM2_H264Writer_t* writer, ARSTREAM2_H264_SliceContext_t *slice, ARSTREAM2_H264_SpsContext_t *sps, ARSTREAM2_H264_PpsContext_t *pps)
{
    int ret = 0;
    int bitsWritten = 0;

    if (pps->entropy_coding_mode_flag)
    {
        // cabac_alignment_one_bit
        // UNSUPPORTED
        return -1;
    }

    // mb_skip_run
    ret = writeBits_expGolomb_ue(writer, slice->sliceMbCount, 1);
    if (ret < 0)
    {
        return -1;
    }
    bitsWritten += ret;

    return bitsWritten;
}


static int ARSTREAM2_H264Writer_WriteGrayISliceData(ARSTREAM2_H264Writer_t* writer, ARSTREAM2_H264_SliceContext_t *slice, ARSTREAM2_H264_SpsContext_t *sps, ARSTREAM2_H264_PpsContext_t *pps)
{
    int ret = 0;
    int bitsWritten = 0;
    unsigned int i;

    if (pps->entropy_coding_mode_flag)
    {
        // cabac_alignment_one_bit
        // UNSUPPORTED
        return -1;
    }

    // macroblock_layer

    for (i = 0; i < writer->sliceContext.sliceMbCount; i++)
    {
        // mb_type = 3 (I_16x16_2_0_0: Intra16x16PredMode = 2/DC, CodedBlockPatternLuma = 0, CodedBlockPatternChroma = 0)
        ret = writeBits_expGolomb_ue(writer, 3, 1);
        if (ret < 0)
        {
            return -1;
        }
        bitsWritten += ret;
        
        // mb_pred -> intra_chroma_pred_mode = 0 (DC)
        ret = writeBits_expGolomb_ue(writer, 0, 1);
        if (ret < 0)
        {
            return -1;
        }
        bitsWritten += ret;

        // mb_qp_delta = 0
        ret = writeBits_expGolomb_se(writer, 0, 1);
        if (ret < 0)
        {
            return -1;
        }
        bitsWritten += ret;

        // residual(0, 15) -> residual_luma(i16x16DClevel, i16x16AClevel, level4x4, level8x8, 0, 15) -> residual_block_cavlc(i16x16DClevel, 0, 15, 16) -> coeff_token = 1 (nC = 0)
        ret = writeBits(writer, 1, 1, 1);
        if (ret < 0)
        {
            return -1;
        }
        bitsWritten += ret;
    }

    return bitsWritten;
}


eARSTREAM2_ERROR ARSTREAM2_H264Writer_RewriteNonRefPSliceNalu(ARSTREAM2_H264Writer_Handle writerHandle, void *sliceContext, const uint8_t *pbInputBuf, unsigned int inputSize, uint8_t *pbOutputBuf, unsigned int outputBufSize, unsigned int *outputSize)
{
    ARSTREAM2_H264Writer_t *writer = (ARSTREAM2_H264Writer_t*)writerHandle;
    int ret = 0, bitsWritten = 0;

    if ((!writerHandle) || (!pbOutputBuf) || (outputBufSize == 0) || (!outputSize))
    {
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }

    if (!writer->isSpsPpsContextValid)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_WRITER_TAG, "Invalid SPS/PPS context");
        return ARSTREAM2_ERROR_INVALID_STATE;
    }

    // Slice context
    if (sliceContext)
    {
        if (((ARSTREAM2_H264_SliceContext_t*)sliceContext)->adaptive_ref_pic_marking_mode_flag == 1)
        {
            // UNSUPPORTED
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_WRITER_TAG, "Slice context: adaptive_ref_pic_marking_mode_flag==1 is not supported");
            return ARSTREAM2_ERROR_UNSUPPORTED;
        }
        memcpy(&writer->sliceContext, sliceContext, sizeof(ARSTREAM2_H264_SliceContext_t));
        writer->sliceContext.nal_ref_idc = 0;
    }
    else
    {
        // UNSUPPORTED
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_WRITER_TAG, "Slice context is not provided");
        return ARSTREAM2_ERROR_UNSUPPORTED;
    }

    /* NB: changing nal_ref_idc to 0 means that dec_ref_pic_marking() will no more be present in the slice header.
     * Therefore adaptive_ref_pic_marking_mode_flag will not be present and the original slice_data() syntax must
     * be shifted left by 1 bit.
     */

    writer->pNaluBuf = pbOutputBuf;
    writer->naluBufSize = outputBufSize;
    writer->naluSize = 0;

    // Reset the bitstream cache
    writer->cache = 0;
    writer->cacheLength = 0;
    writer->oldZeroCount = 0;

    // NALU start code
    if (writer->config.naluPrefix)
    {
        ret = writeBits(writer, 32, ARSTREAM2_H264_BYTE_STREAM_NALU_START_CODE, 0);
        if (ret < 0)
        {
            return ARSTREAM2_ERROR_INVALID_STATE;
        }
        bitsWritten += ret;
    }

    // forbidden_zero_bit
    // nal_ref_idc
    // nal_unit_type
    ret = writeBits(writer, 8, ((writer->sliceContext.nal_ref_idc & 3) << 5) | writer->sliceContext.nal_unit_type, 0);
    if (ret < 0)
    {
        return ARSTREAM2_ERROR_INVALID_STATE;
    }
    bitsWritten += ret;

    // slice_header
    ret = ARSTREAM2_H264Writer_WriteSliceHeader(writer, &writer->sliceContext, &writer->spsContext, &writer->ppsContext);
    if (ret < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_WRITER_TAG, "Error: ARSTREAM2_H264Writer_WriteSliceHeader() failed (%d)", ret);
        return ARSTREAM2_ERROR_INVALID_STATE;
    }
    bitsWritten += ret;

    if (ret != (int)writer->sliceContext.sliceHeaderLengthInBits - 1)
    {
        // UNSUPPORTED
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_WRITER_TAG, "Unsupported slice header length (%d vs %d)", ret, writer->sliceContext.sliceHeaderLengthInBits);
        return ARSTREAM2_ERROR_UNSUPPORTED;
    }

    // slice_data
    int length = inputSize - 1 - writer->sliceContext.sliceHeaderLengthInBits / 8;
    if (length < 1)
    {
        // UNSUPPORTED
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_WRITER_TAG, "Unsupported length (%d)", length);
        return ARSTREAM2_ERROR_UNSUPPORTED;
    }

    int dstBitOffset = bitsWritten & 7;
    int dstOffset = bitsWritten / 8;
    ret = bitstreamByteAlign(writer, 1);
    if (ret < 0)
    {
        return ARSTREAM2_ERROR_INVALID_STATE;
    }
    uint8_t *pDst = (ret == 0) ? writer->pNaluBuf : writer->pNaluBuf - 1;
    uint8_t dst;
    bitsWritten = (bitsWritten / 8) * 8;

    /* destination zero bytes count */
    int dstNumZeros = 0;
    if ((dstOffset > 0) && (*(pDst - 1) == 0x00))
    {
        dstNumZeros++;
        if ((dstOffset > 1) && (*(pDst - 2) == 0x00))
        {
            dstNumZeros++;
        }
    }

    int srcBitOffset = writer->sliceContext.sliceHeaderLengthInBits & 7;
    int srcOffset = 1 + writer->sliceContext.sliceHeaderLengthInBits / 8;
    const uint8_t *pSrc = pbInputBuf + srcOffset;
    uint8_t src;

    int shiftL = (srcBitOffset - dstBitOffset + 8) % 8;
    int notshiftL = 8 - shiftL;
    int mask = (1 << shiftL) - 1;

    /* source zero bytes count */
    int srcNumZeros = 0;
    if ((srcOffset > 0) && (*(pSrc - 1) == 0x00))
    {
        srcNumZeros++;
        if ((srcOffset > 1) && (*(pSrc - 2) == 0x00))
        {
            srcNumZeros++;
        }
    }

    /* source init */
    if (srcBitOffset)
    {
        src = *pSrc++;
        length--;
        if ((srcNumZeros == 2) && (src == 0x03))
        {
            if (length < 1)
            {
                // UNSUPPORTED
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_WRITER_TAG, "Unsupported length-2 (%d)", length);
                return ARSTREAM2_ERROR_UNSUPPORTED;
            }

            /* remove the 0x03 byte */
            srcNumZeros = 0;
            src = *pSrc++;
            length--;
            if (src == 0x00)
            {
                srcNumZeros++;
            }
            else
            {
                srcNumZeros = 0;
            }
        }
        else if (src == 0x00)
        {
            srcNumZeros++;
        }
        else
        {
            srcNumZeros = 0;
        }
        src &= ((1 << (8 - srcBitOffset)) - 1);
    }
    else
    {
        src = 0;
    }

    /* destination init */
    if (dstBitOffset)
    {
        dst = *pDst;
        if (dstBitOffset)
        {
            dst &= (((1 << dstBitOffset) - 1) << (8 - dstBitOffset));
        }
        dst |= (src << shiftL);
    }
    else
    {
        dst = 0;
    }

    /* main loop */
    int i;
    for (i = 0; i < length; i++)
    {
        src = *pSrc++;
        if ((srcNumZeros == 2) && (src == 0x03))
        {
            /* remove the 0x03 byte */
            srcNumZeros = 0;
            src = *pSrc++;
            length--;
        }
        if (src == 0x00)
        {
            srcNumZeros++;
        }
        else
        {
            srcNumZeros = 0;
        }

        dst |= ((src >> notshiftL) & mask);
        if ((dstNumZeros == 2) && (dst <= 0x03))
        {
            /* 0x000000 or 0x000001 or 0x000002 or 0x000003 => insert 0x03 */
            *(pDst++) = 0x03;
            bitsWritten += 8;
            dstNumZeros = 0;
        }
        if (dst == 0)
        {
            dstNumZeros++;
        }
        else
        {
            dstNumZeros = 0;
        }
        *(pDst++) = dst;
        bitsWritten += 8;

        dst = src << shiftL;
    }

    /* last byte */
    if ((dstNumZeros == 2) && (dst <= 0x03))
    {
        /* 0x000000 or 0x000001 or 0x000002 or 0x000003 => insert 0x03 */
        *(pDst++) = 0x03;
        bitsWritten += 8;
        dstNumZeros = 0;
    }
    if (dst == 0)
    {
        dstNumZeros++;
    }
    else
    {
        dstNumZeros = 0;
    }
    *(pDst++) = dst;
    bitsWritten += 8;

    if (pDst[-1] == 0x00)
    {
        /* If the last byte was 0x80, after the shift the rbsp_stop_one_bit is on the previous byte
         * and we should not output a useless 0x00 byte => drop the last 0x00 byte */
        bitsWritten -= 8;
    }

    *outputSize = bitsWritten / 8;

    return ARSTREAM2_OK;
}


eARSTREAM2_ERROR ARSTREAM2_H264Writer_WriteSkippedPSliceNalu(ARSTREAM2_H264Writer_Handle writerHandle, unsigned int firstMbInSlice, unsigned int sliceMbCount, void *sliceContext, uint8_t *pbOutputBuf, unsigned int outputBufSize, unsigned int *outputSize)
{
    ARSTREAM2_H264Writer_t *writer = (ARSTREAM2_H264Writer_t*)writerHandle;
    int ret = 0, bitsWritten = 0;

    if ((!writerHandle) || (!pbOutputBuf) || (outputBufSize == 0) || (!outputSize))
    {
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }

    if (!writer->isSpsPpsContextValid)
    {
        return ARSTREAM2_ERROR_INVALID_STATE;
    }

    // Slice context
    if (sliceContext)
    {
        memcpy(&writer->sliceContext, sliceContext, sizeof(ARSTREAM2_H264_SliceContext_t));
        writer->sliceContext.first_mb_in_slice = firstMbInSlice;
        writer->sliceContext.sliceMbCount = sliceMbCount;
        writer->sliceContext.slice_type = (writer->sliceContext.slice_type >= 5) ? ARSTREAM2_H264_SLICE_TYPE_P_ALL : ARSTREAM2_H264_SLICE_TYPE_P;
        writer->sliceContext.sliceTypeMod5 = writer->sliceContext.slice_type % 5;
        writer->sliceContext.redundant_pic_cnt = 0;
        writer->sliceContext.direct_spatial_mv_pred_flag = 0;
        writer->sliceContext.slice_qp_delta = 0;
        writer->sliceContext.disable_deblocking_filter_idc = 2; // disable deblocking across slice boundaries
        writer->sliceContext.slice_alpha_c0_offset_div2 = 0;
        writer->sliceContext.slice_beta_offset_div2 = 0;
    }
    else
    {
        // UNSUPPORTED
        return ARSTREAM2_ERROR_UNSUPPORTED;
    }

    writer->pNaluBuf = pbOutputBuf;
    writer->naluBufSize = outputBufSize;
    writer->naluSize = 0;

    // Reset the bitstream cache
    writer->cache = 0;
    writer->cacheLength = 0;
    writer->oldZeroCount = 0;

    // NALU start code
    if (writer->config.naluPrefix)
    {
        ret = writeBits(writer, 32, ARSTREAM2_H264_BYTE_STREAM_NALU_START_CODE, 0);
        if (ret < 0)
        {
            return ARSTREAM2_ERROR_INVALID_STATE;
        }
        bitsWritten += ret;
    }

    // forbidden_zero_bit
    // nal_ref_idc
    // nal_unit_type
    ret = writeBits(writer, 8, ((writer->sliceContext.nal_ref_idc & 3) << 5) | writer->sliceContext.nal_unit_type, 0);
    if (ret < 0)
    {
        return ARSTREAM2_ERROR_INVALID_STATE;
    }
    bitsWritten += ret;

    // slice_header
    ret = ARSTREAM2_H264Writer_WriteSliceHeader(writer, &writer->sliceContext, &writer->spsContext, &writer->ppsContext);
    if (ret < 0)
    {
        return ARSTREAM2_ERROR_INVALID_STATE;
    }
    bitsWritten += ret;

    // slice_data
    ret = ARSTREAM2_H264Writer_WriteSkippedPSliceData(writer, &writer->sliceContext, &writer->spsContext, &writer->ppsContext);
    if (ret < 0)
    {
        return ARSTREAM2_ERROR_INVALID_STATE;
    }
    bitsWritten += ret;

    // rbsp_slice_trailing_bits

    // rbsp_trailing_bits
    ret = writeBits(writer, 1, 1, 1);
    if (ret < 0)
    {
        return ARSTREAM2_ERROR_INVALID_STATE;
    }
    bitsWritten += ret;

    ret = bitstreamByteAlign(writer, 1);
    if (ret < 0)
    {
        return ARSTREAM2_ERROR_INVALID_STATE;
    }
    bitsWritten += ret;

    //TODO: cabac_zero_word

    *outputSize = writer->naluSize;

    return ARSTREAM2_OK;
}


eARSTREAM2_ERROR ARSTREAM2_H264Writer_WriteGrayISliceNalu(ARSTREAM2_H264Writer_Handle writerHandle, unsigned int firstMbInSlice, unsigned int sliceMbCount, void *sliceContext, uint8_t *pbOutputBuf, unsigned int outputBufSize, unsigned int *outputSize)
{
    ARSTREAM2_H264Writer_t *writer = (ARSTREAM2_H264Writer_t*)writerHandle;
    int ret = 0, bitsWritten = 0;

    if ((!writerHandle) || (!pbOutputBuf) || (outputBufSize == 0) || (!outputSize))
    {
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }

    if (!writer->isSpsPpsContextValid)
    {
        return ARSTREAM2_ERROR_INVALID_STATE;
    }

    // Slice context
    if (sliceContext)
    {
        memcpy(&writer->sliceContext, sliceContext, sizeof(ARSTREAM2_H264_SliceContext_t));
        writer->sliceContext.first_mb_in_slice = firstMbInSlice;
        writer->sliceContext.sliceMbCount = sliceMbCount;
        writer->sliceContext.slice_type = (writer->sliceContext.slice_type >= 5) ? ARSTREAM2_H264_SLICE_TYPE_I_ALL : ARSTREAM2_H264_SLICE_TYPE_I;
        writer->sliceContext.sliceTypeMod5 = writer->sliceContext.slice_type % 5;
        writer->sliceContext.redundant_pic_cnt = 0;
        writer->sliceContext.direct_spatial_mv_pred_flag = 0;
        writer->sliceContext.slice_qp_delta = 0;
        writer->sliceContext.disable_deblocking_filter_idc = 2; // disable deblocking across slice boundaries
        writer->sliceContext.slice_alpha_c0_offset_div2 = 0;
        writer->sliceContext.slice_beta_offset_div2 = 0;
    }
    else
    {
        // UNSUPPORTED
        return ARSTREAM2_ERROR_UNSUPPORTED;
    }

    writer->pNaluBuf = pbOutputBuf;
    writer->naluBufSize = outputBufSize;
    writer->naluSize = 0;

    // Reset the bitstream cache
    writer->cache = 0;
    writer->cacheLength = 0;
    writer->oldZeroCount = 0;

    // NALU start code
    if (writer->config.naluPrefix)
    {
        ret = writeBits(writer, 32, ARSTREAM2_H264_BYTE_STREAM_NALU_START_CODE, 0);
        if (ret < 0)
        {
            return ARSTREAM2_ERROR_INVALID_STATE;
        }
        bitsWritten += ret;
    }

    // forbidden_zero_bit
    // nal_ref_idc
    // nal_unit_type
    ret = writeBits(writer, 8, ((writer->sliceContext.nal_ref_idc & 3) << 5) | writer->sliceContext.nal_unit_type, 0);
    if (ret < 0)
    {
        return ARSTREAM2_ERROR_INVALID_STATE;
    }
    bitsWritten += ret;

    // slice_header
    ret = ARSTREAM2_H264Writer_WriteSliceHeader(writer, &writer->sliceContext, &writer->spsContext, &writer->ppsContext);
    if (ret < 0)
    {
        return ARSTREAM2_ERROR_INVALID_STATE;
    }
    bitsWritten += ret;

    // slice_data
    ret = ARSTREAM2_H264Writer_WriteGrayISliceData(writer, &writer->sliceContext, &writer->spsContext, &writer->ppsContext);
    if (ret < 0)
    {
        return ARSTREAM2_ERROR_INVALID_STATE;
    }
    bitsWritten += ret;

    // rbsp_slice_trailing_bits

    // rbsp_trailing_bits
    ret = writeBits(writer, 1, 1, 1);
    if (ret < 0)
    {
        return ARSTREAM2_ERROR_INVALID_STATE;
    }
    bitsWritten += ret;

    ret = bitstreamByteAlign(writer, 1);
    if (ret < 0)
    {
        return ARSTREAM2_ERROR_INVALID_STATE;
    }
    bitsWritten += ret;

    //TODO: cabac_zero_word

    *outputSize = writer->naluSize;

    return ARSTREAM2_OK;
}


eARSTREAM2_ERROR ARSTREAM2_H264Writer_SetSpsPpsContext(ARSTREAM2_H264Writer_Handle writerHandle, const void *spsContext, const void *ppsContext)
{
    ARSTREAM2_H264Writer_t *writer = (ARSTREAM2_H264Writer_t*)writerHandle;

    if ((!writerHandle) || (!spsContext) || (!ppsContext))
    {
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }

    memcpy(&writer->spsContext, spsContext, sizeof(ARSTREAM2_H264_SpsContext_t));
    memcpy(&writer->ppsContext, ppsContext, sizeof(ARSTREAM2_H264_PpsContext_t));
    writer->isSpsPpsContextValid = 1;

    return ARSTREAM2_OK;
}


eARSTREAM2_ERROR ARSTREAM2_H264Writer_Init(ARSTREAM2_H264Writer_Handle* writerHandle, ARSTREAM2_H264Writer_Config_t* config)
{
    ARSTREAM2_H264Writer_t* writer;

    if (!writerHandle)
    {
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }

    writer = (ARSTREAM2_H264Writer_t*)malloc(sizeof(*writer));
    if (!writer)
    {
        return ARSTREAM2_ERROR_ALLOC;
    }
    memset(writer, 0, sizeof(*writer));

    if (config)
    {
        memcpy(&writer->config, config, sizeof(writer->config));
    }

    *writerHandle = (ARSTREAM2_H264Writer_Handle*)writer;

    return ARSTREAM2_OK;
}


eARSTREAM2_ERROR ARSTREAM2_H264Writer_Free(ARSTREAM2_H264Writer_Handle writerHandle)
{
    ARSTREAM2_H264Writer_t* writer = (ARSTREAM2_H264Writer_t*)writerHandle;

    if (!writerHandle)
    {
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }

    free(writer);

    return ARSTREAM2_OK;
}

