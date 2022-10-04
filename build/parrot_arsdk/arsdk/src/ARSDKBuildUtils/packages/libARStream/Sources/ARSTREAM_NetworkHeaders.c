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
 * @file ARSTREAM_NetworkHeaders.c
 * @brief Stream headers on network
 * @date 03/22/2013
 * @author nicolas.brulez@parrot.com
 */

#include <config.h>

/*
 * System Headers
 */

/*
 * Private Headers
 */
#include "ARSTREAM_NetworkHeaders.h"

/*
 * ARSDK Headers
 */
#include <libARSAL/ARSAL_Print.h>

/*
 * Macros
 */
#define ARSTREAM_NETWORK_HEADERS_TAG "ARSTREAM_NetworkHeaders"

/*
 * Types
 */

/*
 * Internal functions declarations
 */

/**
 * @brief Computes the Hamming weight of a 32 bit integer
 * The Hamming weight is the number of '1' bits in the integer binary representation
 * @param input The integer to test
 * @return The Hamming weight of the integer
 */
uint32_t ARSTREAM_NetworkHeaders_HammingWeight32 (uint32_t input);

/*
 * Internal functions implementation
 */

uint32_t ARSTREAM_NetworkHeaders_HammingWeight32 (uint32_t input)
{
    uint32_t tst = input;
    tst = tst - ((tst >> 1) & 0x55555555);
    tst = (tst & 0x33333333) + ((tst >> 2) & 0x33333333);
    return (((tst + (tst >> 4)) & 0x0F0F0F0F) * 0x01010101) >> 24;
}

/*
 * Implementation
 */

int ARSTREAM_NetworkHeaders_AckPacketAllFlagsSet (ARSTREAM_NetworkHeaders_AckPacket_t *packet, int maxFlag)
{
    int res = 1;
    if (0 < maxFlag && maxFlag <= 64)
    {
        // Check only in first packet
        uint64_t lo_mask = (1ll << maxFlag) - 1ll;
        res = ((packet->lowPacketsAck & lo_mask) == lo_mask) ? 1 : 0;
    }
    else if (64 < maxFlag && maxFlag <= 128)
    {
        // We need to check for the second packet also
        uint64_t lo_mask = UINT64_MAX;
        uint64_t hi_mask = (1ll << (maxFlag-64)) - 1ll;
        int hi_res = ((packet->highPacketsAck & hi_mask) == hi_mask) ? 1 : 0;
        int lo_res = ((packet->lowPacketsAck & lo_mask) == lo_mask) ? 1 : 0;
        res = (hi_res == 1 && lo_res == 1) ? 1 : 0;
    }
    else
    {
        // We ask for more bits that we have, return 'false'
        res = 0;
    }
    return res;
}

int ARSTREAM_NetworkHeaders_AckPacketFlagIsSet (ARSTREAM_NetworkHeaders_AckPacket_t *packet, int flag)
{
    int retVal = 0;
    if (0 <= flag && flag < 64)
    {
        retVal = ((packet->lowPacketsAck & (1ll << flag)) != 0) ? 1 : 0;
    }
    else if (64 <= flag && flag < 128)
    {
        retVal = ((packet->highPacketsAck & (1ll << (flag - 64))) != 0) ? 1 : 0;
    }
    return retVal;
}

void ARSTREAM_NetworkHeaders_AckPacketReset (ARSTREAM_NetworkHeaders_AckPacket_t *packet)
{
    packet->highPacketsAck = 0ll;
    packet->lowPacketsAck = 0ll;
}

void ARSTREAM_NetworkHeaders_AckPacketResetUpTo (ARSTREAM_NetworkHeaders_AckPacket_t *packet, int maxFlag)
{
    if (0 <= maxFlag && maxFlag < 64)
    {
        packet->highPacketsAck = UINT64_MAX;
        packet->lowPacketsAck = UINT64_MAX << maxFlag;
    }
    else if (64 <= maxFlag && maxFlag < 128)
    {
        packet->highPacketsAck = UINT64_MAX << (maxFlag - 64);
        packet->lowPacketsAck = 0ll;
    }
    else
    {
        packet->highPacketsAck = 0ll;
        packet->lowPacketsAck = 0ll;
    }
}

void ARSTREAM_NetworkHeaders_AckPacketSetFlag (ARSTREAM_NetworkHeaders_AckPacket_t *packet, int flagToSet)
{
    if (0 <= flagToSet && flagToSet < 64)
    {
        packet->lowPacketsAck |= (1ll << flagToSet);
    }
    else if (64 <= flagToSet && flagToSet < 128)
    {
        packet->highPacketsAck |= (1ll << (flagToSet-64));
    }
}

void ARSTREAM_NetworkHeaders_AckPacketSetFlags (ARSTREAM_NetworkHeaders_AckPacket_t *dst, ARSTREAM_NetworkHeaders_AckPacket_t *src)
{
    dst->highPacketsAck |= src->highPacketsAck;
    dst->lowPacketsAck  |= src->lowPacketsAck;
}

int ARSTREAM_NetworkHeaders_AckPacketUnsetFlag (ARSTREAM_NetworkHeaders_AckPacket_t *packet, int flagToRemove)
{
    int retVal = 0;
    if (0 <= flagToRemove && flagToRemove < 64)
    {
        packet->lowPacketsAck &= ~(1ll << flagToRemove);
    }
    else if (64 <= flagToRemove && flagToRemove < 128)
    {
        packet->highPacketsAck &= ~(1ll << flagToRemove);
    }

    if (0ll == packet->lowPacketsAck &&
        0ll == packet->highPacketsAck)
    {
        retVal = 1;
    }
    return retVal;
}

int ARSTREAM_NetworkHeaders_AckPacketUnsetFlags (ARSTREAM_NetworkHeaders_AckPacket_t *dst, ARSTREAM_NetworkHeaders_AckPacket_t *src)
{
    int retVal = 0;
    dst->highPacketsAck &= ~(src->highPacketsAck);
    dst->lowPacketsAck  &= ~(src->lowPacketsAck);
    if (0ll == dst->lowPacketsAck &&
        0ll == dst->highPacketsAck)
    {
        retVal = 1;
    }
    return retVal;
}

uint32_t ARSTREAM_NetworkHeaders_AckPacketCountSet (ARSTREAM_NetworkHeaders_AckPacket_t *packet, int nb)
{
    uint32_t retVal = 0;

    // Get subpacket
    uint32_t tst = (uint32_t)(packet->lowPacketsAck);
    // Mask if needed
    tst = (nb < 32) ? tst & ((1 << nb) - 1) : tst;
    // Add its Hamming weight to the result
    retVal += ARSTREAM_NetworkHeaders_HammingWeight32 (tst);

    // repeat for other three subpackets (if nb is big enough to reach them)
    if (nb > 32)
    {
        tst = (uint32_t)(packet->lowPacketsAck >> 32);
        tst = (nb < 64) ? tst & ((1 << (nb-32)) - 1) : tst;
        retVal += ARSTREAM_NetworkHeaders_HammingWeight32 (tst);
    }
    if (nb > 64)
    {
        tst = (uint32_t)(packet->highPacketsAck);
        tst = (nb < 96) ? tst & ((1 << (nb-64)) - 1) : tst;
        retVal += ARSTREAM_NetworkHeaders_HammingWeight32 (tst);
    }
    if (nb > 96)
    {
        tst = (uint32_t)(packet->highPacketsAck >> 32);
        tst = (nb < 128) ? tst & ((1 << (nb-96)) - 1) : tst;
        retVal += ARSTREAM_NetworkHeaders_HammingWeight32 (tst);
    }
    return retVal;
}

uint32_t ARSTREAM_NetworkHeaders_AckPacketCountNotSet (ARSTREAM_NetworkHeaders_AckPacket_t *packet, int nb)
{
    uint32_t retVal = 0;

    // Get subpacket
    uint32_t tst = (uint32_t)(packet->lowPacketsAck);
    // Mask if needed
    tst = (nb < 32) ? tst | (0xFFFFFFFF << nb) : tst;
    // Add its Hamming weight to the result
    retVal += (32 - ARSTREAM_NetworkHeaders_HammingWeight32 (tst));

    // repeat for other three subpackets (if nb is big enough to reach them)
    if (nb > 32)
    {
        tst = (uint32_t)(packet->lowPacketsAck >> 32);
        tst = (nb < 64) ? tst | (0xFFFFFFFF << (nb - 32)) : tst;
        retVal += (32 - ARSTREAM_NetworkHeaders_HammingWeight32 (tst));
    }
    if (nb > 64)
    {
        tst = (uint32_t)(packet->highPacketsAck);
        tst = (nb < 96) ? tst | (0xFFFFFFFF << (nb - 64)) : tst;
        retVal += (32 - ARSTREAM_NetworkHeaders_HammingWeight32 (tst));
    }
    if (nb > 96)
    {
        tst = (uint32_t)(packet->highPacketsAck >> 32);
        tst = (nb < 128) ? tst | (0xFFFFFFFF << (nb - 96)) : tst;
        retVal += (32 - ARSTREAM_NetworkHeaders_HammingWeight32 (tst));
    }
    return retVal;
}

static void ARSTREAM_NetworkHeaders_InternalAckPacketDump (const char *prefix, ARSTREAM_NetworkHeaders_AckPacket_t *packet, eARSAL_PRINT_LEVEL level)
{
    ARSAL_PRINT (level, ARSTREAM_NETWORK_HEADERS_TAG, "Packet dump: %s", prefix);
    if (packet == NULL)
    {
        ARSAL_PRINT (level, ARSTREAM_NETWORK_HEADERS_TAG, "Packet is NULL");
    }
    else
    {
        ARSAL_PRINT (level, ARSTREAM_NETWORK_HEADERS_TAG, " - Frame number : %d", packet->frameNumber);
        ARSAL_PRINT (level, ARSTREAM_NETWORK_HEADERS_TAG, " - HI 64 bits : %016" PRIu64, packet->highPacketsAck);
        ARSAL_PRINT (level, ARSTREAM_NETWORK_HEADERS_TAG, " - LO 64 bits : %016" PRIu64, packet->lowPacketsAck);
    }
}

void ARSTREAM_NetworkHeaders_AckPacketDump (const char *prefix, ARSTREAM_NetworkHeaders_AckPacket_t *packet)
{
    ARSTREAM_NetworkHeaders_InternalAckPacketDump (prefix, packet, ARSAL_PRINT_DEBUG);
}
