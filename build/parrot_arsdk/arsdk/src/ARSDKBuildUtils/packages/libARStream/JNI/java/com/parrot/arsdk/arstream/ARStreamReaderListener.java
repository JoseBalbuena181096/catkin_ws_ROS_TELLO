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
package com.parrot.arsdk.arstream;

import com.parrot.arsdk.arsal.ARNativeData;

/**
 * This interface describes a listener of ARStreamReader events
 */
public interface ARStreamReaderListener
{
    /**
     * This callback can be called in different cases:<br>
     *  - Frame complete:<br>
     *    - 'currentFrame' contains a complete frame and won't be used by ARStreamReader again<br>
     *    - 'isFlushFrame' is true if the complete frame occured after a buffer flush (on most cases, the complete frame is an I-Frame)<br>
     *    - 'nbSkippedFrames' contains the number of skipped frames since the last complete frame<br>
     *    - 'newBufferCapacity' is undefined<br>
     *    - Return value is the storing location of the next frame<br>
     *  - Frame too small:<br>
     *    - 'currentFrame' should not be modified (it is still used by the ARStreamReader)<br>
     *    - 'isFlushFrame' is undefined<br>
     *    - 'nbSkippedFrames' is undefined<br>
     *    - 'newBufferCapacity' holds a suitable capacity for the new buffer.<br>
     *    - Return value is a new ARNativeData, greater than the currentFrame buffer.<br>
     *      -- If the returned value is null or smaller than currentFrame, the frame is discarded<br>
     *  - Copy complete: (only called after a Frame too small)<br>
     *    - 'currentFrame' is the previous frame buffer, which can now be freed (or reused)<br>
     *    - 'isFlushFrame' is undefined<br>
     *    - 'nbSkippedFrames' is undefined<br>
     *    - 'newBufferCapacity' is undefined<br>
     *    - Return value is unused and should be null<br>
     *  - Cancel:<br>
     *    - 'currentFrame' is the frame buffer which is cancelled, and which can be freed<br>
     *    - 'isFlushFrame' is undefined<br>
     *    - 'nbSkippedFrames' in undefined<br>
     *    - 'newBufferCapacity' is undefined<br>
     *    - Return value is unused and should be null
     * @param cause The event that triggered this call (see global func description)
     * @param currentFrame The frame buffer for the event (see global func description)
     * @param isFlushFrame Indicates if the frame forced a sender flush. This is typically set on I-Frames (see global func description)
     * @param nbSkippedFrames The number of frames skipped since last complete frame (see global func description)
     * @param[inout] newBufferCapacity Capacity of the next buffer to use
     * @return The new frame buffer, or null (depending on cause, see global func description)
     */
    public ARNativeData didUpdateFrameStatus (ARSTREAM_READER_CAUSE_ENUM cause, ARNativeData currentFrame, boolean isFlushFrame, int nbSkippedFrames, int newBufferCapacity);
}
