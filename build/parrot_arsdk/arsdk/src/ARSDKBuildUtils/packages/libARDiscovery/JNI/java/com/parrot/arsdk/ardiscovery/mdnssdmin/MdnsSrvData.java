/*
 Copyright (C) 2015 Parrot SA

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

package com.parrot.arsdk.ardiscovery.mdnssdmin;

/**
 * Store mdns SRV data
 */
public class MdnsSrvData
{
    /** Service port */
    private final int port;
    /** Service dns address */
    private final String target;
    /** Service time to live */
    private final long ttl;

    /**
     * Constructor
     * @param port Service port
     * @param target Service dns address
     * @param ttl Service time to live
     */
    public MdnsSrvData(int port, String target, long ttl)
    {
        this.port = port;
        this.target = target;
        this.ttl = ttl;
    }

    /**
     * Gets the service port
     * @return the service port
     */
    public int getPort()
    {
        return port;
    }

    /**
     * Gets the service target name
     * @return the service target name
     */
    public String getTarget()
    {
        return target;
    }

    /**
     * Gets the time to live
     * @return the time to live
     */
    public long getTtl()
    {
        return ttl;
    }
}
