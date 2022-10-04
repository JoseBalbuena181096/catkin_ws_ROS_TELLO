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

package com.parrot.arsdk.ardiscovery.mdnssdmin.internal;

/**
 * A Mdsn Record identified by a name and a type.
 *
 * Correspond to an entry in dns response or additional data.
 */
public class MdnsRecord
{
    /**
     * Record types.
     * Contains only the subset of types managed by mdsnsdmin
     */
    enum Type
    {
        /** Address record: data is a String containing an IP Address */
        A((byte) 1),
        /** PTR record: data is a String containing a host name */
        PTR((byte) 12),
        /** TXT record: data is a String array containing TXTs */
        TXT((byte) 16),
        /** SRV record: data is a MdnsSrv containing the service port and target */
        SRV((byte) 33);

        /** Type numerical value */
        private final byte val;

        /**
         * Constructor
         * @param val type numerical value
         */
        private Type(byte val)
        {
            this.val = val;
        }

        /**
         * Gets a Type by its numerical value
         * @param val type numerical value
         * @return corresponding type, or null
         */
        static Type get(int val)
        {
            for (Type type : values())
            {
                if (type.val == val)
                {
                    return type;
                }
            }
            return null;
        }
    }

    /** Record name */
    protected final String name;
    /** Record type */
    protected final Type type;

    /**
     * Constructor
     * @param name record name
     * @param type record type
     */
    public MdnsRecord(String name, Type type)
    {
        this.name = name;
        this.type = type;
    }

    /**
     * Gets the record type
     * @return record type.
     */
    public Type getType()
    {
        return this.type;
    }

    @Override
    public boolean equals(Object o)
    {
        if (this == o) return true;
        if (!(o instanceof MdnsRecord)) return false;

        MdnsRecord mdnsEntry = (MdnsRecord) o;

        if (!name.equals(mdnsEntry.name)) return false;
        if (type != mdnsEntry.type) return false;

        return true;
    }

    @Override
    public int hashCode()
    {
        int result = name.hashCode();
        result = 31 * result + type.hashCode();
        return result;
    }
}
