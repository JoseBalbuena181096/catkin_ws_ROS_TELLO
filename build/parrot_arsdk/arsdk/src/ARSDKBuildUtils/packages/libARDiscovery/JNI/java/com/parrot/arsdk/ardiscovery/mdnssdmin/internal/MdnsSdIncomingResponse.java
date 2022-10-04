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

import com.parrot.arsdk.ardiscovery.mdnssdmin.MdnsSrvData;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

/**
 * A mdns incoming response message
 */
public class MdnsSdIncomingResponse
{
    /** All parsed entries, by record. Class of an entry value depends of its type */
    private final Map<MdnsRecord, Object> entries;

    /**
     * Constructor
     * @param data udp payload received
     */
    public MdnsSdIncomingResponse(byte[] data)
    {
        this.entries = new HashMap<MdnsRecord, Object>();
        new Decoder().decode(data);
    }

    /**
     * Gets an A entry by name
     * @param name the name to lookup
     * @return A value (an ip address) or null if not found
     */
    public String getAddress(String name)
    {
        return (String) this.entries.get(new MdnsRecord(name, MdnsRecord.Type.A));
    }

    /**
     * Gets an PTR entry by name
     * @param name the name to lookup
     * @return PTR value (an domain name) or null if not found
     */
    public String getPtr(String name)
    {
        return (String) this.entries.get(new MdnsRecord(name, MdnsRecord.Type.PTR));
    }

    /**
     * Gets an SRV entry by name
     * @param name the name to lookup
     * @return SRV values (ip, name, ttl) or null if not found
     */
    public MdnsSrvData getService(String name)
    {
        return (MdnsSrvData) this.entries.get(new MdnsRecord(name, MdnsRecord.Type.SRV));

    }

    /**
     * Gets an TXT entry by name
     * @param name the name to lookup
     * @return TXT values (an array of strings) or null if not found
     */
    public String[] getTexts(String name)
    {
        return (String[]) this.entries.get(new MdnsRecord(name, MdnsRecord.Type.TXT));
    }

    /**
     * DNS message decoder
     */
    private class Decoder
    {
        /** current pos in the buffer */
        private int pos;
        /** Udp received bytes */
        private byte[] buffer;

        /**
         * Constructor
         *
         * @param data Udp received data
         */
        @SuppressWarnings("UnusedDeclaration")
        public void decode(byte[] data)
        {
            buffer = data;
            pos = 0;
            // header
            int id = readU16();
            int flags = readU16();
            int questionsCnt = readU16();
            int answersCnt = readU16();
            int authoritiesCnt = readU16();
            int additionalRRsCnt = readU16();

            // sanity check
            if (id == 0 && questionsCnt >= 0 && answersCnt >= 0 && authoritiesCnt >= 0 && additionalRRsCnt >= 0)
            {
                // skip questions if any
                for (int cnt = 0; cnt < questionsCnt; cnt++)
                {
                    String name = readName();
                    int type = readU16();
                    int cls = readU16();
                }

                // read answers
                for (int cnt = 0; cnt < answersCnt; cnt++)
                {
                    readResourceRecord();
                }

                // read authorities
                for (int cnt = 0; cnt < authoritiesCnt; cnt++)
                {
                    readResourceRecord();
                }

                // read additional records
                for (int cnt = 0; cnt < additionalRRsCnt; cnt++)
                {
                    readResourceRecord();
                }
            }
        }

        /**
         * Read a received resource record
         */
        @SuppressWarnings("UnusedDeclaration")
        private void readResourceRecord()
        {
            String name = readName();
            MdnsRecord.Type type = MdnsRecord.Type.get(readU16());
            int cls = readU16();
            long ttl = readU32();
            int datalen = readU16();
            if (type != null)
            {
                switch (type)
                {
                    case A:
                        if (datalen == 4)
                        {
                            String address = String.format(Locale.US, "%d.%d.%d.%d", readU8(), readU8(), readU8(), readU8());
                            entries.put(new MdnsRecord(name, type), address);
                        }
                        break;

                    case PTR:
                        String domainName = readName();
                        entries.put(new MdnsRecord(name, type), domainName);
                        break;

                    case TXT:
                        int end = pos + datalen;
                        List<String> txts = new ArrayList<String>();
                        while (pos < end)
                        {
                            txts.add(readString());
                        }
                        entries.put(new MdnsRecord(name, type), txts.toArray(new String[txts.size()]));
                        break;

                    case SRV:
                        int priority = readU16();
                        int weight = readU16();
                        int port = readU16();
                        String target = readName();
                        entries.put(new MdnsRecord(name, type), new MdnsSrvData(port, target, ttl));
                        break;
                    default:
                }
            } else
            {
                // unhandled types: skip data
                pos += datalen;
            }
        }

        /**
         * Read an unsigned 8 bits int
         * @return unsigned 8 bits int at current position
         */
        private int readU8()
        {
            return ((int) buffer[pos++]) & 0xFF;
        }

        /**
         * Read an unsigned 16 bits int
         * @return unsigned 16 bits int at current position
         */
        private int readU16()
        {
            return ((readU8() << 8) & 0xFFFF) | ((readU8()) & 0xFF);
        }

        /**
         * Read an unsigned 32 bits int
         * @return unsigned 32 bits int at current position
         */
        private long readU32()
        {
            return (((long) readU16()) << 16) | (((long) readU16()));
        }

        /**
         * Read a dns name
         * @return dns name at current position
         */
        private String readName()
        {
            StringBuilder sb = new StringBuilder();
            readNameSegment(sb);
            return sb.toString();
        }

        /**
         * Read a dns name fragment and append it to a string builder
         * @param sb string builder to append the fragment to
         */
        private void readNameSegment(StringBuilder sb)
        {
            int len = readU8();
            while (len != 0)
            {
                if ((len & 0xC0) != 0)
                {
                    // reference to other string
                    int offset = (len & 0x003F) << 8 | readU8() & 0xFF;
                    int savedPos = pos;
                    pos = offset;
                    readNameSegment(sb);
                    pos = savedPos;
                    len = 0;
                } else
                {
                    for (int cnt = 0; cnt < len; cnt++)
                    {
                        sb.append((char) buffer[pos++]);
                    }
                    sb.append('.');
                    len = readU8();
                }
            }
        }

        /**
         * Read a dns string
         * @return string at current position
         */
        public String readString()
        {
            StringBuilder sb = new StringBuilder();
            int len = readU8();
            for (int cnt = 0; cnt < len; cnt++)
            {
                sb.append((char) buffer[pos++]);
            }
            return sb.toString();
        }
    }
}
