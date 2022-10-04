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

import android.os.Handler;
import android.os.HandlerThread;
import android.os.Message;

import com.parrot.arsdk.ardiscovery.mdnssdmin.internal.MdnsSdIncomingResponse;
import com.parrot.arsdk.ardiscovery.mdnssdmin.internal.MdnsSdOutgoingQuery;
import com.parrot.arsdk.arsal.ARSALPrint;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.MulticastSocket;
import java.net.NetworkInterface;

/**
 * A minimal implementation of MDSN service discovery
 * <p/>
 * This implementation doesn't handle caching and time to live. It's specially customised to discover
 * one single device on the network. It heavily send queries to ensure the first device is discovered
 * quickly.
 */
public class MdnsSdMin
{
    /**
     * Listener notified when a service has been found or is about to be removed.
     * <p/>
     * Note: callback are called in an internal thread!
     */
    public interface Listener
    {
        /**
         * Notify that a service has been found
         *
         * @param name        unqualified service name
         * @param serviceType service type
         * @param ipAddress   service ip address
         * @param port        service port
         * @param txts        service additional data
         */
        public void onServiceAdded(String name, String serviceType, String ipAddress, int port, String[] txts);

        /**
         * Notify that a device is about to be removed
         *
         * @param name        unqualified service name
         * @param serviceType service type
         */
        public void onServiceRemoved(String name, String serviceType);
    }

    /**
     * Array of service to search for and notify
     */
    private final String[] services;
    /**
     * Prebuilt mdns query
     */
    private final MdnsSdOutgoingQuery query;
    /**
     * Client listener
     */
    private final Listener listener;
    /**
     * mdns multicast socket
     */
    private MulticastSocket socket;
    /**
     * Thread listening on the multicast socket and decoding received mdns packets
     */
    private Thread receiveThread;
    /**
     * Thread sending mdns queries
     */
    private HandlerThread queryThread;
    /**
     * Handler of the queryThread
     */
    private Handler queryHandler;

    /**
     * Logging tag
     */
    private final static String TAG = "MdnsSdMin";
    /**
     * Interval between queries
     */
    private final static int QUERY_INTERVAL_MS = 250;
    /**
     * Duration to send queries for at startup
     */
    private final static int QUERY_DURATION_MS = 5 * 1000;
    /**
     * MDNS multicast group address
     */
    private final String MDNS_MULTICAST_ADDR = "224.0.0.251";
    /**
     * MDNS multicast port
     */
    private final int MDNS_MULTICAST_PORT = 5353;

    /**
     * Constructor
     *
     * @param services array of services to look-for an notify
     * @param listener listener notified when a service has been found or is about to be removed
     */
    public MdnsSdMin(String[] services, Listener listener)
    {
        this.services = services;
        this.listener = listener;
        // create the query
        query = new MdnsSdOutgoingQuery(services);
    }

    /**
     * Starts service discovery
     * @param netInterface interface to bind to, null to use the interface defined by the routing table
     */
    public void start(NetworkInterface netInterface)
    {
        ARSALPrint.d(TAG, "Starting MdsnSd");
        if (socket == null)
        {
            // create multicast socket
            try
            {
                socket = new MulticastSocket(MDNS_MULTICAST_PORT);
                if (netInterface != null)
                {
                    // if a interface has been specified, bind the multicast socket to this interface
                    socket.setNetworkInterface(netInterface);
                    socket.joinGroup(new InetSocketAddress(InetAddress.getByName(MDNS_MULTICAST_ADDR), MDNS_MULTICAST_PORT), netInterface);
                }
                else
                {
                    socket.joinGroup(InetAddress.getByName(MDNS_MULTICAST_ADDR));
                }
                socket.setTimeToLive(255);
                // start the receiver thread
                receiveThread = new ReceiverThread(socket);
                receiveThread.start();
                // start the query thread
                queryThread = new QueryThread(socket);
                queryThread.start();
            }
            catch (IOException e)
            {
                ARSALPrint.e(TAG, "unable to start MdsnSd", e);
            }
        }
    }

    /**
     * Stops service discovery
     */
    public void stop()
    {
        ARSALPrint.d(TAG, "Stopping MdsnSd");
        if (socket != null)
        {
            socket.close();
            socket = null;
            receiveThread = null;
            if (queryThread != null)
            {
                queryThread.quit();
                queryThread = null;
            }
        }
    }

    /**
     * Send queries to discover services.
     * <p/>
     * This methods send a lot of queries at regular interval. Pending queries can be cancelled by
     * calling {@link #cancelSendQueries}
     */
    public void sendQueries()
    {
        ARSALPrint.d(TAG, "Sending queries");
        for (int t = 0; t < QUERY_DURATION_MS; t += QUERY_INTERVAL_MS)
        {
            queryHandler.sendMessageDelayed(queryHandler.obtainMessage(0), t);
        }
    }

    /**
     * Cancel sending queries previously queued by {@link #sendQueries}
     */
    public void cancelSendQueries()
    {
        ARSALPrint.d(TAG, "Cancel sending queries");
        queryHandler.removeMessages(0);
    }

    /**
     * Thread receiving mdns udp packets
     */
    private class ReceiverThread extends Thread
    {
        /**
         * UDP socket
         */
        private final DatagramSocket socket;

        /**
         * Constructor
         *
         * @param socket udp socket (already openend) to use
         */
        public ReceiverThread(DatagramSocket socket)
        {
            super("MdnsSd-receiver");
            this.socket = socket;
        }

        @Override
        public void run()
        {
            byte[] buffer =  new byte[1500];
            while (!socket.isClosed())
            {
                try
                {
                    DatagramPacket packet = new DatagramPacket(buffer, buffer.length);
                    socket.receive(packet);
                    MdnsSdIncomingResponse r = new MdnsSdIncomingResponse(packet.getData());
                    handleResponse(r);
                } catch (Throwable e)
                {
                    // Catch all exceptions, to protect ourself agains bad dns packets
                    ARSALPrint.w(TAG, "Ignoring received packet due to " + e.getMessage());
                }
            }
        }

        /**
         * Handled a mdns response
         *
         * @param response the response message
         */
        private void handleResponse(MdnsSdIncomingResponse response)
        {
            // iterate expected PTRs
            for (String question : services)
            {
                final String ptr = response.getPtr(question);
                if (ptr != null)
                {
                    // found a ptr, the corresponding service
                    final MdnsSrvData srv = response.getService(ptr);
                    if (srv != null)
                    {
                        final String address = response.getAddress(srv.getTarget());
                        final String[] txts = response.getTexts(ptr);
                        if (address != null)
                        {
                            // ptr is the full qualified name. extract device and service name
                            int pos = -1;
                            if (ptr.endsWith(question))
                            {
                                pos = ptr.length() - question.length();
                            }
                            final String name = ptr.substring(0, pos > 0 ? pos - 1 : ptr.length());
                            if (srv.getTtl() > 0)
                            {
                                ARSALPrint.d(TAG, "New service " + name);
                                listener.onServiceAdded(name, question, address, srv.getPort(), txts);
                            } else
                            {
                                ARSALPrint.d(TAG, "Service removed " + name);
                                listener.onServiceRemoved(name, question);
                            }
                        }
                    }
                }
            }
        }
    }

    /**
     * Thread used to send queries
     */
    private class QueryThread extends HandlerThread
    {
        /**
         * UDP socket
         */
        private final DatagramSocket socket;

        /**
         * Constructor
         *
         * @param socket udp socket (already openend) to use
         */
        public QueryThread(DatagramSocket socket)
        {
            super("MdnsSd-query");
            this.socket = socket;
        }

        @Override
        protected void onLooperPrepared()
        {
            if (!socket.isClosed())
            {
                queryHandler = new Handler(getLooper())
                {
                    @Override
                    public void handleMessage(Message msg)
                    {
                       ARSALPrint.d(TAG, "sending query packet");
                       try
                        {
                            byte[] buf = query.encode();
                            DatagramPacket packet = new DatagramPacket(buf, buf.length,
                                    InetAddress.getByName(MDNS_MULTICAST_ADDR), MDNS_MULTICAST_PORT);
                            socket.send(packet);
                        } catch (IOException e)
                        {
                            ARSALPrint.e(TAG, "unable to start query", e);
                        }
                    }
                };
                // do the first query
                sendQueries();
            } else
            {
                // socket has been closed, exit the looper
                getLooper().quit();
            }
        }
    }

}
