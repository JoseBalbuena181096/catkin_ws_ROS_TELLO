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
package com.parrot.arsdk.arsal;

import com.parrot.arsdk.arsal.ARSAL_PRINT_LEVEL_ENUM;
import java.lang.reflect.Method;
import java.text.SimpleDateFormat;
import java.util.Date;

import java.io.PrintWriter;
import java.io.StringWriter;
import java.net.UnknownHostException;

public class ARSALPrint
{

    private ARSALPrint ()
    {
        // Do nothing in this constructor
    }

    public static boolean setMinimumLogLevel(ARSAL_PRINT_LEVEL_ENUM level)
    {
        return nativeSetMinLevel(level.getValue());
    }

    public static ARSAL_PRINT_LEVEL_ENUM getMinimumLogLevel()
    {
        int i_val = nativeGetMinLevel();
        ARSAL_PRINT_LEVEL_ENUM ret = ARSAL_PRINT_LEVEL_ENUM.getFromValue(i_val);
        return ret;
    }

    private static String getStackTraceString(Throwable tr)
    {
        if (tr == null)
        {
            return "";
        }

        // This is to reduce the amount of log spew that apps do in the non-error
        // condition of the network being unavailable.
        Throwable t = tr;
        while (t != null)
        {
            if (t instanceof UnknownHostException)
            {
                return "";
            }
            t = t.getCause();
        }

        StringWriter sw = new StringWriter();
        PrintWriter pw = new PrintWriter(sw);
        tr.printStackTrace(pw);
        return sw.toString();
    }

    public static void v (String tag, String message, Throwable t)
    {
        print (ARSAL_PRINT_LEVEL_ENUM.ARSAL_PRINT_VERBOSE, tag, message + '\n' + getStackTraceString(t));
    }

    public static void wtf (String tag, String message, Throwable t)
    {
        print (ARSAL_PRINT_LEVEL_ENUM.ARSAL_PRINT_FATAL, tag, message + '\n' + getStackTraceString(t));
    }

    public static void i (String tag, String message, Throwable t)
    {
        print (ARSAL_PRINT_LEVEL_ENUM.ARSAL_PRINT_INFO, tag, message + '\n' + getStackTraceString(t));
    }

    public static void d (String tag, String message, Throwable t)
    {
        print (ARSAL_PRINT_LEVEL_ENUM.ARSAL_PRINT_DEBUG, tag, message + '\n' + getStackTraceString(t));
    }

    public static void w (String tag, String message, Throwable t)
    {
        print (ARSAL_PRINT_LEVEL_ENUM.ARSAL_PRINT_WARNING, tag, message + '\n' + getStackTraceString(t));
    }

    public static void e (String tag, String message, Throwable t)
    {
        print (ARSAL_PRINT_LEVEL_ENUM.ARSAL_PRINT_ERROR, tag, message + '\n' + getStackTraceString(t));
    }

    public static void v (String tag, String message)
    {
        print (ARSAL_PRINT_LEVEL_ENUM.ARSAL_PRINT_VERBOSE, tag, message);
    }

    public static void wtf (String tag, String message)
    {
        print (ARSAL_PRINT_LEVEL_ENUM.ARSAL_PRINT_FATAL, tag, message);
    }

    public static void i (String tag, String message)
    {
        print (ARSAL_PRINT_LEVEL_ENUM.ARSAL_PRINT_INFO, tag, message);
    }

    public static void d (String tag, String message)
    {
        print (ARSAL_PRINT_LEVEL_ENUM.ARSAL_PRINT_DEBUG, tag, message);
    }

    public static void w (String tag, String message)
    {
        print (ARSAL_PRINT_LEVEL_ENUM.ARSAL_PRINT_WARNING, tag, message);
    }

    public static void e (String tag, String message)
    {
        print (ARSAL_PRINT_LEVEL_ENUM.ARSAL_PRINT_ERROR, tag, message);
    }

    public static void print (ARSAL_PRINT_LEVEL_ENUM level, String tag, String message)
    {
        nativePrint (level.getValue(), tag, message);
    }

    private static native void nativePrint (int level, String tag, String message);
    private static native boolean nativeSetMinLevel(int level);
    private static native int nativeGetMinLevel();
}
