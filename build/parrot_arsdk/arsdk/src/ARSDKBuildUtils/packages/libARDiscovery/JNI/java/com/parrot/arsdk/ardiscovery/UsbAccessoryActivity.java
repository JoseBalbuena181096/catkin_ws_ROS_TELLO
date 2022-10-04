/*
    Copyright (C) 2016 Parrot SA

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

package com.parrot.arsdk.ardiscovery;

import android.app.Activity;
import android.app.ActivityManager;
import android.content.Context;
import android.content.Intent;
import android.hardware.usb.UsbManager;
import android.os.Build;
import android.os.Bundle;

public abstract class UsbAccessoryActivity extends Activity
{
    protected abstract Class getBaseActivity();

    @Override
    protected void onCreate(Bundle savedInstanceState)
    {
        super.onCreate(savedInstanceState);
        Intent intent = getIntent();
        if (UsbManager.ACTION_USB_ACCESSORY_ATTACHED.equals(intent.getAction()))
        {
            boolean found = false;
            ActivityManager am = (ActivityManager) getSystemService(Context.ACTIVITY_SERVICE);

            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.LOLLIPOP)
            {
                // Post lollipop, use getAppTasks()
                for (ActivityManager.AppTask at : am.getAppTasks())
                {
                    ActivityManager.RecentTaskInfo ti = at.getTaskInfo();
                    if (ti.id != getTaskId())
                    {
                        at.moveToFront();
                        found = true;
                        break;
                    }
                }
            }
            else
            {
                // Pre lollipop, use getRunningTasks()
                for (ActivityManager.RunningTaskInfo rti : am.getRunningTasks(10))
                {
                    if (rti.baseActivity.getPackageName().equals(this.getPackageName()) &&
                        !rti.baseActivity.getClassName().equals(this.getComponentName().getClassName()))
                    {
                        am.moveTaskToFront(rti.id, 0);
                        found = true;
                        break;
                    }
                }
            }


            if (found)
            {
                // broadcast accessory plugged-in
                Intent broadcastIntent = new Intent(UsbAccessoryMux.ACTION_USB_ACCESSORY_ATTACHED);
                broadcastIntent.putExtras(intent.getExtras());
                sendBroadcast(broadcastIntent);
            }
            else
            {
                Intent i = new Intent(this, getBaseActivity());
                i.addFlags(Intent.FLAG_ACTIVITY_NEW_TASK);
                startActivity(i);
            }
        }
        finish();
    }
}
