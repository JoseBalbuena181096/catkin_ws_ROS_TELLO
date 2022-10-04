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

#import <libARDiscovery/USBAccessoryManager.h>
#import "USBAccessoryManagerImpl.h"

/**
 Wrapper on concrete USBAccessoryManager implementation. By default forward to USBAccessoryManagerEA.
 An alternate implementation that connected to the USB accessory through "USB Debug Bridge" application 
 by wifi can be selected by specifying an entry in application plist or a command line option (see constants bellow).
 */

// plist key containing usb accessory mux proxy ip address
NSString *const UsbAccessoryDebugIpPlist = @"UsbAccessoryDebugIp";
// argument passed to the command line, i.e: -usbproxy 192.168.2.1
NSString *const UsbAccessoryDebugCmdLine = @"usbproxy";

@interface USBAccessoryManager ()
@property (nonatomic, strong) id<USBAccessoryManagerImpl> impl;
@end

@implementation USBAccessoryManager


+ (USBAccessoryManager *)sharedInstance
{
#if defined(USE_USB_ACCESSORY) && defined (BUILD_LIBMUX)
    static USBAccessoryManager *_sharedInstance = nil;
    static dispatch_once_t onceToken;
    dispatch_once(&onceToken, ^{
        _sharedInstance = [[USBAccessoryManager alloc] init];

        NSString* proxyIpAddr;
        // check if proxy config has been passed on the command line
        NSUserDefaults *standardDefaults = [NSUserDefaults standardUserDefaults];
        proxyIpAddr = [standardDefaults stringForKey:UsbAccessoryDebugCmdLine];
        if (proxyIpAddr == nil) {
            // check if proxy is configured in plist
            proxyIpAddr = [[NSBundle mainBundle] objectForInfoDictionaryKey:UsbAccessoryDebugIpPlist];
        }

        if (proxyIpAddr == nil) {
            // default case, use External Accessory implementation
            _sharedInstance.impl = [[USBAccessoryManagerEA alloc] initWithOwner: _sharedInstance];
        } else  {
            // use debug proxy
            NSLog(@"USBAccessoryManager: uses accessory proxy on ip address : %@", proxyIpAddr);
            _sharedInstance.impl = [[USBAccessoryManagerProxy alloc] initWithOwner: _sharedInstance ipAddr:proxyIpAddr];
        }
    });
    return _sharedInstance;
#else
    return nil;
#endif
}

-(void)setDelegate:(id<USBAccessoryManagerDelegate>)newValue {
    _delegate = newValue;
    [_impl delegateDidChange];
}

- (eARDISCOVERY_ERROR)muxDiscoveryConnect:(NSString*)name model:(NSString*)model deviceId:(NSString*)serial json:(NSString*)jsonStr callback:(void (^)(uint32_t status, const char* json))connectionCbBlock
{
    eARDISCOVERY_ERROR res = ARDISCOVERY_ERROR_DEVICE_OPERATION_NOT_SUPPORTED;
    if (_impl) {
         res = [_impl muxDiscoveryConnect:name model:model deviceId:serial json:jsonStr callback:connectionCbBlock];
    }
    return res;
}

- (void)muxDiscoveryCancelConnect
{
    [_impl muxDiscoveryCancelConnect];
}

@end

@implementation USBAccessoryManager (USBAccessoryManagerImpl)

-(void)notifyDidAddDeviceWithConnectionId:(NSUInteger)connectionId name:(NSString *)name mux:(struct mux_ctx *)mux serial:(NSString *)serial productType:(eARDISCOVERY_PRODUCT)productType
{
    [_delegate USBAccessoryManager:self didAddDeviceWithConnectionId:connectionId name:name mux:mux serial:serial productType:productType];
}

- (void)notifyDidRemoveDeviceWithConnectionId:(NSUInteger)connectionId
{
    [_delegate USBAccessoryManager:self didRemoveDeviceWithConnectionId:connectionId];
}


@end
