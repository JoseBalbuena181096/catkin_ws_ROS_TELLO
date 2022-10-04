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

#import <Foundation/Foundation.h>
#import <libARDiscovery/ARDISCOVERY_Error.h>
#import <libARDiscovery/ARDISCOVERY_Discovery.h>

@class USBAccessoryManager;

/**
 internal USBAccessoryManager category to notify delegate
 */
@interface USBAccessoryManager (USBAccessoryManagerImpl)
-(void)notifyDidAddDeviceWithConnectionId:(NSUInteger)connectionId name:(NSString *)name mux:(struct mux_ctx *)mux serial:(NSString *)serial productType:(eARDISCOVERY_PRODUCT)productType;
- (void)notifyDidRemoveDeviceWithConnectionId:(NSUInteger)connectionId;
@end


/** 
 Protocol defining a USBAccessoryManager implementation
 */
@protocol USBAccessoryManagerImpl <NSObject>
@required
- (eARDISCOVERY_ERROR)muxDiscoveryConnect:(NSString*)name model:(NSString*)model deviceId:(NSString*)serial json:(NSString*)jsonStr callback:(void (^)(uint32_t status, const char* json))connectionCbBlock;
- (void)muxDiscoveryCancelConnect;
- (void)delegateDidChange;
@end


/**
 Real implementation based on External Accessory
 */
@interface USBAccessoryManagerEA : NSObject <USBAccessoryManagerImpl>
- (instancetype)initWithOwner:(USBAccessoryManager*)owner;
@end

/**
 Debug implementation using mux proxy on ip
 */
@interface USBAccessoryManagerProxy : NSObject <USBAccessoryManagerImpl>
- (instancetype)initWithOwner:(USBAccessoryManager*)owner ipAddr:(NSString*)ipAddr;
@end
