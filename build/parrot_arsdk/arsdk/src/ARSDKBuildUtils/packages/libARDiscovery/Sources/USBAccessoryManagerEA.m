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

#if defined(USE_USB_ACCESSORY) && defined (BUILD_LIBMUX)

#import <libARDiscovery/USBAccessoryManager.h>
#import <libARDiscovery/ARDISCOVERY_MuxDiscovery.h>
#import <libARSAL/ARSAL_Thread.h>
#import <libmux.h>
#import <ExternalAccessory/ExternalAccessory.h>
#import <libpomp.h>
#import <errno.h>
#import <USBAccessoryManagerImpl.h>

#define USB_INPUT_BUFFER_SIZE       64000

NSString *const UISupportedExternalAccessoryProtocols = @"UISupportedExternalAccessoryProtocols";

@interface USBAccessoryManagerEA () <EAAccessoryDelegate, NSStreamDelegate>
@property (nonatomic, weak) USBAccessoryManager* owner;
@property (nonatomic, strong) EAAccessory* accessory;
@property (nonatomic, strong) EASession *session;
@property (nonatomic, strong) NSThread *sessionThread;
@property (nonatomic, assign) ARSAL_Thread_t muxThread;
@property (nonatomic, assign) struct mux_ctx *usbMux;
@property (nonatomic, assign) struct MuxDiscoveryCtx *muxDiscovery;
@property (nonatomic, strong) dispatch_semaphore_t connectSemaphore;
@property (nonatomic, strong) dispatch_semaphore_t sessionEndSemaphore;
@property (nonatomic, copy) void (^connectionCbBlock)(uint32_t status, const char* json);
@property (nonatomic, strong) NSObject *dataToWriteLock;
@property (nonatomic, strong) NSMutableData *dataToWrite;

// store information of latest discovered device
@property (nonatomic, strong) NSString *deviceSerial;
@property (nonatomic, strong) NSString *deviceName;
@property (nonatomic, assign) eARDISCOVERY_PRODUCT productType;
@end

@implementation USBAccessoryManagerEA

- (instancetype)initWithOwner:(USBAccessoryManager*)owner{
    self = [super init];
    if (self)
    {
        _owner = owner;
        [[NSNotificationCenter defaultCenter] addObserver:self selector:@selector(accessoryDidConnect:) name:EAAccessoryDidConnectNotification object:nil];
        [[EAAccessoryManager sharedAccessoryManager] registerForLocalNotifications];
        _dataToWrite = [[NSMutableData alloc] init];
        _dataToWriteLock = [[NSObject alloc] init];
        NSMutableArray *accessoryList = [[NSMutableArray alloc] initWithArray:[[EAAccessoryManager sharedAccessoryManager] connectedAccessories]];
        for(EAAccessory *connectedAccessory in accessoryList)
        {
            [self tryToOpenSessionForAccessory:connectedAccessory];
        }
    }
    return self;
}


-(void)dealloc
{
    [[NSNotificationCenter defaultCenter] removeObserver:self];
    [self cleanUp];
}


- (void)delegateDidChange {
    if (_owner.delegate && _usbMux && _deviceSerial) {
        // notify if a device was discovered
        [_owner notifyDidAddDeviceWithConnectionId:_accessory.connectionID name:_deviceName mux:_usbMux serial:_deviceSerial productType:_productType];
    }
}

#pragma mark - CleanUp
- (void)cleanUp
{
    NSLog(@"%s clean up", __FUNCTION__);
    @synchronized (self) {
        if(self.session != nil)
        {
            [self performSelector:@selector(closeSession) onThread:self.sessionThread withObject:nil waitUntilDone:YES];
            [self.sessionThread cancel];
            // Wake up the run loop to evaluate the cancel
            [self performSelector:@selector(emptyMessage) onThread:self.sessionThread withObject:nil waitUntilDone:NO];
            if(self.sessionEndSemaphore)
            {
                dispatch_semaphore_wait(self.sessionEndSemaphore, DISPATCH_TIME_FOREVER);
                self.sessionEndSemaphore = nil;
            }
            self.sessionThread = nil;
        }
        if(self.muxDiscovery != NULL)
        {
            if(self.connectSemaphore != NULL)
            {
                dispatch_semaphore_signal(self.connectSemaphore);
            }

            ARDiscovery_MuxDiscovery_dispose(self.muxDiscovery);
            self.muxDiscovery = NULL;
            self.connectionCbBlock = NULL;
        }
        if(self.usbMux != NULL)
        {
            mux_stop(self.usbMux);
            if (self.muxThread)
            {
                ARSAL_Thread_Join(self.muxThread, NULL);
                ARSAL_Thread_Destroy(&_muxThread);
            }
            mux_unref(self.usbMux);
            self.usbMux = NULL;
        }
        self.accessory = nil;
        self.deviceName = nil;
        self.deviceSerial = nil;
    }
}

- (void)emptyMessage
{
}

#pragma mark - EAAccessory
#pragma mark EAAccessory Notifications
- (void)accessoryDidConnect:(NSNotification*)notification
{
    NSLog(@"%s Notification: %@", __FUNCTION__, notification.userInfo);
    EAAccessory *connectedAccessory = [[notification userInfo] objectForKey:EAAccessoryKey];

    if(self.usbMux == NULL)
    {
        [self tryToOpenSessionForAccessory:connectedAccessory];
    }
}

#pragma mark EAAccessoryDelegate
- (void)accessoryDidDisconnect:(EAAccessory *)disconnectedAccessory
{
    NSLog(@"%s Accessory: %@", __FUNCTION__, disconnectedAccessory);
    if ([disconnectedAccessory connectionID] == [self.accessory connectionID])
    {
        NSLog(@"%s Disconnect the current accessory", __FUNCTION__);
        [_owner notifyDidRemoveDeviceWithConnectionId:disconnectedAccessory.connectionID];
        [self cleanUp];
    }
}

#pragma mark EASession lifecycle
// open an EASession with the current accessory
- (void)openSession:(NSString *)protocol
{
    self.session = [[EASession alloc] initWithAccessory:self.accessory forProtocol:protocol];

    if (self.session)
    {
        [[self.session inputStream] setDelegate:self];
        [[self.session inputStream] scheduleInRunLoop:[NSRunLoop currentRunLoop] forMode:NSDefaultRunLoopMode];
        [[self.session inputStream] open];

        [[self.session outputStream] setDelegate:self];
        [[self.session outputStream] scheduleInRunLoop:[NSRunLoop currentRunLoop] forMode:NSDefaultRunLoopMode];
        [[self.session outputStream] open];

        dispatch_async(dispatch_get_main_queue(), ^{
            [self createMux];
        });
    }
    else
    {
        [self.sessionThread cancel];
        self.sessionThread = nil;
        NSLog(@"%s Open session failed", __FUNCTION__);
    }
}

// close the session with the current accessory.
- (void)closeSession
{
    [[self.session inputStream] setDelegate:nil];
    [[self.session inputStream] close];
    [[self.session inputStream] removeFromRunLoop:[NSRunLoop currentRunLoop] forMode:NSDefaultRunLoopMode];
    [[self.session outputStream] setDelegate:nil];
    [[self.session outputStream] close];
    [[self.session outputStream] removeFromRunLoop:[NSRunLoop currentRunLoop] forMode:NSDefaultRunLoopMode];

    self.session = nil;
}

- (void)tryToOpenSessionForAccessory:(EAAccessory*)connectedAccessory
{
    NSString *parrotMuxProtocol = [[[NSBundle mainBundle] objectForInfoDictionaryKey:UISupportedExternalAccessoryProtocols] objectAtIndex:0];

    NSLog(@"%s Try to connect accessory : %@. Protocol : %@", __FUNCTION__, connectedAccessory.name, parrotMuxProtocol);

    if(connectedAccessory)
    {
        for (NSString* protocol in [connectedAccessory protocolStrings])
        {
            if([protocol isEqualToString:parrotMuxProtocol])
            {
                self.accessory = connectedAccessory;
                [self.accessory setDelegate:self];

                if(self.sessionThread == nil)
                {
                    self.sessionEndSemaphore = dispatch_semaphore_create(0);
                    self.sessionThread = [[NSThread alloc] initWithTarget:self selector:@selector(sessionThreadMain) object:nil];
                    [self.sessionThread start];
                }
                [self performSelector:@selector(openSession:) onThread:self.sessionThread withObject:protocol waitUntilDone:YES];
                break;
            }
        }
    }
}

#pragma mark EASession Thread main method
- (void)sessionThreadMain
{
    NSRunLoop *runLoop = [NSRunLoop currentRunLoop];
    NSDate* futureDate = [NSDate dateWithTimeIntervalSinceNow:1.0];
    NSTimer *keepAliveTimer = [[NSTimer alloc] initWithFireDate:futureDate interval:0.1 target:self selector:@selector(emptyMessage) userInfo:nil repeats:YES];
    [runLoop addTimer:keepAliveTimer forMode:NSDefaultRunLoopMode];

    while (![NSThread currentThread].isCancelled && [runLoop runMode:NSDefaultRunLoopMode beforeDate:[NSDate distantFuture]])
    {
    }
    NSLog(@"%s Thread Exit", __FUNCTION__);

    //clean up
    [keepAliveTimer invalidate];
    if (self.sessionEndSemaphore)
    {
        dispatch_semaphore_signal(self.sessionEndSemaphore);
    }
    [NSThread exit];
}

#pragma mark EASession NSStreamDelegate
// handle stream event
- (void)stream:(NSStream *)aStream handleEvent:(NSStreamEvent)eventCode
{
    switch (eventCode) {
        case NSStreamEventNone:
            NSLog(@"%s NSStreamEventNone", __FUNCTION__);
            break;
        case NSStreamEventOpenCompleted:
            //NSLog(@"%s NSStreamEventOpenCompleted", __FUNCTION__);
            break;
        case NSStreamEventHasBytesAvailable:
            //NSLog(@"%s NSStreamEventHasBytesAvailable", __FUNCTION__);
            [self readDataFromAccessory];
            break;
        case NSStreamEventHasSpaceAvailable:
            //NSLog(@"%s NSStreamEventHasSpaceAvailable", __FUNCTION__);
            [self writeDataToAccessory];
            break;
        case NSStreamEventErrorOccurred:
            NSLog(@"%s NSStreamEventErrorOccurred", __FUNCTION__);
            break;
        case NSStreamEventEndEncountered:
            //NSLog(@"%s NSStreamEventEndEncountered", __FUNCTION__);
            [self closeSession];
            [self tryToOpenSessionForAccessory:self.accessory];
            break;
        default:
            NSLog(@"%s default", __FUNCTION__);
            break;
    }
}

#pragma mark EASession NSStream IO
// low level write method - write data to the accessory while there is space available and data to write
- (void)writeDataToAccessory
{
    @synchronized(self.dataToWriteLock) {

        while (([[self.session outputStream] hasSpaceAvailable]) && ([self.dataToWrite length] > 0))
        {
            NSInteger bytesWritten = [[self.session outputStream] write:[self.dataToWrite bytes] maxLength:[self.dataToWrite length]];
            //NSLog(@"%s Write to accessory. bytesWritten: %ld / %ld", __FUNCTION__, (long)bytesWritten, (long)_dataToWrite.length);
            if (bytesWritten == -1)
            {
                NSLog(@"%s write error", __FUNCTION__);
                break;
            }
            else if (bytesWritten > 0)
            {
                [self.dataToWrite replaceBytesInRange:NSMakeRange(0, bytesWritten) withBytes:NULL length:0];
            }
        }
    }
}

// low level read method - read data while there is data and space available in the input buffer
- (void)readDataFromAccessory
{
    while ([[self.session inputStream] hasBytesAvailable])
    {
        NSInteger bytesRead = 0;
        void *data = NULL;
        struct pomp_buffer *buf = pomp_buffer_new_get_data(USB_INPUT_BUFFER_SIZE, &data);

        bytesRead = [[self.session inputStream] read:(uint8_t *)data maxLength:USB_INPUT_BUFFER_SIZE];
        //NSLog(@"%s Read data from accessory. bytesRead: %ld", __FUNCTION__, (long)bytesRead);

        pomp_buffer_set_len(buf, (size_t)bytesRead);

        while ([self writeDataToMux:buf] != 0);

        pomp_buffer_unref(buf);
    }
}

#pragma mark - Mux
// create a new mux with no file descriptor and a tx callback
- (void)createMux
{
    @synchronized (self) {
        if(self.usbMux == NULL)
        {
            struct mux_ops ops;
            ops.tx = libmux_mux_ops_tx_callback;
            ops.chan_cb = libmux_mux_ops_channel_cb;
            ops.fdeof = NULL;
            ops.userdata = (__bridge void *)self;
            self.usbMux = mux_new(-1, NULL, &ops, 0);

            if(self.usbMux != NULL)
            {
                ARSAL_Thread_Create(&_muxThread, runMuxThread, (__bridge void *)self);
                [self createMuxDiscovery];
            }
            else
            {
                NSLog(@"%s Failed to create Mux", __FUNCTION__);
            }
        }
    }
}

static void* runMuxThread(void* arg)
{
    USBAccessoryManagerEA *this = (__bridge USBAccessoryManagerEA *)(arg);
    if(this)
    {
        mux_run(this.usbMux);
    }
    return NULL;
}

- (void)createMuxDiscovery
{
    if(self.usbMux != NULL)
    {
        self.muxDiscovery = ARDiscovery_MuxDiscovery_new(self.usbMux, device_added_cb, device_removed_cb, eof_cb, (__bridge void *)self);
        if(self.muxDiscovery == NULL)
        {
            NSLog(@"%s Failed to create MuxDiscovery", __FUNCTION__);
        }
    }
}

// read data from mux - high level method called by the tx callback to write data to the accessory
- (int)readDataFromMux:(struct pomp_buffer *)buf
{
    int retval = 0;
    const void *data = NULL;
    size_t len = 0;

    if(self.session == nil)
    {
        retval = -ENOENT;
    }

    if(retval == 0)
    {
        retval = pomp_buffer_get_cdata(buf, &data, &len, NULL);
    }

    if (retval == 0)
    {
        //NSLog(@"%s read data from mux. Lenght : %d", __FUNCTION__, (int)len);
        NSData *nsdata = [NSData dataWithBytesNoCopy:(uint8_t*)data length:len freeWhenDone:NO];

        @synchronized(self.dataToWriteLock)
        {
            if (self.dataToWrite == nil)
            {
                self.dataToWrite = [[NSMutableData alloc] init];
            }
            [self.dataToWrite appendData:nsdata];
        }

        [self performSelector:@selector(writeDataToAccessory) onThread:self.sessionThread withObject:nil waitUntilDone:NO];
    }

    return retval;
}

// write to the mux - high level method to write data to the mux when data have been read from the accessory
- (int)writeDataToMux:(struct pomp_buffer *)buf
{
    int retval = 0;

    if(buf != NULL && self.usbMux != NULL)
    {
        //NSLog(@"%s write data to mux", __FUNCTION__);
        retval = mux_decode(self.usbMux, buf);
    }

    return retval;
}

// tx callback - called by the mux when there is data to transfer to the accessory
static int libmux_mux_ops_tx_callback(struct mux_ctx *ctx, struct pomp_buffer *buf, void *userdata)
{
    int ret = 0;
    USBAccessoryManagerEA *this = (__bridge USBAccessoryManagerEA *)userdata;
    if (this)
    {
        ret = [this readDataFromMux:buf];
    }

    return ret;
}

static void libmux_mux_ops_channel_cb(struct mux_ctx *ctx, uint32_t chanid, enum mux_channel_event event, struct pomp_buffer *buf, void *userdata)
{
    USBAccessoryManagerEA *this = (__bridge USBAccessoryManagerEA *)userdata;
    if (this)
    {
        //NSLog(@"USBAccessoryManagerEA %s Mux Channel callback. Channel Id : %d. Channel Event : %@", __FUNCTION__, chanid, event == MUX_CHANNEL_RESET ? @"RESET" : @"DATA");
    }
}

#pragma mark - MuxDiscovery
- (eARDISCOVERY_ERROR)muxDiscoveryConnect:(NSString*)name model:(NSString*)model deviceId:(NSString*)serial json:(NSString*)jsonStr callback:(void (^)(uint32_t status, const char* json))connectionCbBlock
{
    eARDISCOVERY_ERROR err = ARDISCOVERY_ERROR;

    if(self.usbMux)
    {
        struct MuxConnectionCtx *muxConnection = ARDiscovery_MuxConnection_new(self.usbMux, device_conn_resp_cb, (__bridge void *)self);
        if (muxConnection != NULL) {
            self.connectSemaphore = dispatch_semaphore_create(0);

            if(self.muxDiscovery != NULL)
            {
                self.connectionCbBlock = connectionCbBlock;
                int result =  ARDiscovery_MuxConnection_sendConnReq(muxConnection, [name UTF8String], [model UTF8String], [serial UTF8String], [jsonStr UTF8String]);
                if(result == 0)
                {
                    //NSLog(@"%s Wait for connection from mux", __FUNCTION__);
                    dispatch_semaphore_wait(self.connectSemaphore, DISPATCH_TIME_FOREVER);
                    err = ARDISCOVERY_OK;
                }
            }
            self.connectSemaphore = nil;
            ARDiscovery_MuxConnection_dispose(muxConnection);
        }
    }
    return err;
}

- (void)muxDiscoveryCancelConnect
{
    //NSLog(@"%s Cancel connect", __FUNCTION__);
    if(self.connectSemaphore != NULL)
    {
        dispatch_semaphore_signal(self.connectSemaphore);
    }
    if(self.muxDiscovery != NULL)
    {
        ARDiscovery_MuxDiscovery_dispose(self.muxDiscovery);
        self.muxDiscovery = NULL;
    }
}

- (void)restartMuxDiscovery
{
    [_owner notifyDidRemoveDeviceWithConnectionId:self.accessory.connectionID];

    @synchronized (self) {
        if(self.connectSemaphore != NULL)
        {
            dispatch_semaphore_signal(self.connectSemaphore);
        }

        if(self.muxDiscovery != NULL)
        {
            ARDiscovery_MuxDiscovery_dispose(self.muxDiscovery);
            self.muxDiscovery = NULL;
        }

        [self createMuxDiscovery];
    }
}

static void device_added_cb(const char *name, uint32_t type, const char *id, void *userdata)
{
    USBAccessoryManagerEA *this = (__bridge USBAccessoryManagerEA *)userdata;
    if(this)
    {
        //NSLog(@"USBAccessoryManagerEA %s Device added cb. Notify delegate : %@. Device name : %@", __FUNCTION__, this.delegate, [NSString stringWithUTF8String:name]);
        this.deviceName = [NSString stringWithUTF8String:name];
        this.deviceSerial = [NSString stringWithUTF8String:id];
        this.productType = ARDISCOVERY_getProductFromProductID((uint16_t)type);
        [this.owner notifyDidAddDeviceWithConnectionId:this.accessory.connectionID name:this.deviceName mux:this.usbMux serial:this.deviceSerial productType:this.productType];
    }
}

static void device_removed_cb(const char *name, uint32_t type, const char *id, void *userdata)
{
    USBAccessoryManagerEA *this = (__bridge USBAccessoryManagerEA *)userdata;
    if(this)
    {
        //NSLog(@"USBAccessoryManagerEA %s Device removed cb. Notify delegate : %@", __FUNCTION__, this.delegate);
        if(this.connectSemaphore != NULL)
        {
            dispatch_semaphore_signal(this.connectSemaphore);
        }
    }
}

static void device_conn_resp_cb(uint32_t status, const char* json, void *userdata)
{
    USBAccessoryManagerEA *this = (__bridge USBAccessoryManagerEA *)userdata;
    if(this)
    {
        //NSLog(@"USBAccessoryManager %s Connection response cb", __FUNCTION__);
        if(this.connectionCbBlock != NULL)
        {
            this.connectionCbBlock(status, json);
            this.connectionCbBlock = NULL;
            if(this.connectSemaphore != NULL)
            {
                dispatch_semaphore_signal(this.connectSemaphore);
            }
        }
    }
}

static void eof_cb(void *userdata)
{
    USBAccessoryManagerEA *this = (__bridge USBAccessoryManagerEA *)userdata;
    if(this)
    {
        //NSLog(@"USBAccessoryManagerEA %s EOF cb", __FUNCTION__);

        dispatch_async(dispatch_get_main_queue(), ^{
            [this restartMuxDiscovery];
        });
    }
}

@end

#endif
