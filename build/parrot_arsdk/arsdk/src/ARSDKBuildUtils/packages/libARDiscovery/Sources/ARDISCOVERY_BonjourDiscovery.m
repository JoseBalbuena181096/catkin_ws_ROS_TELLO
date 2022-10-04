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
//
//  ARDiscovery.m
//  ARSDK 3
//
//  Created by Nicolas BRULEZ on 08/03/13.
//  Copyright (c) 2013 Parrot SA. All rights reserved.
//
#include <arpa/inet.h>
#include <libARSAL/ARSAL_Print.h>
#import <libARDiscovery/ARDISCOVERY_BonjourDiscovery.h>
#import <libARDiscovery/ARDISCOVERY_Discovery.h>
#import <netdb.h>
#import <libARDiscovery/ARDISCOVERY_MuxDiscovery.h>

#ifdef USE_USB_ACCESSORY
#import <libARDiscovery/USBAccessoryManager.h>
#endif

#define ARDISCOVERY_BONJOURDISCOVERY_TAG            "ARDISCOVERY_BonjourDiscovery"

#define kServiceNetControllerType                   @"_arsdk-ff3._udp"
#define kServiceNetDomain                           @ARDISCOVERY_SERVICE_NET_DEVICE_DOMAIN
#define kServiceNetDeviceFormat                     @ARDISCOVERY_SERVICE_NET_DEVICE_FORMAT"."

#define ARBLESERVICE_BLE_MANUFACTURER_DATA_LENGTH   8
#define ARBLESERVICE_PARROT_BT_VENDOR_ID            0X0043  // Parrot Company ID registered by Bluetooth SIG (Bluetooth Specification v4.0 Requirement)
#define ARBLESERVICE_PARROT_USB_VENDOR_ID           0x19cf  // Official Parrot USB Vendor ID

#define kServiceResolutionTimeout                   5.f    // Time in seconds
#define kServiceBLERefreshTime                      10.f    // Time in seconds

#define CHECK_VALID(DEFAULT_RETURN_VALUE)       \
    do                                          \
    {                                           \
        if (! self.valid)                       \
        {                                       \
            return DEFAULT_RETURN_VALUE;        \
        }                                       \
    } while (0)

#pragma mark - ARBLEService implementation

@implementation ARBLEService
@end

@implementation ARUSBService
@end

@implementation ARService
@synthesize signal;

- (ARDISCOVERY_Device_t *) createDevice:(eARDISCOVERY_ERROR *)err
{
    ARDISCOVERY_Device_t *ret;

    if (!err)
        return nil;

    ret = ARDISCOVERY_Device_New(err);

    if (*err != ARDISCOVERY_OK) {
        ARDISCOVERY_Device_Delete(&ret);
        return ret;
    }


    switch(self.network_type) {
    case ARDISCOVERY_NETWORK_TYPE_NET:
    {
        if (![[ARDiscovery sharedInstance] resolveServiceSync:self]) {
            *err = ARDISCOVERY_ERROR_BUSY;
            goto exit;
        }
        NSString *ip = [[ARDiscovery sharedInstance] convertNSNetServiceToIp:self];
        int port = (int)[(NSNetService *)self.service port];
        *err = ARDISCOVERY_Device_InitWifi(ret, self.product, [self.name UTF8String], [ip UTF8String], port);
    }
    break;

    case ARDISCOVERY_NETWORK_TYPE_BLE:
    {
        ARBLEService* bleService = (ARBLEService *)self.service;
        *err = ARDISCOVERY_Device_InitBLE (ret, self.product, (__bridge ARNETWORKAL_BLEDeviceManager_t)(bleService.centralManager), (__bridge ARNETWORKAL_BLEDevice_t)(bleService.peripheral));
    }
    break;

    case ARDISCOVERY_NETWORK_TYPE_USBMUX:
    {
        *err = ARDISCOVERY_Device_InitUSB(ret, self.product, ((ARUSBService*)self.service).usbMux);
    }
    break;

    default:
        *err = ARDISCOVERY_ERROR_BAD_PARAMETER;
        break;
    }

exit:
    if (*err != ARDISCOVERY_OK)
        ARDISCOVERY_Device_Delete(&ret);

    return ret;
}

- (BOOL)isEqual:(id)object
{
    BOOL result = YES;
    ARService *otherService = (ARService *)object;

    if((otherService != nil) && (self.network_type == otherService.network_type))
    {
        if (self.network_type == ARDISCOVERY_NETWORK_TYPE_NET)
        {
            NSNetService *netService = (NSNetService *) self.service;
            NSNetService *otherNETService = (NSNetService *) otherService.service;

            result = ([netService.name isEqual:otherNETService.name]);
        }
        else if (self.network_type == ARDISCOVERY_NETWORK_TYPE_BLE)
        {
            ARBLEService *bleService = (ARBLEService *) self.service;
            ARBLEService *otherBLEService = (ARBLEService *) otherService.service;

            result = ([[bleService.peripheral.identifier UUIDString] isEqual: [otherBLEService.peripheral.identifier UUIDString]]);
        }
        else if (self.network_type == ARDISCOVERY_NETWORK_TYPE_USBMUX)
        {
            ARUSBService *usbService = (ARUSBService *) self.service;
            ARUSBService *otherUSBService = (ARUSBService *) otherService.service;

            result = (usbService.connectionId == otherUSBService.connectionId);
        }
        else
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARDISCOVERY_BONJOURDISCOVERY_TAG, "Unknown network media type.");
            result = NO;
        }
    }
    else
    {
        result = NO;
    }

    return result;
}

@end

#pragma mark Private part

@interface ARDiscovery () <NSNetServiceBrowserDelegate, NSNetServiceDelegate, CBCentralManagerDelegate>

#pragma mark - Supported products list
@property (strong, nonatomic) NSSet *supportedProducts;

#pragma mark - Controller/Devices Services list
@property (strong, nonatomic) NSMutableDictionary *controllersServicesList;
@property (strong, nonatomic) NSMutableDictionary *devicesServicesList;

#pragma mark - Current published service
@property (strong, nonatomic) NSNetService *currentPublishedService;
@property (strong, nonatomic) NSNetService *tryPublishService;

#pragma mark - Services browser / resolution
@property (strong, nonatomic) ARService *currentResolutionService;
@property (strong, nonatomic) NSNetServiceBrowser *controllersServiceBrowser;
@property (strong, nonatomic) NSMutableArray *devicesServiceBrowsers;

#pragma mark - Services CoreBluetooth
@property (strong, nonatomic) dispatch_queue_t centralManagerQueue;
@property (strong, nonatomic) ARSAL_CentralManager *centralManager;
@property (nonatomic, assign) BOOL centralManagerInitialized;
@property (strong, nonatomic) NSMutableDictionary *devicesBLEServicesTimerList;

#pragma mark - Object properly created
@property (nonatomic) BOOL valid;

#pragma mark - Object properly created
@property (nonatomic) BOOL isNSNetDiscovering;
@property (nonatomic) BOOL isCBDiscovering;
@property (nonatomic) BOOL askForCBDiscovering;
@end

#ifdef USE_USB_ACCESSORY
@interface ARDiscovery () <USBAccessoryManagerDelegate>
@end
#endif


#pragma mark Implementation
@implementation ARDiscovery {
    BOOL resolveResult;
    dispatch_semaphore_t resolveSem;
    BOOL resolveSyncBusy;
}

@synthesize controllersServicesList;
@synthesize devicesServicesList;
@synthesize devicesBLEServicesTimerList;
@synthesize currentPublishedService;
@synthesize tryPublishService;
@synthesize currentResolutionService;
@synthesize controllersServiceBrowser;
@synthesize devicesServiceBrowsers;
@synthesize valid;
@synthesize centralManagerQueue;
@synthesize centralManager;
@synthesize centralManagerInitialized;
@synthesize askForCBDiscovering;
@synthesize isNSNetDiscovering;
@synthesize isCBDiscovering;

#pragma mark - Init
+ (ARDiscovery *)sharedInstance
{
    static ARDiscovery *_sharedInstance = nil;
    static dispatch_once_t onceToken;
    dispatch_once(&onceToken, ^{
            _sharedInstance = [[ARDiscovery alloc] init];

            /**
             * Services list init
             */
            _sharedInstance.controllersServicesList = [[NSMutableDictionary alloc] init];
            _sharedInstance.devicesServicesList = [[NSMutableDictionary alloc] init];
            _sharedInstance.devicesBLEServicesTimerList = [[NSMutableDictionary alloc] init];

            /**
             * Supported products list init with all products of eARDISCOVERY_PRODUCT
             */
            NSMutableSet *allProducts = [[NSMutableSet alloc] init];
            for(eARDISCOVERY_PRODUCT product = 0 ; product < ARDISCOVERY_PRODUCT_MAX ; product++)
            {
                [allProducts addObject:[NSNumber numberWithInt:product]];
            }
            _sharedInstance.supportedProducts = [[NSSet alloc] initWithSet:allProducts];

            /**
             * Current published service init
             */
            _sharedInstance.currentPublishedService = nil;
            _sharedInstance.tryPublishService = nil;

            /**
             * Services browser / resolution init
             */
            _sharedInstance.controllersServiceBrowser = [[NSNetServiceBrowser alloc] init];
            [_sharedInstance.controllersServiceBrowser setDelegate:_sharedInstance];
            _sharedInstance.devicesServiceBrowsers = [[NSMutableArray alloc] init];
            for (int i = 0; i < ARDISCOVERY_PRODUCT_MAX; ++i)
            {
                NSNetServiceBrowser *browser = [[NSNetServiceBrowser alloc] init];
                [browser setDelegate:_sharedInstance];
                [_sharedInstance.devicesServiceBrowsers addObject:browser];
            }

            _sharedInstance.currentResolutionService = nil;

            /**
             * Creation was done as a shared instance
             */
            _sharedInstance.valid = YES;

            /**
             * Discover is not in progress
             */
            _sharedInstance.centralManagerInitialized = NO;
            _sharedInstance.centralManagerQueue = dispatch_queue_create( "com.parrot.ardiscovery.ble-centralmanager-queue", DISPATCH_QUEUE_SERIAL);
            _sharedInstance.centralManager = [[ARSAL_CentralManager alloc] initWithQueue:_sharedInstance.centralManagerQueue];
            [_sharedInstance.centralManager addDelegate: _sharedInstance];
            _sharedInstance.isNSNetDiscovering = NO;
            _sharedInstance.isCBDiscovering = NO;
            _sharedInstance.askForCBDiscovering = NO;

            _sharedInstance->resolveResult = NO;
            _sharedInstance->resolveSem = dispatch_semaphore_create(0);
            _sharedInstance->resolveSyncBusy = NO;
        });

    return _sharedInstance;
}

#pragma mark - Set supported products list
- (void)setSupportedProducts:(NSSet*)products
{
    @synchronized (self)
    {
        if(_supportedProducts != nil)
        {
            NSLog(@"%s Supported products list was not empty", __FUNCTION__);
        }
        _supportedProducts = products;
    }
}

#pragma mark - Getters
- (NSArray *)getCurrentListOfDevicesServices
{
    NSArray *array = nil;
    CHECK_VALID(array);
    @synchronized (self)
    {
        array = [[self.devicesServicesList allValues] copy];
    }
    return array;
}

- (NSArray *)getCurrentListOfControllersServices
{
    NSArray *array = nil;
    CHECK_VALID(array);
    @synchronized (self)
    {
        array = [[self.controllersServicesList allValues] copy];
    }
    return array;
}

- (NSString *)getCurrentPublishedServiceName
{
    NSString *name = nil;
    CHECK_VALID(name);
    @synchronized (self)
    {
        name = [[self.currentPublishedService name] copy];
    }
    return name;
}

- (void)removeDeviceService:(ARService*)aService
{
    [self.devicesServicesList removeObjectForKey:aService.name];
}

#pragma mark - Discovery
- (BOOL)isNetServiceValid:(NSNetService *)aNetService
{
    for (int i = 0; i < ARDISCOVERY_PRODUCT_MAX; ++i)
    {
        NSString *deviceType = [NSString stringWithFormat:kServiceNetDeviceFormat, ARDISCOVERY_getProductID(i)];
        if ([aNetService.type isEqualToString:deviceType])
            return YES;
    }
    return NO;
}

- (void)resolveService:(ARService *)aService
{
    CHECK_VALID();
    @synchronized (self)
    {
        if(self.currentResolutionService != nil)
        {
            [[self.currentResolutionService service] stop];
        }

        self.currentResolutionService = aService;
        [((NSNetService*)[self.currentResolutionService service]) setDelegate:self];
        [[self.currentResolutionService service] resolveWithTimeout:kServiceResolutionTimeout];
    }
}

- (BOOL)resolveServiceSync:(ARService *)aService
{
    CHECK_VALID(NO);
    @synchronized(self)
    {
        if(self->resolveSyncBusy)
            return NO;
        self->resolveSyncBusy = YES;
    }
    [self resolveService:aService];
    dispatch_semaphore_wait(self->resolveSem, DISPATCH_TIME_FOREVER);
    @synchronized(self)
    {
        self->resolveSyncBusy = NO;
    }
    return self->resolveResult;
}

- (void)start
{
    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDISCOVERY_BONJOURDISCOVERY_TAG, "%s:%d", __FUNCTION__, __LINE__);

    if (!isNSNetDiscovering)
    {
        /**
         * Start NSNetServiceBrowser
         */
        [controllersServiceBrowser searchForServicesOfType:kServiceNetControllerType inDomain:kServiceNetDomain];
        for (NSUInteger i = 0; i < [devicesServiceBrowsers count]; ++i)
        {
            NSNetServiceBrowser *browser = [devicesServiceBrowsers objectAtIndex:i];
            [browser searchForServicesOfType:[NSString stringWithFormat:kServiceNetDeviceFormat, ARDISCOVERY_getProductID(i)] inDomain:kServiceNetDomain];
        }

        isNSNetDiscovering = YES;
    }

    if(!isCBDiscovering)
    {
        if (centralManagerInitialized)
        {
            /**
             * Start CoreBluetooth discovery
             */
            [centralManager scanForPeripheralsWithServices:nil options:[NSDictionary dictionaryWithObjectsAndKeys:[NSNumber numberWithBool:YES], CBCentralManagerScanOptionAllowDuplicatesKey, nil]];
            isCBDiscovering = YES;
        }
        else
        {
            askForCBDiscovering = YES;
        }
    }

#ifdef USE_USB_ACCESSORY
    [[USBAccessoryManager sharedInstance] setDelegate:self];
#endif
}

- (void)pauseBLE
{
    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDISCOVERY_BONJOURDISCOVERY_TAG, "%s:%d", __FUNCTION__, __LINE__);

    if (centralManagerInitialized && isCBDiscovering)
    {
        /**
         * Stop CBCentralManager
         */
        isCBDiscovering = NO;
        [centralManager stopScan];
    }
}

- (void)stop
{
    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDISCOVERY_BONJOURDISCOVERY_TAG, "%s:%d", __FUNCTION__, __LINE__);

    [self removeAllServices];

    if (isNSNetDiscovering)
    {
        /**
         * Stop NSNetServiceBrowser
         */
        [controllersServiceBrowser stop];
        for (NSNetServiceBrowser *browser in devicesServiceBrowsers)
        {
            [browser stop];
        }
        isNSNetDiscovering = NO;
    }

    if (centralManagerInitialized && isCBDiscovering)
    {
        /**
         * Stop CBCentralManager
         */
        isCBDiscovering = NO;
        [centralManager stopScan];
    }

#ifdef USE_USB_ACCESSORY
    [[USBAccessoryManager sharedInstance] setDelegate:nil];
#endif
}

- (void)removeAllServices
{
    @synchronized (self)
    {
        [self.devicesServicesList removeAllObjects];
        [self sendDevicesListUpdateNotification];

        [self.controllersServicesList removeAllObjects];
        [self sendControllersListUpdateNotification];
    }
}

- (void)removeAllBLEServices
{
    @synchronized (self)
    {
        NSMutableArray *bleDevices = [[NSMutableArray alloc] init];
        for (NSString *key in devicesServicesList)
        {
            ARService *aService = [devicesServicesList objectForKey:key];
            if(aService.network_type == ARDISCOVERY_NETWORK_TYPE_BLE)
            {
                [bleDevices addObject:key];
            }
        }

        [devicesServicesList removeObjectsForKeys:bleDevices];
        [self sendDevicesListUpdateNotification];
    }
}

- (void)removeAllUSBServices
{
    @synchronized (self)
    {
        NSMutableArray *usbDevices = [[NSMutableArray alloc] init];
        for (NSString *key in devicesServicesList)
        {
            ARService *aService = [devicesServicesList objectForKey:key];
            if(aService.network_type == ARDISCOVERY_NETWORK_TYPE_USBMUX)
            {
                [usbDevices addObject:key];
            }
        }

        [devicesServicesList removeObjectsForKeys:usbDevices];
        [self sendDevicesListUpdateNotification];
    }
}

- (void)removeAllWifiServices
{
    @synchronized (self)
    {
        NSMutableArray *wifiDevices = [[NSMutableArray alloc] init];
        for (NSString *key in devicesServicesList)
        {
            ARService *aService = [devicesServicesList objectForKey:key];
            if(aService.network_type == ARDISCOVERY_NETWORK_TYPE_NET)
            {
                [wifiDevices addObject:key];
            }
        }

        [devicesServicesList removeObjectsForKeys:wifiDevices];
        [self sendDevicesListUpdateNotification];
    }
}

- (NSString *)convertNSNetServiceToIp:(ARService *)aService
{
    NSString *name = nil;
    NSData *address = nil;
    struct sockaddr_in *socketAddress = nil;
    NSString *ipString = nil;
    int port;
    NSUInteger i;

    name = [[aService service] name];
    NSArray *adresses = ((NSNetService *)[aService service]).addresses;
    for (i = 0 ; i < [adresses count] ; i++)
    {
        address = [adresses objectAtIndex:i];
        socketAddress = (struct sockaddr_in *) [address bytes];
        if (socketAddress->sin_family == AF_INET)//AF_INET -> IPv4, AF_INET6 -> IPv6
        {
            char ip[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, &socketAddress->sin_addr, ip, INET_ADDRSTRLEN);
            ipString = [NSString stringWithFormat: @"%s", ip];
            port = ntohs(socketAddress->sin_port);
        }
    }

    // This will print the IP and port for you to connect to.
    NSLog(@"%@", [NSString stringWithFormat:@"Resolved:%@-->%@:%u\n", [[aService service] hostName], ipString, port]);

    return ipString;
}

#pragma mark - Publication
- (NSString *)uniqueNameFromServiceName:(NSString *)sname isController:(BOOL)isController
{
    NSString *rname = [sname copy];

    int addCount = 1;

    NSArray *servicesCopy;
    if (isController)
    {
        servicesCopy = [self getCurrentListOfControllersServices];
    }
    else
    {
        servicesCopy = [self getCurrentListOfDevicesServices];
    }
    BOOL rnameIsUnique = YES;
    do {
        rnameIsUnique = YES;
        for (NSNetService *ns in servicesCopy) {
            if ([rname isEqualToString:[ns name]])
            {
                rnameIsUnique = NO;
                break;
            }
        }
        if (! rnameIsUnique)
        {
            rname = [sname stringByAppendingFormat:@"%d", addCount++];
        }
    } while (! rnameIsUnique);
    return rname;
}

- (void)publishControllerServiceWithName:(NSString *)serviceName
{
    CHECK_VALID();
    @synchronized (self)
    {
        NSString *uniqueName = [self uniqueNameFromServiceName:serviceName isController:YES];
        [self.tryPublishService stop];
        self.tryPublishService = [[NSNetService alloc] initWithDomain:kServiceNetDomain type:kServiceNetControllerType name:uniqueName port:9];
        [self.tryPublishService setDelegate:self];
        [self.tryPublishService publish];
    }
}

- (void)unpublishService
{
    CHECK_VALID();
    @synchronized (self)
    {
        [self.tryPublishService stop];
        self.tryPublishService = nil;
        self.currentPublishedService = nil;
        [self sendPublishNotification];
    }
}

#pragma mark - NSNetServiceBrowser Delegate
- (void)netServiceBrowser:(NSNetServiceBrowser *)aNetServiceBrowser didFindService:(NSNetService *)aNetService moreComing:(BOOL)moreComing
{
    NSLog(@"Service found : %@, %@", aNetService.name, aNetService.type);
    if(aNetService.TXTRecordData == nil)
    {
        NSLog (@"Service's TXTRecordData is nil. Updated controllers list notification is not sent");
        return;
    }

    @synchronized (self)
    {
        if ([aNetService.type isEqualToString:kServiceNetControllerType])
        {
            ARService *aService = [self.controllersServicesList objectForKey:aNetService.name];
            if (aService == nil)
            {
                aService = [[ARService alloc] init];
                aService.service = aNetService;
                aService.network_type = ARDISCOVERY_NETWORK_TYPE_NET;
            }
            aService.name = [aNetService name];
            aService.signal = [NSNumber numberWithInt:0];
            NSDictionary *dict = [NSNetService dictionaryFromTXTRecordData:aNetService.TXTRecordData];
            if(dict != nil && [dict objectForKey:[NSString stringWithUTF8String:ARDISCOVERY_SERVICE_NET_RSSI_SIGNAL_KEY]] != nil)
            {
                aService.signal = [dict objectForKey:[NSString stringWithUTF8String:ARDISCOVERY_SERVICE_NET_RSSI_SIGNAL_KEY]];
            }
            aService.product = ARDISCOVERY_PRODUCT_MAX;

            [self.controllersServicesList setObject:aService forKey:aService.name];
            if (!moreComing)
            {
                [self sendControllersListUpdateNotification];
            }
        }
        else
        {
            ARService *aService   = [self.devicesServicesList objectForKey:aNetService.name];

            if (aService == nil)
            {
                aService = [[ARService alloc] init];
                aService.service = aNetService;
                aService.network_type = ARDISCOVERY_NETWORK_TYPE_NET;
            }
            else
            {
                NSString *serviceType = [NSString stringWithFormat:kServiceNetDeviceFormat, ARDISCOVERY_getProductID(aService.product)];
                if(![aNetService.type isEqualToString:serviceType])
                {
                    aService = [[ARService alloc] init];
                    aService.service = aNetService;
                    aService.network_type = ARDISCOVERY_NETWORK_TYPE_NET;
                }
            }

            aService.name = [aNetService name];
            aService.signal = [NSNumber numberWithInt:0];
            NSDictionary *dict = [NSNetService dictionaryFromTXTRecordData:aNetService.TXTRecordData];
            if(dict != nil && [dict objectForKey:[NSString stringWithUTF8String:ARDISCOVERY_SERVICE_NET_RSSI_SIGNAL_KEY]] != nil)
            {
                aService.signal = [dict objectForKey:[NSString stringWithUTF8String:ARDISCOVERY_SERVICE_NET_RSSI_SIGNAL_KEY]];
            }
            aService.product = ARDISCOVERY_PRODUCT_MAX;

            for (int i = 0; (aService.product == ARDISCOVERY_PRODUCT_MAX) && (i < ARDISCOVERY_PRODUCT_MAX); ++i)
            {
                NSString *deviceType = [NSString stringWithFormat:kServiceNetDeviceFormat, ARDISCOVERY_getProductID(i)];
                if ([aNetService.type isEqualToString:deviceType])
                {
                    aService.product = i;
                }
            }

            if (aService.product != ARDISCOVERY_PRODUCT_MAX && [self.supportedProducts containsObject:[NSNumber numberWithInt:aService.product]])
            {
                [self.devicesServicesList setObject:aService forKey:aService.name];
                if (!moreComing)
                {
                    [self sendDevicesListUpdateNotification];
                }
            }
            else
            {
#ifdef DEBUG
                NSLog (@"Found an unknown service : %@", aNetService);
#endif
            }
        }
    }
}

- (void)netServiceBrowser:(NSNetServiceBrowser *)aNetServiceBrowser didRemoveService:(NSNetService *)aNetService moreComing:(BOOL)moreComing
{
    @synchronized (self)
    {
        NSLog(@"netServiceBrowser %@ didRemoveService %@ moreComing: %i", aNetServiceBrowser, aNetService, moreComing);
        if ([self isNetServiceValid:aNetService])
        {
            ARService *aService = (ARService *)[self.devicesServicesList objectForKey:aNetService.name];
            if (aService != nil)
            {
                NSLog(@"Removed service %@ : %@", aService.name, NSStringFromClass([[aService service] class]));
                [self.devicesServicesList removeObjectForKey:aService.name];
                if (!moreComing)
                {
                    [self sendDevicesListUpdateNotification];
                }
            }
        }
        else if ([[aNetService type] isEqual:kServiceNetControllerType])
        {
            ARService *aService = (ARService *)[self.controllersServicesList objectForKey:aNetService.name];
            if (aService != nil)
            {
                NSLog(@"Removed service %@ : %@", aService.name, NSStringFromClass([[aService service] class]));
                [self.controllersServicesList removeObjectForKey:aService.name];
                if (!moreComing)
                {
                    [self sendControllersListUpdateNotification];
                }
            }
        }
        else
        {
#ifdef DEBUG
            NSLog (@"Removed an unknown service : %@", aNetService);
#endif
        }
    }
}

#pragma mark - NSNetService Delegate
- (void)netService:(NSNetService *)service didNotPublish:(NSDictionary *)errorDict
{
    @synchronized (self)
    {
        self.currentPublishedService = nil;
        [self sendPublishNotification];
    }
}

- (void)netServiceDidPublish:(NSNetService *)service
{
    @synchronized (self)
    {
        self.currentPublishedService = service;
        [self sendPublishNotification];
    }
}

- (void)netService:(NSNetService *)service didNotResolve:(NSDictionary *)errorDict
{
    @synchronized (self)
    {
        self->resolveResult = NO;
        self.currentResolutionService = nil;
        [self sendNotResolveNotification];
        [service stop];
        if (self->resolveSyncBusy)
            dispatch_semaphore_signal(self->resolveSem);
    }
}

- (void)netServiceDidResolveAddress:(NSNetService *)service
{
    @synchronized (self)
    {
        self->resolveResult = YES;
        [self sendResolveNotification];
        [service stop];
        if (self->resolveSyncBusy)
            dispatch_semaphore_signal(self->resolveSem);
    }
}

#pragma mark - Refresh BLE services methods
- (void)deviceBLERemoveServices:(ARService *)aService
{
    @synchronized (self)
    {
        CBPeripheral *peripheral = ((ARBLEService *) aService.service).peripheral;
        NSLog(@"Removed service %@ : %@", aService.name, NSStringFromClass([[aService service] class]));
        [self.devicesBLEServicesTimerList removeObjectForKey:[peripheral.identifier UUIDString]];
        [self.devicesServicesList removeObjectForKey:[peripheral.identifier UUIDString]];
        [self sendDevicesListUpdateNotification];
    }
}

- (void)deviceBLETimeout:(NSTimer *)timer
{
    ARService *aService = [timer userInfo];
    [self deviceBLERemoveServices:aService];
}

#pragma mark - CBCentralManagerDelegate methods
- (void)centralManagerDidUpdateState:(CBCentralManager *)central
{
    NSString *sNewState = @"New CBCentralManager state :";
    switch(central.state)
    {
        case CBCentralManagerStatePoweredOn:
            NSLog(@"%@ CBCentralManagerStatePoweredOn", sNewState);
            centralManagerInitialized = YES;
            if(askForCBDiscovering)
            {
                askForCBDiscovering = NO;
                [self start];
            }
            break;

        case CBCentralManagerStateResetting:
            NSLog(@"%@ CBCentralManagerStateResetting", sNewState);
            centralManagerInitialized = NO;
            isCBDiscovering = NO;
            askForCBDiscovering = YES;

            break;

        case CBCentralManagerStateUnsupported:
            NSLog(@"%@ CBCentralManagerStateUnsupported", sNewState);
            centralManagerInitialized = NO;
            break;

        case CBCentralManagerStateUnauthorized:
            NSLog(@"%@ CBCentralManagerStateUnauthorized", sNewState);
            centralManagerInitialized = NO;
            break;

        case CBCentralManagerStatePoweredOff:
            NSLog(@"%@ CBCentralManagerStatePoweredOff", sNewState);
            centralManagerInitialized = NO;
            isCBDiscovering = NO;
            askForCBDiscovering = YES;

            [self removeAllBLEServices];

            break;

        default:
        case CBCentralManagerStateUnknown:
            NSLog(@"%@ CBCentralManagerStateUnknown", sNewState);
            centralManagerInitialized = NO;
            break;
    }
}

- (void)centralManager:(CBCentralManager *)central didDiscoverPeripheral:(CBPeripheral *)peripheral advertisementData:(NSDictionary *)advertisementData RSSI:(NSNumber *)RSSI
{
    @synchronized (self)
    {
        if([peripheral name] != nil)
        {
            if ( [self isParrotBLEDevice:advertisementData] )
            {
                NSData *manufacturerData = [advertisementData valueForKey:CBAdvertisementDataManufacturerDataKey];
                uint16_t *ids = (uint16_t *) manufacturerData.bytes;
                uint8_t *ids8 = (uint8_t *) manufacturerData.bytes;

                ARService *aService = [self.devicesServicesList objectForKey:[peripheral.identifier UUIDString]];
                if(aService == nil)
                {
                    eARDISCOVERY_PRODUCT product = ARDISCOVERY_PRODUCT_MAX;
                    for (int i = 0; (product == ARDISCOVERY_PRODUCT_MAX) && (i < ARDISCOVERY_PRODUCT_MAX) ; i++)
                    {
                        if (ids[2] == ARDISCOVERY_getProductID(i))
                            product = i;
                    }

                    if([self.supportedProducts containsObject:[NSNumber numberWithInt:product]])
                    {
                        NSLog(@"New device %@", [advertisementData objectForKey:CBAdvertisementDataLocalNameKey]);
                        ARBLEService *bleService = [[ARBLEService alloc] init];
                        bleService.centralManager = self.centralManager;
                        bleService.peripheral = peripheral;
                        bleService.connectionState = [self connectionStateForValue:(ids[3] & 0x03)];
                        if (manufacturerData.length > 8) {
                            bleService.hasMinicam = ((ids8[8]) & 0x01);
                        } else {
                            bleService.hasMinicam = NO;
                        }

                        aService = [[ARService alloc] init];
                        aService.service = bleService;
                        aService.network_type = ARDISCOVERY_NETWORK_TYPE_BLE;
                        aService.name = [advertisementData objectForKey:CBAdvertisementDataLocalNameKey];
                        aService.signal = RSSI;
                        aService.product = product;

                        [self.devicesServicesList setObject:aService forKey:[peripheral.identifier UUIDString]];
                        [self sendDevicesListUpdateNotification];
                    }
                }
                else
                {
                    BOOL sendNotification = NO;
                    NSString *name = [advertisementData objectForKey:CBAdvertisementDataLocalNameKey];
                    if(![aService.name isEqualToString:name])
                    {
                        aService.name = name;
                        sendNotification = YES;
                    }

                    if([aService.signal compare:RSSI] != NSOrderedSame)
                    {
                        aService.signal = RSSI;
                        sendNotification = YES;
                    }

                    eARDISCOVERY_CONNECTION_STATE connectionState = [self connectionStateForValue:(ids[3] & 0x03)];
                    BOOL hasMinicam = NO;
                    if (manufacturerData.length > 8) {
                        hasMinicam = ((ids8[8]) & 0x01);
                    }

                    ARBLEService *bleService = aService.service;
                    if (bleService.connectionState != connectionState ||
                        bleService.hasMinicam != hasMinicam) {
                        bleService.connectionState = connectionState;
                        bleService.hasMinicam = hasMinicam;
                        sendNotification = YES;
                    }

                    if(sendNotification)
                    {
                        [self sendDevicesListUpdateNotification];
                    }
                }

                if (aService != nil)
                {
                    // dispatch synchronously in the main thread because the timer is not fired otherwise
                    dispatch_async(dispatch_get_main_queue(), ^{
                        NSTimer *timer = (NSTimer *)[self.devicesBLEServicesTimerList objectForKey:[peripheral.identifier UUIDString]];
                        if(timer != nil)
                        {
                            [timer invalidate];
                            timer = nil;
                        }

                        timer = [NSTimer scheduledTimerWithTimeInterval:kServiceBLERefreshTime target:self selector:@selector(deviceBLETimeout:) userInfo:aService repeats:NO];
                        [self.devicesBLEServicesTimerList setObject:timer forKey:[peripheral.identifier UUIDString]];
                    });
                }
            }
        }
    }
}

- (void)centralManager:(CBCentralManager *)central didDisconnectPeripheral:(CBPeripheral *)peripheral error:(NSError *)error
{
    NSLog(@"centralManager %@ didDisconnectPeripheral %@ error: %@", central, peripheral, error);
    NSTimer *timer = (NSTimer *)[self.devicesBLEServicesTimerList objectForKey:[peripheral.identifier UUIDString]];

    if(timer != nil)
    {
        ARService *aService = [timer userInfo];

        [timer invalidate];
        timer = nil;
        NSLog(@"Will remove BLE service: %@", aService);
        [self deviceBLERemoveServices:aService];

    }
}

- (BOOL)isParrotBLEDevice:(NSDictionary *)advertisementData
{
    /* Read the advertisementData to check if it is a PARROT Delos device with the good version */

    BOOL res = NO;
    NSData *manufacturerData = [advertisementData valueForKey:CBAdvertisementDataManufacturerDataKey];

    if ((manufacturerData != nil) && (manufacturerData.length >= ARBLESERVICE_BLE_MANUFACTURER_DATA_LENGTH))
    {
        uint16_t *ids = (uint16_t*) manufacturerData.bytes;

#ifdef DEBUG
        NSLog(@"manufacturer Data: BTVendorID:0x%.4x USBVendorID:0x%.4x USBProduitID=0x%.4x versionID=0x%.4x", ids[0], ids[1], ids[2], ids[3]);
#endif

        if ((ids[0] == ARBLESERVICE_PARROT_BT_VENDOR_ID) &&
            (ids[1] == ARBLESERVICE_PARROT_USB_VENDOR_ID))
        {
            /* Compare with all known BLE product IDs */
            uint16_t prod_id = ids[2];
            eARDISCOVERY_PRODUCT product = ARDISCOVERY_getProductFromProductID(prod_id);
            if (ARDISCOVERY_getProductFamily(product) == ARDISCOVERY_PRODUCT_FAMILY_MINIDRONE) {
                res = YES;
            }
        }
    }

    return res;
}


/**
 * Get the connection state of the product from the scan record
 * @param value: the value
 * @return the connection state related to the given value.
 *         return ARDISCOVERY_CONNECTION_STATE_UNKNOWN if a value is given but unknown (forward compatibility)
 */
- (eARDISCOVERY_CONNECTION_STATE)connectionStateForValue:(uint8_t)value {
    return (value < ARDISCOVERY_CONNECTION_STATE_MAX) ? value : ARDISCOVERY_CONNECTION_STATE_UNKNOWN;
}

#ifdef USE_USB_ACCESSORY

#pragma mark - USBAccessoryManagerDelegate methods
- (void)USBAccessoryManager:(USBAccessoryManager*)usbAccessoryManager didAddDeviceWithConnectionId:(NSUInteger)connectionId name:(NSString *)name mux:(struct mux_ctx *)mux serial:(NSString *)serial productType:(eARDISCOVERY_PRODUCT)productType
{
    @synchronized (self)
    {
        if(mux != NULL)
        {
            ARService *aService = [self.devicesServicesList objectForKey:[NSNumber numberWithUnsignedInteger:connectionId]];
            if(aService == nil)
            {
                NSLog(@"%s New USB Service. Product name : %@. ConnectionId : %@",__FUNCTION__, name, [NSNumber numberWithUnsignedInteger:connectionId]);
                ARUSBService *usbService = [[ARUSBService alloc] init];
                usbService.usbMux = mux;
                usbService.serial = serial;
                usbService.connectionId = connectionId;

                aService = [[ARService alloc] init];
                aService.service = usbService;
                aService.network_type = ARDISCOVERY_NETWORK_TYPE_USBMUX;
                aService.name = name;
                aService.signal = [NSNumber numberWithInt:0];
                aService.product = productType;

                [self.devicesServicesList setObject:aService forKey:[NSNumber numberWithUnsignedInteger:connectionId]];
                [self sendDevicesListUpdateNotification];
            }
            else
            {
                BOOL sendNotification = NO;
                if(![aService.name isEqualToString:name])
                {
                    aService.name = name;
                    sendNotification = YES;
                }

                if(((ARUSBService*)aService.service).usbMux != mux)
                {
                    ((ARUSBService*)aService.service).usbMux = mux;
                    sendNotification = YES;
                }

                if(aService.product != productType)
                {
                    aService.product = productType;
                    sendNotification = YES;
                }

                if(sendNotification)
                {
                    [self sendDevicesListUpdateNotification];
                }
            }
        }
    }
}

- (void)USBAccessoryManager:(USBAccessoryManager*)usbAccessoryManager didRemoveDeviceWithConnectionId:(NSUInteger)connectionId
{
    @synchronized (self)
    {
        ARService *aService = [self.devicesServicesList objectForKey:[NSNumber numberWithUnsignedInteger:connectionId]];
        if(aService != nil)
        {
            NSLog(@"Removed service %@ : %@", aService.name, NSStringFromClass([[aService service] class]));
            [self.devicesServicesList removeObjectForKey:[NSNumber numberWithUnsignedInteger:connectionId]];
            [self sendDevicesListUpdateNotification];
        }
    }
}
#endif


#pragma mark - Notification sender
- (void)sendPublishNotification
{
    NSDictionary *userInfos = @{kARDiscoveryServiceName: ([self getCurrentPublishedServiceName] != nil) ? [self getCurrentPublishedServiceName] : @""};
    [[NSNotificationCenter defaultCenter] postNotificationName:kARDiscoveryNotificationServicePublished object:self userInfo:userInfos];
}

- (void)sendDevicesListUpdateNotification
{
    NSDictionary *userInfos = @{kARDiscoveryServicesList: [self getCurrentListOfDevicesServices]};
    [[NSNotificationCenter defaultCenter] postNotificationName:kARDiscoveryNotificationServicesDevicesListUpdated object:self userInfo:userInfos];
}

- (void)sendControllersListUpdateNotification
{
    NSDictionary *userInfos = @{kARDiscoveryServicesList: [self getCurrentListOfControllersServices]};
    [[NSNotificationCenter defaultCenter] postNotificationName:kARDiscoveryNotificationServicesControllersListUpdated object:self userInfo:userInfos];
}

- (void)sendResolveNotification
{
    NSDictionary *userInfos = @{kARDiscoveryServiceResolved: self.currentResolutionService};
    [[NSNotificationCenter defaultCenter] postNotificationName:kARDiscoveryNotificationServiceResolved object:self userInfo:userInfos];
}

- (void)sendNotResolveNotification
{
    [[NSNotificationCenter defaultCenter] postNotificationName:kARDiscoveryNotificationServiceNotResolved object:self userInfo:nil];
}

@end
