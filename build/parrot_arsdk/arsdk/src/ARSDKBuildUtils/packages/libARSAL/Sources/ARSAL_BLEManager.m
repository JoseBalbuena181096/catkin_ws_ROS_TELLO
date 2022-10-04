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
/**
 * @file ARSAL_BLEManager.m
 * @brief BLE network manager allow to send over ble network.
 * @brief wifi network manager allow to send over wifi network.
 * @date 06/11/2013
 * @author frederic.dhaeyer@parrot.com
 */

/*****************************************
 *
 *             inport file :
 *
 *****************************************/
#include "libARSAL/ARSAL_BLEManager.h"
#include "libARSAL/ARSAL_Error.h"

#define ARSAL_BLEMANAGER_ENABLE_DEBUG (0)
#define ARSAL_BLEMANAGER_CONNECTION_TIMEOUT_SEC (5)

#pragma mark CBUUID (String Extraction extension)
@implementation CBUUID (StringExtraction)
- (NSString *)representativeString
{
    NSData *data = [self data];

    NSUInteger bytesToConvert = [data length];
    const unsigned char *uuidBytes = [data bytes];
    NSMutableString *outputString = [NSMutableString stringWithCapacity:16];

    for (NSUInteger currentByteIndex = 0; currentByteIndex < bytesToConvert; currentByteIndex++)
    {
        switch (currentByteIndex)
        {
        case 3:
        case 5:
        case 7:
        case 9:[outputString appendFormat:@"%02x-", uuidBytes[currentByteIndex]]; break;
        default:[outputString appendFormat:@"%02x", uuidBytes[currentByteIndex]];
        }

    }

    return outputString;
}

- (NSString *)shortUUID
{
    NSString *uuid = [self representativeString];
    if (uuid.length > 4)
    {
        uuid = [uuid substringWithRange:NSMakeRange(4, 4)];
    }
    return uuid;
}
@end


@implementation ARSALBLEManagerNotificationData

@synthesize characteristic = _characteristic;
@synthesize value = _value;

- (id)initWithValue:(CBCharacteristic*)characteristic value:(NSData*)value
{
    self = [super init];
    if (self != nil)
    {
        _characteristic = characteristic;
        _value = value;
    }
    return self;
}

@end

@implementation ARSALBLEManagerNotification

@synthesize characteristics = _characteristics;
@synthesize notificationsArray = _notificationsArray;

- (id)init
{
    self = [super init];
    if (self != nil)
    {
        ARSAL_Sem_Init(&readCharacteristicsSem, 0, 0);
        ARSAL_Mutex_Init(&readCharacteristicMutex);
        _characteristics = nil;
        _notificationsArray = [NSMutableArray array];
    }
    return self;
}

- (void)dealloc
{
    ARSAL_Sem_Destroy(&readCharacteristicsSem);
    ARSAL_Mutex_Destroy(&readCharacteristicMutex);
}

- (void)addNotification:(ARSALBLEManagerNotificationData*)notificationData
{
    ARSAL_Mutex_Lock(&readCharacteristicMutex);

    [_notificationsArray addObject:notificationData];

    ARSAL_Mutex_Unlock(&readCharacteristicMutex);

    ARSAL_Sem_Post(&readCharacteristicsSem);
}

- (int)getAllNotifications:(NSMutableArray*)notificationsArray maxCount:(int)maxCount
{
    ARSAL_Mutex_Lock(&readCharacteristicMutex);

    NSMutableArray *removeNotifications = [NSMutableArray array];
    for (int i=0; (i < maxCount) && (i < (int)[_notificationsArray count]); i++)
    {
        NSObject *obj = _notificationsArray[i];
        [notificationsArray addObject:obj];
        [removeNotifications addObject:obj];
    }
    [_notificationsArray removeObjectsInArray:removeNotifications];
    [removeNotifications removeAllObjects];

    ARSAL_Mutex_Unlock(&readCharacteristicMutex);
    return (int)[notificationsArray count];
}

- (eARSAL_ERROR)waitNotification:(NSNumber *)timeout
{
    eARSAL_ERROR retVal = ARSAL_OK;
    const struct timespec connectionTimeout = {
        .tv_sec = [timeout doubleValue],
        .tv_nsec = 0,
    };

    if (timeout == nil)
    {
        int ret = ARSAL_Sem_Wait(&readCharacteristicsSem);
        if (ret != 0)
        {
            retVal = ARSAL_ERROR_SYSTEM;
        }
        // NO ELSE
    }
    else
    {
        int ret = ARSAL_Sem_Timedwait(&readCharacteristicsSem,&connectionTimeout);
        if (ret != 0)
        {
            if (errno == ETIMEDOUT)
            {
                retVal = ARSAL_ERROR_BLE_TIMEOUT;
            }
            else
            {
                retVal = ARSAL_ERROR_SYSTEM;
            }
        }
        // NO ELSE
    }
    return retVal;
}

- (BOOL)signalNotification
{
    int ret = ARSAL_Sem_Post(&readCharacteristicsSem);
    return (ret == 0) ? YES : NO;
}

- (void)resetNotification
{
    while (ARSAL_Sem_Trywait(&readCharacteristicsSem) == 0)
    {
        /* Do nothing*/
    }
}
@end

#pragma mark ARSAL_BLEManager implementation
@interface ARSAL_BLEManager ()

@property (nonatomic, assign) eARSAL_ERROR discoverServicesError;
@property (nonatomic, assign) eARSAL_ERROR discoverCharacteristicsError;
@property (nonatomic, assign) eARSAL_ERROR configurationCharacteristicError;
@property (nonatomic, assign) eARSAL_ERROR writeCharacteristicError;

@property (nonatomic, assign) BOOL askDisconnection;
@property (nonatomic, assign) BOOL isDiscoveringServices;
@property (nonatomic, assign) BOOL isDiscoveringCharacteristics;
@property (nonatomic, assign) BOOL isConfiguringCharacteristics;
@property (nonatomic, assign) BOOL isWritingCharacteristic;
@property (nonatomic, strong) NSMutableDictionary *registeredNotificationCharacteristics;
@property (nonatomic, assign) ARSAL_Mutex_t regNotCharacteristicsMutex;

- (void)ARSAL_BLEManager_Init;
@end

@implementation ARSAL_BLEManager
@synthesize discoverServicesError = _discoverServicesError;
@synthesize discoverCharacteristicsError = _discoverCharacteristicsError;
@synthesize configurationCharacteristicError = _configurationCharacteristicError;
@synthesize activePeripheral = _activePeripheral;
@synthesize characteristicsNotifications = _characteristicsNotifications;
@synthesize writeCharacteristicError = _writeCharacteristicError;
@synthesize delegate = _delegate;

SYNTHESIZE_SINGLETON_FOR_CLASS(ARSAL_BLEManager, ARSAL_BLEManager_Init);

// Initializing function to synhtesize singleton
- (void)ARSAL_BLEManager_Init
{
    _activePeripheral = nil;
    _discoverServicesError = ARSAL_OK;
    _discoverCharacteristicsError = ARSAL_OK;
    _characteristicsNotifications = [NSMutableArray array];
    _askDisconnection = NO;
    _isDiscoveringServices = NO;
    _isDiscoveringCharacteristics = NO;
    _isConfiguringCharacteristics = NO;
    _isWritingCharacteristic = NO;
    _registeredNotificationCharacteristics = [NSMutableDictionary dictionary];

    ARSAL_Sem_Init(&connectionSem, 0, 0);
    ARSAL_Sem_Init(&disconnectionSem, 0, 0);
    ARSAL_Sem_Init(&discoverServicesSem, 0, 0);
    ARSAL_Sem_Init(&discoverCharacteristicsSem, 0, 0);
    ARSAL_Sem_Init(&configurationSem, 0, 0);
    ARSAL_Sem_Init(&writeCharacteristicSem, 0, 0);
    ARSAL_Mutex_Init(&_regNotCharacteristicsMutex);
}

- (void)dealloc
{
    ARSAL_Sem_Destroy(&connectionSem);
    ARSAL_Sem_Destroy(&disconnectionSem);
    ARSAL_Sem_Destroy(&discoverServicesSem);
    ARSAL_Sem_Destroy(&discoverCharacteristicsSem);
    ARSAL_Sem_Destroy(&configurationSem);
    ARSAL_Sem_Destroy(&writeCharacteristicSem);
    ARSAL_Mutex_Destroy(&_regNotCharacteristicsMutex);
}

- (BOOL)isPeripheralConnected
{
    BOOL ret = NO;
    @synchronized (self)
    {
        if (self.activePeripheral != nil)
        {
            ret = YES;
        }
    }

    return ret;
}

- (eARSAL_ERROR)discoverNetworkServices:(NSArray *)servicesUUIDs
{
    eARSAL_ERROR result = ARSAL_OK;

    @synchronized (self)
    {
        // If there is an active peripheral, disconnecting it
        if(_activePeripheral != nil)
        {
            _isDiscoveringServices = YES;

            _discoverServicesError = ARSAL_OK;
            [self.activePeripheral discoverServices:nil];
            ARSAL_Sem_Wait(&discoverServicesSem);
            result = self.discoverServicesError;
            _discoverServicesError = ARSAL_OK;

            _isDiscoveringServices = NO;
        }
        else
        {
            result = ARSAL_ERROR_BLE_NOT_CONNECTED;
        }
    }

    return result;
}

- (eARSAL_ERROR)setNotificationCharacteristic:(CBCharacteristic *)characteristic
{
    eARSAL_ERROR result = ARSAL_OK;
    @synchronized (self)
    {
        // If there is an active peripheral, disconnecting it
        if(_activePeripheral != nil)
        {
            _isConfiguringCharacteristics = YES;

            _configurationCharacteristicError = ARSAL_OK;
            [self.activePeripheral setNotifyValue:YES forCharacteristic:characteristic];
            ARSAL_Sem_Wait(&configurationSem);
            result = self.configurationCharacteristicError;
            _configurationCharacteristicError = ARSAL_OK;

            _isConfiguringCharacteristics = NO;
        }
        else
        {
            result = ARSAL_ERROR_BLE_NOT_CONNECTED;
        }
    }

    return result;
}

- (eARSAL_ERROR)discoverNetworkCharacteristics:(NSArray *)characteristicsUUIDs forService:(CBService *)service
{
    eARSAL_ERROR result = ARSAL_OK;
    @synchronized (self)
    {
        // If there is an active peripheral, disconnecting it
        if(self.activePeripheral != nil)
        {
            _isDiscoveringCharacteristics = YES;

            NSLog(@"Service : %@", [service.UUID representativeString]);
            _discoverCharacteristicsError = ARSAL_OK;
            [_activePeripheral discoverCharacteristics:nil forService:service];
            ARSAL_Sem_Wait(&discoverCharacteristicsSem);
            result = _discoverCharacteristicsError;
            _discoverCharacteristicsError = ARSAL_OK;

            _isDiscoveringCharacteristics = NO;
        }
        else
        {
            result = ARSAL_ERROR_BLE_NOT_CONNECTED;
        }
    }

    return result;
}

- (eARSAL_ERROR)connectToPeripheral:(CBPeripheral *)peripheral withCentralManager:(ARSAL_CentralManager *)centralManager
{
#if ARSAL_BLEMANAGER_ENABLE_DEBUG
    NSLog(@"%s:%d : %@", __FUNCTION__, __LINE__, peripheral);
#endif
    eARSAL_ERROR result = ARSAL_OK;
    const struct timespec connectionTimeout = {
        .tv_sec = ARSAL_BLEMANAGER_CONNECTION_TIMEOUT_SEC,
        .tv_nsec = 0,
    };

    @synchronized (self)
    {

        [centralManager addDelegate:self];

        // If there is an active peripheral, disconnecting it
        if(_activePeripheral != nil)
        {
            [self disconnectPeripheral:_activePeripheral withCentralManager:centralManager];
        }

        // Connection to the new peripheral
        [centralManager connectPeripheral:peripheral options:[NSDictionary dictionaryWithObjectsAndKeys:[NSNumber numberWithBool:YES], CBConnectPeripheralOptionNotifyOnDisconnectionKey, nil]];

        if(ARSAL_Sem_Timedwait(&connectionSem, &connectionTimeout) != 0)
        {
            /* disconnect timeout */
            [centralManager cancelPeripheralConnection:peripheral];
        }

        if (_activePeripheral != nil)
        {
            _activePeripheral.delegate = self;
        }
        else
        {
            /* Connection failed */
            result = ARSAL_ERROR_BLE_CONNECTION;
        }

    }

    return result;
}

- (void)disconnectPeripheral:(CBPeripheral *)peripheral withCentralManager:(ARSAL_CentralManager *)centralManager
{
#if ARSAL_BLEMANAGER_ENABLE_DEBUG
    NSLog(@"%s:%d : %@", __FUNCTION__, __LINE__, peripheral);
#endif
    @synchronized (self)
    {
        if (self.activePeripheral != nil)
        {
            _askDisconnection = YES;

            [centralManager cancelPeripheralConnection:_activePeripheral];

            ARSAL_Sem_Wait(&disconnectionSem);

            [centralManager removeDelegate:self];

            _askDisconnection = NO;
        }
    }
}

- (BOOL)writeData:(NSData *)data toCharacteristic:(CBCharacteristic *)characteristic
{
    BOOL result = NO;
    if((_activePeripheral != nil) && (characteristic != nil) && (data != nil))
    {
        [_activePeripheral writeValue:data forCharacteristic:characteristic type:CBCharacteristicWriteWithoutResponse];
        result = YES;
    }

    return result;
}

- (eARSAL_ERROR)writeDataWithResponse:(NSData *)data toCharacteristic:(CBCharacteristic *)characteristic
{
    eARSAL_ERROR result = ARSAL_OK;

    if((_activePeripheral != nil) && (characteristic != nil) && (data != nil) && (_isWritingCharacteristic == NO))
    {
        _isWritingCharacteristic = YES;
        _writeCharacteristicError = ARSAL_OK;
        [_activePeripheral writeValue:data forCharacteristic:characteristic type:CBCharacteristicWriteWithResponse];

        ARSAL_Sem_Wait(&writeCharacteristicSem);
        _isWritingCharacteristic = NO;
        result = _writeCharacteristicError;
        _writeCharacteristicError = ARSAL_OK;
    }
    else
    {
        result = ARSAL_ERROR_BLE_CONNECTION;
    }
    return result;
}


- (void)registerNotificationCharacteristics:(NSArray *)characteristicsArray toKey:(NSString*)readCharacteristicsKey
{
#if ARSAL_BLEMANAGER_ENABLE_DEBUG
    NSLog(@"%s:%d", __FUNCTION__, __LINE__);
#endif

    ARSALBLEManagerNotification *notification = [[ARSALBLEManagerNotification alloc] init];
    [notification setCharacteristics:characteristicsArray];

    ARSAL_Mutex_Lock(&_regNotCharacteristicsMutex);
    [_registeredNotificationCharacteristics setObject:notification forKey:readCharacteristicsKey];
    ARSAL_Mutex_Unlock(&_regNotCharacteristicsMutex);
}

- (BOOL)unregisterNotificationCharacteristics:(NSString*)readCharacteristicsKey
{
    BOOL result = NO;
#if ARSAL_BLEMANAGER_ENABLE_DEBUG
    NSLog(@"%s:%d", __FUNCTION__, __LINE__);
#endif

    ARSAL_Mutex_Lock(&_regNotCharacteristicsMutex);
    ARSALBLEManagerNotification *notification = [_registeredNotificationCharacteristics objectForKey:readCharacteristicsKey];
    if (notification != nil)
    {
        result = YES;
        [_registeredNotificationCharacteristics removeObjectForKey:readCharacteristicsKey];
    }
    ARSAL_Mutex_Unlock(&_regNotCharacteristicsMutex);

    return result;
}

- (BOOL)cancelReadNotification:(NSString *)readCharacteristicsKey
{
    BOOL result = NO;
#if ARSAL_BLEMANAGER_ENABLE_DEBUG
    NSLog(@"%s:%d", __FUNCTION__, __LINE__);
#endif

    ARSAL_Mutex_Lock(&_regNotCharacteristicsMutex);
    ARSALBLEManagerNotification *notification = [_registeredNotificationCharacteristics objectForKey:readCharacteristicsKey];
    ARSAL_Mutex_Unlock(&_regNotCharacteristicsMutex);

    if (notification != nil)
    {
        [notification signalNotification];
        result = YES;
    }

    return result;
}

- (BOOL)resetReadNotification:(NSString *)readCharacteristicsKey
{
    BOOL result = NO;
#if ARSAL_BLEMANAGER_ENABLE_DEBUG
    NSLog(@"%s:%d", __FUNCTION__, __LINE__);
#endif

    ARSAL_Mutex_Lock(&_regNotCharacteristicsMutex);
    ARSALBLEManagerNotification *notification = [_registeredNotificationCharacteristics objectForKey:readCharacteristicsKey];
    ARSAL_Mutex_Unlock(&_regNotCharacteristicsMutex);

    if (notification != nil)
    {
        [notification resetNotification];
        result = YES;
    }

    return result;
}

- (eARSAL_ERROR)readNotificationData:(NSMutableArray *)notificationArray maxCount:(int)maxCount timeout:(NSNumber *)timeout toKey:(NSString *)readCharacteristicsKey
{
    eARSAL_ERROR error = ARSAL_OK;
#if ARSAL_BLEMANAGER_ENABLE_DEBUG
    NSLog(@"%s:%d", __FUNCTION__, __LINE__);
#endif

    if([self isPeripheralConnected])
    {
        ARSAL_Mutex_Lock(&_regNotCharacteristicsMutex);
        ARSALBLEManagerNotification *notification = [_registeredNotificationCharacteristics objectForKey:readCharacteristicsKey];
        ARSAL_Mutex_Unlock(&_regNotCharacteristicsMutex);

        if (notification != nil)
        {
            error = [notification waitNotification:timeout];
            if (error == ARSAL_OK)
            {
                if ([notification.notificationsArray count] > 0)
                {
                    [notification getAllNotifications:notificationArray maxCount:maxCount];
                }
                else
                {
                    error = ARSAL_ERROR_BLE_NO_DATA;
                }
            }
        }
        else
        {
            error = ARSAL_ERROR_BLE_CONNECTION;
        }
    }
    else
    {
        error = ARSAL_ERROR_BLE_CONNECTION;
    }

    return error;
}

- (void)readData:(CBCharacteristic *)characteristic
{
#if ARSAL_BLEMANAGER_ENABLE_DEBUG
    NSLog(@"%s:%d", __FUNCTION__, __LINE__);
#endif

    [_activePeripheral readValueForCharacteristic:characteristic];
}

#pragma mark CBCentralManagerDelegate
- (void)centralManagerDidUpdateState:(CBCentralManager *)central
{
#if ARSAL_BLEMANAGER_ENABLE_DEBUG
    NSLog(@"%s:%d", __FUNCTION__, __LINE__);
#endif

    switch(central.state)
    {
    case CBCentralManagerStatePoweredOn:
        NSLog(@"CBCentralManagerStatePoweredOn");
        break;

    case CBCentralManagerStateResetting:
        NSLog(@"CBCentralManagerStateResetting");
        break;

    case CBCentralManagerStateUnsupported:
        NSLog(@"CBCentralManagerStateUnsupported");
        break;

    case CBCentralManagerStateUnauthorized:
        NSLog(@"CBCentralManagerStateUnauthorized");
        break;

    case CBCentralManagerStatePoweredOff:
        NSLog(@"CBCentralManagerStatePoweredOff");
            if (_activePeripheral != nil)
            {
                [self onDisconnectPeripheral];
            }
        break;

    default:
    case CBCentralManagerStateUnknown:
        NSLog(@"CBCentralManagerStateUnknown");
        break;
    }
}

- (void)centralManager:(CBCentralManager *)central didConnectPeripheral:(CBPeripheral *)peripheral
{
#if ARSAL_BLEMANAGER_ENABLE_DEBUG
    NSLog(@"%s:%d -> %@", __FUNCTION__, __LINE__, peripheral);
#endif

    self.activePeripheral = peripheral;

    ARSAL_Sem_Post(&connectionSem);
}

- (void) centralManager:(CBCentralManager *)central didFailToConnectPeripheral:(CBPeripheral *)peripheral error:(NSError *) error
{
#if ARSAL_BLEMANAGER_ENABLE_DEBUG
    NSLog(@"%s:%d : %@", __FUNCTION__, __LINE__, peripheral);
#endif

    ARSAL_Sem_Post(&connectionSem);
}

- (void)centralManager:(CBCentralManager *)central didDisconnectPeripheral:(CBPeripheral *)peripheral error:(NSError *)error
{
#if ARSAL_BLEMANAGER_ENABLE_DEBUG
    NSLog(@"%s:%d : %@", __FUNCTION__, __LINE__, peripheral);
#endif

    if((_activePeripheral != nil) && (_activePeripheral == peripheral))
    {
        [self onDisconnectPeripheral];
    }
}

- (void) onDisconnectPeripheral
{
#if ARSAL_BLEMANAGER_ENABLE_DEBUG
    NSLog(@"%s:%d", __FUNCTION__, __LINE__);
#endif

    _activePeripheral.delegate = nil;
    _activePeripheral = nil;

    /* Post disconnectionSem only if the disconnect is asked */
    if(_askDisconnection)
    {
        ARSAL_Sem_Post(&disconnectionSem);
    }

    /* if activePeripheral is discovering services */
    if(_isDiscoveringServices)
    {
        _discoverServicesError = ARSAL_ERROR_BLE_NOT_CONNECTED;
        ARSAL_Sem_Post(&discoverServicesSem);
    }

    /* if activePeripheral is discovering Characteristics */
    if(_isDiscoveringCharacteristics)
    {
        _discoverCharacteristicsError = ARSAL_ERROR_BLE_NOT_CONNECTED;
        ARSAL_Sem_Post(&discoverCharacteristicsSem);
    }

    /* if activePeripheral is configuring Characteristics */
    if(_isConfiguringCharacteristics)
    {
        _configurationCharacteristicError = ARSAL_ERROR_BLE_NOT_CONNECTED;
        ARSAL_Sem_Post(&configurationSem);
    }

    /* if activePeripheral is writing Characteristics */
    if (_isWritingCharacteristic)
    {
        _writeCharacteristicError = ARSAL_ERROR_BLE_NOT_CONNECTED;
        ARSAL_Sem_Post(&writeCharacteristicSem);
    }

    /* Notify delegate */
    if(!_askDisconnection)
    {
        if ((_delegate != nil) && ([_delegate respondsToSelector:@selector(onBLEDisconnect)]))
        {
            [_delegate onBLEDisconnect];
        }
    }
}

#pragma mark CBPeripheralDelegate
- (void)peripheral:(CBPeripheral *)peripheral didDiscoverServices:(NSError *)error
{
#if ARSAL_BLEMANAGER_ENABLE_DEBUG
    NSLog(@"%s:%d - %@", __FUNCTION__, __LINE__, peripheral);
#endif

    if (error != nil)
    {
        _discoverServicesError = ARSAL_ERROR_BLE_SERVICES_DISCOVERING;
    }

    ARSAL_Sem_Post(&discoverServicesSem);
}

- (void)peripheral:(CBPeripheral *)peripheral didDiscoverCharacteristicsForService:(CBService *)service error:(NSError *)error
{
#if ARSAL_BLEMANAGER_ENABLE_DEBUG
    NSLog(@"%s:%d - %@", __FUNCTION__, __LINE__, peripheral);
#endif

    if (error != nil)
    {
        _discoverCharacteristicsError = ARSAL_ERROR_BLE_CHARACTERISTICS_DISCOVERING;
    }

    ARSAL_Sem_Post(&discoverCharacteristicsSem);
}

- (void)peripheral:(CBPeripheral *)peripheral didWriteValueForCharacteristic:(CBCharacteristic *)characteristic error:(NSError *)error
{
#if ARSAL_BLEMANAGER_ENABLE_DEBUG
    NSLog(@"%s:%d - %@ : %@", __FUNCTION__, __LINE__, peripheral, [error localizedDescription]);
#endif

    if (_isWritingCharacteristic)
    {
        if (error != nil)
        {
            _writeCharacteristicError = ARSAL_ERROR_BLE_SERVICES_DISCOVERING;
        }

        ARSAL_Sem_Post(&writeCharacteristicSem);
    }
}

- (void)peripheral:(CBPeripheral *)peripheral didUpdateValueForCharacteristic:(CBCharacteristic *)characteristic error:(NSError *)error
{
#if ARSAL_BLEMANAGER_ENABLE_DEBUG
    NSLog(@"%s:%d - %@ : %d, %@", __FUNCTION__, __LINE__, [characteristic.UUID representativeString], [characteristic.value length], [error localizedDescription]);
#endif

    ARSALBLEManagerNotification *foundNotification = nil;

    ARSAL_Mutex_Lock(&_regNotCharacteristicsMutex);
    for (NSString* key in [_registeredNotificationCharacteristics allKeys])
    {
        ARSALBLEManagerNotification *notification = [_registeredNotificationCharacteristics objectForKey:key];

        for (CBCharacteristic *characteristicItem in [notification characteristics])
        {
            if ([characteristicItem.UUID.data isEqualToData:characteristic.UUID.data])
            {
                foundNotification = notification;
                break;
            }
        }

        if (foundNotification != nil)
        {
            break;
        }
    }
    ARSAL_Mutex_Unlock(&_regNotCharacteristicsMutex);

    if (foundNotification != nil)
    {
        ARSALBLEManagerNotificationData *notificationData = [[ARSALBLEManagerNotificationData alloc] initWithValue:characteristic value:[characteristic value]];
        [foundNotification addNotification:notificationData];
    }
}

- (void)peripheral:(CBPeripheral *)peripheral didUpdateNotificationStateForCharacteristic:(CBCharacteristic *)characteristic error:(NSError *)error
{
#if ARSAL_BLEMANAGER_ENABLE_DEBUG
    NSLog(@"%s:%d - %@ : %@", __FUNCTION__, __LINE__, [characteristic.UUID representativeString], [error localizedDescription]);
#endif

    if (error != nil)
    {
        _configurationCharacteristicError = ARSAL_ERROR_BLE_CHARACTERISTIC_CONFIGURING;
    }

    ARSAL_Sem_Post(&configurationSem);
}

- (void)peripheralDidUpdateName:(CBPeripheral *)peripheral
{
#if ARSAL_BLEMANAGER_ENABLE_DEBUG
    NSLog(@"%s:%d -> %@", __FUNCTION__, __LINE__, peripheral);
#endif
}

- (void)unlock
{
    @synchronized (self)
    {
        NSDictionary *regCharacteristics = nil;

        /* post all Semaphore to unlock the all the functions */
    #if ARSAL_BLEMANAGER_ENABLE_DEBUG
        NSLog(@"%s:%d", __FUNCTION__, __LINE__);
    #endif

        ARSAL_Sem_Post(&connectionSem);
        ARSAL_Sem_Post(&discoverServicesSem);
        ARSAL_Sem_Post(&discoverCharacteristicsSem);
        ARSAL_Sem_Post(&configurationSem);
        ARSAL_Sem_Post(&writeCharacteristicSem);

        ARSAL_Mutex_Lock(&_regNotCharacteristicsMutex);
        regCharacteristics = [_registeredNotificationCharacteristics copy];
        ARSAL_Mutex_Unlock(&_regNotCharacteristicsMutex);

        for (ARSALBLEManagerNotification *notification in [regCharacteristics allValues])
        {
            [notification signalNotification];
        }

        /* disconnectionSem is not post because:
         * if the connection is fail, disconnect is not call.
         * if the connection is successful, the BLE callback is always called.
         * the disconnect function is called after the join of the network threads.
         */
    }
}

- (void)reset
{
    @synchronized (self)
    {
        NSDictionary *regCharacteristics = nil;

        /* reset all Semaphores */
#if ARSAL_BLEMANAGER_ENABLE_DEBUG
        NSLog(@"%s:%d", __FUNCTION__, __LINE__);
    #endif

        while (ARSAL_Sem_Trywait(&connectionSem) == 0)
        {
            /* Do nothing*/
        }

        while (ARSAL_Sem_Trywait(&discoverServicesSem) == 0)
        {
            /* Do nothing*/
        }

        while (ARSAL_Sem_Trywait(&discoverCharacteristicsSem) == 0)
        {
            /* Do nothing*/
        }

        while (ARSAL_Sem_Trywait(&disconnectionSem) == 0)
        {
            /* Do nothing*/
        }

        while (ARSAL_Sem_Trywait(&configurationSem) == 0)
        {
            /* Do nothing*/
        }

        while (ARSAL_Sem_Trywait(&writeCharacteristicSem) == 0)
        {
            /* Do nothing*/
        }

        ARSAL_Mutex_Lock(&_regNotCharacteristicsMutex);
        regCharacteristics = [_registeredNotificationCharacteristics copy];
        [_registeredNotificationCharacteristics removeAllObjects];
        ARSAL_Mutex_Unlock(&_regNotCharacteristicsMutex);

        for (ARSALBLEManagerNotification *notification in [regCharacteristics allValues])
        {
            [notification signalNotification];
        }
    }
}

@end
