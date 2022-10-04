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
//  ARSAL_CentralManager.m
//  ARFreeFlight
//
//  Created by Frédéric D'HAEYER on 12/03/14.
//  Copyright (c) 2014 Parrot SA. All rights reserved.
//
#import "libARSAL/ARSAL_CentralManager.h"

@interface ARSAL_CentralManager () <CBCentralManagerDelegate>
@property (nonatomic, strong) NSMutableArray *delegateArray;
@property (nonatomic, strong) CBCentralManager *centralManager;
@end

@implementation ARSAL_CentralManager
- (void)initARCBCentralManager:(dispatch_queue_t)queue options:(NSDictionary *)options
{
    _delegateArray = [NSMutableArray array];
    _centralManager = [[CBCentralManager alloc] initWithDelegate:self queue:queue options:options];
}

- (id)init
{
    self = [super init];
    if(self)
    {
        [self initARCBCentralManager:nil options:nil];
    }
    
    return self;
}

- (id)initWithQueue:(dispatch_queue_t)queue
{
    self = [super init];
    if(self)
    {
        [self initARCBCentralManager:queue options:nil];
    }
    
    return self;
}

- (id)initWithDelegate:(id<CBCentralManagerDelegate>)delegate queue:(dispatch_queue_t)queue options:(NSDictionary *)options
{
    self = [super init];
    if(self)
    {
        [self initARCBCentralManager:queue options:options];
    }
    
    return self;
}

- (void)dealloc
{
    _centralManager.delegate = nil;
    _centralManager = nil;
    _delegateArray = nil;
}

- (void)addDelegate:(id<CBCentralManagerDelegate>)delegate
{
    @synchronized (self)
    {
        if((delegate != nil) && (![_delegateArray containsObject:delegate]))
        {
            [_delegateArray addObject:delegate];
        }
    }
}

- (void)removeDelegate:(id<CBCentralManagerDelegate>)delegate
{
    @synchronized (self)
    {
        if((delegate != nil) && ([_delegateArray containsObject:delegate]))
        {
            [_delegateArray removeObject:delegate];
        }
    }
}

#pragma mark CBCentralManagerDelegate required method
- (void)centralManagerDidUpdateState:(CBCentralManager *)central
{
    NSMutableArray *delegateArray = nil;
    @synchronized (self)
    {
        delegateArray = [_delegateArray copy];
    }
    
    for(id<CBCentralManagerDelegate>delegate in delegateArray)
    {
        [delegate centralManagerDidUpdateState:central];
    }
}

#pragma mark CBCentralManagerDelegate optional method
- (void)centralManager:(CBCentralManager *)central willRestoreState:(NSDictionary *)dict
{
    NSMutableArray *delegateArray = nil;
    @synchronized (self)
    {
        delegateArray = [_delegateArray copy];
    }
    
    for(id<CBCentralManagerDelegate>delegate in delegateArray)
    {
        if([delegate respondsToSelector:@selector(centralManager:willRestoreState:)])
            [delegate centralManager:central willRestoreState:dict];
    }
}

- (void)centralManager:(CBCentralManager *)central didDiscoverPeripheral:(CBPeripheral *)peripheral advertisementData:(NSDictionary *)advertisementData RSSI:(NSNumber *)RSSI
{
    NSMutableArray *delegateArray = nil;
    @synchronized (self)
    {
        delegateArray = [_delegateArray copy];
    }
    
    for(id<CBCentralManagerDelegate>delegate in delegateArray)
    {
        if([delegate respondsToSelector:@selector(centralManager:didDiscoverPeripheral:advertisementData:RSSI:)])
            [delegate centralManager:central didDiscoverPeripheral:peripheral advertisementData:advertisementData RSSI:RSSI];
    }
}

- (void)centralManager:(CBCentralManager *)central didConnectPeripheral:(CBPeripheral *)peripheral
{
    NSMutableArray *delegateArray = nil;
    @synchronized (self)
    {
        delegateArray = [_delegateArray copy];
    }
    
    for(id<CBCentralManagerDelegate>delegate in delegateArray)
    {
        if([delegate respondsToSelector:@selector(centralManager:didConnectPeripheral:)])
            [delegate centralManager:central didConnectPeripheral:peripheral];
    }
}

- (void)centralManager:(CBCentralManager *)central didFailToConnectPeripheral:(CBPeripheral *)peripheral error:(NSError *)error
{
    NSMutableArray *delegateArray = nil;
    @synchronized (self)
    {
        delegateArray = [_delegateArray copy];
    }
    
    for(id<CBCentralManagerDelegate>delegate in delegateArray)
    {
        if([delegate respondsToSelector:@selector(centralManager:didFailToConnectPeripheral:error:)])
            [delegate centralManager:central didFailToConnectPeripheral:peripheral error:error];
    }
}

- (void)centralManager:(CBCentralManager *)central didDisconnectPeripheral:(CBPeripheral *)peripheral error:(NSError *)error
{
    NSMutableArray *delegateArray = nil;
    @synchronized (self)
    {
        delegateArray = [_delegateArray copy];
    }
    
    for(id<CBCentralManagerDelegate>delegate in delegateArray)
    {
        if([delegate respondsToSelector:@selector(centralManager:didDisconnectPeripheral:error:)])
            [delegate centralManager:central didDisconnectPeripheral:peripheral error:error];
    }
}

- (void)connectPeripheral:(CBPeripheral *)peripheral options:(NSDictionary *)options
{
    [_centralManager connectPeripheral:peripheral options:options];
}

- (void)cancelPeripheralConnection:(CBPeripheral *)peripheral
{
    [_centralManager cancelPeripheralConnection:peripheral];
}

- (void)scanForPeripheralsWithServices:(NSArray *)serviceUUIDs options:(NSDictionary *)options
{
    [_centralManager scanForPeripheralsWithServices:serviceUUIDs options:options];
}

- (void)stopScan
{
    [_centralManager stopScan];
}


- (NSArray *)retrieveConnectedPeripheralsWithServices:(NSArray *)serviceUUIDs
{
    return [_centralManager retrieveConnectedPeripheralsWithServices:serviceUUIDs];
}

- (NSArray *)retrievePeripheralsWithIdentifiers:(NSArray *)identifiers
{
    return [_centralManager retrievePeripheralsWithIdentifiers:identifiers];
}

@end
