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
 * @file ARDISCOVERY_JNI_Discovery.c
 * @brief libARDiscovery JNI dicovery c file.
 **/

/*****************************************
 *
 *             include file :
 *
 *****************************************/

#include <jni.h>
#include <stdlib.h>

#include <libARSAL/ARSAL_Print.h>

#include <libARDiscovery/ARDISCOVERY_Discovery.h>


/*****************************************
 *
 *             define :
 *
 *****************************************/

#define ARDISCOVERY_JNIDISCOVERY_TAG "JNIDiscovery"

/*****************************************
 *
 *             implementation :
 *
 *****************************************/

 /**
 * @brief Converts a product enumerator in product ID
 * This function is the only one knowing the correspondance
 * between the enumerator and the products' IDs.
 * @param env reference to the java environment
 * @param thizz reference to the object calling this function
 * @param product The product's enumerator
 * @return The corresponding product ID
 */
JNIEXPORT jint JNICALL
Java_com_parrot_arsdk_ardiscovery_ARDiscoveryService_nativeGetProductID (JNIEnv *env, jclass thizz, int product)
{
    return ARDISCOVERY_getProductID (product);
}

 /**
 * @brief Converts a product ID in product name
 * This function is the only one knowing the correspondance
 * between the products' IDs and the product names.
 * @param product The product ID
 * @return The corresponding product name
 */
JNIEXPORT jstring JNICALL
Java_com_parrot_arsdk_ardiscovery_ARDiscoveryService_nativeGetProductName (JNIEnv *env, jclass thizz, int product)
{
    const char *nativeName;
    jstring jName = NULL;

    nativeName = ARDISCOVERY_getProductName (product);

    if (nativeName != NULL)
    {
        jName = (*env)->NewStringUTF(env, nativeName);
    }

    return jName;
}

/**
 * @brief Converts a product ID in product path name
 * This function is the only one knowing the correspondance
 * between the products' IDs and the product path names.
 * @param product The product ID
 * @return The corresponding product path name
 */
JNIEXPORT jstring JNICALL
Java_com_parrot_arsdk_ardiscovery_ARDiscoveryService_nativeGetProductPathName (JNIEnv *env, jclass thizz, int product)
{
    char nativePathName[128];
    jstring jPathName = NULL;

    nativePathName[0] = '\0';

    ARDISCOVERY_getProductPathName (product, nativePathName, sizeof(nativePathName));

    jPathName = (*env)->NewStringUTF(env, nativePathName);

    return jPathName;
}

 /**
 * @brief Converts a product product name in product ID
 * This function is the only one knowing the correspondance
 * between the product names and the products' IDs.
 * @param name The product's product name
 * @return The corresponding product ID
 */
JNIEXPORT jint JNICALL
Java_com_parrot_arsdk_ardiscovery_ARDiscoveryService_nativeGetProductFromName (JNIEnv *env, jclass thizz, jstring name)
{
    const char *nativeName = (*env)->GetStringUTFChars(env, name, 0);
    jint product;

    product = ARDISCOVERY_getProductFromName (nativeName);

    if (nativeName != NULL)
    {
         (*env)->ReleaseStringUTFChars(env, name, nativeName);
    }

    return product;
}

/**
 * @brief get ARDISCOVERY_SERVICE_NET_DEVICE_FORMAT
 * @return value of ARDISCOVERY_SERVICE_NET_DEVICE_FORMAT
 */
 JNIEXPORT jstring JNICALL
Java_com_parrot_arsdk_ardiscovery_ARDiscoveryService_nativeGetDefineNetDeviceFormat (JNIEnv *env, jclass class)
{
    return  (*env)->NewStringUTF(env, ARDISCOVERY_SERVICE_NET_DEVICE_FORMAT);
}

/**
 * @brief get ARDISCOVERY_SERVICE_NET_DEVICE_DOMAIN
 * @return value of ARDISCOVERY_SERVICE_NET_DEVICE_DOMAIN
 */
 JNIEXPORT jstring JNICALL
Java_com_parrot_arsdk_ardiscovery_ARDiscoveryService_nativeGetDefineNetDeviceDomain (JNIEnv *env, jclass class)
{
    return  (*env)->NewStringUTF(env, ARDISCOVERY_SERVICE_NET_DEVICE_DOMAIN);
}

 /**
 * @brief Converts a product ID to product enumerator
 * This function is the only one knowing the correspondance
 * between the ID and the products' enumerator.
 * @param env reference to the java environment
 * @param thizz reference to the object calling this function
 * @param productID The product's ID
 * @return The corresponding product enumerator
 */
JNIEXPORT jint JNICALL
Java_com_parrot_arsdk_ardiscovery_ARDiscoveryService_nativeGetProductFromProductID (JNIEnv *env, jclass thizz, jint productID)
{
    return ARDISCOVERY_getProductFromProductID (productID);
}

 /**
 * @brief Converts a product to product family
 * This function is the only one knowing the correspondance
 * between the the product's and the product's families.
 * @param env reference to the java environment
 * @param thizz reference to the object calling this function
 * @param product The product
 * @return The corresponding family enumerator
 */
JNIEXPORT jint JNICALL
Java_com_parrot_arsdk_ardiscovery_ARDiscoveryService_nativeGetProductFamily (JNIEnv *env, jclass thizz, jint product)
{
    return ARDISCOVERY_getProductFamily (product);
}
