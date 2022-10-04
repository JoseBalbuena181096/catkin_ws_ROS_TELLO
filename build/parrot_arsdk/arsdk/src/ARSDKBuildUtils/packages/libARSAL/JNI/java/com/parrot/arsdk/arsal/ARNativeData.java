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

/**
 * Root class for all C data that may pass through different libs of AR.SDK<br>
 * <br>
 * AR.SDK Is mainly implemented in C, and thus libs often use C structures to hold data.<br>
 * This is a wrapper class onto these kind of data, so java can pass them around libs without copy into JVM memory space.
 */
public class ARNativeData
{
    /* ************** */
    /* DEFAULT VALUES */
    /* ************** */

    /**
     * Tag for ARSALPrint prints
     */
    private static final String TAG = ARNativeData.class.getSimpleName();

    /**
     * Default size for native data
     */
    public static final int DEFAULT_SIZE = 128;

    /* *********************** */
    /* INTERNAL REPRESENTATION */
    /* *********************** */

    /**
     * Storage of the C Pointer
     */
    protected long pointer;
    /**
     * Storage of the allocated size of the C Pointer
     */
    protected int capacity;
    /**
     * How many bytes are used in the buffer
     */
    protected int used;

    /**
     * Check validity before all native calls
     */
    protected boolean valid;

    /**
     * Dummy throwable to keep the constructors call stack
     */
    private Throwable constructorCallStack;

    /* ************ */
    /* CONSTRUCTORS */
    /* ************ */

    /**
     * Generic constructor<br>
     * This constructor allocate a <code>ARNativeData</code> with the given capacity
     * @param size Initial capacity of the <code>ARNativeData</code>
     */
    public ARNativeData (int size) {
        this.pointer = allocateData (size);
        this.capacity = 0;
        this.valid = false;
        if (this.pointer != 0) {
            this.capacity = size;
            this.valid = true;
        }
        this.used = 0;
        this.constructorCallStack = new Throwable();
    }

    /**
     * Default size constructor
     * This constructor allocate a <code>ARNativeData</code> with the default capacity
     */
    public ARNativeData () {
        this (DEFAULT_SIZE);
    }

    /**
     * Copy constructor<br>
     * This constructor creates a new <code>ARNativeData</code> with a capacity<br>
     * equal to the original objects data size
     * @param data The original <code>ARNativeData</code> to copy
     */
    public ARNativeData (ARNativeData data) {
        this (data.getDataSize ());
        if (! copyData (pointer, capacity, data.getData (), data.getDataSize ())) {
            dispose ();
        }
    }

    /**
     * Copy constructor (from native)<br>
     * This constructor creates a new <code>ARNativeData</code> with a capacity<br>
     * equal to the original data size
     * @param data The original data buffer to copy
     * @param dataSize The original data buffer length
     */
    public ARNativeData(long data, int dataSize) {
        this (dataSize);
        if (! copyData (pointer, capacity, data, dataSize)) {
            dispose ();
        }
    }

    /**
     * Copy constructor, with specified allocation size<br>
     * This constructor creates a new <code>ARNativeData</code> with a capacity<br>
     * equal to the greatest value between the specified <code>capacity</code> and the original<br>
     * object data size
     * @param data The original <code>ARNativeData</code> to copy
     * @param capacity The minimum capacity in bytes for this <code>ARNativeData</code>
     */
    public ARNativeData (ARNativeData data, int capacity) {
        int totalSize = data.getDataSize ();
        if (capacity > totalSize) {
            totalSize = capacity;
        }
        this.pointer = allocateData (totalSize);
        this.capacity = 0;
        this.valid = false;
        if (this.pointer != 0) {
            this.capacity = totalSize;
            this.valid = true;
        }
        this.used = 0;
        this.constructorCallStack = new Throwable();
        if (! copyData (this.pointer, this.capacity, data.getData (), data.getDataSize ())) {
            dispose ();
        }
    }

    /* ********** */
    /* DESTRUCTOR */
    /* ********** */

    protected void finalize () throws Throwable {
        try {
            if (valid) {
                ARSALPrint.w (TAG, this + ": Finalize error -> dispose () was not called !", this.constructorCallStack);
                dispose ();
            }
        }
        finally {
            super.finalize ();
        }
    }

    /* ********* */
    /* TO STRING */
    /* ********* */

    /**
     * Gets a <code>String</code> representation of the <code>ARNativeData</code>
     * @return A human-readable description of the <code>ARNativeData</code>
     */
    public String toString () {
        return "" + this.getClass().getSimpleName() + " { Valid : " + valid + " | " +
            "Used/Capacity (bytes) : " + used + "/" + capacity + " | " +
            "C Pointer : " + pointer + " }";
    }

    /* ************** */
    /* IMPLEMENTATION */
    /* ************** */

    /**
     * Checks the object validity
     * @return <code>true</code> if the object is valid (buffer properly alloc and usable)<br><code>false</code> if the object is invalid (alloc error, disposed object)
     */
    public boolean isValid () {
        return valid;
    }

    /**
     * Sets the used size of the <code>ARNativeData</code>
     * @param size New used size of the command
     * @return <code>true</code> if the size was set<br><code>false</code> if the size was invalid (bigger than capacity) or if the <code>ARNativeData</code> is invalid
     */
    public boolean setUsedSize (int size) {
        if ((! valid) || (size > capacity))
            return false;
        this.used = size;
        return true;
    }

    /**
     * Gets the capacity of the <code>ARNativeData</code><br>
     * This function returns 0 (no capacity) if the data is not valid
     * @return Capacity of the native data buffer
     */
    public int getCapacity () {
        if (valid)
            return capacity;
        return 0;
    }

    /**
     * Gets the C data pointer (equivalent C type : uint8_t *)<br>
     * This function returns 0 (C-NULL) if the data is not valid (i.e. disposed or uninitialized)
     * @return Native data pointer
     */
    public long getData () {
        if (valid)
            return pointer;
        return 0;
    }

    /**
     * Get the C data size (in bytes)<br>
     * This function returns 0 (no data) if the data is not valid
     * @return Size of the native data
     */
    public int getDataSize () {
        if (valid)
            return used;
        return 0;
    }

    /**
     * Gets a byte array which is a copy of the C native data<br>
     * This function allow Java code to access (but not modify) the content of the <code>ARNativeData</code><br>
     * This function creates a new byte [], and thus should not be called in a loop
     * @return A Java copy of the native data (byte equivalent)
     */
    public byte [] getByteData () {
        if (valid)
            return generateByteArray (pointer, capacity, used);
        return null;
    }

	/**
     * Copy data from byte array into C native buffer
     * This function allow Java code to copy into the content of the <code>ARNativeData</code><br>
     * @param src byte buffer from java world   
	 * @param dataSize number of bytes in the array
     * @return <code>true</code> if the copy was done without error<br><code>false</code> if an error occured. In this case, the content of the internal buffer is undefined, so <code>used</code> should be set to zero
     */
    public boolean copyByteData (byte [] src, int dataSize) {
        boolean ret = false;
        if (valid && copyJavaData (pointer, capacity, src, dataSize)) {
            setUsedSize (dataSize);
            ret = true;
        }
        return ret;
    }

    /**
     * Marks a native data as unused (so C-allocated memory can be freed)<br>
     * A disposed data is marked as invalid
     */
    public void dispose () {
        if (valid)
            freeData (pointer);
        this.valid = false;
        this.pointer = 0;
        this.capacity = 0;
        this.used = 0;
    }
    
    /**
     * Ensures the capacity is at least the minimum capacity set.
     * @warning if the capacity is least of minimumCapacity, the buffer will try to be re-allocated  
     * @param minimumCapacity minimum capacity to ensure
     * @return <code>true</code> if the capacity is at least minimumCapacity <br><code>false</code> if the reallocation of the buffer with at least the minimum capacity has failed
     * 
     */
    public boolean ensureCapacityIsAtLeast (int minimumCapacity)
    {
        boolean retVal = false;
        long newPointer = 0;
        
        if (this.capacity >= minimumCapacity)
        {
            retVal = true;
        }
        else
        {
            newPointer = reallocateData (this.pointer, minimumCapacity);
            if (newPointer != 0)
            {
                this.capacity = minimumCapacity;
                this.pointer = newPointer;
                retVal = true;
            }
            /* else retVal = false */
        }
        
        return retVal;
    }

    /* **************** */
    /* NATIVE FUNCTIONS */
    /* **************** */

    /**
     * Memory allocation in native memory space<br>
     * Allocates a memory buffer of size <code>capacity</code> and return its C-Pointer
     * @param capacity Size, in bytes, of the buffer to allocate
     * @return C-Pointer on the buffer, or 0 (C-NULL) if the alloc failed
     */
    private native long allocateData (int capacity);
    
    /**
     * Memory reallocation in native memory space<br>
     * Reallocates a memory buffer of size <code>capacity</code> and return its C-Pointer
     * @param pointer native pointer to reallocate
     * @param capacity Size, in bytes, of the new buffer to reallocated
     * @return C-Pointer on the new buffer, or 0 (C-NULL) if the realloc failed
     */
    private native long reallocateData (long pointer, int capacity);

    /**
     * Memory release in native memory space<br>
     * Frees a memory buffer from its C-Pointer<br>
     * This call is needed because JVM do not know about native memory allocs
     * @param data C-Pointer on the buffer to free
     */
    private native void freeData (long data);

    /**
     * Copy data to a byte array<br>
     * This function is needed to implement the <code>getByteData</code> function
     * @param data C-Pointer to the internal buffer
     * @param capacity Capacity of the internal buffer (avoid overflows)
     * @param used Used bytes in the internal buffer (size to copy)
     * @return A byte array, which content is the byte-equal copy of the native buffer
     */
    private native byte [] generateByteArray (long data, int capacity, int used);

    /**
     * Copy data from another <code>ARNativeData</code><br>
     * This function is used in the copy constructors.
     * @param dst C-Pointer on this object's internal buffer
     * @param dstCapacity Capacity of this object's internal buffer
     * @param src C-Pointer on the initial object's internal buffer
     * @param srcLen Used bytes in the initial object's internal buffer
     * @return <code>true</code> if the copy was done without error<br><code>false</code> if an error occured. In this case, the content of the internal buffer is undefined, so <code>used</code> should be set to zero
     */
    private native boolean copyData (long dst, int dstCapacity, long src, int srcLen);

	/**
     * Copy data from another <code>ARNativeData</code><br>
     * This function is used in the copy constructors.
     * @param dst C-Pointer on this object's internal buffer
     * @param dstCapacity Capacity of this object's internal buffer
     * @param src byte buffer from java world
     * @param srcLen Used bytes in the initial object's internal buffer
     * @return <code>true</code> if the copy was done without error<br><code>false</code> if an error occured. In this case, the content of the internal buffer is undefined, so <code>used</code> should be set to zero
     */
    private native boolean copyJavaData (long dst, int dstCapacity, byte[] src, int srcLen);

}
