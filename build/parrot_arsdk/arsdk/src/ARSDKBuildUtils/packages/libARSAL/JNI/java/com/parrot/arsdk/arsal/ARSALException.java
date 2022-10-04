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
 * Exception class: ARSALException of ARSAL library
 * @date 30/12/2013
 * @author david.flattin.ext@parrot.com
 */
public class ARSALException extends Exception
{	
    private ARSAL_ERROR_ENUM error;
    
    /**
     * ARSALException constructor
     * @return void
     */
    public ARSALException()
    {
        error = ARSAL_ERROR_ENUM.ARSAL_ERROR;
    }
    
    /**
     * ARSALException constructor
     * @param error ARSAL_ERROR_ENUM error code
     * @return void
     */
    public ARSALException(ARSAL_ERROR_ENUM error) 
    {
        this.error = error;
    }
    
    /**
     * ARSALException constructor
     * @param error int error code
     * @return void
     */
    public ARSALException(int error) 
    {
        this.error = ARSAL_ERROR_ENUM.getFromValue(error);
    }
    
    /**
     * Gets ARSAL ERROR code
     * @return {@link ARSAL_ERROR_ENUM} error code
     */
    public ARSAL_ERROR_ENUM getError()
    {
        return error;
    }
    
    /**
     * Sets ARSAL ERROR code
     * @param error {@link ARSAL_ERROR_ENUM} error code     
     * @return void
     */
    public void setError(ARSAL_ERROR_ENUM error)
    {
        this.error = error;
    }
    
    /**
     * Gets ARSALException string representation
     * @return String Exception representation
     */
    public String toString ()
    {
        String str;
        
        if (null != error)
        {
            str = "ARSALException [" + error.toString() + "]";
        }
        else
        {
            str = super.toString();
        }
        
        return str;
    }
}

