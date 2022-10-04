package com.parrot.arsdk.ardiscovery;

public class ARDiscoveryException extends Exception 
{
    private ARDISCOVERY_ERROR_ENUM error;
    
    public ARDiscoveryException()
    {
        error = ARDISCOVERY_ERROR_ENUM.ARDISCOVERY_ERROR;
    }
    
    public ARDiscoveryException (ARDISCOVERY_ERROR_ENUM error) 
    {
        this.error = error;
    }
    
    public ARDiscoveryException (int error) 
    {
        this.error = ARDISCOVERY_ERROR_ENUM.getFromValue(error);
    }
    
    public ARDISCOVERY_ERROR_ENUM getError()
    {
        return error;
    }
    
    public void setError(ARDISCOVERY_ERROR_ENUM error)
    {
        this.error = error;
    }
    
}

