
extern float gyrocal[3];
extern float accelcal[3];

#include "drv_fmc.h"

unsigned writecount;
unsigned options1;
unsigned options2;

void savecal()
{
	int error = 0;
	int addresscount = 0;
	// load non modified flash values
	
	
	
	// erase page
	if (!error) error = fmc_erasepage( );
	
	//write values
	if ( !error)
	{
		fmc_write( addresscount , 0x00AC );
		addresscount++;
		for ( int x = 0 ; x < 3; x++)
		{
		int fl = *(int*)&gyrocal[x];
		fmc_write(  addresscount , fl );
		addresscount++;			
		}
		
		for ( int x = 0 ; x < 3; x++)
		{
		int fl = *(int*)&accelcal[x];
		fmc_write(  addresscount , fl );
		addresscount++;			
		}
		writecount++;
		fmc_write(  addresscount , writecount );
		addresscount++;
		fmc_write(  addresscount , options1 );
		addresscount++;	
		fmc_write(  addresscount , options2 );
		//addresscount++;	
	}
	
	
	return;
}

void loadcal(void)
{
	int addresscount = 0;
	
	if ( fmc_read( addresscount ) == 0x00AC )
	{
		addresscount++;
		for ( int x = 0 ; x < 3; x++)
		{
		int result = fmc_read( addresscount );
		gyrocal[x] = *(float*)&result;
		addresscount++;			
		}
		
		for ( int x = 0 ; x < 3; x++)
		{
		int result = fmc_read( addresscount );
		//float fl = *(float*)&result;
		accelcal[x] = *(float*)&result;
		addresscount++;			
		}
		writecount = fmc_read( addresscount );
		if ( writecount == 0xFFFFFFFF )
		{
			writecount = 0;
		}
		addresscount++;
		options1 = fmc_read( addresscount );
		addresscount++;
		options2 = fmc_read( addresscount );
	}
	else
	{
		//load defaults
		writecount = 0;
	}
	
	
}













