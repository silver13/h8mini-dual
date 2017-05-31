

#include "drv_fmc.h"
#include "defines.h"


void loadcal_old(void);
void fmc_write_inc(unsigned int * address, int data);
unsigned int checksum(void);

extern float gyrocal[3];
extern float accelcal[3];

unsigned writecount;
unsigned options1;
unsigned options2;

extern float pidkp_flash[PIDNUMBER];
extern float pidki_flash[PIDNUMBER];
extern float pidkd_flash[PIDNUMBER];

extern float pidkp[PIDNUMBER];
extern float pidki[PIDNUMBER];
extern float pidkd[PIDNUMBER];

float saved_pidkp_flash[PIDNUMBER];
float saved_pidki_flash[PIDNUMBER];
float saved_pidkd_flash[PIDNUMBER];



// identifying string
#define FLASH_HEADER 0x00AE

//int load_debug = 0;
// 0 - no data found
// 1 - normal
// 2 - mismatch
// 3 - checksum error

void savecal()
{

	unsigned int addresscount = 0;
	// load non modified flash values

	if( fmc_erasepage() ) return;

	//write values
      fmc_write(addresscount, FLASH_HEADER);
      addresscount++;
      for (int x = 0; x < 3; x++)
        {
            int val = *(int *)&gyrocal[x];
            fmc_write(addresscount, val);
            addresscount++;
     
            val = *(int *)&accelcal[x];
            fmc_write(addresscount, val);
            addresscount++;
        }
      writecount++;
        
      fmc_write(addresscount, writecount);
      addresscount++;

     fmc_write(addresscount, options1);
      addresscount++;
       
     fmc_write(addresscount, options2);
        addresscount++;
        
        for ( int i = 0 ; i < 3; i++)
        {
            // pid file pids
            fmc_write(addresscount, *(int *)&pidkp_flash[i]);         
            addresscount++;
                      
            fmc_write(addresscount, *(int *)&pidki_flash[i]);           
            addresscount++;
             
            fmc_write(addresscount, *(int *)&pidkd_flash[i]);            
            addresscount++;
            // current pids
            fmc_write(addresscount, *(int *)&pidkp[i]);           
            addresscount++;
          
            fmc_write(addresscount, *(int *)&pidki[i]);           
            addresscount++;
           
            fmc_write(addresscount, *(int *)&pidkd[i]);
            addresscount++;  
         
        }
        
       // checksum       
        fmc_write( 255, checksum() );

}

unsigned int checksum(void)
{
unsigned int sum = 0x12345678;
    
 for ( int i = 0 ; i < 255; i++)
     sum+= fmc_read(i);

return sum;    
}

void copy_flash_pids()
{
    for ( int i = 0 ; i < 3; i++)
    {
       // update saved pids
        saved_pidkp_flash[i] = pidkp_flash[i];
        saved_pidki_flash[i] = pidki_flash[i];
        saved_pidkd_flash[i] = pidkd_flash[i];
       // update current pids
        pidkp[i] = pidkp_flash[i];
        pidki[i] = pidki_flash[i];
        pidkd[i] = pidkd_flash[i];
    } 
//    load_debug = 2;
}

// check if pid sets are identical
void check_pid_set( void)
{
    
    for ( int i = 0 ; i < 3; i++)
       {
           if ( saved_pidkp_flash[i] != pidkp_flash[i] 
               || saved_pidki_flash[i] != pidki_flash[i]
                || saved_pidkd_flash[i] != pidkd_flash[i] ) 
               copy_flash_pids();
       }
  
}


void loadcal(void)
{
// load old data if present    
    loadcal_old();
    
	int addresscount = 0;

	if (fmc_read(addresscount) == FLASH_HEADER)
	  {
		  addresscount++;
		  for (int x = 0; x < 3; x++)
		    {
			    int result = fmc_read(addresscount);
			    gyrocal[x] = *(float *)&result;
			    addresscount++;
		
			    result = fmc_read(addresscount);
			    accelcal[x] = *(float *)&result;
			    addresscount++;
		    }
            
		  writecount = fmc_read(addresscount);

		  addresscount++;
		  options1 = fmc_read(addresscount);
		  addresscount++;
		  options2 = fmc_read(addresscount);
          addresscount++;
            
          // saved flash pids 
		  for (int x = 0; x < 3; x++)
            { 
          // pid.c file pids
			    int result = fmc_read(addresscount);
			    saved_pidkp_flash[x] = *(float *)&result;
			    addresscount++;
              
			    result = fmc_read(addresscount);
			    saved_pidki_flash[x] = *(float *)&result;
			    addresscount++;            
              
			    result = fmc_read(addresscount);
			    saved_pidkd_flash[x] = *(float *)&result;
			    addresscount++;

          // current pids
           
			    result = fmc_read(addresscount);
			    pidkp[x] = *(float *)&result;
			    addresscount++;
              
			     result = fmc_read(addresscount);
			    pidki[x] = *(float *)&result;
			    addresscount++;
             
			    result = fmc_read(addresscount);
			    pidkd[x] = *(float *)&result;
			    addresscount++;
            }
            
            if ( checksum() != fmc_read(255) )
            {
                copy_flash_pids();
//                load_debug = 3;
                return;
            }
            
//            load_debug = 1;
            
            // check if pid.c file pids are still the same, if not update
            check_pid_set();
            
	  }
	else
	  {
		  //load defaults
		  writecount = 0;
          
          // use flash defaults
          copy_flash_pids();
	  }


}

// for now the old style settings will also be loaded
void loadcal_old(void)
{
	int addresscount = 0;

	if (fmc_read(addresscount) == 0x00AC)
	  {
		  addresscount++;
		  for (int x = 0; x < 3; x++)
		    {
			    int result = fmc_read(addresscount);
			    gyrocal[x] = *(float *)&result;
			    addresscount++;
		    }

		  for (int x = 0; x < 3; x++)
		    {
			    int result = fmc_read(addresscount);
			    accelcal[x] = *(float *)&result;
			    addresscount++;
		    }
            
		  writecount = fmc_read(addresscount);

		  addresscount++;
		  options1 = fmc_read(addresscount);
		  addresscount++;
		  options2 = fmc_read(addresscount);
          addresscount++;
   
            
	  }
//	else
	  {
		  //load defaults
		//  writecount = 0;
	  }


}

