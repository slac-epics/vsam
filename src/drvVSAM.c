/* drvVSAM.c - Driver Support Routines for VME Smart Analog Monitor
 *
 *	Author:		Susanna Jacobson
 *	Date:		10-24-97
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>         /* for modf() */

#ifdef __rtems__
#include <rtems.h>
#include <rtems/error.h>
#include <bsp.h>
#include <syslog.h>

#if defined(__PPC__)
#include <bsp/VME.h>
#include <bsp/bspExt.h>
#endif

#else
#include <vxWorks.h>
#include <tickLib.h>
#include <logLib.h>
#include <vxLib.h>
#include <sysLib.h>         /* for sysBusToLocalAdrs */
#include <taskLib.h>
#include <memLib.h>
#include <vme.h>
#include <time.h>           /* for CLOCKS_PER_SEC    */
#endif

#include	"dbDefs.h"
#include	"dbScan.h"
#include	"drvSup.h"
#include        "errMdef.h"        /* errMessage()         */
#include        "devLib.h"         /* devRegisterAddress() */

#include	"VSAM.h"           /* VSAM_NUM_CHANS,etc   */
#include        "VSAMUtils.h"      /* for VSAM_testMem()   */
#include        "epicsExport.h"
#include        "iocsh.h"

/* Messages - informational and error */
static char *noCard_c    = "VSAM Card %hd not found at (A24) address 0x%8.8lx\n";
static char *cardFound_c = "VSAM Card %hd found at (A24) address 0x%8.8lx\n";

#ifdef DEBUG
static char *cardNotFound_c   = "verifyVSAM: card %hd not found\n";
static char *chanOutOfRange   = "verifyVSAM: chan limit %d but chan %d\n";
static char *invParam_c       = "verifyVSAM: unknown param char %c\n";
static char *invRange_c       = "translateVSAMChannel: can't allocate range struct\n";
#endif


/* Local variables */
/* array for keeping track of cards found at initialization */
static VSAMMEM	       	**p_VSAM = 0;
static VSAMCNFG           CNFG_as[VSAM_MAX_CARDS];

static short ai_cards_found  = 0;
static short ai_num_cards    = VSAM_MAX_CARDS;


/* local function prototypes */
static long init();
static long report(int level);
static int VSAM_clear( VSAMMEM *pVSAM );


/* Global variables        */
/* VSAM driver entry table */

struct {
	long		number;
	DRVSUPFUN	report;
	DRVSUPFUN	init;
} drvVSAM={
	2,
	report,
	init};



/* Driver initialization routines */

static long init()
{
    int              status=OK;
    VSAMMEM         *pVSAM = NULL;          /* local VME address */
    size_t           len = sizeof(VSAMMEM);
    short            i_card;
    char             name_c[20];
    epicsAddressType type=atVMEA24;
    volatile void *pPhysicalAddress = NULL;


    /* Register all VSAM's found eariler */
     if ( !p_VSAM || !ai_cards_found ) {
       #ifdef DEBUG
          printf("No VSAM cards found\n");
       #endif
       return(status);
     }

    for ( i_card=0; i_card<VSAM_MAX_CARDS; i_card++ ) {  
       if ( CNFG_as[i_card].present ) {  
          sprintf(name_c,"VSAM-%.2hd",i_card );
          pPhysicalAddress = (volatile void *)pVSAM; 
          status = devRegisterAddress( (const char *)name_c,type,
                                       CNFG_as[i_card].bus_addr,
                                       len,&pPhysicalAddress );
          if (status==OK) {
             CNFG_as[i_card].registered = TRUE;
             #ifdef DEBUG
               printf( "DRVSUP init VSAM card %1d found", i_card );
             #endif
          }
          else {
            ai_cards_found--;
            p_VSAM[i_card] = 0;
            CNFG_as[i_card].pVSAM = 0;
	  }
       }/* End of if */    
    } /* End of i_card FOR loop */  
    return( OK );
}

/*
 * VSAM_init - set all VSAM's to normal (slow) scan, analog channel data.
 * This function should be called from a vxWorks
 * script prior to invoking iocInit. 
 */
int VSAM_init( long *base_addr,short num_cards, short first )
{
     int           status=OK;
     size_t        bcnt;
     unsigned long ioBase;
     int           nelm;
#ifdef __rtems__
     volatile unsigned long  *pCpuAddr;
     volatile unsigned long  **ppPhysicalAddr;
     epicsAddressType type = atVMEA24;              /* A24/D32 address space */
#else
     unsigned int     type = VME_AM_STD_SUP_DATA;   /* A24/D32 address space */
#endif
     short         i_card;
     short         n_cards = first + num_cards;
     VSAMMEM     **ppcards_present = NULL;
     VSAMMEM      *pVSAM = NULL;  

    /* 
     * Allocal memory to store information about
     * all VSAM modules found in this ioc.
     */
     if ( !p_VSAM ) {
       if ( num_cards>0 ) { 
         ai_num_cards = min(num_cards,VSAM_MAX_CARDS);
       }

       /* Allocate array of pointers for module base address */
       bcnt   = sizeof(*p_VSAM);
       nelm   = ai_num_cards;
       ppcards_present = (VSAMMEM **) calloc(nelm,bcnt);
       if ( !ppcards_present ) return( ERROR );
       p_VSAM = ppcards_present; 
     }
     else if ( num_cards+first > ai_num_cards ) {
       return(ERROR);
     }
     else {
       ppcards_present = p_VSAM;
     }

     /* map address of the VSAM card into the standard address space */
#ifdef __rtems__
      pCpuAddr= (volatile unsigned long *)pVSAM;
      ppPhysicalAddr = &pCpuAddr;
      status = BSP_vme2local_adrs(type,(size_t)base_addr,(unsigned long *)ppPhysicalAddr);
      if (status) {
        status =  S_dev_addrMapFail;
    	printf( "VSAM_init: A24 address 0x%08lx is invalid",(long unsigned int)base_addr);
   	return( status );
     }
#else
     status = sysBusToLocalAdrs(type,(char *)base_addr,(char **)&pVSAM);
     if(status != OK){
        logMsg( "VSAM_init: A24 address 0x%08lx is invalid",
              (long unsigned int)base_addr,0,0,0,0,0 );
        return( status );
     }
#endif

     /* mark each VSAM found into the card present array */
     for (i_card=first,status=OK; i_card<n_cards; i_card++, ppcards_present++) {
       if ( *ppcards_present != 0 )
         printf("VSAM_init: Card %hd already initialized",i_card);
       else {
         ioBase = (unsigned long)base_addr + (sizeof(VSAMMEM) * i_card);
        *ppcards_present          = pVSAM;  /* address of VSAM card found  */
         CNFG_as[i_card].pVSAM    = pVSAM;
         CNFG_as[i_card].bus_addr = ioBase;
         CNFG_as[i_card].present  = TRUE;
         ai_cards_found++;                  /* keep track of cards found */
       }
     }/* End if i_card FOR loop */

     return( status );
}


/*
 * VSAM_present - probe VSAM memory location to 
 * verify if card is in crate. 
 */
int VSAM_present( short card,VSAMMEM *pVSAM )
{
     int	     status;
     int             found = FALSE;
     long            lval;
#ifdef __rtems__
     double          nsec  = 0.2;
     unsigned        wcnt  = sizeof(lval)>>1;
#else
     int             bcnt  = sizeof(lval);
     int             nticks;
     int             mode  = VX_READ;
#endif
     short           chan;
     VSAMCNFG       *pCNFG = &CNFG_as[card];
     int             attempts=0;

     /*   status = devReadProbe(wcnt,(void*)pVSAM,(void*)&lval); */
#ifdef __rtems__
     status = bspExtMemProbe ((void*)pVSAM, 0, wcnt,(void *)&lval); 
     if (status!=RTEMS_SUCCESSFUL) {
        return S_dev_noDevice;
        printf(noCard_c,(int)card,(int)pVSAM);
     }
     else {
        status = OK;
        printf(cardFound_c,card,(unsigned long)pVSAM);
#else
     status = vxMemProbe((char *)pVSAM,mode,bcnt,(char *)&lval)
     if ( status !=OK ) {
        logMsg(noCard_c,(int)card,(int)pVSAM,0,0,0,0);
     }
     else {
        logMsg(cardFound_c,(int)card,(int)pVSAM,0,0,0,0);
#endif
       /* Before doing anything, ensure that data and registers 
          are initially zero, because we have seen the case where
          garbage is present in vsam records after a reboot. 
          If a '1' happens to be in the "little endian" bit, we
          know that this will overwrite the CALIB SUCCESS state in
          the status register for as long as the "little endian" bit
          is held high, regardless of the processing of the CALIBRATION
          bit at 1 Hz.                         dayle 29mar2002 */

        #ifdef DEBUG
            printf("Before the clear");
            VSAM_testMem( pVSAM );
        #endif

        VSAM_clear( pVSAM );

        #ifdef DEBUG
            printf("After the clear");
            VSAM_testMem( pVSAM );
        #endif

       /* 
        * Set mode to read revision. Allow time for
        * the revision to appear in memory after
        * the mode has been set. Remember that
	* the version number is returned in all
	* channels in floating point format. 
	* NOTE:D1 must be cleared to resume
	* analog processing
        *
        */ 
   	pVSAM->mode_control |= SET_FIRMWARE; /* dayle changed this from '=' to 
                                                "|=" to not wipe out anything 
                                                already set. 03/28/02 */
        /* Luchini, changed from 1 tick to 10 on 05/09/01 */
#ifdef __rtems__
        status = rtems_task_wake_after(nsec);
        if(status != RTEMS_SUCCESSFUL){
           printf("rtems_task_wake_after %s\n", rtems_status_text (status));
	}
#else
        nticks = CLOCKS_PER_SEC/10;
        nticks = max(1,nticks);
        taskDelay(nticks);  /* Luchini, changed from 1 tick to 10 on 05/09/01 */
#endif
        for ( chan=0; chan<VSAM_NUM_CHANS; chan++ ) { 
          pCNFG->fw_version[chan] = pVSAM->data[chan];
	}
        found = TRUE; 

       /* 
	* Reset the MODE CONTROL register to 
        * normal scan, analog data and big-endian mode.
        */
   	pVSAM->mode_control = 0;

       /* Dayle added check that it's calibrated and if not, wait for it
          on 03/19/02. 
          In all my testing, CALIB SUCCESS was ALWAYS true at this point
          so this code could be removed. dayle 03/29/02*/
        #ifdef DEBUG
           printf("pVSAM->status is %d",(pVSAM->status));
        #endif
        if (pVSAM->status & CALIB_SUCCESS){
           /* then the calibration bit is set, so fine */ 
           printf("Calibration bit is set. Proceed as normal."); 
        }
        else {
           /* then the calibration bit is not set, so retry */
           attempts = 0;

#ifdef __rtems__
           nsec = 1.0;
#else
           nticks=60;
#endif
           while (!(pVSAM->status & CALIB_SUCCESS) || (attempts<10)) {
#ifdef __rtems__
               status = rtems_task_wake_after (nsec);
#else
               taskDelay(nticks);
#endif
               attempts++;
           }/* End of while statement */ 
           if (attempts==10) {
               printf("Waited 10 seconds for calibration to complete. Gave up.");
           }
           else {
               printf("Detected calibration successful after %2d seconds",attempts);
           }
       }/* end of if */ 
    }

    return( found );
}


/* Zero all data values iand registers upon initialization */ 
int VSAM_clear( VSAMMEM *pVSAM )
{
    int i;
    unsigned long *ptr;

    for (i=0; i<32; i++) {
        pVSAM->data[i] = 0.0;
    }

    ptr = (unsigned long *) pVSAM->range;
    /* There are 32 range values of type char, but they are accessed
       via A24/D32 address space */
    for (i=0; i<8; i++) {
        *ptr = 0;  
        ptr++;
    }

    ptr = (unsigned long *) pVSAM->ac;
    /* There are 32 ac values of type short, but they are accessed
       via A24/D32 address space */
    for (i=0; i<16; i++) {
        *ptr = 0; 
        ptr++;
    }
    pVSAM->reset = 0;
    pVSAM->mode_control = 0;
    /* Note: can't zero status register because it's read-only */
    pVSAM->pad = 0;
    pVSAM->diag_mode = 0;
    for (i=0; i<3; i++) {
        pVSAM->padding[i] = 0;
    } 
    return OK;
}


/* Routines called by init_record() in device support */

/* 
 * verifyVSAM - check validity of card, channel, and other parameters.
 */
int verifyVSAM( short  card,short channel,char   parm)
{
	VSAMMEM		*pVSAM;

    if ((card >= ai_num_cards) || (card < 0)) {
#ifdef DEBUG
	printf(cardNotFound_c,card);
#endif
	return(-1);
    }
    /* verify that specified card is present */
    pVSAM = p_VSAM[card];
    if (pVSAM == 0) {
#ifdef DEBUG
	printf(cardNotFound_c,card);
#endif
	return(1);
    }
    if ((channel >= VSAM_NUM_CHANS+4) || (channel < 0)) {
#ifdef DEBUG
	printf(chanOutOfRange,VSAM_NUM_CHANS,channel);
#endif
	return(-2);
    }
    if ((channel < VSAM_NUM_CHANS) &&
	(parm != RANGE_TYPE)        && 
        (parm != DATA_TYPE)         && 
        (parm != AC_TYPE))
    {
#ifdef DEBUG
	printf(invParam_c,parm);
#endif
	return(-2);
    }

    return(0);
}

/*
 * checkVSAMAi - check that channel specifies a valid analog input.
 *
 * To be called after verifyVSAM() has returned 0.
 *
 * Only the hardware channels (0-31) can be read as floats.
 */
int checkVSAMAi(short channel)
{ 
   if (channel < VSAM_NUM_CHANS) return(0);
   return(-2);
}

/*
 * checkVSAMBi - check that channel and parameter specify a valid binary input.
 *
 * To be called after verifyVSAM() has returned 0.
 *
 * Only the Status Register can be read as binary inputs.
 */
int checkVSAMBi( short channel,char parm )
{
   if (channel != STATUS_CHANNEL) return(-2);
   return(0);
}

/*
 * checkVSAMBo - check that channel and parameter specify a valid binary output.
 *
 * To be called after verifyVSAM() has returned 0.
 *
 * Only the following can be written to:
 *	Reset Register, Mode Control Register, and Test Mode Register.
 */
int checkVSAMBo( short	channel)
{
  if ((channel < VSAM_NUM_CHANS) || 
      (channel == STATUS_CHANNEL))
    return(-2);
  return(0);
}

/*
 * bo_VSAM_read - get initial values for binary outputs and mbbo's.
 */
int bo_VSAM_read( short	        card,
                  short         channel,
                  unsigned long	mask,
                  unsigned long	*pval )
{
	VSAMMEM		*pVSAM;

    if (channel == MODE_CHANNEL) {
	pVSAM = p_VSAM[card];
	if (pVSAM == 0) return -1;
	*pval = pVSAM->status & mask;
    }
    else {
	*pval = 0;
    }
    return(0);
}

/*
 * getVSAMBitMask - get single bit mask from bit specification.
 *
 * Bit specification is character for bit number: '0', '1', etc.
 * This is only relevant for the Mode Control and Status Registers.
 */
int getVSAMBitMask( short          channel,
                    char           spec,
                    unsigned long *pmask)
{
    int	shift,max_shift;

    if (channel == MODE_CHANNEL) max_shift = 2;
    else if (channel == STATUS_CHANNEL) max_shift = 3;
    else return(0);

    shift = spec - '0';
    if ((shift < 0) || (shift > max_shift)) return(-2);
    *pmask = 1 << shift;
    return(0);
}

/*
 * translateVSAMChannel - get address of 4-byte location containing ch's data.
 *
 * VSAM is D32 only, so byte and two-byte items must be extracted from longs.
 */
int translateVSAMChannel(short    channel,
                         char	  spec,
                         VSAMPVT *ppvt)
{
    short     rem;
    VSAMPVT  *rpvt = NULL;

    switch ((int)spec) {
	case RANGE_TYPE:
	    rem = channel%4;
	    ppvt->lchan = channel-rem;
	    rem = rem*8;
	    ppvt->mask = 0xff << rem;
	    ppvt->shift = rem;
	    break;

	case AC_TYPE:
	    rpvt = malloc(sizeof(VSAMPVT));
	    if (rpvt == NULL) {
#ifdef DEBUG
	printf( invRange_c );
#endif
		return -2;
	    }
	    translateVSAMChannel(channel, RANGE_TYPE, rpvt);
	    ppvt->prange = rpvt;
	    rem = channel%2;
	    ppvt->lchan = channel-rem;
	    rem = rem*16;
	    ppvt->mask = 0xffff << rem;
	    ppvt->shift = rem;
	    break;

	default:
	    ppvt->lchan = channel;
	    ppvt->mask = 0xffffffff;
	    ppvt->shift = 0;
	    break;
    }
    return(0);
}


/* Input and Output routines */

/*
 * ai_VSAM_read - Read floating point value:
 *			analog data or firmware revision number,
 *			value represented by range byte,
 *			or value represented by AC measurement (short).
 */
int ai_VSAM_read( short	    card,
                  short	    channel,
                  char      type,
                  VSAMPVT  *ppvt,
                  float	   *prval)
{
    unsigned long    rlong;
    short	     rshort;
    float	     rfloat;
    double	     dfactor,
	             dpp;
    VSAMMEM         *pVSAM;


    pVSAM = p_VSAM[card];
    if (pVSAM == 0) return -1;

    /* VSAM is D32 only, so bytes and shorts must be extracted here */
    switch ((int)type) {
	case RANGE_TYPE:
	    return(getVSAMRange(pVSAM, ppvt, prval));
	    break;

	case AC_TYPE:
   	    /* AC peak-to-peak voltage is ranges[range]*ac/(2**14)     */
	    /* no AC info unless normal scan and analog data requested */
	    if (pVSAM->status & (FAST_SCAN_MODE|FIRMWARE_REV)) return(-1);

	    /* first get range... */
	    if (getVSAMRange(pVSAM, ppvt->prange, &rfloat) != 0) return(-1);

	    dfactor = (double)rfloat/(double)AC_DIVISOR;

	    /* now get AC measurement */
	    rlong  = *(unsigned long *)&pVSAM->ac[ppvt->lchan];
	    rshort = (short)((rlong & ppvt->mask) >> ppvt->shift);
	    dpp    = dfactor * (double)rshort;
	    *prval = (float)dpp;
	    break;

	default:
	    rfloat = pVSAM->data[channel];
	    *prval = rfloat;
	    break;
    }
    return(0);
}

/*
 * getVSAMRange - derive floating-point range value for channel
 */
int getVSAMRange( VSAMMEM  *pMem,
                  VSAMPVT  *ppvt,
                  float	   *pval)
{
    int                 status = 0;
    unsigned long	rlong,i_range;
    static const float  ranges[] = { 10.24, 5.12, 2.56, 1.28, 0.64, 
                                     0.32,  0.16, 0.08, 0.04, 0.02, 
                                     0.01 };

    rlong = *(unsigned long *)&pMem->range[ppvt->lchan];
    i_range = (char)((rlong & ppvt->mask) >> ppvt->shift);
    if ( i_range>MAX_RANGE_BYTE ) 
      status = -1;
    else
      *pval = ranges[ i_range ];
    return( status );
}

/*
 * input_VSAM_driver - return single- or multi-bit binary value.
 */
int input_VSAM_driver( short           card,
                       short           lchan,
                       char            type,
                       unsigned long   mask,
                       unsigned long  *pval)
{
    unsigned long	lval = 0;
    VSAMMEM		*pVSAM;

    pVSAM = p_VSAM[card];
    if (pVSAM == 0) return -1;

    if (lchan < VSAM_NUM_CHANS) {
	if (type == RANGE_TYPE) {
	    lval = *(unsigned long *)&pVSAM->range[lchan];
	}
	else if (type == AC_TYPE) {
	    lval = *(unsigned long *)&pVSAM->ac[lchan];
	}
	else {
	   return(-2);
	}
    }
    else {
	lval = pVSAM->status;
    }
    *pval = lval & mask;
    return(0);
}

/*
 * output_VSAM_driver - reset VSAM, set test mode, or write to mode control reg.
 */
int output_VSAM_driver( short		card,
                        short		channel,
                        unsigned long	mask,
                        unsigned long	*pval)
{
	unsigned long	sval,
			lval,
			rval;
	VSAMMEM		*pVSAM;

    pVSAM = p_VSAM[card];
    if (pVSAM == 0) return -1;

    switch ((int)channel) {
	case RESET_CHANNEL:
	    pVSAM->reset = 0;
	    break;
	case DIAG_CHANNEL:
	    pVSAM->diag_mode = 0;
	    break;
	default:
	    /* Only three bits of mode control register are used */
	    sval = pVSAM->status & MODE_MASK;
	    rval = *pval;
	    if (mask == MODE_MASK) lval = rval & mask;	/* multi-bit output */
	    else {
		if (rval & mask) lval = sval | mask;	/* set single bit */
		else lval = sval & ~mask;		/* clear single bit */
	    }
	    pVSAM->mode_control = lval;
	    break;
    }
    return(0);
}

/* Driver report routines */

static long report(int	level)
{
    return(VSAM_io_report(level));
}

/*
 * VSAM_io_report - report VSAM's present.  For level 1 report,
 *			print raw data for each channel.
 */
long VSAM_io_report( char level )
{
    short	i;
    VSAMMEM     *pMem = NULL;


    if ( !ai_num_cards ) {
        printf("No VSAM Modules present\n");
        return(OK);
    }
    for (i = 0; i < ai_num_cards; i++) {
      if (p_VSAM[i]) {
          if (level == 0 )
	      printf("VSAM:\tcard %hd\tA24: 0x%06lx\n", i, (unsigned long)p_VSAM[i]);
	  else if (level == 1) 
	      VSAM_rval_report(i,0);
	  else if (level==2)
              VSAM_rval_report(i,1);
          else {
              pMem = p_VSAM[i];
	      printf("VSAM:\tcard %hd\tA24: 0x%06lx\t status: 0x%08lx\n", 
                     i, (unsigned long)pMem, (unsigned long)pMem->status);
	  }
      }
    }/* End of FOR loop */
    return OK;
}

/*
 * VSAM_rval_report - report raw analog data, range, and AC measurement
 *			for each channel.
 * 
 * called by VSAM_io_report() if level is 1
 */
void VSAM_rval_report( short int card, short int flag )
{
    short	    i;
    double          version_base=0.0;
    double          version_frac=0.0;
    VSAMMEM	    *pMem  = p_VSAM[card];
    VSAMCNFG        *pCnfg = &CNFG_as[card];


    printf("STATUS reg: 0x%08lx\n",(unsigned long)pMem->status);
    for (i=0; i<VSAM_NUM_CHANS; i++) {
        if ( flag ) {
           version_frac  = modf((double)pCnfg->fw_version[i],&version_base); 
	   printf("\tch %2hd: data %e\t firmware ver: %d\n", 
               i, pMem->data[i],(int)version_base);
	}
        else
	   printf("\tch %2hd: data %e\n",i, pMem->data[i]);
    }
}


/*
 * VSAM_getAdrs - return the VSAM card base address
 */
int VSAM_get_adrs( short card,VSAMMEM **ppVSAM )
{
    int status = ERROR;
    if ((card >= ai_num_cards) || (card < 0)) return(status);

    /* verify that specified card is present */
    if ( p_VSAM[card] ) 
    {
       if ( !ppVSAM )
         printf("VSAM Card %hd A24 Address 0x%8.8lx\n",
                card,(unsigned long)p_VSAM[card]);
       else 
       {
         *ppVSAM = p_VSAM[card];
         status = OK;
       }
    }
    return(status);
}

/*
 * VSAM_getAdrs - return the VSAM card firmware veresion
 * 
 *  This function obtains the firmware version number from
 *  a local structure which is populated during the VSAM
 *  initialzation. Therefire, this function should only be 
 *  called after the initialization function has been envoked.
 */
int VSAM_version( short card,unsigned short *pversion )
{
    int status = ERROR;

    if ((card >= ai_num_cards) || (card < 0))  return(status);

     /* verify that specified card is present */
    if ( CNFG_as[card].registered ) 
    {
      if ( !pversion ) 
         printf("VSAM Card %hd firmware version %f\n",
                card,CNFG_as[card].fw_version[0]);
      else 
      {
        *pversion = (unsigned short)CNFG_as[card].fw_version[0];
         status = OK;
      }
    }
    return(status);
}
