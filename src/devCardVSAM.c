/*

   **MEMBER**=xxxLIBS:xxxLIB
   **CVS**=xxx
=============================================================
 
  Abs:  Input Device Support for VME 
        Smart Analog Monitor Module (VSAM)
 
  Name: devCardVSAM.c 
          init    - Init device support
          read    - read device support
  
  Proto: devSup.h
 
  Auth: 02-Aug-2001, K. Luchini       (LUCHINI) 
  Rev : dd-mmm-yyyy, Reviewer's Name  (USERNAME)
 
-------------------------------------------------------------
  Mod:
        dd-mmm-yyyy, First Lastname (USERNAME):
           Comments 
  
=============================================================
*/


/* Include Files */
#include   "vxWorks.h"        /* OK,ERROR,TRUE,FALSE      */
#include   "stdioLib.h"       /* NULL                     */
#include   "types.h"          /* VME_IO                   */
#include   "string.h"         /* strcpy                   */

#include   "dbFldTypes.h"     /* DBF_FLOAT                */
#include   "alarm.h"          /* READ_ALARM,INVALID_ALARM */
#include   "dbScan.h"         /* IOSCANPVT                */
#include   "recSup.h"         /* recGblRecordError        */
#include   "devSup.h"         /* DEVSUPFUN, S_dev_badBus  */
#include   "vmeCardRecord.h"  /* struct vmeCardRecord     */
#include   "VSAM.h"           /* VSAM_get_adrs(),etc      */
#include   "db_proto.h"       /* alarmStatusChk()         */




/* Local Prototypes */
static long init( void *rec_p );
static long read( void *rec_p );


/* 
 * GLOBAL strucuture
 * Device support entry table 
 */
struct {
  long	     number;        
  DEVSUPFUN  report;  
  DEVSUPFUN  init;        
  DEVSUPFUN  init_record;     
  DEVSUPFUN  get_ioint_info; 
  DEVSUPFUN  read;            
}devCardVSAM = { 5,NULL,NULL,init,NULL,read };


 

/*=============================================================
 
  Abs:  Synchronous Device Support Initialization
        for a VME Smart Analog Monitor Module (VSAM)
 
  Name: init 
 
  Args: rec_p                     Record information  
          Use:  struct
          Type: void *
          Acc:  read-write access
          Mech: By reference

  Rem:  This device support routine is called by the record
        support function init_record(). Its purpose it to 
        initializes the VME Card Record.
                             
  Side:  None

  Ret: long 
         OK               - Successful operation 
         S_dev_badBus     - Failure, due to bad bus type
         S_dev_badCard    - Failure, due to card not present
         S_db_badChoice   - Failure, due to FTVL field not set to FLOAT

=============================================================*/
static long init( void *rec_p )
{  
   long                   status=OK;    /* return status         */
   struct vmeCardRecord  *modu_ps;     /* waveformrecord info   */
   struct vmeio          *vmeio_ps;    /* VME info              */  
   static char           *taskName_c = "devModuVSAM( init )\n";
   VSAMMEM               *pVSAM=NULL;

  /*
   * Is this device supported?
   */
   modu_ps = (struct vmeCardRecord *)rec_p;
   switch ( modu_ps->inp.type )
   {
       case VME_IO: 
        /* 
         * Parse the INP parameter string.
         * This string was input by DCT
         * upon record creation.
         */
         vmeio_ps = &modu_ps->inp.value.vmeio; 
         modu_ps->amod = vmeAMOD_A24;
         modu_ps->dsiz = vmeDSIZ_D32;
         status = VSAM_get_adrs( vmeio_ps->card,&pVSAM );
         if ( status==ERROR ) {
            status = S_dev_badCard;
            strcpy(modu_ps->val,"Card Not Found");
            recGblRecordError( status,rec_p,taskName_c );
	 }
         else {
            modu_ps->addr = (unsigned long)pVSAM;
            modu_ps->csiz = sizeof(VSAMMEM);
            modu_ps->dpvt = (void *)pVSAM;
            VSAM_version( vmeio_ps->card,&modu_ps->ver );
            strcpy(modu_ps->val,"Successful");
	 }
	 break;

      default: /* Bus type is not supported */
         status = S_dev_badBus;
         strcpy(modu_ps->val,"Invalid Bus Type");
         break; 
     
   } /* End of switch statment */


   if ( status!=OK ) 
      recGblRecordError( status,rec_p,taskName_c );
   else
      modu_ps->udf = FALSE;  /* Init completed successfully */
   return( status );
}



/*=============================================================
 
  Abs: VME Card input device suppot        
 
  Name: read 
 
  Args: rec_p                   Record Information
          Use:  struct
          Type: void *
          Acc:  read-write access
          Mech: By reference


  Rem: This routine processes the input VME Card record. 
       

  Side: None

  Ret: long 
            OK             - Successful operation 
            INVALID_ALARM - Failure occured during read  
            Otherwise, see return from 

=============================================================*/
static long read( void *rec_p )
{
    long                  status=OK;                /* return status        */ 
    unsigned short        cur_stat= READ_ALARM;     /* alarm status         */
    unsigned short        cur_sev = INVALID_ALARM;  /* alarm severity       */
    struct vmeCardRecord *modu_ps=NULL;             /* record info          */
    static const char    *taskName_c = "devModuVSAM( read )\n";
    VSAMMEM              *pVSAM = NULL;

   /* 
    * If the private device infor has not been
    * filled in then we have a problem and so
    * just exit successfully.  Otherwise, continue.
    */ 
    modu_ps = (struct vmeCardRecord *)rec_p;
    if ( !modu_ps->dpvt ) {
      alarmStatusChk(cur_stat,cur_sev,status,rec_p,taskName_c );
      status = ERROR;
    }
    else {
        pVSAM = (VSAMMEM *)modu_ps->dpvt;
        modu_ps->mstt = pVSAM->status;
    }
    return( status );
}
