/* VSAMUtils.c - VME Smart Analog Monitor test utilities
 *
 *      Author:         Dayle Kotturi
 *      Date:           03-29-02
 */

#include <stdio.h>       /* for printf() */
 
#include "epicsVersion.h"
#include "VSAM.h"
#include "VSAMUtils.h"

int VSAM_testMem (const VSAMMEM * pVSAM) {
    unsigned long *data;
    unsigned long tmpdata;
    int i;
    VSAMSHORTS s;
    VSAMCHARS c;

    for (i=0; i<32; i++) {
        printf ("data[%2d]     Addr = %lu, Value = %f\n", 
            i, (long) &pVSAM->data[i], pVSAM->data[i]);
    }
    data = (unsigned long *) pVSAM->range;
    for (i=0; i<8; i++) {
        tmpdata = *data;
        c.a = (CHARAMASK & tmpdata); 
        c.b = (CHARBMASK & tmpdata) >> 8; 
        c.c = (CHARCMASK & tmpdata) >> 16; 
        c.d = (CHARDMASK & tmpdata) >> 24; 
        /* range is an unsigned char, but use int to print out value */
        printf ("range[%2d]    Addr = %lu, Values = %d, %d, %d, %d\n", 
            i, (long) data, c.a, c.b, c.c, c.d);
        ++data; 
    }
    for (i=0; i<16; i++) {
        tmpdata = *data;
        s.a = (SHORTAMASK & tmpdata); 
        s.b = (SHORTBMASK & tmpdata) >> 16; 
        printf ("ac[%2d]       Addr = %lu, Values = %d, %d\n", 
            i, (long) data, s.a, s.b);
        ++data; 
    }
    printf ("reset        Addr = %lu, Value = %lu\n", 
        (long) &pVSAM->reset, pVSAM->reset);

    printf ("mode_control Addr = %lu, Value = %lu\n", 
        (long) &pVSAM->mode_control, pVSAM->mode_control);

    printf ("status       Addr = %lu, Value = %lu\n", 
        (long) &pVSAM->status, pVSAM->status);

    printf ("pad          Addr = %lu, Value = %lu\n", 
        (long) &pVSAM->pad, pVSAM->pad);

    printf ("diag_mode    Addr = %lu, Value = %lu\n", 
        (long) &pVSAM->diag_mode, pVSAM->diag_mode);

    for (i=0; i<3; i++) {
        printf ("padding[%1d]   Addr = %lu, Value = %lu\n", 
            i, (long) &pVSAM->padding[i], pVSAM->padding[i]);
    }

return OK;
}

int checkStatus (const VSAMMEM * pVSAM ) {
    if (pVSAM->status & CALIB_SUCCESS)
        printf ("Calibration OK\n");
    else
        printf ("Calibration ERROR\n");

    printf ("status        Addr = %lu, Value = %lu, Mask = %d\n", 
        (long) &pVSAM->status, pVSAM->status, CALIB_SUCCESS);
return OK;
}

int VSAM_resetMode (VSAMMEM * pVSAM) {
    pVSAM->mode_control = 0x0;
    printf ("mode_control  Addr = %lu, Value = %lu\n", 
        (long) pVSAM->mode_control, pVSAM->mode_control);
    printf ("status        Addr = %lu, Value = %lu\n", 
        (long) &pVSAM->status, pVSAM->status);
return OK;
}

int VSAM_setModeMask(VSAMMEM * pVSAM) {
    pVSAM->mode_control = MODE_MASK;
    printf ("mode_control  Addr = %lu, Value = %lu\n", 
        (long) &pVSAM->mode_control, pVSAM->mode_control);
    printf ("status        Addr = %lu, Value = %lu\n", 
        (long) &pVSAM->status, pVSAM->status);
return OK;
} 

int VSAM_setLittleEndian(VSAMMEM * pVSAM) {
    pVSAM->mode_control |= 0x4;
    printf ("mode_control  Addr = %lu, Value = %lu\n", 
        (long) &pVSAM->mode_control, pVSAM->mode_control);
    printf ("status        Addr = %lu, Value = %lu\n", 
        (long) &pVSAM->status, pVSAM->status);
return OK;
}

int VSAM_setBigEndian(VSAMMEM * pVSAM) {
    pVSAM->mode_control &= ~0x4;
    printf ("mode_control  Addr = %lu, Value = %lu\n", 
        (long) &pVSAM->mode_control, pVSAM->mode_control);
    printf ("status        Addr = %lu, Value = %lu\n",
        (long) &pVSAM->status, pVSAM->status);
return OK;
}

int VSAM_setFastScan(VSAMMEM * pVSAM) {
    pVSAM->mode_control |= 0x1;
    printf ("mode_control  Addr = %lu, Value = %lu\n", 
        (long) &pVSAM->mode_control, pVSAM->mode_control);
    printf ("status        Addr = %lu, Value = %lu\n",
        (long) &pVSAM->status, pVSAM->status);
return OK;
}

int VSAM_setNormalScan(VSAMMEM * pVSAM) {
    pVSAM->mode_control &= ~0x1;
    printf ("mode_control  Addr = %lu, Value = %lu\n", 
        (long) &pVSAM->mode_control, pVSAM->mode_control);
    printf ("status        Addr = %lu, Value = %lu\n",
        (long) &pVSAM->status, pVSAM->status);
return OK;
}     

int VSAM_setFirmwareRev(VSAMMEM * pVSAM) {
    pVSAM->mode_control |= 0x2;
    printf ("mode_control  Addr = %lu, Value = %lu\n", 
        (long) &pVSAM->mode_control, pVSAM->mode_control);
    printf ("status        Addr = %lu, Value = %lu\n",
        (long) &pVSAM->status, pVSAM->status);
return OK;
}     

int VSAM_setAnalogChData(VSAMMEM * pVSAM) {
    pVSAM->mode_control &= ~0x2;     
    printf ("mode_control  Addr = %lu, Value = %lu\n", 
        (long) &pVSAM->mode_control, pVSAM->mode_control);
    printf ("status        Addr = %lu, Value = %lu\n",
        (long) &pVSAM->status, pVSAM->status);
return OK;
}     

