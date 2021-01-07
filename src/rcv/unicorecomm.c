/*------------------------------------------------------------------------------
* unicorecomm.c : Unicorecomm high precision GNSS receiver functions
*
*          Copyright (C) 2017-2019 by T.TAKASU, All rights reserved.
*          Copyright (C) 2020 by Yuneec, All rights reserved.
*
* reference :
*     [1] Unicorecomm Inc., Reference commands Manual for High Precision GNSS
*         Board and Module, Version V1.0
*
* version : $Revision:$ $Date:$
* history : 2020/12/31 1.0  new
*-----------------------------------------------------------------------------*/
#include "rtklib.h"

#define UNICORESYNC1 0xAA        /* unicore message start sync code 1 */
#define UNICORESYNC2 0x44        /* unicore message start sync code 2 */
#define UNICORESYNC3 0x12        /* unicore message start sync code 3 */

#define UNICOREHLEN  28          /* unicore message header length (bytes) */

#define ID_RANGE    43          /* message id: unicore range measurement */
#define ID_RANGECMP 140         /* message id: unicore range compressed */
#define ID_GPSEPHEM 7           /* message id: unicore gps ephemeris */
#define ID_GLOEPHEMERIS 723     /* message id: unicore glonass ephemeris */
#define ID_GALEPHEMERIS 1122    /* message id: unicore galileo ephemeris */
#define ID_BD2EPHEM 1047        /* message id: unicore beidou ephemeris BX305 */
#define ID_EVENTALL 308         /* message id: unicore eventall position & time information */

#define MAXVAL      8388608.0

#define OFF_FRQNO   -7          /* offset of glonass freq number */

/* get fields (little-endian) ------------------------------------------------*/
#define U1(p) (*((unsigned char *)(p)))
static unsigned short U2(unsigned char *p) {unsigned short u; memcpy(&u,p,2); return u;}
static unsigned int   U4(unsigned char *p) {unsigned int   u; memcpy(&u,p,4); return u;}
static int            I4(unsigned char *p) {int            i; memcpy(&i,p,4); return i;}
static float          R4(unsigned char *p) {float          r; memcpy(&r,p,4); return r;}
static double         R8(unsigned char *p) {double         r; memcpy(&r,p,8); return r;}

/* extend sign ---------------------------------------------------------------*/
static int exsign(unsigned int v, int bits)
{
    return (int)(v&(1<<(bits-1))?v|(~0u<<bits):v);
}
/* adjust weekly rollover of gps time ----------------------------------------*/
static gtime_t adjweek(gtime_t time, double tow)
{
    double tow_p;
    int week;
    tow_p=time2gpst(time,&week);
    if      (tow<tow_p-302400.0) tow+=604800.0;
    else if (tow>tow_p+302400.0) tow-=604800.0;
    return gpst2time(week,tow);
}
/* get observation data index ------------------------------------------------*/
static int obsindex(obs_t *obs, gtime_t time, int sat)
{
    int i,j;
    
    if (obs->n>=MAXOBS) return -1;
    for (i=0;i<obs->n;i++) {
        if (obs->data[i].sat==sat) return i;
    }
    obs->data[i].time=time;
    obs->data[i].sat=sat;
    for (j=0;j<NFREQ+NEXOBS;j++) {
        obs->data[i].L[j]=obs->data[i].P[j]=0.0;
        obs->data[i].D[j]=0.0;
        obs->data[i].SNR[j]=obs->data[i].LLI[j]=0;
        obs->data[i].code[j]=CODE_NONE;
    }
    obs->n++;
    return i;
}

/* decode unicore tracking status -----------------------------------------------
* deocode unicore tracking status
* args   : unsigned int stat I  tracking status field
*          int    *sys   O      system (SYS_???)
*          int    *code  O      signal code (CODE_L??)
*          int    *track O      tracking state
*                         0=idle                      7=freq-lock loop
*                         2=wide freq band pull-in    9=channel alignment
*                         3=narrow freq band pull-in 10=code search
*                         4=phase lock loop          11=aided phase lock loop
*          int    *plock O      phase-lock flag   (0=not locked, 1=locked)
*          int    *clock O      code-lock flag    (0=not locked, 1=locked)
*          int    *parity O     parity known flag (0=not known,  1=known)
*          int    *halfc O      phase measurement (0=half-cycle not added,
*                                                  1=added)
* return : signal frequency (0:L1,1:L2,2:L5,3:L6,4:L7,5:L8,-1:error)
*-----------------------------------------------------------------------------*/
static int decode_trackstat(unsigned int stat, int *sys, int *code, int *track,
                            int *plock, int *clock, int *parity, int *halfc)
{
    int satsys,sigtype,freq=0;
    
    *track =stat&0x1F;
    *plock =(stat>>10)&1;
    *parity=(stat>>11)&1;
    *clock =(stat>>12)&1;
    satsys =(stat>>16)&7;
    *halfc =(stat>>28)&1;
    sigtype=(stat>>21)&0x1F;

    switch (satsys) {
        case 0: *sys=SYS_GPS; break;
        case 1: *sys=SYS_GLO; break;
        case 2: *sys=SYS_SBS; break;
        case 3: *sys=SYS_GAL; break;
        case 4: *sys=SYS_CMP; break;
        case 5: *sys=SYS_QZS; break;
        default:
            trace(2,"unicore unknown system: sys=%d\n",satsys);
            return -1;
    }
    if (*sys==SYS_GPS) {
        switch (sigtype) {
            case  0: freq=0; *code=CODE_L1C; break; /* L1C/A */
            case  9: freq=1; *code=CODE_L2W; break; /* L2Pcodeless */
            default: freq=-1; break;
        }
    }
    else if (*sys==SYS_QZS) {
        switch (sigtype) {
            case  0: freq=0; *code=CODE_L1C; break; /* L1C/A */
            case  9: freq=1; *code=CODE_L2C; break; /* L2C/A */
            default: freq=-1; break;
        }
    }
    else if (*sys==SYS_GLO) {
        switch (sigtype) {
            case  0: freq=0; *code=CODE_L1C; break; /* L1C/A */
            case  5: freq=1; *code=CODE_L2C; break; /* L2C/A */
            default: freq=-1; break;
        }
    }
    else if (*sys==SYS_GAL) {
        switch (sigtype) {
            case  1: freq=0; *code=CODE_L1B; break; /* E1B */
            case  2: freq=0; *code=CODE_L1C; break; /* E1C */
            case 17: freq=1; *code=CODE_L7Q; break; /* E5bQ */
            default: freq=-1; break;
        }
    }
    else if (*sys==SYS_CMP) {
        switch (sigtype) {
            case  0: freq=0; *code=CODE_L1I; break; /* B1I */
            case 17: freq=1; *code=CODE_L7I; break; /* B2I */
            default: freq=-1; break;
        }
    }
    else if (*sys==SYS_SBS) {
        switch (sigtype) {
            case  0: freq=0; *code=CODE_L1C; break; /* L1C/A */
            case  6: freq=2; *code=CODE_L5I; break; /* L5I */
            default: freq=-1; break;
        }
    }
    if (freq<0) {
        trace(2,"unicore signal type error: sys=%d sigtype=%d\n",*sys,sigtype);
        return -1;
    }
    return freq;
}
/* check code priority and return obs position -------------------------------*/
static int checkpri(const char *opt, int sys, int code, int freq)
{
    int nex=NEXOBS; /* number of extended obs data */
    
    if (sys==SYS_GPS) {
        if (strstr(opt,"-GL1P")&&freq==0) return code==CODE_L1P?0:-1;
        if (strstr(opt,"-GL2X")&&freq==1) return code==CODE_L2X?1:-1;
        if (code==CODE_L1P) return nex<1?-1:NFREQ;
        if (code==CODE_L2X) return nex<2?-1:NFREQ+1;
    }
    else if (sys==SYS_GLO) {
        if (strstr(opt,"-RL2C")&&freq==1) return code==CODE_L2C?1:-1;
        if (code==CODE_L2C) return nex<1?-1:NFREQ;
    }
    else if (sys==SYS_GAL) {
        if (strstr(opt,"-EL1B")&&freq==0) return code==CODE_L1B?0:-1;
        if (code==CODE_L1B) return nex<1?-1:NFREQ;
        if (code==CODE_L8Q) return nex<3?-1:NFREQ+2;
    }
    return freq<NFREQ?freq:-1;
}
/* decode rangeb -------------------------------------------------------------*/
static int decode_rangeb(raw_t *raw)
{
    double psr,adr,dop,snr,lockt,tt;
    char *msg;
    int i,index,nobs,prn,sat,sys,code,freq,pos;
    int track,plock,clock,parity,halfc,lli,gfrq;
    unsigned char *p=raw->buff+UNICOREHLEN;
    
    trace(3,"decode_rangeb: len=%d\n",raw->len);
    
    nobs=U4(p);
    
    if (raw->outtype) {
        msg=raw->msgtype+strlen(raw->msgtype);
        sprintf(msg," nobs=%2d",nobs);
    }
    if (raw->len<UNICOREHLEN+4+nobs*44) {
        trace(2,"unicore rangeb length error: len=%d nobs=%d\n",raw->len,nobs);
        return -1;
    }
    for (i=0,p+=4;i<nobs;i++,p+=44) {
        
        /* decode tracking status */
        if ((freq=decode_trackstat(U4(p+40),&sys,&code,&track,&plock,&clock,
                                   &parity,&halfc))<0) continue;
        
        /* obs position */
        if ((pos=checkpri(raw->opt,sys,code,freq))<0) continue;
        
        prn=U2(p);
        if      (sys==SYS_GLO) prn-=37;
        
        if (!(sat=satno(sys,prn))) {
            trace(3,"unicore rangeb satellite number error: sys=%d,prn=%d\n",sys,prn);
            continue;
        }
        if (sys==SYS_GLO&&!parity) continue; /* invalid if GLO parity unknown */
        
        gfrq =U2(p+ 2);
        psr  =R8(p+ 4);
        adr  =R8(p+16);
        dop  =R4(p+28);
        snr  =R4(p+32);
        lockt=R4(p+36);
        
        /* set glonass frequency channel number */
        if (sys==SYS_GLO&&raw->nav.geph[prn-1].sat!=sat) {
            raw->nav.geph[prn-1].frq=gfrq-7;
        }
        if (raw->tobs[sat-1][pos].time!=0) {
            tt=timediff(raw->time,raw->tobs[sat-1][pos]);
            lli=lockt-raw->lockt[sat-1][pos]+0.05<=tt?LLI_SLIP:0;
        }
        else {
            lli=0;
        }
        if (!parity) lli|=LLI_HALFC;
        if (halfc  ) lli|=LLI_HALFA;
        raw->tobs [sat-1][pos]=raw->time;
        raw->lockt[sat-1][pos]=lockt;
        raw->halfc[sat-1][pos]=halfc;
        
        if (!clock) psr=0.0;     /* code unlock */
        if (!plock) adr=dop=0.0; /* phase unlock */
        
        if (fabs(timediff(raw->obs.data[0].time,raw->time))>1E-9) {
            raw->obs.n=0;
        }
        if ((index=obsindex(&raw->obs,raw->time,sat))>=0) {
            raw->obs.data[index].L  [pos]=-adr;
            raw->obs.data[index].P  [pos]=psr;
            raw->obs.data[index].D  [pos]=(float)dop;
            raw->obs.data[index].SNR[pos]=
                0.0<=snr&&snr<255.0?(unsigned char)(snr*4.0+0.5):0;
            raw->obs.data[index].LLI[pos]=(unsigned char)lli;
            raw->obs.data[index].code[pos]=code;
        }
    }
    return 1;
}
/* decode rangecmpb ----------------------------------------------------------*/
static int decode_rangecmpb(raw_t *raw)
{
    double psr,adr,adr_rolls,lockt,tt,dop,snr,wavelen;
    int i,index,nobs,prn,sat,sys,code,freq,pos;
    int track,plock,clock,parity,halfc,lli;
    char *msg;
    unsigned char *p=raw->buff+UNICOREHLEN;
    
    trace(3,"decode_rangecmpb: len=%d\n",raw->len);
    
    nobs=U4(p);
    
    if (raw->outtype) {
        msg=raw->msgtype+strlen(raw->msgtype);
        sprintf(msg," nobs=%2d",nobs);
    }
    if (raw->len<UNICOREHLEN+4+nobs*24) {
        trace(2,"unicore rangecmpb length error: len=%d nobs=%d\n",raw->len,nobs);
        return -1;
    }
    for (i=0,p+=4;i<nobs;i++,p+=24) {
        
        /* decode tracking status */
        if ((freq=decode_trackstat(U4(p),&sys,&code,&track,&plock,&clock,
                                   &parity,&halfc))<0) continue;
        
        /* obs position */
        if ((pos=checkpri(raw->opt,sys,code,freq))<0) continue;
        
        prn=U1(p+17);
        if      (sys==SYS_GLO) prn-=37;
        
        if (!(sat=satno(sys,prn))) {
            trace(3,"unicore rangecmpb satellite number error: sys=%d,prn=%d\n",sys,prn);
            continue;
        }
        if (sys==SYS_GLO&&!parity) continue; /* invalid if GLO parity unknown */
        
        dop=exsign(U4(p+4)&0xFFFFFFF,28)/256.0;
        psr=(U4(p+7)>>4)/128.0+U1(p+11)*2097152.0;
        
        if ((wavelen=satwavelen(sat,freq,&raw->nav))<=0.0) {
            if (sys==SYS_GLO) wavelen=CLIGHT/(freq==0?FREQ1_GLO:FREQ2_GLO);
            else wavelen=lam_carr[freq];
        }
        adr=I4(p+12)/256.0;
        adr_rolls=(psr/wavelen+adr)/MAXVAL;
        adr=-adr+MAXVAL*floor(adr_rolls+(adr_rolls<=0?-0.5:0.5));
        
        lockt=(U4(p+18)&0x1FFFFF)/32.0; /* lock time */
        
        if (raw->tobs[sat-1][pos].time!=0) {
            tt=timediff(raw->time,raw->tobs[sat-1][pos]);
            lli=(lockt<65535.968&&lockt-raw->lockt[sat-1][pos]+0.05<=tt)?LLI_SLIP:0;
        }
        else {
            lli=0;
        }
        if (!parity) lli|=LLI_HALFC;
        if (halfc  ) lli|=LLI_HALFA;
        raw->tobs [sat-1][pos]=raw->time;
        raw->lockt[sat-1][pos]=lockt;
        raw->halfc[sat-1][pos]=halfc;
        
        snr=((U2(p+20)&0x3FF)>>5)+20.0;
        if (!clock) psr=0.0;     /* code unlock */
        if (!plock) adr=dop=0.0; /* phase unlock */
        
        if (fabs(timediff(raw->obs.data[0].time,raw->time))>1E-9) {
            raw->obs.n=0;
        }
        if ((index=obsindex(&raw->obs,raw->time,sat))>=0) {
            raw->obs.data[index].L  [pos]=adr;
            raw->obs.data[index].P  [pos]=psr;
            raw->obs.data[index].D  [pos]=(float)dop;
            raw->obs.data[index].SNR[pos]=
                0.0<=snr&&snr<255.0?(unsigned char)(snr*4.0+0.5):0;
            raw->obs.data[index].LLI[pos]=(unsigned char)lli;
            raw->obs.data[index].code[pos]=code;
        }
    }
    return 1;
}
/* decode gpsphemb -----------------------------------------------------------*/
static int decode_gpsephemb(raw_t *raw)
{
    unsigned char *p=raw->buff+UNICOREHLEN;
    eph_t eph={0};
    char *msg;
    double tow,toc,n,ura,tt;
    int prn,week,zweek,iode2,as;
    
    trace(3,"decode_gpsephemb: len=%d\n",raw->len);
    
    if (raw->len<UNICOREHLEN+224) {
        trace(2,"unicore gpsephemb length error: len=%d\n",raw->len);
        return -1;
    }
    prn        =U2(p); p+=4;
    
    if (raw->outtype) {
        msg=raw->msgtype+strlen(raw->msgtype);
        sprintf(msg," prn=%3d",prn);
    }
    if (!(eph.sat=satno(SYS_GPS,prn))) {
        trace(2,"unicore gpsephemb prn error: prn=%d\n",prn);
        return -1;
    }
    tow        =R8(p); p+=8;
    eph.svh    =(int)U4(p); p+=4;
    eph.iode   =(int)U4(p); p+=4;
    iode2      =(int)U4(p); p+=4;
    week       =(int)U4(p); p+=4;
    zweek      =U4(p); p+=4;
    eph.toes   =R8(p); p+=8;
    eph.A      =R8(p); p+=8;
    eph.deln   =R8(p); p+=8;
    eph.M0     =R8(p); p+=8;
    eph.e      =R8(p); p+=8;
    eph.omg    =R8(p); p+=8;
    eph.cuc    =R8(p); p+=8;
    eph.cus    =R8(p); p+=8;
    eph.crc    =R8(p); p+=8;
    eph.crs    =R8(p); p+=8;
    eph.cic    =R8(p); p+=8;
    eph.cis    =R8(p); p+=8;
    eph.i0     =R8(p); p+=8;
    eph.idot   =R8(p); p+=8;
    eph.OMG0   =R8(p); p+=8;
    eph.OMGd   =R8(p); p+=8;
    eph.iodc   =(int)U4(p); p+=4;
    toc        =R8(p); p+=8;
    eph.tgd[0] =R8(p); p+=8;
    eph.f0     =R8(p); p+=8;
    eph.f1     =R8(p); p+=8;
    eph.f2     =R8(p); p+=8;
    as         =(int)U4(p); p+=4; /* AS-ON */
    n          =R8(p); p+=8;
    ura        =R8(p); p+=8;
    
    if (eph.iode!=iode2) {
        trace(2,"unicore gpsephemb iode error: iode=%d %d\n",eph.iode,iode2);
        return -1;
    }
    eph.week=adjgpsweek(week);
    eph.toe=gpst2time(eph.week,eph.toes);
    tt=timediff(eph.toe,raw->time);
    if      (tt<-302400.0) eph.week++;
    else if (tt> 302400.0) eph.week--;
    eph.toe=gpst2time(eph.week,eph.toes);
    eph.toc=gpst2time(eph.week,toc);
    eph.ttr=adjweek(eph.toe,tow);
    eph.sva=uraindex(ura);
    
    if (!strstr(raw->opt,"-EPHALL")) {
        if (timediff(raw->nav.eph[eph.sat-1].toe,eph.toe)==0.0&&
            raw->nav.eph[eph.sat-1].iode==eph.iode&&
            raw->nav.eph[eph.sat-1].iodc==eph.iodc) return 0; /* unchanged */
    }
    raw->nav.eph[eph.sat-1]=eph;
    raw->ephsat=eph.sat;
    return 2;
}
/* decode gloephemerisb ------------------------------------------------------*/
static int decode_gloephemerisb(raw_t *raw)
{
    unsigned char *p=raw->buff+UNICOREHLEN;
    geph_t geph={0};
    char *msg;
    double tow,tof,toff;
    int prn,sat,week;
    
    trace(3,"decode_gloephemerisb: len=%d\n",raw->len);
    
    if (raw->len<UNICOREHLEN+144) {
        trace(2,"unicore gloephemerisb length error: len=%d\n",raw->len);
        return -1;
    }
    prn        =U2(p)-37;
    
    if (raw->outtype) {
        msg=raw->msgtype+strlen(raw->msgtype);
        sprintf(msg," prn=%3d",prn);
    }
    if (!(sat=satno(SYS_GLO,prn))) {
        trace(2,"unicore gloephemerisb prn error: prn=%d\n",prn);
        return -1;
    }
    geph.frq   =U2(p+  2)+OFF_FRQNO;
    week       =U2(p+  6);
    tow        =floor(U4(p+8)/1000.0+0.5); /* rounded to integer sec */
    toff       =U4(p+ 12);
    geph.iode  =U4(p+ 20)&0x7F;
    geph.svh   =U4(p+ 24);
    geph.pos[0]=R8(p+ 28);
    geph.pos[1]=R8(p+ 36);
    geph.pos[2]=R8(p+ 44);
    geph.vel[0]=R8(p+ 52);
    geph.vel[1]=R8(p+ 60);
    geph.vel[2]=R8(p+ 68);
    geph.acc[0]=R8(p+ 76);
    geph.acc[1]=R8(p+ 84);
    geph.acc[2]=R8(p+ 92);
    geph.taun  =R8(p+100);
    geph.gamn  =R8(p+116);
    tof        =U4(p+124)-toff; /* glonasst->gpst */
    geph.age   =U4(p+136);
    geph.toe=gpst2time(week,tow);
    tof+=floor(tow/86400.0)*86400;
    if      (tof<tow-43200.0) tof+=86400.0;
    else if (tof>tow+43200.0) tof-=86400.0;
    geph.tof=gpst2time(week,tof);
    
    if (!strstr(raw->opt,"-EPHALL")) {
        if (fabs(timediff(geph.toe,raw->nav.geph[prn-1].toe))<1.0&&
            geph.svh==raw->nav.geph[prn-1].svh) return 0; /* unchanged */
    }
    geph.sat=sat;
    raw->nav.geph[prn-1]=geph;
    raw->ephsat=sat;
    return 2;
}
/* decode galephemerisb ------------------------------------------------------*/
static int decode_galephemerisb(raw_t *raw)
{
    eph_t eph={0};
    unsigned char *p=raw->buff+UNICOREHLEN;
    double tow,sqrtA,af0_fnav,af1_fnav,af2_fnav,af0_inav,af1_inav,af2_inav,tt;
    char *msg;
    int prn,rcv_fnav,rcv_inav,svh_e1b,svh_e5a,svh_e5b,dvs_e1b,dvs_e5a,dvs_e5b;
    int toc_fnav,toc_inav,week,sel_nav=0;
    
    trace(3,"decode_galephemerisb: len=%d\n",raw->len);
    
    if (raw->len<UNICOREHLEN+220) {
        trace(2,"oem4 galephemrisb length error: len=%d\n",raw->len);
        return -1;
    }
    prn       =U4(p);   p+=4;
    rcv_fnav  =U4(p)&1; p+=4;
    rcv_inav  =U4(p)&1; p+=4;
    svh_e1b   =U1(p)&3; p+=1;
    svh_e5a   =U1(p)&3; p+=1;
    svh_e5b   =U1(p)&3; p+=1;
    dvs_e1b   =U1(p)&1; p+=1;
    dvs_e5a   =U1(p)&1; p+=1;
    dvs_e5b   =U1(p)&1; p+=1;
    eph.sva   =U1(p);   p+=1+1; /* SISA index */
    eph.iode  =U4(p);   p+=4;   /* IODNav */
    eph.toes  =U4(p);   p+=4;
    sqrtA     =R8(p);   p+=8;
    eph.deln  =R8(p);   p+=8;
    eph.M0    =R8(p);   p+=8;
    eph.e     =R8(p);   p+=8;
    eph.omg   =R8(p);   p+=8;
    eph.cuc   =R8(p);   p+=8;
    eph.cus   =R8(p);   p+=8;
    eph.crc   =R8(p);   p+=8;
    eph.crs   =R8(p);   p+=8;
    eph.cic   =R8(p);   p+=8;
    eph.cis   =R8(p);   p+=8;
    eph.i0    =R8(p);   p+=8;
    eph.idot  =R8(p);   p+=8;
    eph.OMG0  =R8(p);   p+=8;
    eph.OMGd  =R8(p);   p+=8;
    toc_fnav  =U4(p);   p+=4;
    af0_fnav  =R8(p);   p+=8;
    af1_fnav  =R8(p);   p+=8;
    af2_fnav  =R8(p);   p+=8;
    toc_inav  =U4(p);   p+=4;
    af0_inav  =R8(p);   p+=8;
    af1_inav  =R8(p);   p+=8;
    af2_inav  =R8(p);   p+=8;
    eph.tgd[0]=R8(p);   p+=8; /* BGD: E5A-E1 (s) */
    eph.tgd[1]=R8(p);         /* BGD: E5B-E1 (s) */
    eph.iodc  =eph.iode;
    eph.svh   =(svh_e5b<<7)|(dvs_e5b<<6)|(svh_e5a<<4)|(dvs_e5a<<3)|
               (svh_e1b<<1)|dvs_e1b;
    
    /* ephemeris selection (0:INAV,1:FNAV) */
    if      (strstr(raw->opt,"-GALINAV")) sel_nav=0;
    else if (strstr(raw->opt,"-GALFNAV")) sel_nav=1;
    else if (!rcv_inav&&rcv_fnav) sel_nav=1;
    
    eph.A     =sqrtA*sqrtA;
    eph.f0    =sel_nav?af0_fnav:af0_inav;
    eph.f1    =sel_nav?af1_fnav:af1_inav;
    eph.f2    =sel_nav?af2_fnav:af2_inav;
    
    /* set data source defined in rinex 3.03 */
    eph.code=(sel_nav==0)?((1<<0)|(1<<9)):((1<<1)|(1<<8));
    
    if (raw->outtype) {
        msg=raw->msgtype+strlen(raw->msgtype);
        sprintf(msg," prn=%3d iod=%3d toes=%6.0f",prn,eph.iode,eph.toes);
    }
    if (!(eph.sat=satno(SYS_GAL,prn))) {
        trace(2,"oemv galephemeris satellite error: prn=%d\n",prn);
        return -1;
    }
    tow=time2gpst(raw->time,&week);
    eph.week=week; /* gps-week = gal-week */
    eph.toe=gpst2time(eph.week,eph.toes);
    
    /* for week-handover problem */
    tt=timediff(eph.toe,raw->time);
    if      (tt<-302400.0) eph.week++;
    else if (tt> 302400.0) eph.week--;
    eph.toe=gpst2time(eph.week,eph.toes);
    eph.toc=adjweek(eph.toe,sel_nav?toc_fnav:toc_inav);
    eph.ttr=adjweek(eph.toe,tow);
    
    if (!strstr(raw->opt,"-EPHALL")) {
        if (raw->nav.eph[eph.sat-1].iode==eph.iode&&
            raw->nav.eph[eph.sat-1].code==eph.code) return 0; /* unchanged */
    }
    raw->nav.eph[eph.sat-1]=eph;
    raw->ephsat=eph.sat;
    return 2;
}
/* decode bd2ephemb ----------------------------------------------------------*/
static int decode_bd2ephemb(raw_t *raw)
{
    eph_t eph={0};
    unsigned char *p=raw->buff+UNICOREHLEN;
    double ura;
    char *msg;
    int prn,toc;

    trace(3,"decode_bd2ephemb: len=%d\n",raw->len);

    if (raw->len<UNICOREHLEN+232) {
        trace(2,"unicore bdsephemrisb length error: len=%d\n",raw->len);
        return -1;
    }

    prn       =U4(p+0);     /* PRN */
    eph.week  =U4(p+24);    /* WEEK */
    ura       =R8(p+224);   /* URA */
    eph.svh   =U4(p+12)&1;  /* Health */
    eph.tgd[0]=R8(p+172);   /* TGD1 */
    eph.tgd[1]=R8(p+180);   /* TGD2 */
    eph.iodc  =U4(p+160);   /* AODC */
    toc       =U4(p+164);   /* TOC */
    eph.f0    =R8(p+188);   /* af0 */
    eph.f1    =R8(p+196);   /* af1 */
    eph.f2    =R8(p+204);   /* af2 */
    eph.iode  =U4(p+16);    /* AODE */
    eph.toes  =U4(p+32);    /* TOE */
    eph.e     =R8(p+64);    /* ECC */
    eph.omg   =R8(p+72);    /* w */
    eph.deln  =R8(p+48);    /* Delta N */
    eph.M0    =R8(p+56);    /* M0 */
    eph.OMG0  =R8(p+144);   /* OMG0 */
    eph.OMGd  =R8(p+152);   /* OMGd */
    eph.i0    =R8(p+128);   /* I0 */
    eph.idot  =R8(p+136);   /* IDOT */
    eph.cuc   =R8(p+80);    /* cuc */
    eph.cus   =R8(p+88);    /* cus */
    eph.crc   =R8(p+96);    /* crc */
    eph.crs   =R8(p+104);   /* crs */
    eph.cic   =R8(p+112);   /* cic */
    eph.cis   =R8(p+120);   /* cis */
    eph.A     =R8(p+40);    /* A */
    eph.sva   =uraindex(ura);

    if (raw->outtype) {
        msg=raw->msgtype+strlen(raw->msgtype);
        sprintf(msg," prn=%3d iod=%3d toes=%6.0f",prn,eph.iode,eph.toes);
    }
    if (!(eph.sat=satno(SYS_CMP,prn))) {
        trace(2,"unicore bdsephemeris satellite error: prn=%d\n",prn);
        return -1;
    }
    eph.toe=bdt2gpst(bdt2time(eph.week,eph.toes)); /* bdt -> gpst */
    eph.toc=bdt2gpst(bdt2time(eph.week,toc));      /* bdt -> gpst */
    eph.ttr=raw->time;

    if (!strstr(raw->opt,"-EPHALL")) {
        if (timediff(raw->nav.eph[eph.sat-1].toe,eph.toe)==0.0&&
            raw->nav.eph[eph.sat-1].iode==eph.iode&&
            raw->nav.eph[eph.sat-1].iodc==eph.iodc) return 0; /* unchanged */
    }
    raw->nav.eph[eph.sat-1]=eph;
    raw->ephsat=eph.sat;

    return 2;
}
/* decode unicore message -----------------------------------------------------*/
static int decode_unicore(raw_t *raw)
{
    double tow;
    int msg,week,type=U2(raw->buff+4);
    
    trace(3,"decode_unicore: type=%3d len=%d\n",type,raw->len);
    
    /* check crc32 */
    if (rtk_crc32(raw->buff,raw->len)!=U4(raw->buff+raw->len)) {
        trace(2,"unicore crc error: type=%3d len=%d\n",type,raw->len);
        return -1;
    }
    msg =(U1(raw->buff+6)>>4)&0x3;
    if (!(week=U2(raw->buff+14))) {
        return -1;
    }
    week=adjgpsweek(week);
    tow =U4(raw->buff+16)*0.001;
    raw->time=gpst2time(week,tow);
    
    if (raw->outtype) {
        sprintf(raw->msgtype,"UNICORE%4d (%4d): msg=%d %s",type,raw->len,msg,
                time_str(gpst2time(week,tow),2));
    }
    switch (type) {
        case ID_RANGE         : return decode_rangeb       (raw);
        case ID_RANGECMP      : return decode_rangecmpb    (raw);
        case ID_GPSEPHEM      : return decode_gpsephemb    (raw);
        case ID_GLOEPHEMERIS  : return decode_gloephemerisb(raw);
        case ID_GALEPHEMERIS  : return decode_galephemerisb(raw);
        case ID_BD2EPHEM      : return decode_bd2ephemb    (raw);
/*        case ID_EVENTALL      : return decode_eventall     (raw); */
        /*default               : return decode_fallback     (STRFMT_UNICORE, raw);*/
    }
    return 0;
}
/* sync header ---------------------------------------------------------------*/
static int sync_unicore(unsigned char *buff, unsigned char data)
{
    buff[0]=buff[1]; buff[1]=buff[2]; buff[2]=data;
    return buff[0]==UNICORESYNC1&&buff[1]==UNICORESYNC2&&buff[2]==UNICORESYNC3;
}
/* input unicore raw data from stream -------------------------------------------
* fetch next unicore raw data and input a mesasge from stream
* args   : raw_t *raw   IO     receiver raw data control struct
*          unsigned char data I stream data (1 byte)
* return : status (-1: error message, 0: no message, 1: input observation data,
*                  2: input ephemeris, 3: input sbas message,
*                  9: input ion/utc parameter)
*
* notes  : to specify input options for unicore, set raw->opt to the following
*          option strings separated by spaces.
*
*          -EPHALL : input all ephemerides
*-----------------------------------------------------------------------------*/
extern int input_unicore(raw_t *raw, unsigned char data)
{
    trace(5,"input_unicore: data=%02x\n",data);
    
    /* synchronize frame */
    if (raw->nbyte==0) {
        if (sync_unicore(raw->buff,data)) raw->nbyte=3;
        return 0;
    }
    raw->buff[raw->nbyte++]=data;
    
    if (raw->nbyte==10&&(raw->len=U2(raw->buff+8)+UNICOREHLEN)>MAXRAWLEN-4) {
        trace(2,"unicore length error: len=%d\n",raw->len);
        raw->nbyte=0;
        return -1;
    }
    if (raw->nbyte<10||raw->nbyte<raw->len+4) return 0;
    raw->nbyte=0;
    
    /* decode unicore message */
    return decode_unicore(raw);
}
/* input unicore raw data from file ---------------------------------------------
* fetch next unicore raw data and input a message from file
* args   : raw_t  *raw   IO     receiver raw data control struct
*          int    format I      receiver raw data format (STRFMT_???)
*          FILE   *fp    I      file pointer
* return : status(-2: end of file, -1...9: same as above)
*-----------------------------------------------------------------------------*/
extern int input_unicoref(raw_t *raw, FILE *fp)
{
    int i,data;
    
    trace(4,"input_unicoref:\n");
    
    /* synchronize frame */
    if (raw->nbyte==0) {
        for (i=0;;i++) {
            if ((data=fgetc(fp))==EOF) return -2;
            if (sync_unicore(raw->buff,(unsigned char)data)) break;
            if (i>=4096) return 0;
        }
    }
    if (fread(raw->buff+3,7,1,fp)<1) return -2;
    raw->nbyte=10;
    
    if ((raw->len=U2(raw->buff+8)+UNICOREHLEN)>MAXRAWLEN-4) {
        trace(2,"unicore length error: len=%d\n",raw->len);
        raw->nbyte=0;
        return -1;
    }
    if (fread(raw->buff+10,raw->len-6,1,fp)<1) return -2;
    raw->nbyte=0;
    
    /* decode unicore message */
    return decode_unicore(raw);
}
