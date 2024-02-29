/* 
 * File:   
 * Author: 
 * Comments:
 * Revision history: 
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef EP_OSDP_H
#define	EP_OSDP_H

#include "ep_includes.h"

//============================  OSDP Commands   ==========================================
#define OSDP_POLL             0x60
#define OSDP_ID               0x61
#define OSDP_CAP              0x62
#define OSDP_DIAG             0x63
#define OSDP_LSTAT            0x64 
#define OSDP_ISTAT            0x65
#define OSDP_OSTAT            0x66
#define OSDP_RSTAT            0x67
#define OSDP_OUT              0x68
#define OSDP_LED              0x69
#define OSDP_BUZ              0x6A
#define OSDP_TEXT             0x6B
#define OSDP_TDSET            0x6D
#define OSDP_COMSET           0x6E
#define OSDP_DATA             0x6F
#define OSDP_PROMPT           0x71
#define OSDP_BIOREAD          0x73
#define OSDP_BIOMATCH         0x74
#define OSDP_KEYSET           0x75
#define OSDP_CHLNG            0x76 
#define OSDP_SCRYPT           0x77
#define OSDP_CONT             0x79  
#define OSDP_MFG              0x80
#define OSDP_XWR              0xA1

//============================  OSDP replies   ==========================================
#define OSDP_ACK              0x40
#define OSDP_NAK              0x41
#define OSDP_PDID             0x45
#define OSDP_PDCAP            0x46  
#define OSDP_LSTATR           0x48 
#define OSDP_ISTATR           0x49
#define OSDP_OSTATR           0x4A 
#define OSDP_RSTATR           0x4B 
#define OSDP_RAW              0x50
#define OSDP_FMT              0x51
#define OSDP_TAG              0x52
#define OSDP_KPD              0x53
#define OSDP_COM              0x54
#define OSDP_BIOREADR         0x57   
#define OSDP_FPMATCHR         0x58   
#define OSDP_CCRYPT           0x76 
#define OSDP_RMACI            0x78
#define OSDP_MFGREP           0x90 
#define OSDP_BUSY             0x79
#define OSDP_XRD              0xB1

//======================= OSDP NACK error codes ==============================
#define OSDP_ERR_MSG_CHK              0x01
#define OSDP_ERR_CMD_LEN              0x02
#define OSDP_ERR_UNKNOWN_CMD          0x03
#define OSDP_ERR_SQN                  0x04
#define OSDP_ERR_SECBLOCK             0x05
#define OSDP_ERR_ENCRPT_REQ           0x06
#define OSDP_ERR_BIOTYP_NOTSUP        0x07
#define OSDP_ERR_BIOFRMT_NOTSUP       0x08


//=============================  Secuity block types =================================
#define OSDP_SECBLKTYPE_SCS_11        0x11
#define OSDP_SECBLKTYPE_SCS_12        0x12
#define OSDP_SECBLKTYPE_SCS_13        0x13
#define OSDP_SECBLKTYPE_SCS_14        0x14
#define OSDP_SECBLKTYPE_SCS_15        0x15
#define OSDP_SECBLKTYPE_SCS_16        0x16
#define OSDP_SECBLKTYPE_SCS_17        0x17
#define OSDP_SECBLKTYPE_SCS_18        0x18


#define OSDP_PACKET_SIZE_MAX 200 //Refer to 2.6 of OSDP manual
#define OSDP_INTERCHARACTER_TIMEOUT 20 //in ms. Refer to 2.8 of OSDP manual
#define OSDP_PCKTDESCRPTR_BUF_NUM 1  //Number of OSDP buffer
#define OSDP_PD_ADDR_1STREADER 0x00 //First reader OSDP PD address
#define OSDP_PD_ADDR_2NDREADER 0x01 //Second reader OSDP PD address
#define OSDP_BROADCAST_ADDR 0x7F

// SCS = Secure Channel Session

typedef struct _EPS_OSDP_SCS {
    // EPSTT_OSDSCS_STT OSDP_SCS_State;
    // Keys
    U8 SCBK[16];
    U8 S_ENC[16];
    U8 S_MAC1[16];
    U8 S_MAC2[16];

    // Last sent MAC  to the CP.it is updated for first 
    // time at OSDP_SCRYPT handeling section
    U8 LastSent_MAC[16];
    // Last Received MAC to the PD.it is 
    // updated after every unwrapping procedure.
    U8 LastRcvd_MAC[16];

    //Cryptograms
    U8 Server_Crptgm[16]; //Server (CP) cryptogram
    U8 Client_Crptgm[16]; //Client (PD) cryptogram

    //Random numbers
    U8 RND_A[8];
    U8 RND_B[8];

    //validations

    struct {
        U8 SCBK_Valid : 1;
        U8 S_ENC_Valid : 1;
        U8 S_MAC1_Valid : 1;
        U8 S_MAC2_Valid : 1;

        U8 LastSent_MAC_Valid : 1;
        U8 LastRcvd_MAC_Valid : 1;

        U8 Srvr_Crptgm_Valid : 1;
        U8 Clnt_Crptgm_Valid : 1;

        U8 RND_A_Valid : 1;
        U8 RND_B_Valid : 1;

        U8 NOP : 6;
    } VALIDATION;
}
EPS_OSDP_SCS;

//communication packet state machine

typedef enum _EPSTT_OSDPPCKT_STT {
    STT_NOTHING_RCVD,
    STT_SOM_RCVD,
    STT_ADDR_RCVD,
    STT_LENLSB_RCVD,
    STT_LENMSB_RCVD,
    STT_PAYLOAD_RCVD,
    STT_CHKSUMCRCLSB_RCVD,
    STT_CRCMSB_RCVD
} EPSTT_OSDPPCKT_STT;

typedef union _EPS_OSDP_HEADER {
    // header

    struct {
        U8 som;
        U8 pd_addr;
        U8 len_lsb; // Packet Length including SOM to end of checksum
        U8 len_msb;
    };
    U8 header[4];
} EPS_OSDP_HEADER;

/**
 * Sequence number
 * Set: 16-bit CRC, Clear: 8-bit checksum is present Din the tail of the packet
 * if is set then the Security control block is present in the message
 */
typedef union _EPS_OSDP_CTRL {

    struct {
        U8 sqn : 2;
        U8 cksum : 1;
        U8 scb : 1;
        U8 na : 3;
        U8 multi : 1;
    };
    U8 ctrlByte;
} EPS_OSDP_CTRL;

/**
 * // meta data: valid and onwards
 * Packet buffer contains a valid received OSDP Packet
 * packet content is locked by a process for consumption
 * PcktBuf_Write_Index; //used for push the incoming character into the buffer
 * Packet_Reception_Time; //tick time of reception
 * We need it rather than direct calculation for secure channel session
 */
typedef struct _EPS_OSDP_PACKET {
    EPS_OSDP_HEADER header;
    EPS_OSDP_CTRL ctrl;
    U8 payload[OSDP_PACKET_SIZE_MAX - 5];
    bool valid;
    bool locked;
    U8 prevSqn[2];
    UL32 buffIdx;
    UL32 revTime;
    UL32 dataBlkLen;
} EPS_OSDP_PACKET;

//Refer to the "4.11 osdp_BUZ" command

typedef struct _EPS_BUZCTRL {
    U8 ReaderNum;
    U8 ToneCode;
    U8 OnTime; //in units of 100ms
    U8 OffTime; //in units of 100ms
    U8 Count;
} EPS_BUZCTRL;

void ep_osdpInit(void);
void ep_osdpPacketConsume(void);
void ep_osdpSend(U8* buff, UL32 len);
void ep_osdpProcess(EPS_OSDP_PACKET*);
S8 ep_osdpPktBuff(U8);
void osdpLEDHandler(void* p1stRecAddress, U8 ReaderNo, UL32 RecNumber);

#endif	/* XC_HEADER_TEMPLATE_H */

