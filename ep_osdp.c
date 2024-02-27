/*
 * File:   odsp.c
 * Author: ep88
 *
 * Created on February 23, 2024, 2:19 PM
 */


#include "ep_includes.h"

EPS_OSDP_SCS osdpScs;
//Read Index=Full home for read  Write index=Empty home for write
U8 rIdx = 0;
U8 wIdx = 0;
UL32 UartLstRcvTick = 0;
EPS_OSDP_PACKET osdpBuff[OSDP_PCKTDESCRPTR_BUF_NUM];

static U8 lastSqn[2] = {0, 0};
static bool OSDP_PktDscptrs_Full = false;
static EPSTT_OSDPPCKT_STT state = STT_NOTHING_RCVD;

void ep_osdpInit(void) {
    memset(&osdpScs, 0, sizeof (EPS_OSDP_SCS));
}

void ep_osdpSend(U8* buff, UL32 len) {
    for (UL32 i = 0; i < len; i++) putch(buff[i]);
}

C8 ep_osdpPktBuff(U8 byte, EPS_OSDP_PACKET* pkt) {
    U8 U8_Temp, res;
    US16 U16_Tmp, i, j, pktLen = 0;
    SS16 SU16_Tmp;
    UL32 U32_Tmp, lastRecvTick;

    res = MA_BUSY;
    lastRecvTick = UartLstRcvTick;
    UartLstRcvTick = getCurrentTick(); //save this character reception tick

    // all packet descriptions are full so cant accept any character
    if (OSDP_PktDscptrs_Full) return MA_FULL;

    // If received character stream size is more than standard frame size
    // If there is a gap between received characters more than OSDP_INTERCHARACTER_TIMEOUT
    if (getCurrentTick() - lastRecvTick > OSDP_INTERCHARACTER_TIMEOUT ||
            pkt->buffIdx >= OSDP_PACKET_SIZE_MAX) {
        pkt->buffIdx = 0; //reset buffer write index because sync happened
        state = STT_NOTHING_RCVD; //Reset the buffer status
    }

    //Push character into relevant buffer
    *(&pkt + pkt->buffIdx) = byte;
    pkt->buffIdx++;

    switch (state) {
        case STT_NOTHING_RCVD: // First byte, receiving SOM
            if (pkt->header.som == 0x53) state = STT_SOM_RCVD;
            else pkt->buffIdx = 0;
            break;
        case STT_SOM_RCVD: // SOM received, receiving address
            if ((pkt->header.pd_addr == OSDP_PD_ADDR_1STREADER) ||
                    (pkt->header.pd_addr == OSDP_PD_ADDR_2NDREADER) ||
                    (pkt->header.pd_addr == OSDP_BROADCAST_ADDR))
                state = STT_ADDR_RCVD;
            else {
                pkt->buffIdx = 0;
                state = STT_NOTHING_RCVD;
            }
            break;
        case STT_ADDR_RCVD: // Address received, receiving LEN_LSB
            state = STT_LENLSB_RCVD;
            break;
        case STT_LENLSB_RCVD:// LEN_LSB received, receiving LEN_MSB
            pktLen = ((pkt->header.len_msb << 8) & 0xFF00)
                    | (0x00FF & pkt->header.len_lsb);
            if (pktLen > OSDP_PACKET_SIZE_MAX) {
                pkt->buffIdx = 0;
                state = STT_NOTHING_RCVD;
            } else {
                // we are going to receive payload in next stage
                // payload starts from CTRL, so we need to deduct
                // header length of 4 bytes
                SU16_Tmp = pktLen - 4;
                state = STT_LENMSB_RCVD;
            }
            break;
        case STT_LENMSB_RCVD: // Payload + MAC[0 to 3] or checksum/CRC reception
            //whole packet received
            if (!(--SU16_Tmp)) {
                // 16-bit CRC checking
                if (pkt->ctrl.cksum) {
                    US16 crc;
                    crc = CRC_CCITT_INIT_VALUE; //CRC initial value
                    for (i = 0; i < (pktLen - 2); i++)
                        crc = update_crc_ccitt(crc, (char) *((U8*) & pkt + i));
                    //extract CRC field
                    U16_Tmp = ((*((U8*) & pkt + pktLen - 1) << 8) & 0xFF00)
                            | (*((U8*) & pkt + pktLen - 2) & 0x00FF);

                    if (crc != U16_Tmp)//CRC error
                        res = MA_ERROR;
                    else {
                        pkt->prevSqn[pkt->header.pd_addr] = lastSqn[pkt->header.pd_addr];
                        lastSqn[pkt->header.pd_addr] = pkt->ctrl.sqn & 0x03;
                        pkt->revTime = UartLstRcvTick;
                        if (pkt->ctrl.scb == 0)//no block data
                            pkt->dataBlkLen = pktLen - 7 - pkt->ctrl.cksum;
                        else//with block data
                            pkt->dataBlkLen = pktLen - 7 - pkt->ctrl.cksum - pkt->payload[1];
                        res = MA_OK;
                    }
                    pkt->buffIdx = 0;
                    state = STT_NOTHING_RCVD;
                } else //Checksum checking
                {
                    U16_Tmp = pktLen - 1;
                    U32_Tmp = 0;
                    U8_Temp = *(& pkt + U16_Tmp); //save received checksum
                    while (U16_Tmp--) U32_Tmp += *(&pkt + U16_Tmp);
                    // Take the right most 1 bytes of its two's complements
                    U32_Tmp = ((~U32_Tmp) + 1) & 0x000000FF;
                    if ((U8) U32_Tmp != U8_Temp)
                        res = MA_ERROR;
                    else {
                        pkt->prevSqn[pkt->header.pd_addr] = lastSqn[pkt->header.pd_addr];
                        lastSqn[pkt->header.pd_addr] = pkt->ctrl.sqn & 0x03;
                        pkt->revTime = UartLstRcvTick;
                        if (pkt->ctrl.scb == 0)//no block data
                            pkt->dataBlkLen = pktLen - 7 - pkt->ctrl.cksum;
                        else//with block data
                            pkt->dataBlkLen = pktLen - 7 - pkt->ctrl.cksum - pkt->payload[1];
                        res = MA_OK;
                    }
                    pkt->buffIdx = 0;
                    state = STT_NOTHING_RCVD;
                }
            }
            break;
        default:
            break;
    }
    return res;
}

void ep_osdpPacketConsume(void) {
    bool IntLocalyDisabled = disableTMR0Int();

    //no OSDP packet is waiting for serve
    if ((rIdx == wIdx)
            && (OSDP_PktDscptrs_Full == false)) return;


    // ===================================== Check the SQN integrity ===================================================
    //-------  if SQN is 0 or in sequence then process this received packet ---------
    U8 pdAddr = osdpBuff[rIdx].header.pd_addr;
    U8 currSqn = osdpBuff[rIdx].ctrl.sqn;
    U8 lastSqn = osdpBuff[rIdx].prevSqn[pdAddr];
    bool cond1 = osdpBuff[rIdx].ctrl.sqn == 0;
    bool cond2 = (lastSqn == 3) && (currSqn == 1);
    // need to confirm if this is necessary
    // bool sqnIsLastPlus1 = osdpBuff[rIdx].ctrl.sqn == ((osdpBuff[rIdx].prevSqn[osdpBuff[rIdx].header.addr] + 1) % 4);
    if (cond1 || cond2) {
        ep_osdpProcess(&osdpBuff[rIdx]);
        //Take a shot of this "received and served" OSDP packet. it will be used for resend process
        //        OSDP_LstRcvd_Shadow_Len[osdpBuff[rIdx].header.pd_addr] = (((UL32) osdpBuff[rIdx].OSDP_Packet.HEADER.Len_MSB << 8) & 0xFF00) |
        //                (0x00FF & osdpBuff[rIdx].OSDP_Packet.HEADER.Len_LSB);
        //        memcpy((U8*) &OSDP_LstRcvd_Shadow[osdpBuff[rIdx].header.pd_addr],
        //                (U8*) &osdpBuff[rIdx].OSDP_Packet, OSDP_LstRcvd_Shadow_Len[osdpBuff[rIdx].header.pd_addr]);
        //    }// ----- SQN is the same as previous one --------
        //    else if (osdpBuff[rIdx].OSDP_Packet.payload.CTRL.BITS.SQN ==
        //            osdpBuff[rIdx].Previous_SQN[osdpBuff[rIdx].header.pd_addr]) //Just resend the last sent packet without any further process
        //    {
        //
        //
        //        //check if this received packet is exactly the same as shadow packet (last received packet that served before) then resend last reply again
        //        if (memcmp((U8*) &OSDP_LstRcvd_Shadow[osdpBuff[rIdx].header.pd_addr],
        //                (U8*) &osdpBuff[rIdx].OSDP_Packet,
        //                OSDP_LstRcvd_Shadow_Len[osdpBuff[rIdx].header.pd_addr]) == 0) {
        //            OSDP_Send((U8*) &OSDP_Packet4Send[osdpBuff[rIdx].header.pd_addr], OSDP_LstSent_Len[osdpBuff[rIdx].header.pd_addr]); //Resend the last sent reply(its command is processed befor)
        //            //no need to update OSDP_LstSent_Len because its a resend
        //        } else //SQN error
        //        {
        //
        //        }
    } else //SQN error
    {
        // DEBUG
    }
    //Update Read index
    rIdx = (rIdx + 1) % OSDP_PCKTDESCRPTR_BUF_NUM;
    OSDP_PktDscptrs_Full = false;
    if (IntLocalyDisabled == true) enableTMR0Int();
}