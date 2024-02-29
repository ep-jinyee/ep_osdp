#include "ep_includes.h"

EPS_OSDP_SCS osdpScs;
//Read Index=Full home for read  Write index=Empty home for write
U8 rIdx = 0;
U8 wIdx = 0;

EPS_OSDP_PACKET pktForSend[2], lastRcvPkt[2]; //pktForSend per OSDP header address.we need it for managing SQN per address.this is applicable for shadow too.

UL32 lastRcvlen[2] = {0, 0}, lastSentlen[2] = {0, 0}; //indexes are for per OSDP header address
extern EPS_E2P_CONFIGS E2P_Configs;
extern U8 TRIGGER, UNTRIGGER;
extern U8 tmpr1Change, tmpr2Change, PrvTmpr1, PrvTmpr2;
extern UL32 CardFlashTick[2];
extern bool CardFlashTimeoutEnbled;

static UL32 lastRecvTick = 0;
static UL32 UartLstRcvTick = 0;
static U8 lastSqn[2] = {0, 0};
static bool odspBuffFull = false;
static EPSTT_OSDPPCKT_STT state = STT_NOTHING_RCVD;
static EPS_OSDP_PACKET osdpBuff[OSDP_PCKTDESCRPTR_BUF_NUM];

void ep_osdpInit(void) {
    memset(&osdpScs, 0, sizeof (EPS_OSDP_SCS));
}

void ep_osdpSend(U8* buff, UL32 len) {
    for (UL32 i = 0; i < len; i++) putch(buff[i]);
}

S8 ep_osdpPktBuff(U8 byte) {
    // declare static so that they exist
    // all the time even after exit the func
    static U8 counter;
    static U8 pktlen = 0;
    U8 u8Temp;
    S8 res;
    US16 u16Tmp;
    US16 crc;
    UL32 u32Tmp;

    res = (S8) MA_BUSY;

    //save this character reception tick
    lastRecvTick = UartLstRcvTick;
    UartLstRcvTick = getCurrentTick();

    // buff are full cant accept any byte
    if (odspBuffFull) return (S8) MA_FULL;

    // If received byte stream size is more than standard frame size
    // If there is a gap between received byte more than 
    // OSDP_INTERCHARACTER_TIMEOUT
    if (UartLstRcvTick - lastRecvTick > OSDP_INTERCHARACTER_TIMEOUT
            || osdpBuff[wIdx].buffIdx >= OSDP_PACKET_SIZE_MAX) {
        osdpBuff[wIdx].buffIdx = 0;
        state = STT_NOTHING_RCVD;
    }

    // Push character into buffer, this U8* is very important
    // since our struct has member of different size, U8* tells
    // compiler to point 1 byte and advance 1 byte every time.
    *((U8*) & osdpBuff[wIdx] + osdpBuff[wIdx].buffIdx) = byte;
    osdpBuff[wIdx].buffIdx++;

    switch (state) {
        case STT_NOTHING_RCVD: // First byte, receiving SOM
            if (osdpBuff[wIdx].header.som == 0x53) state = STT_SOM_RCVD;
            else osdpBuff[wIdx].buffIdx = 0;
            break;
        case STT_SOM_RCVD: // SOM received, receiving address
            if ((osdpBuff[wIdx].header.pd_addr == OSDP_PD_ADDR_1STREADER)
                    || (osdpBuff[wIdx].header.pd_addr == OSDP_PD_ADDR_2NDREADER)
                    || (osdpBuff[wIdx].header.pd_addr == OSDP_BROADCAST_ADDR))
                state = STT_ADDR_RCVD;
            else {
                osdpBuff[wIdx].buffIdx = 0;
                state = STT_NOTHING_RCVD;
            }
            break;
        case STT_ADDR_RCVD: // Address received, receiving LEN_LSB
            state = STT_LENLSB_RCVD;
            break;
        case STT_LENLSB_RCVD:// LEN_LSB received, receiving LEN_MSB
            pktlen = ((osdpBuff[wIdx].header.len_msb << 8) & 0xF0)
                    | (osdpBuff[wIdx].header.len_lsb & 0x0F);
            if (pktlen > OSDP_PACKET_SIZE_MAX) {
                osdpBuff[wIdx].buffIdx = 0;
                state = STT_NOTHING_RCVD;
            } else {
                // we are going to receive payload in next stage
                // payload starts from CTRL, so we need to deduct
                // header length of 4 bytes
                counter = pktlen & 0xFB;
                state = STT_LENMSB_RCVD;
            }
            break;
        case STT_LENMSB_RCVD: // Payload + MAC[0 to 3] or checksum/CRC reception
            //whole packet received
            if (!(--counter)) {
                // 16-bit CRC checking
                if (osdpBuff[wIdx].ctrl.cksum) {
                    crc = CRC_CCITT_INIT_VALUE; //CRC initial value
                    for (U8 i = 0; i < (pktlen - 2); i++)
                        crc = update_crc_ccitt(crc, (char) *((U8*) & osdpBuff[wIdx] + i));
                    //extract CRC field
                    u16Tmp = ((UI16) (*((U8*) & osdpBuff[wIdx] + pktlen - 1) << 8) & 0xFF00)
                            | ((UI16) *((U8*) & osdpBuff[wIdx] + pktlen - 2) & 0x00FF);
                    if (crc != u16Tmp)//CRC error
                        res = (S8) MA_ERROR;
                    else {
                        osdpBuff[wIdx].prevSqn[osdpBuff[wIdx].header.pd_addr]
                                = lastSqn[osdpBuff[wIdx].header.pd_addr];
                        lastSqn[osdpBuff[wIdx].header.pd_addr]
                                = osdpBuff[wIdx].ctrl.sqn & 0x03;
                        osdpBuff[wIdx].revTime = UartLstRcvTick;
                        if (osdpBuff[wIdx].ctrl.scb == 0)//no block data
                            osdpBuff[wIdx].dataBlkLen = pktlen - 7 - osdpBuff[wIdx].ctrl.cksum;
                        else//with block data
                            osdpBuff[wIdx].dataBlkLen = pktlen - 7 - osdpBuff[wIdx].ctrl.cksum - osdpBuff[wIdx].payload[0];
                        res = MA_OK;
                    }
                    osdpBuff[wIdx].buffIdx = 0;
                    state = STT_NOTHING_RCVD;
                } else //Checksum checking
                {
                    u16Tmp = pktlen - 1;
                    u32Tmp = 0;
                    u8Temp = *((U8*) & osdpBuff[wIdx] + u16Tmp); //save received checksum
                    while (u16Tmp--) u32Tmp += *((U8*) & osdpBuff[wIdx] + u16Tmp);
                    // Take the right most 1 bytes of its two's complements
                    u32Tmp = ((~u32Tmp) + 1) & 0x000000FF;
                    if ((U8) u32Tmp != u8Temp)
                        res = (S8) MA_ERROR;
                    else {
                        osdpBuff[wIdx].prevSqn[osdpBuff[wIdx].header.pd_addr]
                                = lastSqn[osdpBuff[wIdx].header.pd_addr];
                        lastSqn[osdpBuff[wIdx].header.pd_addr]
                                = osdpBuff[wIdx].ctrl.sqn & 0x03;
                        osdpBuff[wIdx].revTime = UartLstRcvTick;
                        if (osdpBuff[wIdx].ctrl.scb == 0)//no block data
                            osdpBuff[wIdx].dataBlkLen
                                = pktlen - 7 - osdpBuff[wIdx].ctrl.cksum;
                        else//with block data
                            osdpBuff[wIdx].dataBlkLen
                                = pktlen - 7 - osdpBuff[wIdx].ctrl.cksum - osdpBuff[wIdx].payload[1];
                        res = MA_OK;
                    }
                    osdpBuff[wIdx].buffIdx = 0;
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
    if ((rIdx == wIdx) && (odspBuffFull == false)) return;

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
        lastRcvlen[pdAddr] =
                (((UL32) osdpBuff[rIdx].header.len_msb << 8) & 0xFF00)
                | (0x00FF & osdpBuff[rIdx].header.len_lsb);
        memcpy((U8*) & lastRcvPkt[pdAddr], (U8*) & osdpBuff[rIdx], (size_t) lastRcvlen[pdAddr]);
    }// ----- SQN is the same as previous one --------
    else if (currSqn == lastSqn) {
        //Just resend the last sent packet without any further process
        //check if this received packet is exactly the same as shadow packet (last received packet that served before) then resend last reply again
        if (memcmp((U8*) & lastRcvPkt[pdAddr],
                (U8*) & osdpBuff[rIdx],
                (size_t) lastRcvlen[pdAddr]) == 0) {
            ep_osdpSend((U8*) & pktForSend[pdAddr], lastSentlen[pdAddr]); //Resend the last sent reply(its command is processed befor)
            //no need to update lastSentlen because its a resend
        } else //SQN error
        {
            // for debug
        }
    } else //SQN error
    {
        // for debug
    }
    //Update Read index
    rIdx = (rIdx + 1) % OSDP_PCKTDESCRPTR_BUF_NUM;
    odspBuffFull = false;
    if (IntLocalyDisabled == true) enableTMR0Int();
}

void ep_osdpProcess(EPS_OSDP_PACKET *pBuff) {
    U8 cmd, k;
    U8 pdAddr;
    UI16 i, len, rcvlen;
    US16 j;

    pdAddr = pBuff->header.pd_addr;
    if (pBuff->ctrl.scb) cmd = *((U8*) pBuff + 5 + pBuff->payload[0]);
    else cmd = *((U8*) pBuff + 5);

    rcvlen = (U8) ((UI16) pBuff->header.len_msb << 8 & 0xff00)
            | (0x00ff & (UI16) pBuff->header.len_lsb);

    //    memset(pktForSend, 0 sizeof (pktForSend));

    pktForSend[pdAddr].header.som = 0x53;
    pktForSend[pdAddr].header.pd_addr = pdAddr | 0x80;
    pktForSend[pdAddr].ctrl.sqn = 0; //there is no specified rule for SQN from PD to CP
    pktForSend[pdAddr].ctrl.cksum = pBuff->ctrl.cksum;
    pktForSend[pdAddr].ctrl.scb = 0;
    pktForSend[pdAddr].ctrl.na = 0; //Always it has to be 0
    pktForSend[pdAddr].ctrl.multi = 0;

    switch (cmd) {
        case OSDP_POLL:
            //----- check if any wiegand data is ready for consumption (sendig over OSDP here) ------
            if (pdAddr == OSDP_PD_ADDR_1STREADER) k = 0;
            else if (pdAddr == OSDP_PD_ADDR_2NDREADER) k = 1;

            //--- when any change in tampers,just report back the tamper status as poll response (check out "OSDP_LSTAT" response)
            if ((tmpr1Change == 1 && k == 0) || (tmpr2Change == 1 && k == 1)) {

                i = 0;
                pktForSend[pdAddr].payload[++i] = OSDP_LSTATR; //Reply Code
                // -------------  Data Block construction ------------
                if (k == 0) {
                    pktForSend[pdAddr].payload[++i] = ((PrvTmpr1 == 0) ? 0 : 1); //Tamper status
                    tmpr1Change = 0;
                }
                if (k == 1) {
                    pktForSend[pdAddr].payload[++i] = ((PrvTmpr2 == 0) ? 0 : 1); //Tamper status
                    tmpr2Change = 0;
                }
                pktForSend[pdAddr].payload[++i] = 0; //Power status.0 means normal

            }//---- sending back card CSN -----
                //there was data from one of the wiegands for consumption
            else if ((j = ep_wiegaGet(&pktForSend[pdAddr].payload[6], k))
                    && ((getCurrentTick() < (CardFlashTick[k] + CARDFLASH_TIMEOUT))
                    || (CardFlashTimeoutEnbled == false))) {
                i = 0;
                pktForSend[pdAddr].payload[++i] = OSDP_RAW; //Reply Code for Reader Data
                pktForSend[pdAddr].payload[++i] = k; //Reader Number
                pktForSend[pdAddr].payload[++i] = 0x01; //Wiegand data type
                pktForSend[pdAddr].payload[++i] = (U8) j; //LSB of bit count
                pktForSend[pdAddr].payload[++i] = 0x00; //MSB of bit count
                //data section (CSN here) is already filled before by Wieg_Get function.just updating i
                i += (((j - 1) / 8) + 1);

            } else//just send ACK back
            {
                i = 0; //i=0 is for CTRL field
                pktForSend[pdAddr].payload[++i] = OSDP_ACK; //Reply Code
                // -------------  Data Block construction ------------
                //nothing in data block required
                //(i-1) is data block len
            }
            //(i-1) is data block len
            len = (i - 1) + 7 + pktForSend[pdAddr].ctrl.cksum; //total len with CRC/Chksum . (i-1) is for data section of the reply
            pktForSend[pdAddr].header.len_msb = (U8) (len >> 8);
            pktForSend[pdAddr].header.len_lsb = (U8) len;
            // ChkSum/CRC calculation
            j = CRC_ChkSum_Calc((S8*) & pktForSend[pdAddr], len - 1 - pktForSend[pdAddr].ctrl.cksum, pktForSend[pdAddr].ctrl.cksum);
            pktForSend[pdAddr].payload[++i] = (U8) j;
            pktForSend[pdAddr].payload[++i] = (U8) (j >> 8);

            ep_osdpSend((U8*) & pktForSend[pdAddr], len);
            lastSentlen[pdAddr] = len;
            break;
        case OSDP_ID:
            pktForSend[pdAddr].payload[1] = OSDP_PDID; //Reply Code

            // -----------  Data Block construction ------------

            //As MCC18 dosnt support sscanf, we are going to use separate numbers of the EP_OUI
            /*
            sscanf((const S8*)EP_OUI,"%2x%2x%2x",
                   (int*)&pktForSend[pdAddr].payload[2],
                   (int*)&pktForSend[pdAddr].payload[3],
                   (int*)&pktForSend[pdAddr].payload[4]);  
             */
            pktForSend[pdAddr].payload[2] = EP_OUI_1STOCTET;
            pktForSend[pdAddr].payload[3] = EP_OUI_2NDOCTET;
            pktForSend[pdAddr].payload[4] = EP_OUI_3RDOCTET;


            ////pktForSend.OSDP_Payload[5]=EP_MODEL_NUM;
            pktForSend[pdAddr].payload[5] = E2P_Configs.ModelNo;
            ////pktForSend.OSDP_Payload[6]=EP_PRODUCT_VER;
            pktForSend[pdAddr].payload[6] = E2P_Configs.ProductVer;

            pktForSend[pdAddr].payload[7] = (U8) (E2P_Configs.Serial_Num); //LSB first
            pktForSend[pdAddr].payload[8] = (U8) (E2P_Configs.Serial_Num >> 8);
            pktForSend[pdAddr].payload[9] = (U8) (E2P_Configs.Serial_Num >> 16);
            pktForSend[pdAddr].payload[10] = (U8) (E2P_Configs.Serial_Num >> 24);

            //As MCC18 dosnt support sscanf, we are going to use separate numbers of the EP_FW_VER
            /*
            sscanf((const S8*)EP_FW_VER,"%d.%d.%d",
                   (int*)&pktForSend[pdAddr].payload[11],
                   (int*)&pktForSend[pdAddr].payload[12],
                   (int*)&pktForSend[pdAddr].payload[13]);  
             */
            pktForSend[pdAddr].payload[11] = EP_FW_VER_X; //Firmware version major
            pktForSend[pdAddr].payload[12] = EP_FW_VER_Y; //Firmware version minor
            pktForSend[pdAddr].payload[13] = EP_FW_VER_Z; //Firmware version bug

            i = 13;
            //(i-1) is data block len
            len = (i - 1) + 7 + pktForSend[pdAddr].ctrl.cksum; //total len with CRC/Chksum . (i-1) is for data section of the reply
            pktForSend[pdAddr].header.len_msb = (U8) (len >> 8);
            pktForSend[pdAddr].header.len_lsb = (U8) len;
            // ChkSum/CRC calculation
            j = CRC_ChkSum_Calc((S8*) & pktForSend[pdAddr], len - 1 - pktForSend[pdAddr].ctrl.cksum, pktForSend[pdAddr].ctrl.cksum);
            pktForSend[pdAddr].payload[++i] = (U8) j;
            pktForSend[pdAddr].payload[++i] = (U8) (j >> 8);
            ep_osdpSend((U8*) & pktForSend[pdAddr], len);
            lastSentlen[pdAddr] = len;
            break;
        case OSDP_COMSET:
            //if(OSDP_scs.OSDP_SCS_State == STT_SCS_OK)// we only accept comunication and configuration settings in secure channel mde
            //{
            //00 00 12 34 56 78 00 80 25 00 00 30 31 32 33 34 35 36 37 38 39 3A 3B 3C 3D 3E 3F 80 00 00 00 00
            if (pBuff->dataBlkLen == (sizeof (EPS_E2P_CONFIGS) - 1)) //we need exactly E2p configuration structure size minus one byte(for freshness)
            {
                //gathering config data for saving into E2prom
                E2P_Configs.E2P_Freshness = 0x55;

                //OSDP_Payload[1] and OSDP_Payload[2] are SEC_BLK_LEN and SEC_BLK_TYPE (in case of secure channel comunication.DRB dosnt support secure channel)
                E2P_Configs.ModelNo = pBuff->payload[2];
                E2P_Configs.ProductVer = pBuff->payload[3];
                E2P_Configs.Serial_Num = (((UL32) pBuff->payload[4] & 0x000000FF) << 0) |
                        (((UL32) pBuff->payload[5] & 0x000000FF) << 8) |
                        (((UL32) pBuff->payload[6] & 0x000000FF) << 16) |
                        (((UL32) pBuff->payload[7] & 0x000000FF) << 24);

                //E2P_Configs.OSDP.PD_Address=pBuff->OSDP_Packet.payload[8];//change address to new one
                E2P_Configs.OSDP.PD_Address = OSDP_PD_ADDR_1STREADER; //we wont replace the address in case of wiegand to 485 converter

                //in wiegand to 485 converter ,the baud rate has to be fixed on DEFAULTCONFIG_BAUDRATE
                /*
                E2P_Configs.OSDP.Baud_Rate=((pBuff->OSDP_Packet.payload[9]  & 0x000000FF)<<0 )  |
                                           ((pBuff->OSDP_Packet.payload[10] & 0x000000FF)<<8  ) |
                                           ((pBuff->OSDP_Packet.payload[11] & 0x000000FF)<<16 ) |
                                           ((pBuff->OSDP_Packet.payload[12] & 0x000000FF)<<24 );

                if(E2P_Configs.OSDP.Baud_Rate > OSDP_MAX_BAUDRATE)  //<<BUG2>>.check bug list
                    E2P_Configs.OSDP.Baud_Rate = OSDP_MAX_BAUDRATE;
                 */
                E2P_Configs.OSDP.Baud_Rate = DEFAULTCONFIG_BAUDRATE; //we wont replace the baud rate in case of wiegand to 485 converter

                //not using secure channel for wiehgand to 485 converter
                //for(i=0;i<16;i++)
                //    E2P_Configs.OSDP.SCBK_D[i]=pBuff->OSDP_Packet.OSDP_Payload[13+i];

                //write config data on E2prom
                // we will do this later
                //                Eeprom_Write(EEPROM_I2C_ADRS, E2P_ADDR_CONFIG_START, (U8_t*) & E2P_Configs, sizeof (EPS_E2P_CONFIGS));


                // ---- send back the osdp_COM as response -----
                i = 0; //i=0 is for CTRL field
                pktForSend[pdAddr].payload[++i] = OSDP_COM; //Reply Code

                // -------------  Data Block construction ------------
                //send back the received parameters (just address and baud rate)
                pktForSend[pdAddr].payload[++i] = pdAddr; //for wiegand to 485,we send back default address(we couldnt update the address field)

                pktForSend[pdAddr].payload[++i] = (U8) (E2P_Configs.OSDP.Baud_Rate >> 0); //LSB first
                pktForSend[pdAddr].payload[++i] = (U8) (E2P_Configs.OSDP.Baud_Rate >> 8);
                pktForSend[pdAddr].payload[++i] = (U8) (E2P_Configs.OSDP.Baud_Rate >> 16);
                pktForSend[pdAddr].payload[++i] = (U8) (E2P_Configs.OSDP.Baud_Rate >> 24);

                //(i-1) is data block len
                len = (i - 1) + 7 + pktForSend[pdAddr].ctrl.cksum; //total len with CRC/Chksum . (i-1) is for data section of the reply
                pktForSend[pdAddr].header.len_msb = (U8) (len >> 8);
                pktForSend[pdAddr].header.len_lsb = (U8) len;
                // ChkSum/CRC calculation
                j = CRC_ChkSum_Calc((S8*) & pktForSend[pdAddr], len - 1 - pktForSend[pdAddr].ctrl.cksum, pktForSend[pdAddr].ctrl.cksum);
                pktForSend[pdAddr].payload[++i] = (U8) j;
                pktForSend[pdAddr].payload[++i] = (U8) (j >> 8);
                ep_osdpSend((U8*) & pktForSend[pdAddr], len);
                lastSentlen[pdAddr] = len;

            }//OF DataBlock_len is appropriate
            else//send nack because len is not accordingly
            {
                i = 0; //i=0 is for CTRL field
                pktForSend[pdAddr].payload[++i] = OSDP_NAK; //Reply Code
                // -------------  Data Block construction ------------
                pktForSend[pdAddr].payload[++i] = OSDP_ERR_CMD_LEN; //Nack error code for command len
                //(i-1) is data block len
                len = (i - 1) + 7 + pktForSend[pdAddr].ctrl.cksum; //total len with CRC/Chksum . (i-1) is for data section of the reply
                pktForSend[pdAddr].header.len_msb = (U8) (len >> 8);
                pktForSend[pdAddr].header.len_lsb = (U8) len;
                // ChkSum/CRC calculation
                j = CRC_ChkSum_Calc((S8*) & pktForSend[pdAddr], len - 1 - pktForSend[pdAddr].ctrl.cksum, pktForSend[pdAddr].ctrl.cksum);
                pktForSend[pdAddr].payload[++i] = (U8) j;
                pktForSend[pdAddr].payload[++i] = (U8) (j >> 8);
                ep_osdpSend((U8*) & pktForSend[pdAddr], len);
                lastSentlen[pdAddr] = len;
            }
            //}//of if this is a secure channel

            break;
        case OSDP_LSTAT:
            //Buzz(2700,10);//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
            i = 0; //i=0 is for CTRL field
            pktForSend[pdAddr].payload[++i] = OSDP_LSTATR; //Reply Code
            // -------------  Data Block construction ------------
            if (pdAddr == OSDP_PD_ADDR_1STREADER) {
                pktForSend[pdAddr].payload[++i] = ((R1TMP == 0) ? 0 : 1); //Tamper status
                tmpr1Change = 0;
            }
            if (pdAddr == OSDP_PD_ADDR_2NDREADER) {
                pktForSend[pdAddr].payload[++i] = ((R2TMP == 0) ? 0 : 1); //Tamper status
                tmpr2Change = 0;
            }
            pktForSend[pdAddr].payload[++i] = 0; //Power status.0 means normal

            //(i-1) is data block len
            len = (i - 1) + 7 + pktForSend[pdAddr].ctrl.cksum; //total len with CRC/Chksum . (i-1) is for data section of the reply
            pktForSend[pdAddr].header.len_msb = (U8) (len >> 8);
            pktForSend[pdAddr].header.len_lsb = (U8) len;
            // ChkSum/CRC calculation
            j = CRC_ChkSum_Calc((S8*) & pktForSend[pdAddr], len - 1 - pktForSend[pdAddr].ctrl.cksum, pktForSend[pdAddr].ctrl.cksum);
            pktForSend[pdAddr].payload[++i] = (U8) j;
            pktForSend[pdAddr].payload[++i] = (U8) (j >> 8);
            ep_osdpSend((U8*) & pktForSend[pdAddr], len);
            lastSentlen[pdAddr] = len;
            break;
        case OSDP_LED:
            //Buzz(2700,10);//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
            /*      TempCode  OnTime  OffTime  OnColor  OffColor   TimeLsb  TimerMsb  PermanentCode  OnTime  OffTime   OnColor   OffColor
              00 00 02        05      0A       01       02         FF       FF        01             0A      0A        02        00
              00 00 02        05      0A       00       00         50       00        01             0A      0A        00        00
             */
            if (((pBuff->dataBlkLen) % 14 == 0) && //we need exactly factor of 14 bytes data block for each record
                    ((pBuff->dataBlkLen) != 0)) {

                if (pdAddr == OSDP_PD_ADDR_1STREADER) //CP asked for first reader LED control
                    k = 0; //check first reader to see if any data available
                else if (pdAddr == OSDP_PD_ADDR_2NDREADER) //CP asked for Second reader LED control
                    k = 1; //check second reader to see if any data available

                osdpLEDHandler(&pBuff->payload[2], k, pBuff->dataBlkLen / 14);

                i = 0; //i=0 is for CTRL field
                pktForSend[pdAddr].payload[++i] = OSDP_ACK; //Reply Code
                // -------------  Data Block construction ------------
                //nothing in data block required
                //(i-1) is data block len
                len = (i - 1) + 7 + pktForSend[pdAddr].ctrl.cksum; //total len with CRC/Chksum . (i-1) is for data section of the reply
                pktForSend[pdAddr].header.len_msb = (U8) (len >> 8);
                pktForSend[pdAddr].header.len_lsb = (U8) len;
                // ChkSum/CRC calculation
                j = CRC_ChkSum_Calc((S8*) & pktForSend[pdAddr], len - 1 - pktForSend[pdAddr].ctrl.cksum, pktForSend[pdAddr].ctrl.cksum);
                pktForSend[pdAddr].payload[++i] = (U8) j;
                pktForSend[pdAddr].payload[++i] = (U8) (j >> 8);
                ep_osdpSend((U8*) & pktForSend[pdAddr], len);
                lastSentlen[pdAddr] = len;
            } else//send nack because len is not accordingly
            {
                i = 0; //i=0 is for CTRL field
                pktForSend[pdAddr].payload[++i] = OSDP_NAK; //Reply Code
                // -------------  Data Block construction ------------
                pktForSend[pdAddr].payload[++i] = OSDP_ERR_CMD_LEN; //Nack error code for command len
                //(i-1) is data block len
                len = (i - 1) + 7 + pktForSend[pdAddr].ctrl.cksum; //total len with CRC/Chksum . (i-1) is for data section of the reply
                pktForSend[pdAddr].header.len_msb = (U8) (len >> 8);
                pktForSend[pdAddr].header.len_lsb = (U8) len;
                // ChkSum/CRC calculation
                j = CRC_ChkSum_Calc((S8*) & pktForSend[pdAddr], len - 1 - pktForSend[pdAddr].ctrl.cksum, pktForSend[pdAddr].ctrl.cksum);
                pktForSend[pdAddr].payload[++i] = (U8) j;
                pktForSend[pdAddr].payload[++i] = (U8) (j >> 8);

                ep_osdpSend((U8*) & pktForSend[pdAddr], len);
                lastSentlen[pdAddr] = len;
            }
            break;
        case OSDP_BUZ:
            /*      RdrNo   ToneCode   OnTime  OffTime   Count
                   00      02         01      1F        00
             */
            if (pBuff->dataBlkLen == 5) //we need exactly 5 bytes data block for OSDP_BUZ command
            {
                EPS_BUZCTRL* pBuzCtrl_Struct;

                pBuzCtrl_Struct = (EPS_BUZCTRL*) & pBuff->payload[2]; //[0] is packet CTRL    [1] is packet Command

                if (pBuzCtrl_Struct->ToneCode != 0 && pBuzCtrl_Struct->ToneCode < 3) {//if ToneCode is not NOP and also valid

                    //----- first cancel any related current buzer state -------
                    if (pdAddr == OSDP_PD_ADDR_1STREADER) //first reader buzzer control
                    {
                        k = 0;
                        R1D_BUZ = UNTRIGGER; // maybe we had not any sequencer but steady tone so we should stop it directly
                        i = Sqncr_Find_Index(BW_ENTY_1stBUZ);
                    } else if (pdAddr == OSDP_PD_ADDR_2NDREADER) //Second reader buzzer control
                    {
                        k = 1;
                        R2D_BUZ = UNTRIGGER; // maybe we had not any sequencer but steady tone so we should stop it directly
                        i = Sqncr_Find_Index(BW_ENTY_2ndBUZ);
                    }

                    if ((U8) i != 0xFF)//if there is one sequencer related to the buzzer cancel it
                    {
                        //sprintf(GlobalSharedBuf,"\n\rBuzzer sequencer index to release=%i",i);//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
                        //EPF_DBGU_Printk(GlobalSharedBuf);//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
                        //                        StopSequencer(i);
                    }
                    // ---- now trigger buzzer accordingly -----
                    if (pBuzCtrl_Struct->ToneCode == 2 && pBuzCtrl_Struct->OnTime != 0) {
                        if (pBuzCtrl_Struct->OffTime != 0) {
                            if (pBuzCtrl_Struct->Count == 0)//count 0 means buzz iteration should be continued until next command (refer to 4.11 of OSDP)
                                i = 0xffff;
                            else
                                i = ((UI16) pBuzCtrl_Struct->Count) * ((UI16) ((UI16) pBuzCtrl_Struct->OnTime + (UL32) pBuzCtrl_Struct->OffTime))*100;

                            if (k == 0)
                                StartSqncr(BW_ENTY_1stBUZ, (UL32) i, ((UL32) pBuzCtrl_Struct->OnTime) * 100, ((UL32) pBuzCtrl_Struct->OffTime) * 100, 1);
                            else if (k == 1)
                                StartSqncr(BW_ENTY_2ndBUZ, (UL32) i, ((UL32) pBuzCtrl_Struct->OnTime) * 100, ((UL32) pBuzCtrl_Struct->OffTime) * 100, 1);

                            //sprintf(GlobalSharedBuf,"\n\rBuzzer sequencer index to start=%i",j);//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
                            //EPF_DBGU_Printk(GlobalSharedBuf);//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
                        } else //if off time is 0 means steady tone until next command
                        {
                            if (k == 0)
                                R1D_BUZ = TRIGGER;
                            else if (k == 1)
                                R2D_BUZ = TRIGGER;
                        }
                    }
                }//END OF if ToneCode is not NOP
                i = 0; //i=0 is for CTRL field
                pktForSend[pdAddr].payload[++i] = OSDP_ACK; //Reply Code
                // -------------  Data Block construction ------------
                //nothing in data block required
                //(i-1) is data block len
                len = (i - 1) + 7 + pktForSend[pdAddr].ctrl.cksum; //total len with CRC/Chksum . (i-1) is for data section of the reply
                pktForSend[pdAddr].header.len_msb = (U8) (len >> 8);
                pktForSend[pdAddr].header.len_lsb = (U8) len;
                // ChkSum/CRC calculation
                j = CRC_ChkSum_Calc((S8*) & pktForSend[pdAddr], len - 1 - pktForSend[pdAddr].ctrl.cksum, pktForSend[pdAddr].ctrl.cksum);
                pktForSend[pdAddr].payload[++i] = (U8) j;
                pktForSend[pdAddr].payload[++i] = (U8) (j >> 8);
                ep_osdpSend((U8*) & pktForSend[pdAddr], len);
                lastSentlen[pdAddr] = len;
            } else//send nack because len is not accordingly
            {
                i = 0; //i=0 is for CTRL field
                pktForSend[pdAddr].payload[++i] = OSDP_NAK; //Reply Code
                // -------------  Data Block construction ------------
                pktForSend[pdAddr].payload[++i] = OSDP_ERR_CMD_LEN; //Nack error code for command len
                //(i-1) is data block len
                len = (i - 1) + 7 + pktForSend[pdAddr].ctrl.cksum; //total len with CRC/Chksum . (i-1) is for data section of the reply
                pktForSend[pdAddr].header.len_msb = (U8) (len >> 8);
                pktForSend[pdAddr].header.len_lsb = (U8) len;
                // ChkSum/CRC calculation
                j = CRC_ChkSum_Calc((S8*) & pktForSend[pdAddr], len - 1 - pktForSend[pdAddr].ctrl.cksum, pktForSend[pdAddr].ctrl.cksum);
                pktForSend[pdAddr].payload[++i] = (U8) j;
                pktForSend[pdAddr].payload[++i] = (U8) (j >> 8);
                ep_osdpSend((U8*) & pktForSend[pdAddr], len);
                lastSentlen[pdAddr] = len;
            }
            break;
        default:
            break;
    }
}

void osdpLEDHandler(void* p1stRecAddress, U8 ReaderNo, UL32 RecNumber) {
}

void osdpPollHandler() {
}