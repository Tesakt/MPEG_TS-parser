#include "tsTransportStream.h"
#include <cstdio>
#include <fstream>
#include <vector>

//=============================================================================================================================================================================
// xTS_PacketHeader
//=============================================================================================================================================================================


/// @brief Reset - reset all TS packet header fields
void xTS_PacketHeader::Reset()
{
    m_SB = 0;
    m_E = false;
    m_S = false;
    m_T = false;
    m_PID = 0;
    m_TSC = 0;
    m_AFC = 0;
    m_CC = 0;
}

/**
  @brief Parse all TS packet header fields
  @param Input is pointer to buffer containing TS packet
  @return Number of parsed bytes (4 on success, -1 on failure) 
 */
int32_t xTS_PacketHeader::Parse(const std::vector<uint8_t>& Input)
{
    if (Input.empty())
        return NOT_VALID;

    m_SB = Input[0];
    m_E = (Input[1] & 0x80) != 0;
    m_S = (Input[1] & 0x40) != 0;
    m_T = (Input[1] & 0x20) != 0;
    m_PID = ((Input[1] & 0x1F) << 8) | Input[2];
    m_TSC = (Input[3] >> 6) & 0x03;
    m_AFC = (Input[3] >> 4) & 0x03;
    m_CC = Input[3] & 0x0F;

    return 4;
}

void xTS_PacketHeader::Print() const
{
    printf("TS: SB=%02d E=%d S=%d P=%d PID=%5d TSC=%d AF=%d CC=%2d",
           m_SB,
           m_E,
           m_S,
           m_T,
           m_PID,
           m_TSC,
           m_AFC,
           m_CC);

    return;
}

//=============================================================================================================================================================================
// xTS_AdaptationField
//=============================================================================================================================================================================

void xTS_AdaptationField::Reset()
{
    m_AdaptationFieldControl = 0;
    m_AdaptationFieldLength = 0;
    m_RA = false;
    m_DC = false;
    m_SP = false;
    m_PR = false;
    m_OR = false;
    m_SF = false;
    m_TP = false;
    m_EX = false;
    m_PCR = 0;
    return;
}

/**
@brief Parse adaptation field
@param PacketBuffer is pointer to buffer containing TS packet
@param AdaptationFieldControl is value of Adaptation Field Control field of
corresponding TS packet header
@return Number of parsed bytes (length of AF or -1 on failure)
*/

int32_t xTS_AdaptationField::Parse(const std::vector<uint8_t>& PacketBuffer, uint8_t AdaptationFieldControl)
{
    m_AdaptationFieldControl = AdaptationFieldControl;
    xTS m_xTS;

    if (PacketBuffer.empty())
        return NOT_VALID;

    m_AdaptationFieldLength = PacketBuffer[4];
    m_RA = (PacketBuffer[5] & 0x80) != 0;
    m_DC = (PacketBuffer[5] & 0x40) != 0;     
    m_SP = (PacketBuffer[5] & 0x20) != 0;
    m_PR = (PacketBuffer[5] & 0x10) != 0;
    m_OR = (PacketBuffer[5] & 0x08) != 0;
    m_SF = (PacketBuffer[5] & 0x04) != 0;
    m_TP = (PacketBuffer[5] & 0x02) != 0;
    m_EX = (PacketBuffer[5] & 0x01) != 0;

    if (m_PR == true)
    {
        // Pierwsze 33 bity po polu kontrolnym to pola PCR
        m_PCR |= static_cast<uint64_t>(PacketBuffer[6]) << 25;
        m_PCR |= static_cast<uint64_t>(PacketBuffer[7]) << 17;
        m_PCR |= static_cast<uint64_t>(PacketBuffer[8]) << 9;
        m_PCR |= static_cast<uint64_t>(PacketBuffer[9]) << 1;
        m_PCR |= static_cast<uint64_t>(PacketBuffer[10]) >> 7;
        m_PCR *= m_xTS.BaseToExtendedClockMultiplier;

        m_time = static_cast<float>(m_PCR) / m_xTS.ExtendedClockFrequency_Hz;
    }
    return m_AdaptationFieldLength;
}


void xTS_AdaptationField::Print() const
{

    printf("           AF: L=%3d DC=%d RA=%d SP=%d PR=%d OR=%d SF=%d TP=%d EX=%d",
    m_AdaptationFieldLength,
    m_RA,
    m_DC,
    m_SP,
    m_PR,
    m_OR,
    m_SF,
    m_TP,
    m_EX);

    if (m_PR == true)
        printf(" PCR=%d (Time=%fs) Stuffing=0",
        m_PCR,
        m_time);
    else
        printf(" Stuffing=%d", m_AdaptationFieldLength - 1);
}


//=============================================================================================================================================================================
// xPES_PacketHeader
//=============================================================================================================================================================================


void xPES_PacketHeader::Reset() {
    m_PacketStartCodePrefix = 0;
    m_StreamId = 0;
    m_PacketLength = 0;
    m_PTS_DTS = 0;
    m_PresentationTimeStamp = 0;
    m_PTS_time = 0;
    m_DecodeTimeStamp = 0;
    m_DTS_time = 0;
    // Extension header flags
    m_ESCR_flag = 0;
    m_ES_rate_flag = 0;
    m_DSM_trick_mode_flag = 0;
    m_additional_copy_info_flag = 0;
    m_PES_CRC_flag = 0;
    m_PES_extension_flag = 0;

    return;
}

int32_t xPES_PacketHeader::Parse(const std::vector<uint8_t>& Data, int32_t Offset) {
    if (Data.empty())
        return NOT_VALID;
    std::vector<uint8_t> Input(Data.begin() + Offset, Data.end());
    xTS m_xTS;

    m_PacketStartCodePrefix = (static_cast<uint32_t>(Input[0]) << 16) | (static_cast<uint32_t>(Input[1]) << 8) | static_cast<uint32_t>(Input[2]);
    m_StreamId              = Input[3];
    m_PacketLength          = (static_cast<uint16_t>(Input[4]) << 8) | (static_cast<uint16_t>(Input[5]));
    m_PTS_DTS               = Input[7] >> 6;

    m_ESCR_flag = (Input[7] & 0x20) != 0;
    m_ES_rate_flag = (Input[7] & 0x10) != 0;
    m_DSM_trick_mode_flag = (Input[7] & 0x08) != 0;
    m_additional_copy_info_flag = (Input[7] & 0x04) != 0;
    m_PES_CRC_flag = (Input[7] & 0x02) != 0;
    m_PES_extension_flag = (Input[7] & 0x01) != 0;
    
    m_HeaderLength = 9 + (Input[8]);

    if (m_PTS_DTS == 0x02) { // PTS = 1, DTS = 0
        m_PresentationTimeStamp = (static_cast<uint32_t>(Input[9] & 0x0E) << 30) |  // 0x0E = 00001110
                                  (static_cast<uint32_t>(Input[10]) << 22) |        // 0xFF = 11111111
                                  (static_cast<uint32_t>(Input[11] & 0xFE) << 15) | // 0xFE = 11111110
                                  (static_cast<uint32_t>(Input[12]) << 7) |         // 0xFF = 11111111
                                  (static_cast<uint32_t>(Input[13] & 0xFE) >> 1);   // 0xFE = 11111110
        m_PTS_time = static_cast<float>(m_PresentationTimeStamp) / m_xTS.BaseClockFrequency_Hz;

    } else if (m_PTS_DTS == 0x01) { // PTS = 0, DTS = 1
        m_DecodeTimeStamp = (static_cast<uint32_t>(Input[9] & 0x0E) << 30) |        // 0x0E = 00001110
                                  (static_cast<uint32_t>(Input[10]) << 22) |        // 0xFF = 11111111
                                  (static_cast<uint32_t>(Input[11] & 0xFE) << 15) | // 0xFE = 11111110
                                  (static_cast<uint32_t>(Input[12]) << 7) |         // 0xFF = 11111111
                                  (static_cast<uint32_t>(Input[13] & 0xFE) >> 1);   // 0xFE = 11111110
        m_DTS_time = static_cast<float>(m_DecodeTimeStamp) / m_xTS.BaseClockFrequency_Hz;

    } else if (m_PTS_DTS == 0x03) { // PTS = 1, DTS = 1
        m_PresentationTimeStamp = (static_cast<uint32_t>(Input[9] & 0x0E) << 30) |  
                                  (static_cast<uint32_t>(Input[10]) << 22) |
                                  (static_cast<uint32_t>(Input[11] & 0xFE) << 15) |
                                  (static_cast<uint32_t>(Input[12]) << 7) |
                                  (static_cast<uint32_t>(Input[13] & 0xFE) >> 1);
        m_PTS_time = static_cast<float>(m_PresentationTimeStamp) / m_xTS.BaseClockFrequency_Hz;

        m_DecodeTimeStamp = (static_cast<uint32_t>(Input[14] & 0x0E) << 30) |
                                  (static_cast<uint32_t>(Input[15]) << 22) |
                                  (static_cast<uint32_t>(Input[16] & 0xFE) << 15) |
                                  (static_cast<uint32_t>(Input[17]) << 7) |
                                  (static_cast<uint32_t>(Input[18] & 0xFE) >> 1);
        m_DTS_time = static_cast<float>(m_DecodeTimeStamp) / m_xTS.BaseClockFrequency_Hz;
    }

    return m_HeaderLength;
}

void xPES_PacketHeader::Print() const {
    printf("           PES: PSCP=%d SID=%d L=%d ", m_PacketStartCodePrefix, m_StreamId, m_PacketLength);

    if (m_PTS_DTS == 0x02 || m_PTS_DTS == 0x03) {
        printf("PTS=%d (Time=%fs) ", m_PresentationTimeStamp, m_PTS_time);
    } else if (m_PTS_DTS == 0x01) {
        printf("DTS=%d (Time=%fs) ", m_DecodeTimeStamp, m_DTS_time);
    }
    printf("\n");
    return;
}

//=============================================================================================================================================================================
// xPES_Assembler
//=============================================================================================================================================================================

void xPES_Assembler::xBufferReset() {
    m_LastContinuityCounter = -1;
    m_Started = false;
    m_DataOffset = 0;
    m_BufferSize = 0;
    m_Buffer.clear(); // Dodajemy czyszczenie wektora
    return;
}

void xPES_Assembler::Init (int32_t PID) {
    m_PID = PID;
    m_Buffer.clear();
    m_BufferSize = 0;
    m_DataOffset = 0;
    m_LastContinuityCounter = -1;
    m_Started = false;
    return;
}

void xPES_Assembler::xBufferAppend(const std::vector<uint8_t>& Data, int32_t Offset) {
    if (Data.empty() || Data.size() <= 0) {
        return;
    }
    // Dodajemy do wektora dane z bufora
    m_Buffer.insert(m_Buffer.end(), Data.begin() + Offset, Data.end());
    m_BufferSize += Data.size() - Offset;
    return;
}

xPES_Assembler::eResult xPES_Assembler::AbsorbPacket(const std::vector<uint8_t> TransportStreamPacket, const xTS_PacketHeader* PacketHeader, const xTS_AdaptationField* AdaptationField) {     

    // Check if the packet is the start of a new PES packet
    if (PacketHeader->isPayloadStart()) {
        if (PacketHeader->getPID() != m_PID) {
            Init(PacketHeader->getPID());
        }
        xBufferReset();
        m_Started = true;
        m_PESH.Reset();
        m_LastContinuityCounter = PacketHeader->getCC();

        int32_t PES_headerLength = m_PESH.Parse(TransportStreamPacket, xTS::TS_HeaderLength + AdaptationField->getNumBytes());

        if (PES_headerLength == NOT_VALID) {
            m_Started = false;
            return eResult::StreamPackedLost;
        }

        xBufferAppend(TransportStreamPacket, xTS::TS_HeaderLength + AdaptationField->getNumBytes() + PES_headerLength);
        
        m_LastContinuityCounter = PacketHeader->getCC();
        return eResult::AssemblingStarted;
    }

    if (PacketHeader->getCC() != ((m_LastContinuityCounter + 1) & 0x0F)) {
        m_LastContinuityCounter = PacketHeader->getCC();
        m_Started = false;
        return eResult::StreamPackedLost;
    }

    // Check if the packet is a continuation of a PES packet
    if (m_Started && !PacketHeader->hasAdaptationField()) {
        xBufferAppend(TransportStreamPacket, xTS::TS_HeaderLength);
        m_LastContinuityCounter = PacketHeader->getCC();
        return eResult::AssemblingContinue;
    }

    // Check if the packet is the end of a PES packet
    if (m_Started && PacketHeader->hasAdaptationField()) {
        xBufferAppend(TransportStreamPacket, xTS::TS_HeaderLength + AdaptationField->getNumBytes());
        m_LastContinuityCounter = PacketHeader->getCC();
        m_Started = false;
        if (m_PID == 136) { // Write only audio files
            WriteFile();
        }
        return eResult::AssemblingFinished;
    }

    m_Started = false;
    m_LastContinuityCounter = PacketHeader->getCC();

    return eResult::StreamPackedLost;
}


void xPES_Assembler::WriteFile() {
    // Append the buffer to the file
    std::ofstream file(m_FileName, std::ios::out | std::ios::app | std::ios::binary);
    file.write(reinterpret_cast<const char*>(m_Buffer.data()), m_Buffer.size());
    file.close();
    return;
}