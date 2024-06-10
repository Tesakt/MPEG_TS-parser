#pragma once
#include "tsCommon.h"
#include <string>
#include <vector>

/*
MPEG-TS packet:
`        3                   2                   1                   0  `
`      1 0 9 8 7 6 5 4 3 2 1 0 9 8 7 6 5 4 3 2 1 0 9 8 7 6 5 4 3 2 1 0  `
`     +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+ `
`   0 |                             Header                            | `
`     +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+ `
`   4 |                  Adaptation field + Payload                   | `
`     |                                                               | `
` 184 |                                                               | `
`     +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+ `


MPEG-TS packet header:
`        3                   2                   1                   0  `
`      1 0 9 8 7 6 5 4 3 2 1 0 9 8 7 6 5 4 3 2 1 0 9 8 7 6 5 4 3 2 1 0  `
`     +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+ `
`   0 |       SB      |E|S|T|           PID           |TSC|AFC|   CC  | `
`     +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+ `

Sync byte                    (SB ) :  8 bits
Transport error indicator    (E  ) :  1 bit
Payload unit start indicator (S  ) :  1 bit
Transport priority           (T  ) :  1 bit
Packet Identifier            (PID) : 13 bits
Transport scrambling control (TSC) :  2 bits
Adaptation field control     (AFC) :  2 bits
Continuity counter           (CC ) :  4 bits
*/


//=============================================================================================================================================================================

class xTS
{
public:
  static constexpr uint32_t TS_PacketLength  = 188;
  static constexpr uint32_t TS_HeaderLength  = 4;

  static constexpr uint32_t PES_HeaderLength = 6;

  static constexpr uint32_t BaseClockFrequency_Hz         =    90000; //Hz
  static constexpr uint32_t ExtendedClockFrequency_Hz     = 27000000; //Hz
  static constexpr uint32_t BaseClockFrequency_kHz        =       90; //kHz
  static constexpr uint32_t ExtendedClockFrequency_kHz    =    27000; //kHz
  static constexpr uint32_t BaseToExtendedClockMultiplier =      300;
};

//=============================================================================================================================================================================

class xTS_PacketHeader
{
public:
  enum class ePID : uint16_t
  {
    PAT  = 0x0000,
    CAT  = 0x0001,
    TSDT = 0x0002,
    IPMT = 0x0003,
    NIT  = 0x0010, //DVB specific PID
    SDT  = 0x0011, //DVB specific PID
    NuLL = 0x1FFF,
  };

protected:
  uint8_t  m_SB; // Sync byte
  bool     m_E;  // Transport error indicator
  bool     m_S;  // Payload unit start indicator
  bool     m_T;  // Transport priority
  uint16_t m_PID;// Packet Identifier
  uint8_t  m_TSC;// Transport scrambling control
  uint8_t  m_AFC;// Adaptation field control
  uint8_t  m_CC; // Continuity counter
  ePID     m_PIDType; // Typ PID

public:
  void     Reset();
  int32_t  Parse(const std::vector<uint8_t>& Input);
  void     Print() const;

public:
  uint8_t  getSyncByte() const { return m_SB; }  
  bool     hasTransportError() const { return m_E; }
  bool     isPayloadStart() const { return m_S; }
  bool     hasTransportPriority() const { return m_T; }
  bool     hasAdaptationField() const { return (m_AFC == 2 || m_AFC == 3) ? true : false; }
  uint8_t  getAdaptationFieldControl() const { return m_AFC; }
  uint16_t getPID() const { return m_PID; }
  uint8_t  getTSC() const { return m_TSC; }
  uint8_t  getAFC() const { return m_AFC; }
  uint8_t  getCC() const { return m_CC; }
  ePID     getPIDType() const { return m_PIDType; }
};

//=============================================================================================================================================================================

class xTS_AdaptationField
{
protected:
    //setup
    uint8_t m_AdaptationFieldControl;
    //mandatory fields
    uint8_t m_AdaptationFieldLength;
    //optional fields - PCR
    bool m_DC;
    bool m_RA;
    bool m_SP;
    bool m_PR; // Program Clock Reference flag
    bool m_OR;
    bool m_SF;
    bool m_TP;
    bool m_EX;
    uint32_t m_PCR;
    // the time encoded in the PCR field measured in units of the period of the 27 MHz system clock
    // where i is the byte index of the final byte of the program_clock_reference_base field
    float m_time;

public:
    void    Reset();
    int32_t Parse(const std::vector<uint8_t>& PacketBuffer, uint8_t AdaptationFieldControl);
    void    Print() const;

public:
    //mandatory fields
    uint8_t getAdaptationFieldLength () const { return m_AdaptationFieldLength ; }
    //derived values
    uint32_t getNumBytes () const { return m_AdaptationFieldLength + 1; }
};

//=============================================================================================================================================================================

class xPES_PacketHeader
{
  public:
    enum eStreamId : uint8_t
    {
      eStreamId_program_stream_map = 0xBC,
      eStreamId_padding_stream = 0xBE,
      eStreamId_private_stream_2 = 0xBF,
      eStreamId_ECM = 0xF0,
      eStreamId_EMM = 0xF1,
      eStreamId_program_stream_directory = 0xFF,
      eStreamId_DSMCC_stream = 0xF2,
      eStreamId_ITUT_H222_1_type_E = 0xF8,
    };

  protected:
    //PES packet header
    int m_HeaderLength;
    uint32_t m_PacketStartCodePrefix;
    uint8_t m_StreamId;
    uint16_t m_PacketLength;
    uint8_t m_PTS_DTS;
    uint64_t m_PresentationTimeStamp;
    uint64_t m_DecodeTimeStamp;
    float m_PTS_time;
    float m_DTS_time;
    // Extension header
    bool m_ESCR_flag;
    bool m_ES_rate_flag;
    bool m_DSM_trick_mode_flag;
    bool m_additional_copy_info_flag;
    bool m_PES_CRC_flag;
    bool m_PES_extension_flag;
    // Extended header

  public:
    void Reset();
    int32_t Parse(const std::vector<uint8_t>& Input, int32_t Offset);
    void Print() const;

  public:
    //PES packet header
    int getHeaderLength() const {return m_HeaderLength; }
    uint32_t getPacketStartCodePrefix() const { return m_PacketStartCodePrefix; }
    uint8_t getStreamId () const { return m_StreamId; }
    uint16_t getPacketLength () const { return m_PacketLength; }
};

//=============================================================================================================================================================================

class xPES_Assembler
{
  public:
    xPES_Assembler(std::string FileName) : m_FileName(FileName) {}

    enum class eResult : int32_t
    {
      UnexpectedPID = 1,
      StreamPackedLost ,
      AssemblingStarted ,
      AssemblingContinue,
      AssemblingFinished,
    };

  protected:

    int32_t m_PID;
    //buffer
    std::vector<uint8_t> m_Buffer;
    uint32_t m_BufferSize;
    uint32_t m_DataOffset;
    //operation
    int8_t m_LastContinuityCounter;
    bool m_Started;
    xPES_PacketHeader m_PESH;
    std::string m_FileName;

  public:
    void Init (int32_t PID);
    eResult AbsorbPacket(const std::vector<uint8_t> TransportStreamPacket, const xTS_PacketHeader* PacketHeader, const xTS_AdaptationField* AdaptationField);
    void PrintPESH () const { m_PESH.Print(); }
    std::vector<uint8_t> getPacket () { return m_Buffer; }
    int32_t getNumPacketBytes() const { return m_BufferSize + m_PESH.getHeaderLength(); }
    int getHeaderLength() const { return m_PESH.getHeaderLength(); }
    void PrepareFile(std::string FileName);
    void WriteFile();

  protected:
    void xBufferReset ();
    void xBufferAppend(const std::vector<uint8_t>& Data, int32_t Size);
};