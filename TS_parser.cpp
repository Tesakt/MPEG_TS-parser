#include "tsCommon.h"
#include "tsTransportStream.h"
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <vector>

//=============================================================================================================================================================================

int main(int argc, char* argv[], char* envp[])
{
    if (argc != 3) {
        printf("Usage: %s <input_file> <output_file>\n", argv[0]);
        return EXIT_FAILURE;
    }

    const char* inputFileName = argv[1];
    std::ifstream inputFile(inputFileName, std::ios::binary);
    if (!inputFile.is_open()) {
        printf("Failed to open input file: %s\n", inputFileName);
        return EXIT_FAILURE;
    }

    const char* outputFileName = argv[2];
    if (std::ofstream(outputFileName)) {
        std::remove(outputFileName);
    }

    int NumberOfPacketsLost = 0;

    xTS_PacketHeader TS_PacketHeader;
    xTS_AdaptationField TS_PacketAdaptationField;
    xPES_Assembler PES_Assembler(outputFileName);

    int32_t TS_PacketId = 0;
    std::vector<uint8_t> TS_PacketBuffer(xTS::TS_PacketLength);
    while (!(inputFile.eof())) {
        // Read TS packet from file
        inputFile.read(reinterpret_cast<char*>(TS_PacketBuffer.data()), xTS::TS_PacketLength);
        if (!inputFile) {
            printf("Error while reading from file\n");
            break;
        }

        // Parse TS packet header
        TS_PacketHeader.Reset();
        TS_PacketHeader.Parse(TS_PacketBuffer);
        
        TS_PacketAdaptationField.Reset();
        if (TS_PacketHeader.getSyncByte() == 'G' && TS_PacketHeader.getPID() == 136) {
            if (TS_PacketHeader.hasAdaptationField()) {
                TS_PacketAdaptationField.Parse(TS_PacketBuffer, TS_PacketHeader.getAdaptationFieldControl());
            }

            printf("%010d ", TS_PacketId);
            TS_PacketHeader.Print();

            if (TS_PacketHeader.hasAdaptationField()) { 
                printf("\n"); 
                TS_PacketAdaptationField.Print(); 
                printf("\n");
            }

            xPES_Assembler::eResult Result = PES_Assembler.AbsorbPacket(TS_PacketBuffer, &TS_PacketHeader, &TS_PacketAdaptationField);
            switch (Result) {
                case xPES_Assembler::eResult::StreamPackedLost:  
                    printf("\nPcktLost \n"); 
                    NumberOfPacketsLost++; 
                    break;
                case xPES_Assembler::eResult::AssemblingStarted: 
                    printf("\n           Assembling Started  \n"); 
                    PES_Assembler.PrintPESH(); 
                    break;
                case xPES_Assembler::eResult::AssemblingContinue: 
                    printf(" Assembling Continue \n"); 
                    break;
                case xPES_Assembler::eResult::AssemblingFinished: 
                    printf("           Assembling Finished \n"); 
                    printf("           PES: PcktLen=%d HeadLen=%d DataLen=%d\n", PES_Assembler.getNumPacketBytes(), PES_Assembler.getHeaderLength(), PES_Assembler.getNumPacketBytes() - PES_Assembler.getHeaderLength()); 
                    break;
                default: 
                    break;
            }
        }

        TS_PacketId++;
    }

    printf("Number of lost packets: %d\n", NumberOfPacketsLost);
    inputFile.close();

    return EXIT_SUCCESS;
}
