#ifndef __CONSOLE_TAIL_H__
#define __CONSOLE_TAIL_H__

#include <stdint.h>

#pragma pack(push, 1)
#define TailSignatureSize 4
#define TailLenSize 3
#define TailCheckSumSize 4
#define TailMagicNumSize 4

#define TailSignatureStr "CaSt"
#define TailVersion '0'

#ifdef WIN32		// Do not use on firmware has firmware has to deal with variable struct with extra data
struct TAIL_FIXED_STRUCT_V0 {
	uint8_t signature[TailSignatureSize];			// Signature "CaSt"
	uint8_t version;								//
	uint8_t checksum[TailCheckSumSize];				// checksum: except '\r', version, signature and checksum itself
	uint8_t tailLength[TailLenSize];				// Tail length
	uint8_t magicNum[TailMagicNumSize];				// To indentify a connection
	uint8_t sequenceId;
	// Extra data can be extended here
};

struct TAIL_FIXED_STRUCT1_V0 {						
	uint8_t tailLength[TailLenSize];				// Tail length. Hex string
	uint8_t magicNum[TailMagicNumSize];				// To indentify a connection
	uint8_t sequenceId;								
	// Extra data can be extended here
};

uint16_t ConsoleChecksum(uint16_t sum, const uint8_t* buf, uint16_t size);
#endif

struct TAIL_FIXED_STRUCT0 {
	uint8_t signature[TailSignatureSize];			// Signature "CaSt"
	uint8_t version;								// default to '0'
	uint8_t checksum[TailCheckSumSize];				// checksum: except '\r', version, signature and checksum itself
};

struct TAIL_FIXED_STRUCT1 {
	uint8_t tailLength[TailLenSize];				// Tail length. Hex string
	uint8_t magicNum[TailMagicNumSize];				// To indentify a connection
	uint8_t sequenceId;								// To tell the correctness of packet sequence.
};

uint8_t ValidateCommandLine(const char* source, size_t size);

#pragma pack(pop)

#endif