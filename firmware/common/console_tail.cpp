#include <string.h>
#include <stdio.h>

#include "console_tail.h"

void strrev(char *target, const char* source, size_t size);				// reverse a string. Not C null terminated

uint16_t ConsoleChecksum(uint16_t sum, const uint8_t* buf, uint16_t size) {
	for (size_t i = 0; i < size; ++i) {
		sum += *buf;
		++buf;
	}

	return sum;
}

// Stringify macro into string literal
#define stringify(a) _stringify(a)
#define _stringify(a) #a

uint8_t ValidateCommandLine(const char* source, size_t size) {
	static char reverStr[768];		// Firmware has very small stack. so do not define them on stack
	static char buf[16];
	static char tailLengthStr[TailLenSize + 1];
	static int tailLength;
        
	//============   Special case, we don't validate base-64 packet   ============
	if (size < 5) {
		return 0;
	}

	if (memcmp(source, "U 0 0", 5) == 0) {
		return 1;
	}
	//=============================================================================

	if (size < sizeof(struct TAIL_FIXED_STRUCT0) + sizeof(struct TAIL_FIXED_STRUCT1) + 1) {		// 1 means "splitter"
		return 0;
	}

	strrev(reverStr, source, size);

	struct TAIL_FIXED_STRUCT0* tail0 = (struct TAIL_FIXED_STRUCT0*)(reverStr);
	struct TAIL_FIXED_STRUCT1* tail1 = (struct TAIL_FIXED_STRUCT1*)(reverStr + sizeof(struct TAIL_FIXED_STRUCT0));

	if (memcmp(tail0->signature, TailSignatureStr, TailSignatureSize) != 0) {
		return 0;
	}

	if (tail0->version < TailVersion) {	// version compatibility
		return 0;
	}

	uint16_t sum = ConsoleChecksum(0, (uint8_t*)reverStr + sizeof(struct TAIL_FIXED_STRUCT0), size - sizeof(struct TAIL_FIXED_STRUCT0));
	
	sprintf(buf, "%0" stringify(TailCheckSumSize) "X", sum);
	if (strlen(buf) != TailCheckSumSize) {
		return 0;
	}

	if (memcmp(buf, tail0->checksum, TailCheckSumSize) != 0) {
		return 0;
	}

	memcpy(tailLengthStr, tail1->tailLength, TailLenSize);
	tailLengthStr[TailLenSize] = 0;

	int num = sscanf(tailLengthStr, "%d", &tailLength);

	if (num < 1) {
		return 0;
	}

	if (tailLength < (sizeof(struct TAIL_FIXED_STRUCT0) + sizeof(struct TAIL_FIXED_STRUCT1))) {
		return 0;
	}

	if (tailLength > size) {
		return 0;
	}

	if (reverStr[tailLength] != ' ') {
		return 0;
	}

	return 1;
}

// reverse a string. Not C null terminated
void strrev(char *target, const char* source, size_t size) {
	for (size_t i = 0; i < size; ++i) {
		target[i] = source[size - 1 - i];
	}
}

