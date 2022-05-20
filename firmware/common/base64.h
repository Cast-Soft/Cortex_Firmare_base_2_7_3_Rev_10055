#ifndef __BASE64_H__
#define __BASE64_H__

#ifdef __cplusplus
// Encode. return in target and terminated by '\0'. target is the output buffer and targsize is buffer's size
// return: -1 error (buffer too small) or number of bytes in target
extern "C" 
#endif
int b64_ntop(unsigned char const *src, size_t srclength, char *target, size_t targsize);

// Decode. Source must be terminated by '\0'
// target is the output buffer and targsize is buffer's size
// return: -1 error (buffer too small) or number of bytes in target
/* skips all whitespace anywhere.
   converts characters, four at a time, starting at (or after)
   src from base - 64 numbers into three 8 bit bytes in the target area.
   it returns the number of data bytes stored at the target, or -1 on error.
 */
#ifdef __cplusplus
extern "C" 
#endif
int b64_pton(char const *src, unsigned char *target, size_t targsize);

#endif