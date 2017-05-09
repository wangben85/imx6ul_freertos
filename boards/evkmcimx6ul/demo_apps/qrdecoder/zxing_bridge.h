#ifndef __ZXING_BRIDGE_H__
#define __ZXING_BRIDGE_H__

#include <stdint.h>
#include <stdbool.h>


/* ZXING Lib API defination */

#define DECODE_QR       (0)
#define DECODE_1D       (1)


void set_region(int left, int top, int width, int height);
bool get_pixel(int x, int y);
void set_martix_data(int left, int top, int width, int height, uint8_t *data);
int cppInit(int width, int height);
void cppResetBinarizer(void);
uint8_t *cppGetResults(void);
int read_image(uint32_t option, char *format, char *result);
void cppProcessBlockRow(uint8_t *blockRow);
void extractLuminance(uint8_t *out, uint8_t *in, uint32_t count);


#endif
