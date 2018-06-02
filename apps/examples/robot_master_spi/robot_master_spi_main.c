/****************************************************************************
 * examples/robot_master_spi/robot_master_spi.c
 *
 *   Copyright (C) 2008, 2011-2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const unsigned char RMAP_CRCTable[] = {
0x00, 0x91, 0xe3, 0x72, 0x07, 0x96, 0xe4, 0x75,
0x0e, 0x9f, 0xed, 0x7c, 0x09, 0x98, 0xea, 0x7b,
0x1c, 0x8d, 0xff, 0x6e, 0x1b, 0x8a, 0xf8, 0x69,
0x12, 0x83, 0xf1, 0x60, 0x15, 0x84, 0xf6, 0x67,
0x38, 0xa9, 0xdb, 0x4a, 0x3f, 0xae, 0xdc, 0x4d,
0x36, 0xa7, 0xd5, 0x44, 0x31, 0xa0, 0xd2, 0x43,
0x24, 0xb5, 0xc7, 0x56, 0x23, 0xb2, 0xc0, 0x51,
0x2a, 0xbb, 0xc9, 0x58, 0x2d, 0xbc, 0xce, 0x5f,
0x70, 0xe1, 0x93, 0x02, 0x77, 0xe6, 0x94, 0x05,
0x7e, 0xef, 0x9d, 0x0c, 0x79, 0xe8, 0x9a, 0x0b,
0x6c, 0xfd, 0x8f, 0x1e, 0x6b, 0xfa, 0x88, 0x19,
0x62, 0xf3, 0x81, 0x10, 0x65, 0xf4, 0x86, 0x17,
0x48, 0xd9, 0xab, 0x3a, 0x4f, 0xde, 0xac, 0x3d,
0x46, 0xd7, 0xa5, 0x34, 0x41, 0xd0, 0xa2, 0x33,
0x54, 0xc5, 0xb7, 0x26, 0x53, 0xc2, 0xb0, 0x21,
0x5a, 0xcb, 0xb9, 0x28, 0x5d, 0xcc, 0xbe, 0x2f,
0xe0, 0x71, 0x03, 0x92, 0xe7, 0x76, 0x04, 0x95,
0xee, 0x7f, 0x0d, 0x9c, 0xe9, 0x78, 0x0a, 0x9b,
0xfc, 0x6d, 0x1f, 0x8e, 0xfb, 0x6a, 0x18, 0x89,
0xf2, 0x63, 0x11, 0x80, 0xf5, 0x64, 0x16, 0x87,
0xd8, 0x49, 0x3b, 0xaa, 0xdf, 0x4e, 0x3c, 0xad,
0xd6, 0x47, 0x35, 0xa4, 0xd1, 0x40, 0x32, 0xa3,
0xc4, 0x55, 0x27, 0xb6, 0xc3, 0x52, 0x20, 0xb1,
0xca, 0x5b, 0x29, 0xb8, 0xcd, 0x5c, 0x2e, 0xbf,
0x90, 0x01, 0x73, 0xe2, 0x97, 0x06, 0x74, 0xe5,
0x9e, 0x0f, 0x7d, 0xec, 0x99, 0x08, 0x7a, 0xeb,
0x8c, 0x1d, 0x6f, 0xfe, 0x8b, 0x1a, 0x68, 0xf9,
0x82, 0x13, 0x61, 0xf0, 0x85, 0x14, 0x66, 0xf7,
0xa8, 0x39, 0x4b, 0xda, 0xaf, 0x3e, 0x4c, 0xdd,
0xa6, 0x37, 0x45, 0xd4, 0xa1, 0x30, 0x42, 0xd3,
0xb4, 0x25, 0x57, 0xc6, 0xb3, 0x22, 0x50, 0xc1,
0xba, 0x2b, 0x59, 0xc8, 0xbd, 0x2c, 0x5e, 0xcf
};

uint8_t last_crc_in;
uint8_t last_crc_out;

unsigned char spi_buf_out[256];
unsigned char spi_buf_in[256];


/****************************************************************************
 * Public Functions
 ****************************************************************************/
unsigned char RMAP_CalculateCRC(unsigned char INCRC, unsigned char INBYTE)
{
  return RMAP_CRCTable[INCRC ^ INBYTE];
}

unsigned char RMAP_CalculateCRC_mod(unsigned char INCRC, unsigned char INBYTE)
{
  unsigned char x=INBYTE;
  x = (x<<4) | (x>>4);
  x = ((x & 0x33)<<2) | ((x & 0xcc)>>2);
  x = ((x & 0x55)<<1) | ((x & 0xaa)>>1);
  return RMAP_CRCTable[INCRC ^ x];
}

#define SPI_FRAME_SZ 6

void test_spi(unsigned int data)
{
  //volatile uint32_t *spi_dr = (uint32_t *) 0x4001300c;
  volatile uint8_t *spi_dr = (uint8_t *) 0x4001300c;
  volatile uint32_t *spi_sr = (uint32_t *) 0x40013008;

  spi_buf_in[0] = *spi_dr;
  while (((*spi_sr)&2)==0);
  *spi_dr = (uint8_t) data;
  //while (((*spi_sr)&1)==0);
  spi_buf_in[0] = *spi_dr;
}

void send_spi_frame(void)
{
  int i;
  //volatile uint32_t *spi_dr = (uint32_t *) 0x4001300c;
  volatile uint8_t *spi_dr = (uint8_t *) 0x4001300c;
  volatile uint32_t *spi_sr = (uint32_t *) 0x40013008;

  last_crc_in = 0x00;
  last_crc_out = 0x00;

  spi_buf_in[0] = *spi_dr;
  for (i=0; i<SPI_FRAME_SZ; i++) {
    while (((*spi_sr)&2)==0);
    *spi_dr = spi_buf_out[i];
    last_crc_out = RMAP_CalculateCRC_mod(last_crc_out, spi_buf_out[i]);
    while (((*spi_sr)&1)==0);
    spi_buf_in[i] = *spi_dr;
    if (i==5) { /* FIXME : TODO : HACK : fix fpga! */
      unsigned char x=spi_buf_in[i];
      x = (x<<4) | (x>>4);
      x = ((x & 0x33)<<2) | ((x & 0xcc)>>2);
      x = ((x & 0x55)<<1) | ((x & 0xaa)>>1);
      spi_buf_in[i] = x;
    }
    last_crc_in = RMAP_CalculateCRC_mod(last_crc_in, spi_buf_in[i]);
  }
  //while (((*spi_sr)&1)==0);
  spi_buf_in[SPI_FRAME_SZ] = *spi_dr;
}

int master_spi_read_word (unsigned int apb_addr, unsigned int *pdata)
{
  int i;
  uint32_t val;

  for (i=0; i<SPI_FRAME_SZ; i++) {
    spi_buf_in[i] = 0;
    spi_buf_out[i] = 0;
  }

  /* 1) sending APB_ADDR */
  spi_buf_out[0] = 0x30;
  spi_buf_out[1] = (apb_addr>>24) & 0xff;
  spi_buf_out[2] = (apb_addr>>16) & 0xff;
  spi_buf_out[3] = (apb_addr>>8) & 0xff;
  spi_buf_out[4] = (apb_addr) & 0xff;
  spi_buf_out[5] = 0;

  send_spi_frame();

  /* 2) reading data */
  spi_buf_out[0] = 0x50;
  spi_buf_out[1] = 0;
  spi_buf_out[2] = 0;
  spi_buf_out[3] = 0;
  spi_buf_out[4] = 0;
  spi_buf_out[5] = 0;

  send_spi_frame();

  val = 
    (spi_buf_in[1]<<24) + 
    (spi_buf_in[2]<<16) + 
    (spi_buf_in[3]<<8)  + 
    (spi_buf_in[4]);
  *pdata = val;

  return 0;
}

int master_spi_write_word (unsigned int apb_addr, unsigned int data)
{
  int i;

  for (i=0; i<SPI_FRAME_SZ; i++) {
    spi_buf_in[i] = 0;
    spi_buf_out[i] = 0;
  }

  /* 1) sending APB_ADDR */
  spi_buf_out[0] = 0x30;
  spi_buf_out[1] = (apb_addr>>24) & 0xff;
  spi_buf_out[2] = (apb_addr>>16) & 0xff;
  spi_buf_out[3] = (apb_addr>>8) & 0xff;
  spi_buf_out[4] = (apb_addr) & 0xff;
  spi_buf_out[5] = 0;

  send_spi_frame();

  /* 2) writing data */
  spi_buf_out[0] = 0x40;
  spi_buf_out[1] = (data>>24) & 0xff;
  spi_buf_out[2] = (data>>16) & 0xff;
  spi_buf_out[3] = (data>>8) & 0xff;
  spi_buf_out[4] = (data) & 0xff;
  spi_buf_out[5] = 0;

  send_spi_frame();

  return 0;
}


/****************************************************************************
 * hello_spi_main
 ****************************************************************************/
void usage(const char *prog_name)
{
  printf("Usage:\n");
  printf(" %s w <apb_addr> <data>\n", prog_name);
  printf(" %s r <apb_addr>\n", prog_name);
}

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int robot_master_spi_main(int argc, char *argv[])
#endif
{
  int is_write=0;
  int is_read=0;
  int is_test=0;
  unsigned int data = 0x42424242;
  unsigned int apb_addr;
  int i;

  if(argc<3) {
    usage(argv[0]);
    return 1;
  }

  printf("CMD : %s\n", argv[1]);

  if (argv[1][0]=='w') {
    is_write=1;
    if(argc<4) {
      usage(argv[0]);
      return 1;
    }
    apb_addr = strtoul(argv[2], NULL, 16);
    data = strtoul(argv[3], NULL, 16);
  } else if (argv[1][0]=='r') {
    is_read=1;
    apb_addr = strtoul(argv[2], NULL, 16);
  } else if (argv[1][0]=='t') {
    is_test=1;
    data = strtoul(argv[2], NULL, 16);
  } else {
    usage(argv[0]);
    return 1;
  }

  if (is_write==1) {
    master_spi_write_word (apb_addr, data);
    printf(" @0x%.8x : W 0x%.8x \n", apb_addr, data);
    printf(" last_crc_in = %.2x \n", last_crc_in);
    printf(" last_crc_out = %.2x \n", last_crc_out);
    printf(" spi_buf_in = [ ");
    for (i=0; i<SPI_FRAME_SZ; i++) {
      printf("%.2x ", spi_buf_in[i]);
    }
    printf("]\n");
  } else if (is_read==1) {
    master_spi_read_word (apb_addr, &data);
    printf(" @0x%.8x : R 0x%.8x \n", apb_addr, data);
    printf(" last_crc_in = %.2x \n", last_crc_in);
    printf(" last_crc_out = %.2x \n", last_crc_out);
    printf(" spi_buf_in = [ ");
    for (i=0; i<SPI_FRAME_SZ; i++) {
      printf("%.2x ", spi_buf_in[i]);
    }
    printf("]\n");
  } else if (is_test==1) {
    unsigned char tab_data[] = {0xff, 0x12, 0x34, 0x56, 0x78, 0x25};
    //printf(" TEST : data = 0x%.8x \n", data);
    //test_spi(data);
    //last_crc_out = RMAP_CalculateCRC_mod(last_crc_out, (unsigned char)data);
    //printf(" last_crc_out = %.2x \n", last_crc_out);
    last_crc_out = 0;
    last_crc_out = RMAP_CalculateCRC_mod(last_crc_out, tab_data[0]);
    last_crc_out = RMAP_CalculateCRC_mod(last_crc_out, tab_data[1]);
    last_crc_out = RMAP_CalculateCRC_mod(last_crc_out, tab_data[2]);
    last_crc_out = RMAP_CalculateCRC_mod(last_crc_out, tab_data[3]);
    last_crc_out = RMAP_CalculateCRC_mod(last_crc_out, tab_data[4]);
    last_crc_out = RMAP_CalculateCRC_mod(last_crc_out, tab_data[5]);
    printf(" last_crc_out(mod) = %.2x \n", last_crc_out);
  }

  return 0;
}
