#ifndef __GOLDO_FPGA_H__
#define __GOLDO_FPGA_H__
#include "goldo_config.h"

int goldo_fpga_master_spi_read_word (unsigned int apb_addr, 
                                     unsigned int *pdata);

int goldo_fpga_master_spi_write_word (unsigned int apb_addr, 
                                      unsigned int data);

unsigned int goldo_fpga_get_version (void);

/* robot 2018 : 
   servo_id = [0..11]
   new_pw  = [0..0x40000]
*/
int goldo_fpga_cmd_servo (int servo_id, unsigned int new_pw);

/* robot 2018 : 
   pompe droite : motor_id = 0
   pompe gauche : motor_id = 1
   moteur tapis : motor_id = 2
   new_pw  = [0..0x200]
*/
int goldo_fpga_cmd_motor (int motor_id, int new_pw);

/* robot 2018 : 
   stp_id = [0..1]
   new_pos  = [0..0x10000]
*/
int goldo_fpga_cmd_stepper (int stp_id, unsigned int new_pos);

/* robot 2018 : 
   stp_id = [0..1]
*/
int goldo_fpga_get_stepper_pos (int stp_id, unsigned int *new_pos);


#endif /* __GOLDO_FPGA_H__ */
