/*
 * microsd_fz.h
 *
 *  Created on: Jul 25, 2025
 *      Author: liufangzheng
 */

#ifndef INC_MICROSD_FZ_H_
#define INC_MICROSD_FZ_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

#include "main.h"
#include "fatfs.h"
#include "stm32f7xx_hal.h"

//some variables for FatFs
extern FATFS FatFs;   //Fatfs handle
extern FIL fil;       //File handle

extern FRESULT mount_sd_card(void);
extern FRESULT eject_sd_card(void);
extern FRESULT get_statistics(DWORD *free_clusters, DWORD *free_sectors, DWORD *total_sectors);
// extern FRESULT open_file(const char *filename);
extern FRESULT write_data_to_file(const char *filename, uint8_t *data, UINT data_len);
extern FRESULT close_file(void);

#endif /* INC_MICROSD_FZ_H_ */
