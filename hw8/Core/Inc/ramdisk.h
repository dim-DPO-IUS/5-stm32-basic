#ifndef INC_RAMDISK_H_
#define INC_RAMDISK_H_

#define SECTORS (48)
#define SECTOR_SIZE (512)

extern uint8_t ramdisk[SECTORS][SECTOR_SIZE];

#endif /* INC_RAMDISK_H_ */

