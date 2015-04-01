#include "contiki.h"
#include "contiki-lib.h"

#include "cetic-6lbr.h"
#include "nvm-config.h"
#include "nvm-itf.h"

#include <string.h>

#define DEBUG 0
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#define CETIC_6LBR_NVM_ADDRESS (0x100)

MEMB(nvm_memb, nvm_data_t, 1);
LIST(nvm_list);
void
nvm_int(void)
{
  memb_init(&nvm_memb);
  list_init(nvm_list);
}
void 
I2C_EE_BufferRead(uint16_t RomAddr,uint16_t NumByteToRead,nvm_data_t *pRomData)
{
  struct nvm_data_t *m;
  m = list_head(nvm_list);
  memcpy(pRomData, m, NumByteToRead);
}
void
I2C_EE_BufferWrite(uint16_t RomAddr,uint16_t NumByteToWrite,nvm_data_t *pRomData)
{
  struct nvm_data_t *m;
  m = list_head(nvm_list);
  if(m != NULL) {
    list_remove(nvm_list, m);
    memb_free(&nvm_memb, m);
  }
  m = memb_alloc(&nvm_memb);
  memcpy(m, pRomData, NumByteToWrite);
  list_add(nvm_list, m);
}

void
nvm_data_read(void)
{
  PRINTF("Reading 6LBR NVM\r\n");
  I2C_EE_BufferRead(CETIC_6LBR_NVM_ADDRESS,sizeof(nvm_data_t),&nvm_data);
}

void
nvm_data_write(void)
{
  PRINTF("Flashing 6LBR NVM\r\n");
  I2C_EE_BufferWrite(CETIC_6LBR_NVM_ADDRESS,sizeof(nvm_data_t),&nvm_data);
}
