/*
 * Copyright (c) 2013, CETIC.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/**
 * \file
 *         Platform-independant interface for parameters in Non-Volatile Memory
 * \author
 *         6LBR Team <6lbr@cetic.be>
 */

#define LOG6LBR_MODULE "NVM"

#include "contiki.h"
#include "contiki-lib.h"

#include <string.h>
#include <ctype.h>

#include "rpl-private.h"

#include "cetic-6lbr.h"
#include "nvm-config.h"
#include "nvm-itf.h"

#include "net/ip/uip-debug.h"
#define DEBUG 0
#if DEBUG
  #include <stdio.h>
  #include "bsp.h"
  #define PRINTF(...) printf(__VA_ARGS__)
  #define PRINTETHADDR(addr) printf(" %02x:%02x:%02x:%02x:%02x:%02x ",
                                    (*addr)[0], (*addr)[1], (*addr)[2], (*addr)[3], (*addr)[4], (*addr)[5])
#else
  #define PRINTF(...)
  #define PRINTETHADDR(addr)
#endif


nvm_data_t nvm_data;

/*---------------------------------------------------------------------------*/

void
check_nvm(volatile nvm_data_t * nvm_data, int reset)
{
  uint8_t flash = 0;
  uip_ipaddr_t loc_fipaddr;

  if(reset || nvm_data->magic != CETIC_6LBR_NVM_MAGIC
     || nvm_data->version > CETIC_6LBR_NVM_CURRENT_VERSION) {
    //NVM is invalid or we are rollbacking from another version
    //Set all data to default values
    if (!reset) {
      PRINTF("Invalid NVM magic number or unsupported NVM version, reseting it...\n");
    }
    nvm_data->magic = CETIC_6LBR_NVM_MAGIC;
    nvm_data->version = CETIC_6LBR_NVM_VERSION_0;

    CETIC_6LBR_NVM_DEFAULT_ETH_NET_PREFIX(&loc_fipaddr);
    memcpy((uint8_t *)&nvm_data->eth_net_prefix, &loc_fipaddr.u8, 16);

    CETIC_6LBR_NVM_DEFAULT_ETH_IP_ADDR(&loc_fipaddr);
    memcpy((uint8_t *)&nvm_data->eth_ip_addr, &loc_fipaddr.u8, 16);

    CETIC_6LBR_NVM_DEFAULT_WSN_NET_PREFIX(&loc_fipaddr);
    memcpy((uint8_t *)&nvm_data->wsn_net_prefix, &loc_fipaddr.u8, 16);

    CETIC_6LBR_NVM_DEFAULT_WSN_IP_ADDR(&loc_fipaddr);
    memcpy((uint8_t *)&nvm_data->wsn_ip_addr, &loc_fipaddr.u8, 16);

    CETIC_6LBR_NVM_DEFAULT_ETH_DFT_ROUTER(&loc_fipaddr);
    memcpy((uint8_t *)&nvm_data->eth_dft_router, &loc_fipaddr.u8, 16);

    nvm_data->rpl_version_id = CETIC_6LBR_NVM_DEFAULT_RPL_VERSION_ID;

    nvm_data->mode = CETIC_6LBR_NVM_DEFAULT_MODE;

    nvm_data->channel = CETIC_6LBR_NVM_DEFAULT_CHANNEL;

    flash = 1;
  }
  if ( nvm_data->version == CETIC_6LBR_NVM_VERSION_0)
  {
    if (!reset) {
      PRINTF("Migrate NVM version 0 towards 1\n");
    }
    nvm_data->version = CETIC_6LBR_NVM_VERSION_1;

    nvm_data->global_flags = CETIC_6LBR_NVM_DEFAULT_GLOBAL_FLAGS;

    nvm_data->wsn_net_prefix_len = CETIC_6LBR_NVM_DEFAULT_WSN_NET_PREFIX_LEN;
    nvm_data->eth_net_prefix_len = CETIC_6LBR_NVM_DEFAULT_ETH_NET_PREFIX_LEN;

    nvm_data->ra_flags = CETIC_6LBR_NVM_DEFAULT_RA_FLAGS;
    nvm_data->ra_router_lifetime = CETIC_6LBR_NVM_DEFAULT_RA_ROUTER_LIFETIME;
    nvm_data->ra_max_interval = CETIC_6LBR_NVM_DEFAULT_RA_MAX_INTERVAL;
    nvm_data->ra_min_interval = CETIC_6LBR_NVM_DEFAULT_RA_MIN_INTERVAL;
    nvm_data->ra_min_delay = CETIC_6LBR_NVM_DEFAULT_RA_MIN_DELAY;

    nvm_data->ra_prefix_flags = CETIC_6LBR_NVM_DEFAULT_RA_PREFIX_FLAGS;
    nvm_data->ra_prefix_vtime = CETIC_6LBR_NVM_DEFAULT_RA_PREFIX_VTIME;
    nvm_data->ra_prefix_ptime = CETIC_6LBR_NVM_DEFAULT_RA_PREFIX_PTIME;
    nvm_data->ra_rio_flags = CETIC_6LBR_NVM_DEFAULT_RA_RIO_FLAGS;
    nvm_data->ra_rio_lifetime = CETIC_6LBR_NVM_DEFAULT_RA_RIO_LIFETIME;

    nvm_data->rpl_instance_id = CETIC_6LBR_NVM_DEFAULT_RPL_INSTANCE_ID;
    nvm_data->rpl_preference = CETIC_6LBR_NVM_DEFAULT_RPL_PREFERENCE;
    nvm_data->rpl_dio_intdoubl = CETIC_6LBR_NVM_DEFAULT_RPL_DIO_INT_DOUBLING;
    nvm_data->rpl_dio_intmin = CETIC_6LBR_NVM_DEFAULT_RPL_DIO_MIN_INT;
    nvm_data->rpl_dio_redundancy = CETIC_6LBR_NVM_DEFAULT_RPL_DIO_REDUNDANCY;
    nvm_data->rpl_default_lifetime = CETIC_6LBR_NVM_DEFAULT_RPL_DEFAULT_LIFETIME;
    nvm_data->rpl_min_hoprankinc = CETIC_6LBR_NVM_DEFAULT_RPL_MIN_HOP_RANK_INC;
    nvm_data->rpl_lifetime_unit = CETIC_6LBR_NVM_DEFAULT_RPL_LIFETIME_UNIT;

    flash = 1;
  }

  if(flash) {
    nvm_data_write();
  }
}

void
load_nvm_config(void)
{
  nvm_data_read();

  PRINTF("NVM Magic : %x\n", nvm_data.magic);
  PRINTF("NVM Version : %x\n", nvm_data.version);

  check_nvm(&nvm_data, 0);
}

void
store_nvm_config(void)
{
  nvm_data_write();
}
