/*
 * Copyright (C) 2022 by Paul Bachek
 * prbachek@gmail.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
 
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <target/target.h>
#include <helper/log.h>
#include "imp.h"
#include "arm_io.h"

#define AXI_COMMAND_PHASE           (0 << 19)
#define AXI_DATA_PHASE              (1 << 19)

#define CMD_TYPE_UPDATE_REGS_AXI    (0 << 21)
#define CMD_TYPE_MODE_REG           (1 << 21)
#define CMD_TYPE_UPDATE_REGS        (2 << 21)
#define CMD_TYPE_BOTH               (3 << 21)

#define ECC_STATUS_BUSY             (1 << 6)
#define ECC_PAGE_SIZE_0x512         (0 << 0)
#define ECC_PAGE_SIZE_1x512         (1 << 0)
#define ECC_PAGE_SIZE_2x512         (2 << 0)
#define ECC_PAGE_SIZE_4x512         (3 << 0)
#define ECC_MODE_BYPASSED           (0 << 2)
#define ECC_MODE_CALC_ONLY          (1 << 2)
#define ECC_MODE_CALC_RDWR          (2 << 2)
#define ECC_READ_END_BLOCK          (0 << 4)
#define ECC_READ_END_PAGE           (1 << 4)
#define ECC_JUMP_NO                 (0 << 5)
#define ECC_JUMP_COL_CMD            (1 << 5)
#define ECC_JUMP_FULL_CMD           (2 << 5)
#define ECC_IGNORE_ADD_EIGHT        (1 << 7)
#define ECC_EXTRA_BLOCK             (1 << 10)
#define ECC_EXTRA_BLOCK_SIZE_4B     (0 << 11)
#define ECC_EXTRA_BLOCK_SIZE_8B     (1 << 11)
#define ECC_EXTRA_BLOCK_SIZE_16B    (2 << 11)
#define ECC_EXTRA_BLOCK_SIZE_32B    (3 << 11)

// PL350 Register map
enum {
    REG_OFFSET_MEMC_STATUS          = 0x000,
    REG_OFFSET_MEMIF_CFG            = 0x004,
    REG_OFFSET_MEMIF_CFG_SET        = 0x008,
    REG_OFFSET_MEMIF_CFG_CLR        = 0x00C,
    REG_OFFSET_DIRECT_CMD           = 0x010,
    REG_OFFSET_SET_CYCLES           = 0x014,
    REG_OFFSET_SET_OPMODE           = 0x018,
    REG_OFFSET_REFRESH_PERIOD_0     = 0x020,
    REG_OFFSET_REFRESH_PERIOD_1     = 0x024,
    REG_OFFSET_CHIP_CFG_IF0_C0      = 0x100,
    REG_OFFSET_CHIP_CFG_IF0_C1      = 0x120,
    REG_OFFSET_CHIP_CFG_IF0_C2      = 0x140,
    REG_OFFSET_CHIP_CFG_IF0_C3      = 0x160,
    REG_OFFSET_CHIP_CFG_IF1_C0      = 0x180,
    REG_OFFSET_CHIP_CFG_IF1_C1      = 0x1A0,
    REG_OFFSET_CHIP_CFG_IF1_C2      = 0x1C0,
    REG_OFFSET_CHIP_CFG_IF1_C3      = 0x1E0,
    REG_OFFSET_USR_STATUS           = 0x200,
    REG_OFFSET_USR_CFG              = 0x204,
    REG_OFFSET_ECC_IF0              = 0x300,
    REG_OFFSET_ECC_IF1              = 0x400,
    REG_OFFSET_INTEGRATION_TEST     = 0xE00,
    REG_OFFSET_PERIPH_ID0           = 0xFE0,
    REG_OFFSET_PERIPH_ID1           = 0xFE4,
    REG_OFFSET_PERIPH_ID2           = 0xFE8,
    REG_OFFSET_PERIPH_ID3           = 0xFEC,
    REG_OFFSET_PCELL_ID0            = 0xFF0,
    REG_OFFSET_PCELL_ID1            = 0xFF4,
    REG_OFFSET_PCELL_ID2            = 0xFF8,
    REG_OFFSET_PCELL_ID3            = 0xFFC,
};

// Config register offsets per chip select
enum {
    REG_OFFSET_NAND_CYCLES          = 0x00,
    REG_OFFSET_OPMODE               = 0x04,
};  

// ECC register offsets per interface
enum {  
    REG_OFFSET_ECC_STATUS           = 0x00,
    REG_OFFSET_ECC_MEMCFG           = 0x04,
    REG_OFFSET_ECC_MEMCMD1          = 0x08,
    REG_OFFSET_ECC_MEMCMD2          = 0x0C,
    REG_OFFSET_ECC_ADDR0            = 0x10,
    REG_OFFSET_ECC_ADDR1            = 0x14,
    REG_OFFSET_ECC_VAL0             = 0x18,
    REG_OFFSET_ECC_VAL1             = 0x1C,
    REG_OFFSET_ECC_VAL2             = 0x20,
    REG_OFFSET_ECC_VAL3             = 0x24,
    REG_OFFSET_ECC_VAL4             = 0x28,
};

// Private data
struct pl350_priv {
    // Base addresses of buses
    uint32_t axi_addr;
    uint32_t apb_addr;
    
    // Inteface/Chip Select #
    uint8_t if_cs;
    
    // Buffer command cycle operations
    uint8_t cmd_buf;
    uint8_t cmd_buf_valid;
    uint8_t addr_buf[7];
    uint8_t addr_buf_cnt;
    
    // Info for ARM hosted operations
    struct arm_nand_data io;
};

/* 
 * Handle the initial NAND device command
 */
NAND_DEVICE_COMMAND_HANDLER(pl350_device_command) {
    uint32_t axi_addr = 0, apb_addr = 0;
    uint8_t intf, cs;
    struct pl350_priv *priv;
    int retval = ERROR_OK;
    
    LOG_DEBUG("PL350 NAND Device Command");
    
    if (CMD_ARGC != 4 && CMD_ARGC != 6) {
        command_print(CMD, "parameters: mem_base_addr config_base_addr [interface(0-1) chip_select(0-3)]");
        retval = ERROR_COMMAND_ARGUMENT_INVALID;
    }
    
    COMMAND_PARSE_NUMBER(u32, CMD_ARGV[2], axi_addr);
    if (axi_addr == 0) {
        command_print(CMD, "invalid mem_base_addr: %s", CMD_ARGV[2]);
        retval = ERROR_COMMAND_ARGUMENT_INVALID;
    }
    
    COMMAND_PARSE_NUMBER(u32, CMD_ARGV[3], apb_addr);
    if (apb_addr == 0) {
        command_print(CMD, "invalid config_base_addr: %s", CMD_ARGV[3]);
        retval = ERROR_COMMAND_ARGUMENT_INVALID;
    }
    
    if (CMD_ARGC == 6) {
        COMMAND_PARSE_NUMBER(u8, CMD_ARGV[4], intf);
        if (intf > 1) {
            command_print(CMD, "invalid interface(0-1): %s", CMD_ARGV[4]);
            retval = ERROR_COMMAND_ARGUMENT_INVALID;
        }
        
        COMMAND_PARSE_NUMBER(u8, CMD_ARGV[5], cs);
        if (cs > 3) {
            command_print(CMD, "invalid chip_select(0-3): %s", CMD_ARGV[5]);
            retval = ERROR_COMMAND_ARGUMENT_INVALID;
        }
    } else {
        intf = -1;
        cs = -1;
    }
    
    priv = calloc(1, sizeof(*priv));
    if (!priv) {
        command_print(CMD, "Unable to allocate space for controller private data");
        retval = ERROR_COMMAND_ARGUMENT_INVALID;
    }
    
    if (retval == ERROR_OK) {
        priv->axi_addr = axi_addr;
        priv->apb_addr = apb_addr;
        priv->if_cs = (intf << 2) | cs;
        priv->cmd_buf_valid = 0;
        priv->addr_buf_cnt = 0;
        
        priv->io.target = nand->target;
        priv->io.op = ARM_NAND_NONE;
        
        nand->controller_priv = priv;
    }
    
    return retval;
}

/*
 * Sets the timing parameters for the memory device
 */
COMMAND_HANDLER(handle_pl350_set_cycles) {
    struct nand_device *nand;
    struct pl350_priv *priv;
    uint8_t cycles[7];
    uint32_t cycles_val, nand_num;
    int retval = ERROR_OK;
    
    if (CMD_ARGC != 8) {
        command_print(CMD, "parameters: bank_id tRC tWC tREA tWP tCLR tAR tRR");
        retval = ERROR_COMMAND_ARGUMENT_INVALID;
    }

    if (retval == ERROR_OK) {
        COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], nand_num);
        nand = get_nand_device_by_num(nand_num);
        if (!nand) {
            command_print(CMD, "invalid nand device number: %s", CMD_ARGV[0]);
            retval = ERROR_COMMAND_ARGUMENT_INVALID;
        } else {
            priv = nand->controller_priv;
        }
    }
    
    if (retval == ERROR_OK && priv->if_cs > 7) {
        command_print(CMD, "nand device must have interface and chip_select parameters");
        retval = ERROR_COMMAND_ARGUMENT_INVALID;
    }
    
    if (retval == ERROR_OK) {
        for (uint8_t i = 0; i < 7; i++) {
            COMMAND_PARSE_NUMBER(u8, CMD_ARGV[i + 1], cycles[i]);
        }
        
        cycles_val = (cycles[6] & 0xF) << 20 | \
                     (cycles[5] & 0x7) << 17 | \
                     (cycles[4] & 0x7) << 14 | \
                     (cycles[3] & 0x7) << 11 | \
                     (cycles[2] & 0x7) << 8  | \
                     (cycles[1] & 0xF) << 4  | \
                     (cycles[0] & 0xF) << 0;
                     
        retval = target_write_u32(nand->target, priv->apb_addr | REG_OFFSET_SET_CYCLES, cycles_val);
    }
    
    if (retval == ERROR_OK) {
        retval = target_write_u32(nand->target, priv->apb_addr | REG_OFFSET_DIRECT_CMD, priv->if_cs << 23 | CMD_TYPE_UPDATE_REGS);
    }
    
    return retval;
}

/*
 * Sets the interface bus width to 8 or 16 bits
 */
COMMAND_HANDLER(handle_pl350_set_bus_width) {
    struct nand_device *nand;
    struct pl350_priv *priv;
    uint8_t opmode_val;
    uint32_t nand_num;
    int retval = ERROR_OK;
    
    if (CMD_ARGC != 2) {
        command_print(CMD, "parameters: bank_id width (8,16)");
        retval = ERROR_COMMAND_ARGUMENT_INVALID;
    }

    if (retval == ERROR_OK) {
        COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], nand_num);
        nand = get_nand_device_by_num(nand_num);
        if (!nand) {
            command_print(CMD, "invalid nand device number: %s", CMD_ARGV[0]);
            retval = ERROR_COMMAND_ARGUMENT_INVALID;
        } else {
            priv = nand->controller_priv;
        }
    }
    
    if (retval == ERROR_OK && priv->if_cs > 7) {
        command_print(CMD, "nand device must have interface and chip_select parameters");
        retval = ERROR_COMMAND_ARGUMENT_INVALID;
    }
    
    if (retval == ERROR_OK) {
        COMMAND_PARSE_NUMBER(u8, CMD_ARGV[1], opmode_val);
        if (opmode_val != 8 && opmode_val != 16) {
            command_print(CMD, "Invalid bus width (8,16)");
            retval = ERROR_COMMAND_ARGUMENT_INVALID;
        }
    }
    
    if (retval == ERROR_OK) {
        retval = target_write_u32(nand->target, priv->apb_addr | REG_OFFSET_SET_OPMODE, opmode_val >> 4);
    }
    
    if (retval == ERROR_OK) {
        retval = target_write_u32(nand->target, priv->apb_addr | REG_OFFSET_DIRECT_CMD, priv->if_cs << 23 | CMD_TYPE_UPDATE_REGS);
    }
    
    return retval;
}

/*
 * Sub-Command handlers
 */
static const struct command_registration pl350_sub_command_registration[] = {
    {
        .name = "set_cycles",
        .handler = handle_pl350_set_cycles,
        .mode = COMMAND_EXEC,
        .help = "Sets the timing parameters for the memory device",
        .usage = "parameters: bank_id tRC tWC tREA tWP tCLR tAR tRR",
    },
    {
        .name = "set_bus_width",
        .handler = handle_pl350_set_bus_width,
        .mode = COMMAND_EXEC,
        .help = "Sets the interface bus width to 8 or 16 bits",
        .usage = "parameters: bank_id width (8,16)",
    },
    COMMAND_REGISTRATION_DONE
};

/*
 * Command handlers
 */
static const struct command_registration pl350_command_registration[] = {
    {
        .name = "pl350",
        .handler = NULL,
        .mode = COMMAND_ANY,
        .help = "PL350 NAND flash controller",
        .usage = "parameters: mem_base_addr config_base_addr [interface(0-1) chip_select(0-3)]",
        .chain = pl350_sub_command_registration,
    },
    COMMAND_REGISTRATION_DONE
};

/*
 * Initialize NAND controller
 */
static int pl350_init(struct nand_device *nand) {
    int retval = ERROR_OK;
    
    if (nand->target->state != TARGET_HALTED) {
        LOG_ERROR("Target must be halted to use NAND controller");
        retval = ERROR_NAND_OPERATION_FAILED;
    }
    
    return retval;
}

/*
 * Reset NAND controller
 */
static int pl350_reset(struct nand_device *nand) {
    // No software reset capability
    return ERROR_OK;
}
/*
 * Send a command and all stored address bytes from the buffer to
 * the memory device
 */
static int flush_cmd_buf(struct nand_device *nand) {
    struct pl350_priv *priv;
    int retval = ERROR_OK;
    
    priv = nand->controller_priv;
    
    if (priv->cmd_buf_valid) {
        uint32_t addr;
        uint8_t i = 0;
        
        addr = priv->axi_addr | priv->addr_buf_cnt << 21 | AXI_COMMAND_PHASE | priv->cmd_buf << 3;
        do {
            retval = target_write_u8(nand->target, addr + i, priv->addr_buf[i]);
        } while ((++i < priv->addr_buf_cnt) && (retval == ERROR_OK));
        priv->cmd_buf_valid = 0;
        priv->addr_buf_cnt = 0;
    }
    
    return retval;
}
/*
 * Send a single command byte to the memory device
 * The PL35x does not support sending single address bytes 
 * so commands are buffered until they can be sent together with
 * the accompanying address bytes.
 */
static int pl350_command(struct nand_device *nand, uint8_t command) {
    struct pl350_priv *priv;
    int retval = ERROR_OK;
        
    priv = nand->controller_priv;
    
    if (retval == ERROR_OK) {
        // Flush the command/address buffer
        retval = flush_cmd_buf(nand);
    }
        
    if (retval == ERROR_OK) {
        uint32_t addr;
        
        // Do not buffer reset command
        if (command == NAND_CMD_RESET){
            addr = priv->axi_addr | AXI_COMMAND_PHASE | command << 3;
            retval = target_write_u8(nand->target, addr, 0);
        }
        else {
            priv->cmd_buf = command;
            priv->cmd_buf_valid = 1;
        }
    }
    
    return retval;
}

/*
 * Send a single address byte to the memory device
 * The PL35x does not support sending single address bytes 
 * so they are buffered until they can all be sent together
 * with the accompanying command.
 */
static int pl350_address(struct nand_device *nand, uint8_t address) {
    struct pl350_priv *priv;
    int retval = ERROR_OK;
    
    priv = nand->controller_priv;
    
    // Check address buffer count
    if (priv->addr_buf_cnt >= 7) {
        LOG_ERROR("Too many addresses sent");
        retval = ERROR_NAND_OPERATION_FAILED;
    }
    if (retval == ERROR_OK) {
        // Write to address buffer
        priv->addr_buf[priv->addr_buf_cnt++] = address;
    }
    
    return retval;
}

/*
 * Write a byte of data to memory device
 */
static int pl350_write_data(struct nand_device *nand, uint16_t data) {
    struct pl350_priv *priv;
    uint32_t addr;
    int retval = ERROR_OK;
    
    priv = nand->controller_priv;
    
    addr = priv->axi_addr | AXI_DATA_PHASE;
    
    if (retval == ERROR_OK)
        // Flush the command/address buffer
        retval = flush_cmd_buf(nand);
        
    if (retval == ERROR_OK)
        // Write a byte of data to external memory
        retval = target_write_u8(nand->target, addr, data);
    
    return retval;
}

/*
 * Read a byte of data from memory device
 */
static int pl350_read_data(struct nand_device *nand, void *data) {
    struct pl350_priv *priv;
    uint32_t addr;
    int retval = ERROR_OK;
    
    priv = nand->controller_priv;
    
    addr = priv->axi_addr | AXI_DATA_PHASE;
    
    if (retval == ERROR_OK)
        // Flush the command/address buffer
        retval = flush_cmd_buf(nand);
        
    if (retval == ERROR_OK)
        // Read a byte of data from external memory
        retval = target_read_u8(nand->target, addr, data);
    
    return retval;
}

/*
 * Write a block of data to memory device
 */
static int pl350_write_block_data(struct nand_device *nand, uint8_t *data, int size) {
    struct pl350_priv *priv;
    struct arm_nand_data *io;
    uint32_t addr;
    int retval = ERROR_OK;
    
    priv = nand->controller_priv;
    io = &priv->io;
    
    addr = priv->axi_addr | AXI_DATA_PHASE;
    
    if (retval == ERROR_OK)
        // Flush the command/address buffer
        retval = flush_cmd_buf(nand);
    
    io->chunk_size = nand->page_size;
    io->data = addr;
    
    if (retval == ERROR_OK)
        // Execute hosted external memory write
        retval = arm_nandwrite(io, data, size);
    
    return retval;
}

/*
 * Read a block of data from memory device
 */
static int pl350_read_block_data(struct nand_device *nand, uint8_t *data, int size) {
    struct pl350_priv *priv;
    struct arm_nand_data *io;
    uint32_t addr;
    int retval = ERROR_OK;
    
    priv = nand->controller_priv;
    io = &priv->io;
    
    addr = priv->axi_addr | AXI_DATA_PHASE;
    
    if (retval == ERROR_OK)
        // Flush the command/address buffer
        retval = flush_cmd_buf(nand);
    
    io->chunk_size = nand->page_size;
    io->data = addr;
    
    if (retval == ERROR_OK)
        // Execute hosted external memory read
        retval = arm_nandread(io, data, size);
    
    return retval;
}

/*
 * Calculate ECC Hamming code for a data buffer of size bytes
 */
static uint32_t calc_ecc(uint8_t *data, uint32_t size) {
    uint32_t odd, even, numDataBits, numParityBits;
    
    numDataBits = size * 8;
    numParityBits = sizeof(numDataBits) * CHAR_BIT - __builtin_clz(numDataBits) - 1;
    
    odd = 0;
    even = 0;
    
    // Iterate over all input bits
    for (uint32_t i = 0; i < numDataBits; i++) {
        // If data bit is set, XOR select bits in ECC
        if ((data[i/8] >> i%8) & 1) {
            odd ^= i;
            even ^= ~i;
        }
    }
    // Invert final codes
    odd = ~odd;
    even = ~even;
    // Mask out extra upper bits
    odd &= (numDataBits - 1);
    even &= (numDataBits - 1);
    
    return (even << numParityBits) | odd;
}

/*
 * Check ECC Hamming codes and attempt to correct a data buffer of size bytes
 */
static int check_ecc(uint8_t *data, uint32_t size, uint32_t ecc_mem, uint32_t ecc_calc) {
    uint32_t ecc_xor, numDataBits, numParityBits;
    uint8_t ecc_numOnes;
    int retval;
    
    numDataBits = size * 8;
    numParityBits = sizeof(numDataBits) * CHAR_BIT - __builtin_clz(numDataBits) - 1;
    
    ecc_xor = ecc_calc ^ ecc_mem;
    ecc_numOnes = __builtin_popcount(ecc_xor);
    
    if (ecc_numOnes == 0) {
        // No errors
        retval = 0;
    } else if (ecc_numOnes == numParityBits) {
        // Single correctable data error
        data[(ecc_xor >> 3) & (size - 1)] ^= (1 << (ecc_xor & 0x7));
        retval = 1;
    } else if (ecc_numOnes == 1) {
        // Single ECC code error
        retval = 2;
    } else {
        // Multiple errors
        retval = -1;
    }
    
    return retval;
}

/*
 * Write a page to memory and attempt to detect data errors
 */
static int pl350_write_page(struct nand_device *nand, uint32_t page, uint8_t *data, uint32_t data_size, uint8_t *oob, uint32_t oob_size) {
    struct pl350_priv *priv;
    uint8_t *oob_buf, ecc_page_size, num_512_blocks;
    uint32_t ecc_sw, ecc_hw;
    int retval = ERROR_OK;
    
    priv = nand->controller_priv;
    
    // Set parameters for different page size devices
    num_512_blocks = nand->page_size / 512;
    if (!oob_size)
        oob_size = num_512_blocks * 16;
    if (num_512_blocks == 1)
        ecc_page_size = ECC_PAGE_SIZE_1x512;
    else if (num_512_blocks == 2)
        ecc_page_size = ECC_PAGE_SIZE_2x512;
    else if (num_512_blocks == 4)
        ecc_page_size = ECC_PAGE_SIZE_4x512;
    else
        retval = ERROR_NAND_OPERATION_FAILED;
    
    if (retval == ERROR_OK) {
        // Set up ECC module
        retval = target_write_u32(nand->target,
            priv->apb_addr | REG_OFFSET_ECC_IF1 | REG_OFFSET_ECC_MEMCFG,
            ECC_JUMP_FULL_CMD | ECC_READ_END_PAGE | ECC_MODE_CALC_ONLY | ecc_page_size);
    }
    
    oob_buf = oob;
    if (retval == ERROR_OK) {
        // Allocate OOB if buffer isn't provided
        if (!oob_buf) {
            oob_buf = malloc(oob_size);
            if (!oob_buf) {
                LOG_ERROR("Could not allocate memory for OOB");
                retval = ERROR_NAND_OPERATION_FAILED;
            }
        }
    }
    
    // Calculate ECC codes for each 512 byte block
    for (uint8_t i = 0; data && i < num_512_blocks; i++){
        if (retval == ERROR_OK) {
            // Calculate ECC and write to oob buffer
            ecc_sw = calc_ecc(&data[i*512], 512);
            target_buffer_set_u24(nand->target, &oob_buf[oob_size + 3*(i - num_512_blocks)], ecc_sw);
        }
    }
    
    if (retval == ERROR_OK) {
        // Write page to memory
        retval = nand_write_page_raw(nand, page, data, data_size, oob_buf, oob_size);
    }
    
    if (retval == ERROR_OK) {
        uint32_t status;
        
        // Poll ECC module busy
        do {
            // Read status register
            retval = target_read_u32(nand->target, priv->apb_addr | REG_OFFSET_ECC_IF1 | REG_OFFSET_ECC_STATUS, &status);
        } while ((status & ECC_STATUS_BUSY) && (retval == ERROR_OK));
    }
    
    // Check ECC codes for each 512 byte block
    for (uint8_t i = 0; data && i < num_512_blocks; i++){
        if (retval == ERROR_OK) {
            // Read ECC module calculated value
            retval = target_read_u32(nand->target, (priv->apb_addr | REG_OFFSET_ECC_IF1 | REG_OFFSET_ECC_VAL0) + i*4, &ecc_hw);
        }
        if (retval == ERROR_OK) {
            // Invert code and only keep 24 bits
            ecc_hw = ~ecc_hw & 0x00FFFFFF;
            // Get software calculated ECC value
            ecc_sw = target_buffer_get_u24(nand->target, &oob_buf[oob_size + 3*(i - num_512_blocks)]);
        
            // Compare software and hardware calculated ECC
            if (ecc_hw != ecc_sw) {
                LOG_ERROR("SW/HW ECC mismatch, page write failed");
                retval = ERROR_NAND_OPERATION_FAILED;
            }
        }
    }
    
    // Deallocate OOB buffer if caller doesn't want it
    if (!oob)
        free(oob_buf);

    return retval;
}

/*
 * Read a page from memory and attempt to detect/correct data errors
 */
static int pl350_read_page(struct nand_device *nand, uint32_t page, uint8_t *data, uint32_t data_size, uint8_t *oob, uint32_t oob_size) {
    struct pl350_priv *priv;
    uint8_t *oob_buf, ecc_page_size, num_512_blocks, retry;
    uint32_t ecc_sw, ecc_hw, ecc_mem;
    int retval = ERROR_OK;
    
    priv = nand->controller_priv;
    
    // Set parameters for different page size devices
    num_512_blocks = nand->page_size / 512;
    if (!oob_size)
        oob_size = num_512_blocks * 16;
    if (num_512_blocks == 1)
        ecc_page_size = ECC_PAGE_SIZE_1x512;
    else if (num_512_blocks == 2)
        ecc_page_size = ECC_PAGE_SIZE_2x512;
    else if (num_512_blocks == 4)
        ecc_page_size = ECC_PAGE_SIZE_4x512;
    else
        retval = ERROR_NAND_OPERATION_FAILED;
    
    if (retval == ERROR_OK) {
        // Set up ECC module
        retval = target_write_u32(nand->target,
            priv->apb_addr | REG_OFFSET_ECC_IF1 | REG_OFFSET_ECC_MEMCFG,
            ECC_JUMP_FULL_CMD | ECC_READ_END_PAGE | ECC_MODE_CALC_ONLY | ecc_page_size);
    }
    
    oob_buf = oob;
    if (retval == ERROR_OK) {
        // Allocate OOB if buffer isn't provided
        if (!oob_buf) {
            oob_buf = malloc(oob_size);
            if (!oob_buf) {
                LOG_ERROR("Could not allocate memory for OOB");
                retval = ERROR_NAND_OPERATION_FAILED;
            }
        }
    }
    
    if (retval == ERROR_OK) {
        // Read page from memory
        retval = nand_read_page_raw(nand, page, data, data_size, oob_buf, oob_size);
    }
    
    if (retval == ERROR_OK) {
        uint32_t status;
        
        // Poll ECC module busy
        do {
            // Read status register
            retval = target_read_u32(nand->target, priv->apb_addr | REG_OFFSET_ECC_IF1 | REG_OFFSET_ECC_STATUS, &status);
        } while ((status & ECC_STATUS_BUSY) && (retval == ERROR_OK));
    }
    
    // Check ECC codes for each 512 byte block
    for (uint8_t i = 0; data && i < num_512_blocks; i++){
        if (retval == ERROR_OK) {
            // Read ECC module calculated value
            retval = target_read_u32(nand->target, (priv->apb_addr | REG_OFFSET_ECC_IF1 | REG_OFFSET_ECC_VAL0) + i*4, &ecc_hw);
        }
        if (retval == ERROR_OK) {
            // Invert code and only keep 24 bits
            ecc_hw = ~ecc_hw & 0x00FFFFFF;
            // Calculate software ECC value
            ecc_sw = calc_ecc(&data[i*512], 512);
            // Get ECC code from memory
            ecc_mem = target_buffer_get_u24(nand->target, &oob_buf[oob_size + 3*(i - num_512_blocks)]);
        
            // Compare software and hardware calculated ECC
            retry = 0;
            if (ecc_hw != ecc_sw) {
                LOG_ERROR("SW/HW ECC mismatch, retrying page read");
                retry = 1;
                break;
            }
            
            // Check parity values
            switch (check_ecc(&data[i*512], 512, ecc_mem, ecc_hw)) {
            case 1: LOG_INFO("ECC single error corrected");
                break;
            case 2: LOG_INFO("ECC code error detected");
                break;
            case -1: LOG_ERROR("ECC multiple errors detected");
                break;
            }
        }
    }
    
    // Deallocate OOB buffer if caller doesn't want it
    if (!oob)
        free(oob_buf);
    
    // Retry page read if communication error detected
    if (retry)
        retval = pl350_read_page(nand, page, data, data_size, oob, oob_size);

    return retval;
}

/*
 * Check if NAND controller is ready
 */
static int pl350_nand_ready(struct nand_device *nand, int timeout) {
    struct pl350_priv *priv;
    uint32_t status;
    int retval = ERROR_OK;
    
    priv = nand->controller_priv;
    
    // Read memory controller status
    retval = target_read_u32(nand->target, priv->apb_addr | REG_OFFSET_MEMC_STATUS, &status);
    
    if (retval == ERROR_OK) {
        // Check if memory controller is ready
        if (status & 0x1) {
            retval = ERROR_NAND_OPERATION_FAILED;
        }
    }
    
    return (retval == ERROR_OK);
}

// NAND Flash Controller structure to register the driver
struct nand_flash_controller pl350_nand_controller = {
    .name = "pl350",
    .usage = "pl350 static memory controller",
    .commands = pl350_command_registration,
    .nand_device_command = pl350_device_command,
    .init = pl350_init,
    .reset = pl350_reset,
    .command = pl350_command,
    .address = pl350_address,
    .write_data = pl350_write_data,
    .read_data = pl350_read_data,
    .write_block_data = pl350_write_block_data,
    .read_block_data = pl350_read_block_data,
    .write_page = pl350_write_page,
    .read_page = pl350_read_page,
    .nand_ready = pl350_nand_ready,
};
