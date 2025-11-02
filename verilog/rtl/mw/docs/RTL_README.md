# Microwatt RTL Design for OpenFrame Tapeout

This directory contains the RTL design for the Microwatt CPU core integrated with the ChipFoundry OpenFrame platform.

## Design Overview

The design consists of:
- **Microwatt CPU Core**: Open-source POWER ISA processor
- **Wishbone Interconnect**: Standard bus fabric connecting CPU to peripherals
- **Boot ROM**: 4KB read-only memory containing bootloader
- **System SRAM**: 16KB read-write memory using ChipFoundry commercial SRAM
- **GPIO**: 11 general-purpose I/O pins
- **UART**: Serial communication interface

## Memory Map

| Address Range         | Size  | Device      | Description                    |
|-----------------------|-------|-------------|--------------------------------|
| 0x0000_0000-0x0000_0FFF | 4 KB  | Boot ROM    | Read-only bootloader code      |
| 0x0000_1000-0x0000_1FFF | 4 KB  | Unmapped    | Reserved/unused                |
| 0x0000_2000-0x0000_5FFF | 16 KB | SRAM        | Read-write program memory      |
| 0x0000_6000+          | -     | Reserved    | Future accelerator/peripherals |

## File Organization

### Top Level
- `soc_top.sv` - Top-level SoC module (OpenFrame wrapper compatible)

### CPU Core
- `cpu/mw_top.v` - Microwatt wrapper with clean Wishbone interface
- `cpu/mw.v` - Synthesized Microwatt core (VHDL → Verilog conversion)
- `cpu/wb_mux.sv` - Wishbone fabric for address decoding

### Memory Subsystem
- `rtl/wb_rom.v` - Wishbone ROM controller
- `rom.hex` - Boot ROM initialization data (PowerPC machine code)
- `sram/CF_SRAM_4096x32/hdl/bus_wrapper/CF_SRAM_4096x32_wb_wrapper.v` - SRAM Wishbone wrapper
- `sram/CF_SRAM_4096x32/hdl/controllers/ram_controller_wb.v` - SRAM controller
- `sram/CF_SRAM_4096x32/hdl/CF_SRAM_4096x32.v` - SRAM macro (behavioral for sim)

### SRAM Hard Macro Files (for tapeout)
- `sram/CF_SRAM_4096x32/gds/CF_SRAM_4096x32.gds` - SRAM layout
- `sram/CF_SRAM_4096x32/lef/CF_SRAM_4096x32.lef` - SRAM abstract view
- `sram/CF_SRAM_4096x32/lib/CF_SRAM_4096x32_*.lib` - SRAM timing models

## Module Hierarchy

```
soc_top
├── u_cpu (microwatt_top)
│   └── u_soc (soc) - Microwatt core from mw.v
├── u_wb (wb_fabric)
├── u_boot_rom (wb_rom)
└── u_sys_sram (CF_SRAM_4096x32_wb_wrapper)
    ├── i_ram_wb_controller (ram_controller_wb)
    └── i_sram (CF_SRAM_4096x32)
```

## Interface Signals (soc_top)

### Power Pins (ifdef USE_POWER_PINS)
- `vccd1` - 1.8V digital supply
- `vssd1` - Digital ground

### Clock and Reset
- `wb_clk_i` - System clock input
- `wb_rst_i` - System reset input (active high)

### User I/O
- `io_in[10:0]` - User input pins
- `io_out[10:0]` - User output pins
- `io_oeb[10:0]` - Output enable (active low)

### UART
- `uart_rx` - UART receive
- `uart_tx` - UART transmit

## Boot Sequence

1. CPU starts at ROM address 0x0000_0000
2. Boot ROM executes minimal bootloader
3. Current stub: Immediate branch to SRAM (0x0000_2000)
4. User program in SRAM executes

## Synthesis Flow

### For Initial Testing (without hard macro)
```bash
# Use behavioral SRAM model
# Include all files from rtl_filelist.txt
```

### For Tapeout (with hard macro)
```bash
# Replace behavioral SRAM with hard macro instantiation
# Include SRAM LEF, LIB, and GDS files in OpenLane configuration
# Set EXTRA_LEFS, EXTRA_LIBS, EXTRA_GDS_FILES in config.tcl
```

## OpenFrame Integration

The `soc_top` module is designed to be instantiated directly in the OpenFrame user project wrapper:

```verilog
openframe_project_wrapper
└── mprj (soc_top)
    ├── GPIO mapping: clock(gpio[0]), reset(gpio[1]), 
    │                 user_io(gpio[12:2]), uart(gpio[13])
    └── Power connections: vccd1, vssd1
```

## Design Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| Clock Frequency | Target ~50MHz | Adjust in OpenLane config |
| ROM Size | 4 KB (1024 words) | ADDR_SIZE=10 in wb_rom |
| SRAM Size | 16 KB (4096 words) | WIDTH=14 in SRAM wrapper |
| Data Width | 32 bits | Standard word size |
| Address Width | 32 bits | Byte-addressed |

## Next Steps

1. ✅ Create RTL files
2. ✅ Create boot ROM stub
3. ✅ Integrate ChipFoundry SRAM
4. 🔲 Run initial OpenLane synthesis
5. 🔲 Develop full bootloader with UART programming
6. 🔲 Add AI accelerator as third Wishbone slave
7. 🔲 Complete place and route
8. 🔲 Run timing analysis and optimization
9. 🔲 Generate final GDS for tapeout

## License

All RTL sources use Apache-2.0 license (compatible with OpenPOWER and hackathon requirements).

## References

- [Microwatt GitHub](https://git.openpower.foundation/cores/microwatt)
- [ChipFoundry Microwatt Challenge](https://chipfoundry.io/challenges/microwatt)
- [OpenFrame User Project](https://github.com/chipfoundry/openframe_user_project)
- [ChipFoundry Commercial SRAM](https://chipfoundry.io/commercial-sram)

