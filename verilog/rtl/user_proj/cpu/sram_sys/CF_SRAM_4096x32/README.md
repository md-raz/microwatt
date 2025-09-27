# CF_SRAM_4096x32

This is a 4096x32 SRAM macro that is built using four 1024x32 SRAM macros. It provides a Wishbone compliant slave interface for easy integration into SoC designs.

## Structure

The 4096x32 SRAM macro consists of:

1. **CF_SRAM_4096x32.v** - The main SRAM macro that instantiates four 1024x32 SRAMs with address decoding and data multiplexing
2. **CF_SRAM_4096x32_wb_wrapper.v** - Wishbone compliant wrapper that provides the bus interface
3. **controllers/ram_controller_wb.v** - Wishbone controller that translates bus transactions to SRAM signals

## Features

- **Capacity**: 4096 words × 32 bits = 16KB
- **Address Width**: 12 bits (4096 = 2^12)
- **Data Width**: 32 bits
- **Interface**: Wishbone compliant slave
- **Byte Enable**: Supports byte-level writes using wbs_sel_i[3:0]
- **Scan Chain**: Includes scan chain support for testing

## Address Mapping

The 4096x32 SRAM uses the following address mapping:
- Address bits [11:10]: Select which 1024x32 SRAM macro (00, 01, 10, 11)
- Address bits [9:0]: Word address within each 1024x32 SRAM

## Dependencies

This macro depends on the CF_SRAM_1024x32 macro, which must be available in your design. 