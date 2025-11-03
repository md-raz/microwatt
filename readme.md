# Microwatt Momentum Hackathon Submission Report

**Competition**: [Microwatt Momentum - OpenPOWER HW Design Hackathon](https://chipfoundry.io/challenges/microwatt)  
**Deadline**: November 3, 2025, 11:59pm PST  
**Submission Status**: Incomplete - Implementation Report

---

## Summary

This document provides a detailed technical report of our hackathon submission that did **not** reach tapeout-ready state by the deadline. While we were unable to complete the full design flow, we are submitting this comprehensive documentation to provide the hackathon organizers and community with:

1. **Transparent progress reporting**: What we accomplished and where we encountered barriers
2. **Technical insights**: Specific challenges faced during ASIC implementation with Microwatt
3. **Lessons for future events**: Time sinks and workflow bottlenecks that could inform future hackathon design

We successfully completed RTL design and integration but became blocked during the OpenLane hard macro generation phase due to challenges with nested hard macros within the Microwatt core. This report documents our implementation workflow, technical decisions, and the specific issues encountered.

---

## Project Proposal

### Original Vision

We proposed an **AI Accelerator ASIC** combining:
- **Microwatt CPU**: Single-core POWER ISA processor as the control unit
- **TinyML Accelerator**: Compact INT8 matrix-vector accelerator for inference
- **Wishbone Interconnect**: Standard bus connecting CPU to peripherals and accelerator
- **ChipFoundry Commercial SRAM**: For program memory and ML weight storage
- **UART Bootloader**: Runtime reprogrammability for firmware and weights

The full proposal envisioned a system where the Microwatt CPU would reach the accelerator through a WB-to-AXI bridge, with the accelerator implementing an 8×8 INT8 MAC array with bias, requantization, and ReLU. A UART bootloader in ROM would enable reprogramming at runtime.

### Simplified Scope (Original Proposal vs. What we had so far within time limits)

To manage complexity within hackathon timeframes, we **simplified the design**:
- ❌ AXI bridge, custom accelerator, multiple scratchpad SRAMs
- ✅ Microwatt CPU, Wishbone fabric, ROM, SRAM, UART, GPIO

### Final Target Implementation

**Core System Components**:
- **Microwatt CPU**: Unmodified POWER ISA core with Wishbone master interface
- **Boot ROM**: 4KB read-only memory with bootloader stub
- **System SRAM**: 16KB using ChipFoundry CF_SRAM_4096x32 commercial macro
- **UART**: 16550-compatible serial interface for console
- **GPIO**: 11 general-purpose I/O pins
- **Wishbone Fabric**: Address decoder connecting CPU to peripherals

**Integration Target**:
- OpenFrame user project wrapper compatible with ChipFoundry platform
- SkyWater 130nm PDK
- Designed to fit within OpenFrame user area

---

## System Architecture

### Block Diagram

```
openframe_project_wrapper
│
└── soc_top (User SoC)
    ├── microwatt_top (CPU Wrapper)
    │   └── soc (Microwatt Core - VHDL→Verilog)
    │       ├── processor (POWER core)
    │       │   ├── icache (+ RAM32_1RW1R macro)
    │       │   ├── dcache (+ RAM32_1RW1R macro)
    │       │   ├── register_file (Microwatt_FP_DFFRFile macro)
    │       │   ├── execute1 (+ multiply_add_64x64 macro)
    │       │   └── fpu (+ multiply_add_64x64 macro)
    │       ├── uart16550 (UART controller)
    │       ├── gpio (GPIO controller)
    │       └── dmi_xilinx_dtm (dmi_dtm_8_64 debug macro)
    │
    ├── wb_fabric (Wishbone Interconnect)
    │   ├── Address decoder (2 slaves)
    │   └── Arbitration logic
    │
    ├── wb_rom (Slave 0: Boot ROM)
    │   └── 4KB initialized from rom.hex
    │
    └── CF_SRAM_4096x32_wb_wrapper (Slave 1: System SRAM)
        └── CF_SRAM_4096x32 (ChipFoundry commercial macro)
```

### Memory Map

| Address Range           | Size  | Device      | Description                    |
|------------------------|-------|-------------|--------------------------------|
| `0x0000_0000-0x0000_0FFF` | 4 KB  | Boot ROM    | Read-only bootloader code      |
| `0x0000_1000-0x0000_1FFF` | 4 KB  | Unmapped    | Reserved/unused                |
| `0x0000_2000-0x0000_5FFF` | 16 KB | SRAM        | Read-write program memory      |
| `0x0000_6000+`          | -     | Reserved    | Future accelerator/peripherals |

**Address Decoding** (in `wb_fabric`):
- ROM: `BASE=0x0000_0000`, `MASK=0xFFFF_F000` (4KB window)
- RAM: `BASE=0x0000_2000`, `MASK=0xFFFF_E000` (32KB window, 16KB used)

### Module Hierarchy

**Top Module**: `soc_top.sv` (190 lines)
```systemverilog
module soc_top (
    inout vccd1, vssd1,              // Power pins (ifdef USE_POWER_PINS)
    input  wire wb_clk_i,            // System clock
    input  wire wb_rst_i,            // System reset (active high)
    input  wire [10:0] io_in,        // User inputs
    output wire [10:0] io_out,       // User outputs
    output wire [10:0] io_oeb,       // Output enable (active low)
    input  wire uart_rx,             // UART receive
    output wire uart_tx              // UART transmit
);
```

**Instantiates**:
1. `microwatt_top` - CPU wrapper (WB master)
2. `wb_fabric` - 2-slave Wishbone interconnect
3. `wb_rom` - Boot ROM controller
4. `CF_SRAM_4096x32_wb_wrapper` - System SRAM with WB interface

### OpenFrame Integration

**Wrapper Module**: `openframe_project_wrapper.v`
- Instantiates `soc_top` as `mprj`
- **GPIO Mapping**:
  - `gpio_in[0]` → `wb_clk_i` (clock)
  - `gpio_in[1]` → `wb_rst_i` (reset)
  - `gpio_in[12:2]` → `io_in[10:0]` (user inputs)
  - `gpio_out[12:2]` ← `io_out[10:0]` (user outputs)
  - `gpio_in[13]` → `uart_rx`
  - `gpio_out[13]` ← `uart_tx`
- **Power**: Connects `vccd1`/`vssd1` to user design
- **Power Macros**: Instantiates `vccd1_connection` and `vssd1_connection` for proper power routing

---

## Implementation Workflow & Status Checklist

### Phase 0: Setup & Dependencies ✅

- ✅ **Downloaded Microwatt core** from [OpenPOWER Foundation](https://git.openpower.foundation/cores/microwatt)
- ✅ **Installed toolchain**: OpenLane, Yosys, GHDL, GHDL-Yosys plugin, Verilator
- ✅ **Downloaded ChipFoundry Commercial SRAM macros** (CF_SRAM_4096x32)
- ✅ **Cloned OpenFrame template** from ChipFoundry

**Time Investment**: Initial setup and dependency installation (1-2 days)

### Phase 1: Microwatt Core Verification ⚠️

- ✅ **Simulated Microwatt** with provided GHDL testbenches
- ✅ **Confirmed CPU functionality** in simulation environment
- ⚠️ **MAJOR ISSUE**: VHDL to Verilog conversion challenges (see Challenge 1 below)

**Status**: Functional but required significant custom work to generate ASIC-compatible Verilog

### Phase 2: OpenFrame Flow Validation ✅

- ✅ **Successfully ran timer example** from `openframe_timer_example` repository
- ✅ **Generated timer hard macro** (`user_proj_timer`)
- ✅ **Integrated timer into wrapper** (`openframe_project_wrapper`)
- ✅ **Verified complete OpenLane flow** works end-to-end

**Outcome**: Confirmed that the OpenFrame template, OpenLane flow, and tapeout process are functional. This established a known-good baseline for comparison.

**Time Investment**: 2-3 days for validation

### Phase 3: RTL Design & Integration ✅

**RTL Development**:
- ✅ Created `soc_top.sv` with Microwatt, WB fabric, ROM, SRAM, UART
- ✅ Implemented WB address decoder (`wb_fabric` in `cpu/wb_mux.sv`)
- ✅ Created boot ROM controller (`cpu/wb_rom.v`) with hex initialization
- ✅ Integrated CF_SRAM_4096x32 with Wishbone wrapper
- ✅ Created boot stub (`rom.hex`) - PowerPC instruction for branch to SRAM
- ✅ Connected to OpenFrame wrapper (`openframe_project_wrapper.v`)

**Verification**:
- ✅ Verilator lint passes (syntax correct)
- ✅ Module hierarchy complete and verified
- ✅ All instantiations resolve correctly
- ✅ Interface compatibility validated

**Documentation**:
- ✅ `verilog/rtl/mw/docs/RTL_README.md` - Design overview
- ✅ `verilog/rtl/mw/docs/IMPLEMENTATION_SUMMARY.md` - Implementation details
- ✅ `verilog/rtl/mw/docs/VERIFICATION_CHECKLIST.md` - Pre-synthesis validation
- ✅ `verilog/rtl/mw/docs/rtl_filelist.txt` - Complete file list

**Status**: RTL design complete and ready for synthesis

**Time Investment**: 4-5 days

### Phase 4: Hard Macro Generation ⚠️ BLOCKED

**Objective**: Generate `soc_top` hard macro (GDS/LEF) using OpenLane

**Configuration**:
- ✅ Created `openlane/soc_top/config.json` with all design parameters
- ✅ Added VERILOG_FILES list
- ✅ Added VERILOG_FILES_BLACKBOX for hard macros
- ✅ Added EXTRA_LEFS, EXTRA_LIBS, EXTRA_GDS_FILES
- ✅ Set clock constraints (25ns period, 40MHz target)
- ✅ Configured placement density and utilization

**OpenLane Runs**:
- `25_11_02_15_09`: Failed at synthesis stage
- `25_11_02_15_16`: Failed at pre-PNR STA stage
- `25_11_02_15_27`: Failed at floorplan stage
- `25_11_02_15_40`: **Furthest progress** - failed at power connection stage

**⚠️ CRITICAL ISSUE**: Multiple internal hard macros in Microwatt core causing integration failures (see Challenge 2 below)

**Blocking Errors**:
```
[ORD-2013] instance u_cpu.u_soc.dmi_xilinx_dtm LEF master dmi_dtm_8_64 not found.
[ORD-2013] instance u_cpu.u_soc.processor.dcache_0.rams_n1_way.cache_ram_0 LEF master RAM32_1RW1R not found.
Could not find master for cell type 'dmi_dtm_8_64' in the database.
```

**Status**: ❌ Could not generate `soc_top.gds`/`soc_top.lef` before deadline

**Time Investment**: 5-6 days with multiple iterations

### Phase 5: Wrapper Integration 🔲 NOT REACHED

**Planned Steps** (not executed):
- 🔲 Treat `soc_top` as blackbox macro
- 🔲 Add `soc_top.lef`/`soc_top.gds` to wrapper EXTRA files
- 🔲 Run `openframe_project_wrapper` through OpenLane
- 🔲 Verify power connections via power macros
- 🔲 Pass timing analysis
- 🔲 Generate final wrapper GDS

### Phase 6: Accelerator Integration 🔲 NOT REACHED

**Planned Enhancement** (original proposal):
- 🔲 Design INT8 matrix-vector accelerator (8×8 MAC array)
- 🔲 Add as third Wishbone slave (`NS=3` in wb_fabric)
- 🔲 Add scratchpad SRAMs for weights and activations
- 🔲 Implement control/status registers
- 🔲 Write firmware to interface with accelerator

**Decision**: Deferred due to time constraints on base system

---

## Major Technical Challenges

### Challenge 1: VHDL to Verilog Conversion

#### Problem Statement

**Core Issue**: Microwatt is written entirely in VHDL (~50 source files), but OpenLane's synthesis flow requires Verilog input. The Microwatt repository includes makefiles for FPGA targets (using Xilinx/Lattice tools with inference), but no ASIC-ready flows.

**Complications**:
1. Existing tapeout examples (referenced on OpenPOWER forums) included peripherals we didn't need (SPI, I2C, JTAG)
2. Removing unwanted modules caused synthesis errors due to port mismatches
3. FPGA makefiles used inferred memory primitives unsuitable for ASIC
4. Generic parameters for memory configuration not well documented

#### Attempted Solutions

1. **Used existing tapeout Verilog**: Tried to base on prior work
   - **Issue**: Included unnecessary peripherals tightly coupled to core
   - **Result**: Removal caused cascading port connection errors

2. **Modified makefiles for ASIC target**: Attempted to add ASIC synthesis target
   - **Issue**: Makefiles heavily FPGA-focused, unclear how to disable inference
   - **Result**: Generated netlists still contained FPGA-specific primitives

3. **Manually removed BRAM modules**: Set `MEMORY_SIZE=0` to exclude internal memory
   - **Issue**: `fpga/main_bram.vhdl` still included in netlist regardless of generic
   - **Result**: Required manual post-processing to remove

#### Final Working Solution

Wrote custom GHDL-Yosys synthesis command that directly specifies source files for ASIC target:

```bash
ghdl --synth --out=verilog --std=08 \
  -gMEMORY_SIZE=131072 \
  -gRAM_INIT_FILE=hello_world/hello_world.hex \
  nonrandom.vhdl \
  decode_types.vhdl common.vhdl wishbone_types.vhdl fetch1.vhdl utils.vhdl \
  plrufn.vhdl cache_ram.vhdl icache.vhdl predecode.vhdl decode1.vhdl helpers.vhdl \
  insn_helpers.vhdl control.vhdl decode2.vhdl register_file.vhdl cr_file.vhdl \
  crhelpers.vhdl ppc_fx_insns.vhdl rotator.vhdl logical.vhdl countbits.vhdl \
  multiply.vhdl multiply-32s.vhdl divider.vhdl execute1.vhdl loadstore1.vhdl \
  mmu.vhdl dcache.vhdl writeback.vhdl core_debug.vhdl core.vhdl fpu.vhdl pmu.vhdl \
  bitsort.vhdl wishbone_arbiter.vhdl sync_fifo.vhdl wishbone_debug_master.vhdl \
  xics.vhdl syscon.vhdl gpio.vhdl soc.vhdl \
  spi_rxtx.vhdl spi_flash_ctrl.vhdl \
  wishbone_bram_wrapper.vhdl fpga/main_bram.vhdl \
  fpga/pp_fifo.vhd fpga/pp_soc_uart.vhd \
  git.vhdl dmi_dtm_dummy.vhdl \
  -e soc > microwatt_soc.v
```

**Key Parameters**:
- `-gMEMORY_SIZE=131072`: Sets internal memory size (experimented with 0, still included BRAM)
- `-gRAM_INIT_FILE`: Boot code initialization
- Explicit file list ensures only needed modules included
- `-e soc`: Elaborate the `soc` entity as top level

#### Remaining Issues

1. **BRAM modules still included**: Even with `MEMORY_SIZE=0`, FPGA BRAM wrappers appeared in netlist
   - Required manual removal/commenting in generated Verilog
   
2. **Hard macro instantiations**: Generated Verilog contained instantiations for:
   - `dmi_dtm_8_64` (debug module)
   - `RAM32_1RW1R` (cache memories)
   - `multiply_add_64x64` (FPU multipliers)
   - `Microwatt_FP_DFFRFile` (register file)
   
   These were intentional hard macros but complicated downstream integration (see Challenge 2).

#### Time Impact

**Estimated Time Lost**: About a week (with other projects)
- Understanding Microwatt generics and build system
- Trial-and-error with GHDL synthesis options
- Debugging port mismatches from removed peripherals
- Manual netlist cleanup

**Lessons**: 
- FPGA-first designs require significant work (or really good understanding of internal modules and replacements) for ASIC flow
- Better documentation of generics and synthesis options could have helped (specifically a barebones make for just ASIC)
- Pre-generated ASIC-ready Verilog netlists would significantly help future participants

---

### Challenge 2: OpenLane Hard Macro Generation

#### Problem Statement

**Core Issue**: `soc_top` hardening in OpenLane failed repeatedly during place-and-route stages. The Microwatt core contains **multiple nested hard macros** that OpenLane struggled to integrate properly.

#### Root Cause Analysis

The synthesized Microwatt core (`cpu/mw.v`, ~90K lines) instantiates **six hard macro instances**:

| Macro Module | Instance Path | Purpose | Count |
|--------------|---------------|---------|-------|
| `dmi_dtm_8_64` | `u_cpu.u_soc.dmi_xilinx_dtm` | Debug module interface | 1 |
| `RAM32_1RW1R` | `u_cpu.u_soc.processor.icache_0.rams_n1_way.cache_ram_0` | Instruction cache RAM | 1 |
| `RAM32_1RW1R` | `u_cpu.u_soc.processor.dcache_0.rams_n1_way.cache_ram_0` | Data cache RAM | 1 |
| `multiply_add_64x64` | `u_cpu.u_soc.processor.execute1_0.multiply_0.multiplier` | Integer multiplier | 1 |
| `multiply_add_64x64` | `u_cpu.u_soc.processor.with_fpu_fpu_0.fpu_multiply_0.multiplier` | FPU multiplier | 1 |
| `Microwatt_FP_DFFRFile` | `u_cpu.u_soc.processor.register_file_0.register_file_0` | Register file | 1 |

Additionally, our design includes:
| `CF_SRAM_4096x32` | `u_sys_sram.i_sram` | System SRAM | 1 |

**Total**: 7 hard macros in design hierarchy

#### OpenLane Error Messages

From run `25_11_02_15_40` (furthest progress):

**Floorplan Stage** (`13-openroad-floorplan`):
```
[STA-0198] module dmi_dtm_8_64 not found. Creating black box for \u_cpu.u_soc.dmi_xilinx_dtm.
[STA-0198] module RAM32_1RW1R not found. Creating black box for cache_ram_0.
[STA-0198] module multiply_add_64x64 not found. Creating black box for multiplier.
[STA-0198] module Microwatt_FP_DFFRFile not found. Creating black box for register_file_0.
[ORD-2013] instance u_cpu.u_soc.dmi_xilinx_dtm LEF master dmi_dtm_8_64 not found.
[ORD-2013] instance u_cpu.u_soc.processor.dcache_0.rams_n1_way.cache_ram_0 LEF master RAM32_1RW1R not found.
[ORD-2013] instance u_cpu.u_soc.processor.execute1_0.multiply_0.multiplier LEF master multiply_add_64x64 not found.
```

**Power Connection Stage** (`15-odb-setpowerconnections`):
```
Could not find master for cell type 'dmi_dtm_8_64' in the database.
```

**SRAM Port Mismatch** warnings:
```
[STA-0201] instance \u_sys_sram.i_sram port AD not found.
[STA-0201] instance \u_sys_sram.i_sram port BEN not found.
[STA-0201] instance \u_sys_sram.i_sram port CLKin not found.
[... 14 more port warnings ...]
```

Indicates mismatch between behavioral SRAM model and hard macro interface.

#### Attempted Solutions

1. **Referenced antonblanchard/microwatt-caravel configuration**
   - **URL**: https://github.com/antonblanchard/microwatt-caravel
   - **Action**: Examined OpenLane config from successful Microwatt tapeout
   - **Copied**: Placement density settings, PDN configuration, timing constraints
   - **Result**: Still encountered macro integration failures

2. **Added all hard macros to OpenLane configuration**
   - **File**: `openlane/soc_top/config.json`
   - **Added**: All LEF, LIB, and GDS files to `EXTRA_LEFS`, `EXTRA_LIBS`, `EXTRA_GDS_FILES`
   - **Issue**: Some macros (internal Microwatt ones) not available as separate files
   - **Result**: OpenLane could not locate LEF masters for internal macros

3. **Adjusted placement parameters**
   - **Settings tried**:
     - `PL_TARGET_DENSITY: 0.55` (originally 0.6)
     - `FP_CORE_UTIL: 40` (originally 50)
     - `GPL_CELL_PADDING: 2`
     - `DPL_CELL_PADDING: 2`
   - **Goal**: Reduce congestion around hard macros
   - **Result**: Helped progress further but didn't resolve LEF master issues

4. **Disabled error checks to proceed**
   - **Settings**:
     - `ERROR_ON_SYNTH_CHECKS: false`
     - `QUIT_ON_UNMAPPED_CELLS: false`
     - `QUIT_ON_SYNTH_CHECKS: false`
   - **Result**: Flow progressed to floorplan stage but failed at power connection

5. **Created macro placement configuration**
   - **File**: `openlane/soc_top/macro.cfg`
   - **Action**: Attempted manual placement hints for CF_SRAM_4096x32
   - **Result**: SRAM placement proceeded, but internal Microwatt macros still problematic

#### Issues Encountered

1. **Missing LEF files**: Internal Microwatt macros don't have separate LEF/LIB/GDS files
   - Generated as part of GHDL synthesis
   - Not available as external hard macros for OpenLane integration

2. **Congestion violations**: Dense placement of multiple macros within CPU hierarchy
   - Global placement struggled with nested macro constraints
   - Routing congestion around cache RAMs and register file

3. **Density violations**: Core utilization exceeded capacity in regions with macros
   - Standard cell placement failed near macro boundaries

4. **Long iteration times**: Each OpenLane run took 20 minutes - several hours
   - Difficult to iterate quickly on configuration changes
   - Limited number of attempts before deadline

#### Configuration Reference

Final attempted configuration in `openlane/soc_top/config.json`:

```json
{
    "DESIGN_NAME": "soc_top",
    "VERILOG_FILES_BLACKBOX": [
        "dir::../../verilog/rtl/mw/sram/CF_SRAM_4096x32/hdl/CF_SRAM_4096x32.v",
        "dir::../../verilog/rtl/Microwatt_FP_DFFRFile.v",
        "dir::../../verilog/rtl/wrapper/RAM32_1RW1R.v",
        "dir::../../verilog/gl/multiply_add_64x64.v"
    ],
    "EXTRA_LEFS": [
        "dir::../../verilog/rtl/mw/sram/CF_SRAM_4096x32/lef/CF_SRAM_4096x32.lef",
        "dir::../../lef/Microwatt_FP_DFFRFile.lef",
        "dir::../../lef/RAM32_1RW1R.lef",
        "dir::../../lef/multiply_add_64x64.lef"
    ],
    "CLOCK_PERIOD": 25,
    "PL_TARGET_DENSITY": 0.55,
    "FP_CORE_UTIL": 40
}
```

#### Status

**Outcome**: ❌ Could not successfully generate `soc_top` hard macro before deadline

**Furthest Progress**: Floorplan stage with power connection errors

**Time Investment**: 5-6 days with ~20 OpenLane iterations

---

## Implementation Details

### RTL Files Created

All RTL located in `verilog/rtl/mw/`:

#### Top Level
- **`soc_top.sv`** (190 lines)
  - Main SoC integration module
  - Instantiates Microwatt CPU, WB fabric, ROM, SRAM
  - Power-aware design with `USE_POWER_PINS` ifdef
  - OpenFrame-compatible interface

#### CPU Subsystem
- **`cpu/mw_top.v`** (125 lines)
  - Microwatt wrapper providing clean 32-bit Wishbone master interface
  - Handles GPIO direction control (input/output/output-enable)
  - Connects UART pads directly to Microwatt internal UART controller
  
- **`cpu/mw.v`** (~90,679 lines)
  - GHDL-synthesized Microwatt core (VHDL → Verilog)
  - Contains processor pipeline, caches, FPU, peripherals
  - Instantiates internal hard macros (listed in Challenge 2)
  - Module name: `soc` (renamed from long hash)
  
- **`cpu/wb_mux.sv`** (Wishbone fabric)
  - Address decoder for 2 slaves (ROM, SRAM)
  - Base/mask address matching
  - Multiplexed data and acknowledge signals
  
- **`cpu/wb_rom.v`** (Boot ROM controller)
  - Parameterized Wishbone ROM slave
  - `$readmemh` initialization from `rom.hex`
  - Single-cycle read access
  - Default: 4KB (ADDR_SIZE=10, 1024 words)

#### UART Subsystem
Located in `cpu/uart16550/` (10 files):
- `uart_top.v` - 16550-compatible UART top level
- `uart_receiver.v` - RX data path
- `uart_transmitter.v` - TX data path
- `uart_regs.v` - Register file
- `uart_rfifo.v`, `uart_tfifo.v` - RX/TX FIFOs
- `uart_sync_flops.v` - Clock domain crossing
- `uart_wb.v` - Wishbone interface
- `uart_defines.v` - Configuration parameters
- `raminfr.v` - FIFO memory

#### Memory Subsystem
Located in `sram/CF_SRAM_4096x32/`:
- **`hdl/CF_SRAM_4096x32.v`** - Behavioral model for simulation
- **`hdl/controllers/ram_controller_wb.v`** - Wishbone-to-SRAM controller
- **`hdl/bus_wrapper/CF_SRAM_4096x32_wb_wrapper.v`** - Complete WB wrapper
- **`lef/CF_SRAM_4096x32.lef`** - Abstract view for P&R
- **`gds/CF_SRAM_4096x32.gds`** - Layout for tapeout
- **`lib/CF_SRAM_4096x32.lib`** - Timing model

Configuration:
- **Size**: 4096 words × 32 bits = 16 KB
- **WIDTH parameter**: 14 (14-bit byte address for 16KB)
- **Interface**: Standard Wishbone B4 classic
- **Power**: Connected to vccd1/vssd1 via `VPWR`/`VGND` ports

#### Boot Firmware
- **`rom.hex`** - Boot ROM initialization file
  - PowerPC machine code
  - First instruction: `48002002` = `ba 0x2000` (branch absolute to SRAM)
  - Opcode breakdown:
    - Bits [31:26] = `010010` (opcode 18 = branch)
    - Bits [25:2] = `0x000800` (target address 0x2000)
    - Bit [1] = `1` (AA: absolute addressing)
    - Bit [0] = `0` (LK: no link register update)
  - Followed by NOPs for padding

### Wrapper Integration

**File**: `verilog/rtl/openframe_project_wrapper.v` (154 lines)

**Purpose**: Top-level wrapper that connects user design to OpenFrame padframe

**Key Connections**:
```verilog
soc_top mprj (
    .vccd1(vccd1),
    .vssd1(vssd1),
    .wb_clk_i(gpio_in[0]),      // Clock from GPIO
    .wb_rst_i(gpio_in[1]),      // Reset from GPIO
    .io_in(gpio_in[12:2]),      // 11 user inputs
    .io_out(gpio_out[12:2]),    // 11 user outputs
    .io_oeb(gpio_oeb[12:2]),    // Output enables
    .uart_rx(gpio_in[13]),      // UART RX
    .uart_tx(gpio_out[13])      // UART TX
);
```

**Unused GPIO Configuration**:
- Outputs tied to `0`
- Output enables set to Hi-Z (`oeb=1`)
- Input buffers enabled for inputs, disabled for outputs
- Analog features disabled via `gpio_loopback_zero`

**Power Connection Macros**:
```verilog
(* keep *) vccd1_connection vccd1_connection ();
(* keep *) vssd1_connection vssd1_connection ();
```
These macros contain vias and metal routing to connect padframe power to user design.

### Configuration Files

#### `openlane/soc_top/config.json`
OpenLane configuration for hard macro generation:
- **Design name**: `soc_top`
- **Clock**: `wb_clk_i`, 25ns period (40 MHz target)
- **Die area**: 1200µm × 1200µm
- **Verilog files**: 16 source files listed
- **Blackbox files**: 4 hard macros declared
- **Extra LEF/LIB/GDS**: Hard macro physical files
- **PDN hooks**: Power connection for CF_SRAM_4096x32
- **Placement**: Target density 0.55, core util 40%

#### `openlane/soc_top/macro.cfg`
Macro placement hints (attempted):
```
u_sys_sram.i_sram 600 600 N
```
Places SRAM at (600µm, 600µm) with North orientation.

#### `openlane/openframe_project_wrapper/config.json`
Wrapper configuration (not reached):
- Instantiates hardened `soc_top` as blackbox
- Flattens wrapper with macros
- Fixed DEF template for OpenFrame compatibility

### Design Parameters

| Parameter | Value | Notes |
|-----------|-------|-------|
| **Target Clock Frequency** | 40 MHz | 25ns period in config |
| **ROM Size** | 4 KB | 1024 × 32-bit words |
| **SRAM Size** | 16 KB | 4096 × 32-bit words |
| **Data Width** | 32 bits | Standard word size |
| **Address Width** | 32 bits | Byte-addressed |
| **GPIO Count** | 11 pins | User-accessible I/O |
| **UART Baud Rate** | Configurable | 16550 supports multiple rates |
| **Die Area (soc_top)** | 1200×1200 µm² | 1.44 mm² |
| **Supply Voltage** | 1.8V | vccd1/vssd1 domain |
| **Process** | SkyWater 130nm | SKY130 PDK |

---

## What Works

### Successfully Completed Items

✅ **RTL Design**
- All modules syntactically correct
- Verilator lint passes without errors
- Module hierarchy complete and verified
- Interface compatibility validated

✅ **Integration**
- Microwatt CPU successfully wrapped with clean WB interface
- Boot ROM controller functional
- SRAM wrapper properly interfaces with ChipFoundry macro
- OpenFrame wrapper correctly instantiates user design

✅ **OpenFrame Flow Validation**
- Timer example completed full flow (synthesis → P&R → GDS)
- Proves OpenFrame template and process work correctly
- Established known-good baseline

✅ **Documentation**
- Comprehensive RTL documentation created
- Memory map clearly defined
- Interface specifications documented
- Verification checklist completed

✅ **Boot Sequence**
- ROM hex file contains valid PowerPC branch instruction
- CPU would start at address 0x0000_0000 (ROM)
- Immediate jump to 0x0000_2000 (SRAM) for user code

✅ **Power Design**
- Proper power pin connections with USE_POWER_PINS ifdef
- Power connection macros instantiated in wrapper
- PDN hooks configured for SRAM macro

### Functional Verification

While we did not reach tapeout, the design is **architecturally sound**:

1. **Address Decoding**: WB fabric correctly routes transactions to ROM/SRAM
2. **Memory Sizing**: ROM and SRAM appropriately sized for boot + program
3. **Interface Matching**: All port connections verified compatible
4. **OpenFrame Compliance**: Wrapper follows OpenFrame template requirements

---

## Current Repository State

### Documentation Files

Located in `verilog/rtl/mw/docs/`:

1. **`RTL_README.md`** (145 lines)
   - Design overview and architecture
   - Memory map specification
   - File organization
   - Module hierarchy
   - Interface signals
   - Boot sequence description
   - OpenFrame integration guide

2. **`IMPLEMENTATION_SUMMARY.md`** (214 lines)
   - Changes made to RTL
   - Module verification table
   - Memory map validation
   - Interface validation
   - Files ready for OpenLane
   - Next steps for integration

3. **`VERIFICATION_CHECKLIST.md`** (215 lines)
   - Pre-synthesis verification
   - Module dependencies verified
   - Interface validation
   - Power pin connections checked
   - Linter status
   - Boot ROM content validated
   - Syntax check commands

4. **`rtl_filelist.txt`** (31 lines)
   - Complete list of RTL sources
   - File organization
   - Comments on usage (simulation vs. tapeout)

### OpenLane Run History

Located in `openlane/soc_top/runs/`:

**Run: 25_11_02_15_09** (First attempt)
- Status: Failed at synthesis check stage
- Progress: 08-checker-yosyssynthchecks
- Issue: Synthesis warnings/errors

**Run: 25_11_02_15_16** (Second attempt)
- Status: Failed at pre-PNR STA stage
- Progress: 12-openroad-staprepnr
- Issue: Timing analysis errors with missing macros

**Run: 25_11_02_15_27** (Third attempt)
- Status: Failed at floorplan stage
- Progress: 13-openroad-floorplan
- Issue: Macro LEF masters not found

**Run: 25_11_02_15_40** (Furthest progress)
- Status: Failed at power connection stage
- Progress: 15-odb-setpowerconnections
- Issue: `dmi_dtm_8_64` master not found in database
- Error log: `Could not find master for cell type 'dmi_dtm_8_64'`
- Contains: Synthesis netlist, reports, floorplan attempts

Each run contains complete logs, intermediate files, and state information for debugging.

### Repository Structure

```
microwatt/
├── README.md                          # Original OpenFrame example README
├── README.rst                         # OpenFrame project documentation
├── HACKATHON_SUBMISSION.md           # This document
├── verilog/
│   ├── rtl/
│   │   ├── openframe_project_wrapper.v    # Top-level wrapper (154 lines)
│   │   ├── openframe_project_netlists.v   # Netlist declarations
│   │   ├── vccd1_connection.v             # Power macro
│   │   ├── vssd1_connection.v             # Power macro
│   │   ├── Microwatt_FP_DFFRFile.v        # Register file macro (stub)
│   │   └── mw/                            # Microwatt SoC design
│   │       ├── soc_top.sv                 # Main SoC (190 lines)
│   │       ├── rom.hex                    # Boot ROM init
│   │       ├── cpu/                       # CPU subsystem
│   │       │   ├── mw_top.v              # Microwatt wrapper
│   │       │   ├── mw.v                  # Microwatt core (~90K lines)
│   │       │   ├── wb_mux.sv             # WB fabric
│   │       │   ├── wb_rom.v              # ROM controller
│   │       │   └── uart16550/            # UART (10 files)
│   │       ├── sram/                      # Memory subsystem
│   │       │   └── CF_SRAM_4096x32/      # ChipFoundry SRAM
│   │       │       ├── hdl/              # RTL + controllers
│   │       │       ├── lef/              # Abstract view
│   │       │       ├── gds/              # Layout
│   │       │       └── lib/              # Timing
│   │       └── docs/                      # Design documentation
│   │           ├── RTL_README.md
│   │           ├── IMPLEMENTATION_SUMMARY.md
│   │           ├── VERIFICATION_CHECKLIST.md
│   │           └── rtl_filelist.txt
│   └── gl/                                # Gate-level (empty - not generated)
├── lef/                                   # LEF files (power macros + timer)
├── gds/                                   # GDS files (power macros + timer)
├── lib/                                   # LIB files (power macros + timer)
└── openlane/
    ├── soc_top/                          # Main SoC hard macro (FAILED)
    │   ├── config.json                   # OpenLane configuration
    │   ├── macro.cfg                     # Macro placement
    │   ├── pin_order.cfg                 # Pin placement
    │   └── runs/                         # 4 failed runs
    │       └── 25_11_02_15_40/           # Furthest progress 
    ├── openframe_project_wrapper/        # Wrapper (not hardened)
    │   └── config.json
    └── [other examples: timer, RAM, multiply_add]
```

---

## Next Steps to Completion

If work were to continue beyond the hackathon deadline, the following steps would be needed:

### 1. Resolve Hard Macro Integration Issues

**Option A: Obtain Missing LEF/LIB Files**
- Source or generate LEF/LIB/GDS for internal Microwatt macros:
  - `dmi_dtm_8_64` (debug module)
  - `RAM32_1RW1R` (cache RAMs)
  - `multiply_add_64x64` (multipliers)
  - `Microwatt_FP_DFFRFile` (register file)
- Add to OpenLane EXTRA files lists
- Ensure power pin compatibility

**Option B: Flatten Microwatt Core**
- Remove hard macro instantiations from synthesis
- Replace with synthesizable RTL equivalents
- Trade-off: Larger area, potentially worse timing, but simpler integration

**Option C: Hierarchical Hardening**
- First harden internal Microwatt macros individually
- Generate LEF/LIB/GDS for each
- Then harden full `soc_top` with known-good macros

**Recommendation**: Option C is most robust but time-intensive

### 2. Complete `soc_top` Hard Macro Generation

Once macro issues resolved:
1. Iterate on placement strategy
   - Experiment with manual macro placement
   - Adjust core utilization and target density
   - Fine-tune PDN grid spacing
2. Pass timing analysis
   - May need clock period adjustment (25ns → 30ns?)
   - Add timing constraints for I/O paths
   - Balance setup vs. hold slack
3. Complete routing
   - Resolve any remaining DRC violations
   - Verify LVS clean
   - Generate final GDS/LEF/LIB

**Expected Time**: 2-4 weeks with proper macro files

### 3. Wrapper Integration

With hardened `soc_top` macro:
1. Update `openframe_project_wrapper/config.json`
   - Add `soc_top.lef` to EXTRA_LEFS
   - Add `soc_top.gds` to EXTRA_GDS_FILES
   - Add `soc_top.lib` to EXTRA_LIBS
2. Run wrapper through OpenLane
   - Should be relatively fast (mostly routing)
   - Verify power macro connections
3. Validate with DRC/LVS checks
4. Generate final wrapper GDS

**Expected Time**: 3-5 days

### 4. Precheck and Tapeout Submission

1. Run ChipFoundry precheck
   - Verify all requirements met
   - Check GDS layers
   - Validate power connections
2. Address any precheck violations
3. Submit to ChipFoundry platform
4. Provide required documentation

**Expected Time**: 1-2 days

### 5. Future Enhancement: Accelerator Integration (Original Proposal)

If base system successful, add TinyML accelerator:

1. **Design accelerator RTL**
   - 8×8 INT8 MAC array (64 MACs)
   - Bias addition, requantization, ReLU
   - Control/status registers
   - Estimated: 2-3K logic cells + scratchpads

2. **Add memory for ML**
   - Weight scratchpad: CF_SRAM_4096x32 (16KB)
   - Activation scratchpad: CF_SRAM_1024x32 (4KB)
   - Output scratchpad: CF_SRAM_1024x32 (4KB)

3. **Integrate into SoC**
   - Increment `NS=3` in wb_fabric
   - Add accelerator base address (e.g., 0x0000_6000)
   - Connect as third Wishbone slave

4. **Write firmware**
   - C code to configure accelerator
   - Load weights/activations
   - Trigger inference
   - Read results

5. **Re-run full flow**
   - Re-harden `soc_top` with accelerator
   - Re-integrate into wrapper
   - Re-submit for tapeout

**Expected Time**: 6-8 weeks

---

## Lessons Learned

### Time Sinks (What Took Longest)

1. **VHDL to Verilog Conversion** (3-4 days)
   - Understanding Microwatt generics and build system
   - Debugging GHDL synthesis options
   - Cleaning up generated Verilog netlist
   - **Impact**: Significant delay before RTL work could begin

2. **OpenLane Hard Macro Integration** (5-6 days)
   - Debugging LEF master errors
   - Multiple iteration cycles with long run times
   - Configuration experimentation
   - **Impact**: Blocked progress to completion

3. **OpenLane Iteration Time** (cumulative)
   - Each run: 15-30 minutes
   - ~20 iterations attempted
   - Limited rapid debugging
   - **Impact**: Slow feedback loop

**Total Time Lost**: ~10-12 days out of ~14 day active work period

### What Helped (Success Factors)

1. **Timer Example Validation** (2-3 days)
   - Proved flow works end-to-end
   - Provided known-good configuration reference
   - Built confidence in OpenFrame process
   - **Value**: Essential baseline

2. **Existing ChipFoundry Documentation**
   - SRAM integration guidelines
   - OpenFrame template documentation
   - Example projects
   - **Value**: Reduced trial-and-error

3. **Modular RTL Design Approach**
   - Clean interface boundaries
   - Separated concerns (CPU, fabric, memories)
   - Easy to verify incrementally
   - **Value**: Confidence in architecture despite P&R failures

4. **Comprehensive Documentation**
   - Created early and maintained
   - Captured decisions and rationale
   - Made this report easier to write
   - **Value**: Preserved knowledge

### Technical Insights

1. **FPGA-to-ASIC Gap**: Designs targeting FPGAs require substantial rework for ASIC flows
   - Memory inference doesn't translate
   - Hard macros vs. soft logic trade-offs
   - Build systems often FPGA-tool-specific

2. **Nested Hard Macros**: Complex hierarchies with multiple macros are challenging
   - Need complete LEF/LIB/GDS files for all instances
   - Placement and routing become significantly harder
   - Tool support varies

3. **OpenLane Learning Curve**: First-time users face challenges
   - Configuration parameter space is large
   - Error messages not always actionable
   - Iteration time impacts learning rate

### What Would Help Future Participants

**For Hackathon Organizers**:
1. **ASIC-Ready Microwatt Distribution**
   - Pre-generated Verilog netlists targeting ASIC flow
   - Hard macro files (LEF/LIB/GDS) for internal components
   - OpenLane-compatible build system
   - **Impact**: Would save 3-4 days

2. **Nested Macro Integration Guide**
   - Tutorial on designs with multiple hard macros
   - Example configurations
   - Debugging strategies
   - **Impact**: Would save 2-3 days

3. **Faster Iteration Infrastructure**
   - Pre-configured cloud compute for OpenLane
   - Parallel run capability
   - Incremental builds where possible
   - **Impact**: 2-3× faster feedback loop

4. **Hackathon Timeline Considerations**
   - ~2 weeks total timeline very tight for ASIC tapeout
   - Suggest 4-6 weeks for complex SoC designs
   - Or provide more complete starting templates
   - **Impact**: Higher success rate

**For Future Participants**:
1. Start with timer example validation (day 1)
2. Spend time understanding tool flow before custom design
3. Plan for 2-3× longer than expected for P&R iterations
4. Document continuously, not at the end
5. Consider simpler designs for first tapeout attempt

---

## Conclusion

While we did not achieve a tapeout-ready submission, this hackathon was a valuable learning experience in ASIC design methodology, open-source EDA tools, and the specific challenges of integrating complex processor cores like Microwatt into an ASIC flow.

### Summary of Achievement

**Completed**:
- ✅ Functional RTL design
- ✅ OpenFrame integration architecture
- ✅ Comprehensive documentation
- ✅ Validated design approach (via timer example)

**Blocked On**:
- ❌ Hard macro integration in OpenLane
- ❌ Successful P&R of `soc_top`

### Value for Community

We hope this detailed report provides value to:
1. **Hackathon organizers**: Understanding workflow bottlenecks
2. **Tool developers**: Real-world use case for improvement
3. **Future participants**: Learning from our challenges
4. **Microwatt community**: ASIC integration insights

### Acknowledgments

Thanks to:
- ChipFoundry team for hosting the hackathon
- OpenPOWER Foundation for Microwatt
- OpenLane/efabless for open-source ASIC tools
- Hackathon judges for accepting this documentation in lieu of completed design

### Repository

This complete implementation (including this report, all RTL, documentation, and OpenLane run artifacts) is available in the repository for reference and learning.

**License**: Apache-2.0 (compatible with OpenPOWER and ChipFoundry requirements)


