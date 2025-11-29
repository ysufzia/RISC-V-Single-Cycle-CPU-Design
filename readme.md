# RISC-V Single-Cycle CPU Design
A hardware implementation of a RISC-V processor with custom vector instructions, written in Verilog. This CPU design demonstrates fundamental computer architecture concepts including instruction fetch, decode, execute, memory access, and write-back stages—all within a single clock cycle.

## 🎯 Features
- **Single-cycle execution**: Each instruction completes in one clock cycle
- **13 RISC-V instructions** including:
  - Arithmetic: `ADD`, `SUB`, `ADDI`
  - Logical: `AND`, `SRL`
  - Comparison: `SLT` (Set Less Than)
  - Memory: `LW` (Load Word), `SW` (Store Word)
  - Control flow: `BEQ`, `BLT`, `JAL`, `JALR`
  - **Custom vector operations**: `add_v` (vector add), `avg_v` (vector average)
- **32-bit data path** with 32 general-purpose registers
- **Harvard architecture** with separate instruction and data memories
- **Functional ALU** supporting multiple operations including custom vector instructions

## Supported Instructions

| Instruction | Type | Description |
|-------------|------|-------------|
| `ADD`  | R-type | Add two registers |
| `SUB`  | R-type | Subtract two registers |
| `AND`  | R-type | Bitwise AND |
| `SRL`  | R-type | Shift right logical |
| `SLT`  | R-type | Set less than (signed) |
| `ADDI` | I-type | Add immediate |
| `LW`   | I-type | Load word from memory |
| `SW`   | S-type | Store word to memory |
| `BEQ`  | B-type | Branch if equal |
| `BLT`  | B-type | Branch if less than |
| `JAL`  | J-type | Jump and link |
| `JALR` | I-type | Jump and link register |
| `add_v`| Custom | Vector add (4×8-bit SIMD) |
| `avg_v`| Custom | Vector average (4×8-bit SIMD) |

## Test Program: Median Filter

The CPU is validated using a **3-element median filter** implementation in RISC-V assembly. This program:

1. Loads an array from memory
2. Iterates through the array
3. For each element, computes the median of three consecutive values (left, center, right)
4. Stores the filtered result back to memory

The median filter demonstrates:
- Function calls with `JAL`/`JALR`
- Conditional branching with `BEQ`/`BLT`
- Memory operations with `LW`/`SW`
- Arithmetic and logical operations

**Files:**
- `Median_prog.asm` - Assembly source code
- `Median_prog.hex` - Machine code in hexadecimal format


### Key Modules
- **`processor`**: Top-level CPU module integrating all components
- **`pcounter`**: Program counter register
- **`GPR`**: General Purpose Register file (32 registers)
- **`alu`**: Arithmetic Logic Unit with 7 operations
- **`ctrl_unit`**: Instruction decoder and control signal generator
- **`imm_decode`**: Immediate value decoder for I/S/B/U/J formats
- **`inst_mem`**: Instruction memory (ROM)
- **`data_mem`**: Data memory (RAM)

### Prerequisites

- Verilog simulator (ModelSim, Icarus Verilog, Verilator, etc.)
- (Optional) RISC-V assembler for creating custom programs

### Running the Simulation

1. Clone the repository:
```bash
git clone https://github.com/ysufzia/RISC-V-Single-Cycle-CPU-Design.git
cd RISC-V-Single-Cycle-CPU-Design
```

2. Compile the Verilog files:
```bash
iverilog -o cpu_sim RISC-VCPU.v
```

3. Run the simulation:
```bash
vvp cpu_sim
```

### Loading Custom Programs

To run your own programs:

1. Write RISC-V assembly code
2. Assemble to machine code
3. Convert to hex format
4. Replace `memfile_inst.hex` with your program
5. Update `memfile_data.hex` for initial data memory values

## 📊 Performance
- **Clock period**: Determined by critical path (typically ALU + memory access)
- **CPI**: 1.0 (single-cycle design)
- **Throughput**: One instruction per cycle

## 🔧 Custom Vector Instructions
### `add_v` (Vector Add)
- **Opcode**: `0001011` (custom-0)
- **funct3**: `000`
- Performs parallel addition of four 8-bit elements
- Example: `0x12345678 + 0x11111111 = 0x23456789`

### `avg_v` (Vector Average)
- **Opcode**: `0001011` (custom-0)
- **funct3**: `001`
- Computes average of four 8-bit element pairs with proper rounding
- Example: `avg_v(0xFF00FF00, 0x01000100) = 0x80008000`

## 📝 Design Decisions
- **Single-cycle architecture**: Simplifies control logic but limits clock frequency
- **Separate memories**: Harvard architecture allows simultaneous instruction fetch and data access
- **32-bit data path**: Standard RISC-V RV32I base integer instruction set
- **Custom opcode space**: Uses reserved custom-0 opcode for vector extensions

## 👨‍💻 Author

[Yusuf Ziya Dönergöz]