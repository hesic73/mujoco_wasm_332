# MuJoCo WASM 3.3.2 Project Documentation

## Project Overview

This project aims to compile the MuJoCo 3.3.2 physics engine into WebAssembly (WASM) for use in JavaScript applications.

## Directory Structure

```
mujoco_wasm_332/
├── include/            # MuJoCo 3.3.2 header files
│   └── mujoco/
├── lib/                # Compiled MuJoCo 3.3.2 library files
├── src/                # Source code and code generation scripts
│   ├── parse_mjxmacro.py     # Main code generation script
│   ├── functions.py          # MuJoCo API function definitions
│   ├── ast_nodes.py          # AST node definitions
│   ├── main.template.cc      # C++ binding template
│   ├── main.genned.cc        # Generated C++ code
│   ├── mujoco_wasm.template.d.ts  # TypeScript definition template
│   └── CMakeLists.txt
├── dist/               # Build output directory
│   └── mujoco_wasm.d.ts      # Generated TypeScript definitions
├── examples/           # Example files and scenes
├── build/              # CMake build directory
├── CMakeLists.txt      # Main CMake configuration
├── package.json
├── index.html
└── README.md
```

## Build Process

### 1. Code Generation Phase

Run the Python script to generate C++ binding code and TypeScript definitions:

```bash
python src/parse_mjxmacro.py
```

This script will:
- Parse macro definitions from `include/mujoco/mjxmacro.h`
- Parse enum types from `include/mujoco/mjmodel.h`
- Read MuJoCo API function definitions from `src/functions.py`
- Generate Emscripten binding code (C++) to `src/main.genned.cc`
- Generate TypeScript type definitions to `dist/mujoco_wasm.d.ts`

### 2. Compilation Phase

Compile to WebAssembly using Emscripten:

```bash
mkdir build
cd build
emcmake cmake ..
make
```

Compilation output:
- `dist/mujoco_wasm.js` - JavaScript loader
- `dist/mujoco_wasm.wasm` - WebAssembly module
- `dist/mujoco_wasm.d.ts` - TypeScript type definitions

## Core Component Description

### parse_mjxmacro.py

The main code generation script, containing the following functions:

1. **Parse pointer fields** (`parse_pointer_line`)
   - Extract array fields of MuJoCo model and data from macro definitions
   - Generate C++ property accessors
   - Generate TypeScript type definitions

2. **Parse integer fields** (`parse_int_line`)
   - Extract integer type fields from macro definitions
   - Generate corresponding C++ and TypeScript code

3. **Parse enums** 
   - Extract enum definitions from `mjmodel.h`
   - Generate Emscripten enum bindings
   - Generate TypeScript enum definitions

4. **Generate function bindings**
   - Iterate through all MuJoCo API functions defined in `functions.py`
   - Generate appropriate wrapper code based on parameter types
   - Handle special types (strings, array pointers, etc.)

### functions.py

Contains declarations of all MuJoCo API functions, each including:
- Function name
- Return type
- Parameter list (names and types)
- Documentation string

This file is manually maintained and needs to stay in sync with the MuJoCo version.

### main.template.cc

C++ template file containing:
- Framework code for Emscripten bindings
- `Model` and `Simulation` class definitions
- Manually written special bindings
- Struct field bindings (such as `mjOption`, `mjLROpt`)

Marker positions (will be replaced by auto-generated code):
- `// MJMODEL_DEFINITIONS` - Model class method definitions
- `// MJMODEL_BINDINGS` - Model class Emscripten bindings
- `// MJDATA_DEFINITIONS` - Simulation class method definitions
- `// MJDATA_BINDINGS` - Simulation class Emscripten bindings
- `// MODEL_ENUMS` - Enum type bindings

### mujoco_wasm.template.d.ts

TypeScript definition template containing:
- Basic interface definitions
- Exported class and method signatures

Marker positions:
- `// MODEL_INTERFACE` - Model interface members
- `// DATA_INTERFACE` - Simulation interface members
- `// ENUMS` - Enum type definitions

## MuJoCo 3.3.2 Version Compatibility

The following modifications were made when upgrading from older versions to 3.3.2:

### 1. Function Signature Changes

**mj_solveM2**
- Old version: `mj_solveM2(m, d, x, y, n)` - 5 parameters
- New version: `mj_solveM2(m, d, x, y, sqrtInvD, n)` - 6 parameters
- Fix location: `src/functions.py` line 1523

### 2. Removed Functions

**mj_activate and mj_deactivate**
- These two functions have been removed in 3.3.2 (only for backward compatibility)
- Fix: Removed these two function definitions from `src/functions.py`

### 3. mjOption Struct Changes

**Removed fields:**
- `mpr_tolerance` - MPR solver tolerance
- `collision` - Collision mode
- `mpr_iterations` - Maximum number of MPR solver iterations

**Added fields:**
- `ls_tolerance` - CG/Newton linesearch tolerance
- `ccd_tolerance` - Convex collision solver tolerance
- `ls_iterations` - Maximum number of CG/Newton linesearch iterations
- `ccd_iterations` - Maximum number of convex collision solver iterations
- `disableactuator` - Bit flags for disabling actuators by group id
- `sdf_initpoints` - Number of starting points for SDF gradient descent
- `sdf_iterations` - Max number of iterations for SDF gradient descent

Fix location: `src/main.template.cc` lines 235-269

### 4. mjLROpt Struct Changes

**Field name correction:**
- `inteval` → `interval` (spelling error fixed)

Fix location: `src/main.template.cc` line 239

### 5. Enum Parsing Error Fix

**Problem:** Parsing enums would cause IndexError if there is no comment

**Fix:** Added safety check in `parse_mjxmacro.py`
```python
potatos = parts[1] if len(parts) > 1 else ""
if potatos:
    auto_gen_lines["enums_typescript"].append("    /** "+potatos.ljust(40)+" */")
```

Fix location: `src/parse_mjxmacro.py` lines 156-159

## Type Mapping

C++ type to JavaScript/TypeScript mapping:

| C++ Type | TypeScript Type | JavaScript Object |
|---------|----------------|----------------|
| `int*` | `Int32Array` | Int32Array |
| `mjtNum*` | `Float64Array` | Float64Array |
| `float*` | `Float32Array` | Float32Array |
| `mjtByte*` | `Uint8Array` | Uint8Array |
| `char*` | `Uint8Array` | Uint8Array |
| `uintptr_t*` | `BigUint64Array` | BigUint64Array |
| `const char*` | `string` | string |
| `int/float/mjtNum` | `number` | number |

## Usage Example

```typescript
import loadMuJoCo from './dist/mujoco_wasm.js';

const mujoco = await loadMuJoCo();

// Load model
const model = mujoco.Model.load_from_xml("scene.xml");

// Create simulation
const simulation = new mujoco.Simulation(model);

// Run one step
simulation.step();

// Access data
const qpos = simulation.qpos;
const qvel = simulation.qvel;

// Call MuJoCo API functions
simulation.forward();
```

## Common Issues

### 1. Compilation warning: format string is not a string literal

This is because `mju_error` and `mju_warning` functions use format strings. This is a known warning and does not affect functionality.

### 2. How to add new MuJoCo functions?

1. Add function declaration in `src/functions.py`
2. Run `python src/parse_mjxmacro.py`
3. Recompile

### 3. How to handle pointer parameters?

For `mjtNum*` type parameters, use `Float64Array` and pass through `byteOffset`:
```javascript
const array = new Float64Array(10);
simulation.someFunction(array);
```

Note: Only arrays allocated by MuJoCo can be used directly; external arrays may require special handling.

## Development Notes

1. **Keep versions in sync**: Ensure `functions.py` matches the MuJoCo header file version
2. **Test enum parsing**: Ensure all enum values have proper comments or can handle missing comments
3. **Check struct fields**: New versions may add or remove fields, requiring manual updates to `main.template.cc`
4. **Run complete build**: Always run the full generation and compilation process after modifications

## Build Dependencies

- Python 3.x
- Emscripten SDK (emcmake, emmake)
- CMake 3.22+
- Clang/LLVM

## Known Limitations

1. Not all MuJoCo API functions are exported (only functions with valid signatures)
2. Array parameters can only use arrays allocated by MuJoCo
3. Some complex callback functions may not be supported
4. Pointer parameter safety depends on correct type usage

## Contribution Guidelines

When modifying code:
1. Update the corresponding template files (`*.template.*`)
2. Run the code generation script
3. Recompile and test
4. Update this documentation

## License

Follow MuJoCo's license requirements.

