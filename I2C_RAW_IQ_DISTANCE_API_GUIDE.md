# High-Level API Guide: Requesting Raw IQ Distance Frames over I2C

## What this guide covers
- The MCU/host communicates with the STM32 firmware via the I2C “distance detector” register protocol.
- This project supports two IQ endpoint styles:
  - `i2c_iq_custom.c`: IQ request is index-based (you request a bin/index and firmware captures and selects that index).
  - `i2c_distance_iq_stream.c`: IQ request is “capture once, then stream packed IQ samples”.

The register map comes from:
- `Inc/distance_reg_protocol.h`
- `Src/applications/i2c/distance_reg_protocol_access.c`

## Register addresses (subset)
- `DISTANCE_REG_COMMAND_ADDRESS` = `256`
- `DISTANCE_REG_DETECTOR_STATUS_ADDRESS` = `3`
- `DISTANCE_REG_MEASURE_COUNTER_ADDRESS` = `2`
- `DISTANCE_REG_PEAK0_DISTANCE_ADDRESS` = `17`
- `DISTANCE_REG_PEAK0_STRENGTH_ADDRESS` = `27`

## Command write behavior
- Writing a 32-bit value to `DISTANCE_REG_COMMAND_ADDRESS` triggers `i2c_distance_detector_command(value)`.
- Firmware sets a BUSY bit in detector status while processing.
- Host should wait until BUSY clears before reading results.

## IQ readiness signaling
- IQ readiness is communicated via bit `1<<16` in `DISTANCE_REG_DETECTOR_STATUS_ADDRESS`.
- Different IQ endpoint styles use different packing for the actual IQ samples, but the “IQ-ready bit” concept is consistent.

## IQ endpoint style A: `i2c_iq_custom.c` (index-based I/Q)
### IQ request format
- IQ command is written using:
  - upper 16 bits: `IQ_CMD_BASE` (value `6`)
  - lower 16 bits: requested `index`
- Firmware captures IQ and stores averaged IQ for all bins, then sets:
  - “current index” = requested index

### Sequence (one IQ bin)
1. Ensure detector is configured & calibrated for distance measurements.
2. Write command word to `DISTANCE_REG_COMMAND_ADDRESS`:
   - `command = (6<<16) | index`
3. Wait for the detector BUSY bit to clear.
4. Poll/read:
   - `DISTANCE_REG_DETECTOR_STATUS_ADDRESS` and confirm IQ-ready bit is set.
5. Read:
   - `DISTANCE_REG_MEASURE_COUNTER_ADDRESS`:
     - in this endpoint style it returns `iq_num_points` (#bins) when IQ is valid
   - `DISTANCE_REG_PEAK0_DISTANCE_ADDRESS`:
     - returns IQ “real” for the selected `index`
   - `DISTANCE_REG_PEAK0_STRENGTH_ADDRESS`:
     - returns IQ “imag” for the selected `index`

### Scaling of IQ values
- The protocol converts firmware `float` values to register values using `acc_reg_protocol_float_to_uint32_milli` / `acc_reg_protocol_float_to_int32_milli`.
- Practically:
  - divide read register integers by `1000` to recover the `float` IQ real/imag values.

### Getting a full IQ frame
- This endpoint style re-captures when you send a new IQ command.
- To retrieve the whole IQ frame, loop `index = 0 .. (iq_num_points-1)` and repeat the command + read steps for each index.

## IQ endpoint style B: `i2c_distance_iq_stream.c` (capture-once, stream packed IQ)
### IQ request format
- Firmware supports a command value `CAPTURE_IQ` with numeric value `6`.
- Host writes `DISTANCE_REG_COMMAND_ADDRESS = 6` to trigger capture.

### Sequence (capture once, then stream)
1. Ensure detector is configured & calibrated.
2. Write command:
   - `DISTANCE_REG_COMMAND_ADDRESS = 6`
3. Wait for IQ-ready bit to be set in `DISTANCE_REG_DETECTOR_STATUS_ADDRESS`.
4. Read `DISTANCE_REG_MEASURE_COUNTER_ADDRESS`:
   - IQ_METADATA is packed into this register:
     - valid bit (bit 31)
     - frame length (bits 0–15 in 16-bit words)
     - number of sweeps (bits 16–23)
     - data format (bits 24–29)
5. Stream IQ samples using `DISTANCE_REG_PEAK0_DISTANCE_ADDRESS` reads:
   - Each read returns a packed 32-bit word interpreted by firmware as:
     - low 16 bits = I (real)
     - high 16 bits = Q (imag)
   - The firmware returns a `float` that the protocol converts back to the packed integer, so host receives the packed IQ directly in the register value.

## Practical notes
- If you are optimizing for firmware size, prefer the “core + thin app” approach already used in this repo.
- If timing is critical, use the IQ stream style (capture once, stream many reads) rather than index-based repeated captures.

