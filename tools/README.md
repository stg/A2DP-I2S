# Tools

- [`resource_inject.py`](#resource_injectpy) - Injects binary resources into existing C/C++ initializers by symbol name.
- [`polyphase_dude.py`](#polyphase_dudepy) - Designs polyphase filters and can resample WAV files for verification.

## `resource_inject.py`

Converts a binary file into a C initializer and injects it into an existing C/C++ source file by symbol name.  
Used in this repo to produce MP3 resources for sound effects.

### Arguments

`target` [target source]
- Positional target C/C++ source file.

`sources` [input file...]
- One or more binary input files. Relative paths resolve from the target file directory if needed.

`-n`, `--name` [symbol]
- Symbol name or template. `*` expands to sanitized filename and `#` to zero-based source index. Default is `*`.

`-t`, `--type` [`8`|`16le`|`16be`|`32le`|`32be`|`64le`|`64be`]
- Element width and endianness. Default is `8`.

`-c`, `--count` [values per line]
- Values per output line. Use `0` for a single line. Defaults to `16` for 8/16-bit, `8` for 32-bit, `4` for 64-bit.

`-o`, `--output` [patched file]
- Write patched output to a separate file. Relative paths resolve from the target file directory. Otherwise edits in place and creates a `.bak` backup.

`--no-bak`
- When editing in place, replace the target without keeping a `.bak` file.

`--symbols`
- List injectable symbols found in the target file and exit.

### Behavior Notes

- The target symbol must already exist in the target source file.
- Duplicate source or target symbol names cause the tool to fail.
- If the generated symbol is not found, the tool prints `[FAILED] <name>` and exits nonzero.
- For `16/32/64` types, the input size must be exactly divisible by the element width.

### MP3 Injection Examples

Run this command to inject the three MP3 resources into `BluetoothI2S-Core/Resources.cpp`:

```text
python tools\resource_inject.py BluetoothI2S-Core\Resources.cpp sfx\logo.mp3 sfx\connect.mp3 sfx\disconnect.mp3
```

Write patched output to a separate file instead of editing in place:

```text
python tools\resource_inject.py Data.cpp bin\logo.bin -o Data.test.cpp
```

Edit in place without keeping a backup:

```text
python tools\resource_inject.py Data.cpp bin\logo.bin --no-bak
```

Force a specific symbol name:

```text
python tools\resource_inject.py Data.cpp bin\custom_blob.bin -n custom_symbol
```

Use a name template for multiple source files:

```text
python tools\resource_inject.py Data.cpp bin\logo.bin bin\connect.bin -n res_#_*
```

Interpret binary data as 32-bit little-endian values:

```text
python tools\resource_inject.py Data.cpp bin\table.bin -t 32le -c 8
```

List injectable symbols in the target:

```text
python tools\resource_inject.py Data.cpp --symbols
```

Write all values on one line:

```text
python tools\resource_inject.py Data.cpp bin\table.bin -c 0
```

## `polyphase_dude.py`

Designs polyphase interpolation / resampling filters, prints C filter tables, and can also resample WAV files for verification.

Dependencies:
- `numpy`
- `scipy`
- `matplotlib` for `--plot`

### Arguments

`-t`, `--taps` [number of taps]
- Taps per phase. Must be `2` for `linear` and `1` for `nearest`.

`-n`, `--phases` [number of phases]
- Number of phases. Must be at least `1`.

`-q`, `--fbits` [fractional bits]
- Coefficient quantization precision. Default is `15`.

`-m`, `--method` [`sinc-dpss`|`sinc-kaiser`|`sinc-tukey`|`nearest`|`linear`]
- Filter construction method. Default is `sinc-kaiser`.

`-p`, `--wp` [kaiser beta|tukey alpha|dpss nw]
- Window parameter. Defaults are chosen automatically.

`-f`, `--cutoff` [normalized frequency]
- Normalized cutoff relative to input Nyquist. Optional. Default is `1.0`. Forced to `0.5` when `--halfband` is used.

`-s`, `--snorm`
- Normalize each phase independently.

`-b`, `--halfband` [`full`|`odd`|`even`]
- Half-band mode. Forces cutoff to `0.5`.

`--plot`
- Plot impulse, magnitude, and phase response.

`-i`, `--input` [input.wav]
- Optional input WAV to process through the filter.

`-o`, `--output` [output.wav]
- Output WAV. Required with `--input`.

`-r`, `--rate` [output sample rate]
- Output sample rate in Hz. Required with `--input`.

### Examples

Examples below are based on the comments in [BluetoothI2S-Core/Audio.cpp](/d:/Sync/ExternalProjects/A2DP-I2S/BluetoothI2S-Core/Audio.cpp:39).

Generate the half-band SRC stage used for `src_2x_kernel`:

```text
python tools\polyphase_dude.py -n 1 -t 31 -m sinc-kaiser -p 7 -q 27 -b even -s
```

Generate the main fractional SRC filter bank used for `src_polyphase_filterbank`:

```text
python tools\polyphase_dude.py -n 256 -t 16 -m sinc-kaiser -p 7 -f 0.4535 -q 27 -s
```

Generate and plot a design interactively:

```text
python tools\polyphase_dude.py -n 16 -t 16 -m sinc-dpss -f 0.907 -q 31 --plot
```

Resample a WAV file for listening or verification:

```text
python tools\polyphase_dude.py -i input.wav -o output.wav -r 96000 -n 256 -t 16 -m sinc-kaiser -p 7 -f 0.4535 -q 27 -s
```
