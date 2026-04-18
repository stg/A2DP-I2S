#!/usr/bin/env python3
#
# (C) DS prototyp 2025
# Generate polyphase filter bank, optionally apply it to a WAV file.
# Works a charm - and builds symmetric tables for reduced ROM if desired.
#
# Example: polyphase_dude --phases 16 --taps 16 -msinc-dpss --cutoff 0.907 -q31 --plot

import argparse
import numpy as np
from scipy.io import wavfile
from scipy.signal.windows import kaiser, dpss, tukey
from scipy.signal import freqz
import matplotlib.pyplot as plt

# Construct a kernel of specified length, delay and window
def sinc_kernel(delay, phases, taps, window, wp, cutoff, snorm, halfband):
    n = (np.arange(-(taps - 1 / phases) / 2 * phases, (taps) / 2 * phases)) / phases
    if   window == "kaiser": w = kaiser(len(n), wp)
    elif window == "dpss":   w = dpss  (len(n), wp)
    elif window == "tukey":  w = tukey (len(n), wp)
    kernel = np.sinc(cutoff * n) * w
    kernel /= np.sum(kernel) / phases
    delay_kernel = kernel[phases - 1 - delay::phases]
    if halfband == 'odd':
        delay_kernel = kernel[1::2]
    if halfband == 'even':
        delay_kernel = kernel[0::2]
    # Normalize each phase - removes phase energy modulation at cost of slight filter response distortion
    if snorm: delay_kernel /= np.sum(delay_kernel)
    return n[phases - 1 - delay::phases], delay_kernel

# Perform resampling using the constructed polyphase filter bank
def resample(x, bank, rate):
    N_in = len(x)
    M_out = int(np.ceil(N_in * rate))
    y = np.zeros(M_out)

    indexer :int = 0
    t_int: int = 0
    n_out: int = 0
    step: int = int(0x40000000 / rate)

    while t_int < len(x):
        kernel = bank[indexer // (0x40000000 // len(bank))]
        
        acc : int = 0.0
        for k in range(len(kernel)):
            i = t_int + k - len(kernel)
            acc += x[i] * kernel[k]
        
        y[n_out] = acc
        n_out = n_out + 1

        indexer += step;
        while indexer >= 0x40000000:
            indexer = indexer - 0x40000000
            t_int = t_int + 1
        
    return y

# Quantize array
def quantize(h, q):
    return (np.round(h * (1 << q)).astype(int) / (1 << q)).astype(float)

def print_filterbank(bank, q, phases, coeffs):
    # Find bits required
    max_value = 0
    for phase in range(len(bank)):
        for coeff in range(len(bank[phase])):
            value = round(bank[phase][coeff] * (1 << q))
            value = value + 1 if value >= 0 else -bank[phase][coeff]
            if value > max_value: max_value = value
    req_bits = np.ceil(np.log2(max_value))
    # Select word size
    if   req_bits > 31: word_size = 64;
    elif req_bits > 15: word_size = 32;
    elif req_bits >  7: word_size = 16;
    else:               word_size =  8;
    # Print C code
    print(f"#define PP_PHASES {phases:d}")
    print(f"#define PP_COEFFS {coeffs:d}")
    print(f"#define PP_Q {q:d}")
    print(f"static const int{word_size:d}_t pp_filter[PP_PHASES][PP_COEFFS] = {{")
    mirror_prefix = '' # if not comment_mirror else '//'
    for phase in range(len(bank)):
        print(f"    {'' if phase < len(bank)/2 else mirror_prefix:s}{{ {", ".join(f"0x{int(np.round(coef * (1 << q))) & ((1 << word_size) - 1):0{word_size // 4}X}" for coef in bank[phase])} }},")
    print("};")

def construct_filterbank(phases, taps, method, wp, cutoff, fbits, snorm, halfband):
    filterbank = []
    for i in range(phases):
        if method == 'nearest':
            # nearest-neighbour
            x = np.arange(0, taps)
            kernel = np.zeros(taps)
            kernel[0] = 1
        elif method == 'linear':
            # linear interpolation
            x = np.arange(0, taps)
            kernel = np.zeros(taps)
            kernel[0] = 1 - i / phases
            kernel[1] = 0 + i / phases
        else:
            # sinc-window
            x, kernel = sinc_kernel(i, phases, taps, method[5:], wp, cutoff, snorm, halfband)

        # Quantize here to emulate the correct behavior of the filterbank output
        filterbank.append(quantize(kernel, fbits))
    return filterbank
        
def main():
    parser = argparse.ArgumentParser(description="Upsample WAV file using configurable sinc interpolation.")
    parser.add_argument("-i", "--input",    type=str,   default=None,                help="Input WAV file path")
    parser.add_argument("-o", "--output",   type=str,   default=None,                help="Output WAV file path")
    parser.add_argument("-r", "--rate",     type=float, default=None,                help="Desired output sample rate (Hz)")
    parser.add_argument("-t", "--taps",     type=int,   required=True, help="Number of taps in each phase")
    parser.add_argument("-p", "--wp",       type=float, default=None,                help="Window parameter (kaiser: beta, tukey: alpha, dpss: nw)")
    parser.add_argument("-f", "--cutoff",   type=float, default=1.0,                 help="Normalized cutoff (0–1, relative to input Nyquist)")
    parser.add_argument("-n", "--phases",   type=int,   required=True,               help="Polyphase filter bank size")
    parser.add_argument("-q", "--fbits",    type=int,   default=15,                  help="Quantization bits for coefficients (1-63)")
    parser.add_argument("-s", "--snorm",    action='store_true',                     help="Normalize each phase")
    parser.add_argument("-m", "--method",   type=str,   default="sinc-kaiser",       help="Window function (sinc-dpss, sinc-kaiser, sinc-tukey, nearest, linear)",
        choices=["sinc-dpss", "sinc-kaiser", "sinc-tukey", "nearest", "linear"])
    parser.add_argument("-b", "--halfband",   type=str,   default=None,              help="Half-band mode (full, odd, even), implies -f0.5",
        choices=["full", "odd", "even"])
        
    parser.add_argument("--plot", action='store_true')
    args = parser.parse_args()
    
    if args.wp is None:
        if   args.method == "sinc-kaiser": args.wp = 8
        elif args.method == "sinc-dpss":   args.wp = np.log2(args.taps / 16) / 2 + 3
        elif args.method == "sinc-tukey":  args.wp = 1/3
    elif args.method in ["nearest", "linear"]:
            print(f"Do not provide window parameter for {args.method:s}")

    if args.phases < 1:
        raise Exception("Must have at least 1 phase")

    if args.taps < 1:
        raise Exception("Must have at least 1 tap")
    
    if args.method == "linear" and args.taps != 2:
        raise Exception("Method linear requires exactly 2 taps")

    if args.method == "nearest" and args.taps != 1:
        raise Exception("Method nearest requires exactly 1 taps")
    
    if args.halfband:
        args.cutoff = 0.5

    if args.plot and args.halfband in ["odd", "even"]:
        raise Exception("Plotting odd/even half-bands produces no useful output")

    # Construct polyphase filter bank
    bank = construct_filterbank(args.phases, args.taps, args.method, args.wp, args.cutoff, args.fbits, args.snorm, args.halfband)

    if args.input is None:
    
        if not ((args.output is None) and (args.rate is None)):
            raise Exception("Do not provide output(.wav) or rate unless providing input(.wav)")

        # Just print the filterbank
        print_filterbank(bank, args.fbits, args.phases, args.taps)
    
    else:

        if ((args.output is None) or (args.rate is None)):
            raise Exception("Must provide output(.wav) and rate if providing input(.wav)")

        # Read WAV
        fs_in, x = wavfile.read(args.input)

        # Perform resampling
        if x.ndim == 1:
            y = resample(x, bank, args.rate / fs_in)
        else:
            y = np.stack([resample(x[:, ch], bank, args.rate / fs_in) for ch in range(x.shape[1])], axis=-1)

        # Save resampled WAV
        wavfile.write(args.output, int(args.rate), y.astype(x.dtype))
        
        # Print what we have done
        print(f"Resampled {args.input} ({fs_in} Hz) → {args.output} ({args.rate} Hz) using fixed-point polyphase filter")
        print(f"Configuration: taps={args.taps} phases={args.phases} method={args.method} wp={args.wp} q={args.fbits} cutoff={args.cutoff}*Nyquist")

    # Plot impulse, magnitude, phase response for each phase
    if args.plot:
        fig, axs = plt.subplots(3, args.phases, figsize=(16, 8), squeeze=False)
        x = np.arange(args.taps)
        for i in range(len(bank)):
            kernel = bank[i]
            # Plot filter (impulse) response
            f, r = freqz(kernel)
            axs[0,i].plot(x, kernel)
            axs[0,i].set_ylim([-0.25,1.25])
            # Plot magnitude
            r_mag = [20 * np.log10(np.abs(oneH)) for oneH in r]
            axs[1,i].plot(f / np.pi, r_mag)
            axs[1,i].set_ylim([-80,5])
            # Plot phase (wrapped)
            r_pha = [np.angle(oneH) for oneH in r]
            axs[2,i].plot(f / np.pi, r_pha)
            axs[2,i].set_ylim([-np.pi,np.pi])
        plt.tight_layout()
        plt.show()

if __name__ == "__main__":
    main()
