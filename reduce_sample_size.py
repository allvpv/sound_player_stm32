#!/usr/bin/env python3
import numpy as np
import bitstring as bs
import argparse

parser = argparse.ArgumentParser(description='Reduces sample size.')

parser.add_argument('--n', metavar='N', type=int, help='target bitwidth of sample', required=True)
parser.add_argument('--input', type=str, help='input file (signed 16-bit PCM raw data)',
                    required=True)
parser.add_argument('--output', type=str, help='output file (unsigned n-bit PCM raw data)',
                    required=True)

args = parser.parse_args()

k = 16 - args.n
song_samples_reduced = bs.BitArray()

with open(args.input, "rb") as inp:
    while True:
        sample_inp = inp.read(2)

        if sample_inp == b'':
            break
        else:
            assert len(sample_inp) == 2

        sample_num = int.from_bytes(sample_inp, byteorder='little', signed=False)
        sample_unsigned = sample_num ^ 0x8000   # Make unsigned
        sample_shifted = sample_unsigned >> k   # Discard last k bits to make sample n-bit

        song_samples_reduced.append(bs.Bits(uint=sample_shifted, length=args.n))

with open(args.output, "wb") as out:
    out.write(song_samples_reduced.tobytes())

