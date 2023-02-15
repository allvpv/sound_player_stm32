#!/usr/bin/env python3
import argparse

parser = argparse.ArgumentParser(description='Converts to cdata format.')

parser.add_argument('--input', type=str, help='input file: raw binary data', required=True)
parser.add_argument('--output', type=str, help='output file: cdata', required=True)
parser.add_argument('--when_newline', type=int, help='how often insert newline', default=15)

args = parser.parse_args()

with open(args.input, "rb") as inp, open(args.output, "wb") as out:
    time_for_newline = args.when_newline
    first_line = True
    insert_separator = False

    while True:
        byte = inp.read(1)

        if byte == b'':
            break

        if insert_separator:
            out.write(b',')

        if time_for_newline == 0:
            out.write(b'\n')
            time_for_newline = args.when_newline
        elif not first_line:
            out.write(b' ')
            time_for_newline -= 1
        else:
            first_line = False

        sample_num = int.from_bytes(byte, byteorder='little', signed=False)
        out.write(bytes('{:#04x}'.format(sample_num), encoding='utf-8'))
        insert_separator = True

    out.write(b'\n')




