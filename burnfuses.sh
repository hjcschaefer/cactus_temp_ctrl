#!/usr/bin/env bash
avrdude -p m328p -c stk500 -P /dev/tty.PL2303-0000105D -U lfuse:w:0xcc:m -U hfuse:w:0xdb:m -U efuse:w:0x05:m


