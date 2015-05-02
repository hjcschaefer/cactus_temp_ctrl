#!/usr/bin/env bash
avrdude -p m328p -c stk500 -P /dev/tty.PL2303-0000105D -U flash:w:kakteen.hex:i

