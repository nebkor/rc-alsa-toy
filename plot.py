#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Script description here
"""

from __future__ import print_function, absolute_import, division
import numpy as np
import pylab


def run(args):
    raw_data = np.memmap(args.input, dtype='h', mode='r')
    pylab.plot(raw_data)
    pylab.show()


if '__main__' == __name__:
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input')

    args = parser.parse_args()
    run(args)
