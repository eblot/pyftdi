#!/usr/bin/env python3

"""Quick and dirty bytearray vs. array('B') performance test."""

from array import array
from struct import pack
from timeit import timeit
from time import perf_counter

def timing(f, n, a):
    start = perf_counter()
    for _ in range(n):
        f(a); f(a); f(a); f(a); f(a); f(a); f(a); f(a); f(a); f(a)
    finish = perf_counter()
    return '%s\t%f' % (f.__name__, finish - start)

def time_array(addr):
    return array('B', addr)

def time_bytearray(addr):
    return bytearray(addr)

def extend_array(addr):
    b = bytearray()
    b.extend(addr)
    b.extend(b)
    b.extend(b)
    b.extend(b)
    b.extend(b)
    b.extend(b)
    return b

def extend_bytearray(addr):
    b = bytearray()
    b.extend(addr)
    b.extend(b)
    b.extend(b)
    b.extend(b)
    b.extend(b)
    b.extend(b)
    return b

def array_tostring(addr):
    return array('B', addr).tobytes()

def str_bytearray(addr):
    return str(bytearray(addr))

def struct_pack(addr):
    return pack('4B', *addr)

def main():
    count = 100000
    addr = '192.168.4.2'
    addr = tuple([int(i) for i in addr.split('.')])
    print('\t\ttiming\t\tfunc\t\tno func')
    print('%s\t%s\t%s' % (timing(time_array, count, addr),
          timeit('time_array((192,168,4,2))', number=count,
                 setup='from __main__ import time_array'),
          timeit("array('B', (192,168,4,2))", number=count,
                 setup='from array import array')))
    print('%s\t%s\t%s' % (timing(time_bytearray, count, addr),
          timeit('time_bytearray((192,168,4,2))', number=count,
                 setup='from __main__ import time_bytearray'),
          timeit('bytearray((192,168,4,2))', number=count)))
    print('%s\t%s' % (timing(extend_array, count, addr),
          timeit('extend_array((192,168,4,2))', number=count,
                 setup='from __main__ import extend_array')))
    print('%s\t%s' % (timing(extend_bytearray, count, addr),
          timeit('extend_bytearray((192,168,4,2))', number=count,
                 setup='from __main__ import extend_bytearray')))
    print('%s\t%s\t%s' % (timing(array_tostring, count, addr),
          timeit('array_tostring((192,168,4,2))', number=count,
                 setup='from __main__ import array_tostring'),
          timeit("array('B', (192,168,4,2)).tostring()", number=count,
                 setup='from array import array')))
    print('%s\t%s\t%s' % (timing(str_bytearray, count, addr),
          timeit('str_bytearray((192,168,4,2))', number=count,
                 setup='from __main__ import str_bytearray'),
          timeit('str(bytearray((192,168,4,2)))', number=count)))
    print('%s\t%s\t%s' % (timing(struct_pack, count, addr),
          timeit('struct_pack((192,168,4,2))', number=count,
                 setup='from __main__ import struct_pack'),
          timeit("pack('4B', *(192,168,4,2))", number=count,
                 setup='from struct import pack')))

if __name__ == '__main__':
    main()
