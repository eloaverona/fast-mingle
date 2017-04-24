all: mingle
mingle: mingle.c mingle.i
	swig -python -c++ mingle.i
	c++ -fPIC -c mingle.c
	c++ -fPIC -c mingle_wrap.cxx -I/usr/include/python2.7
	c++ -shared mingle.o mingle_wrap.o -o _mingle.so
clean:
	rm -f mingle.o
	rm -f mingle.py
	rm -f _mingle.so
	rm -f mingle_wrap.cxx
	rm -f mingle_wrap.o
	rm -f mingle.pyc