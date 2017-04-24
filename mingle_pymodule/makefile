mingle: mingle.c mingle.i
	swig -python -c++ mingle.i
	c++ -fPIC -c mingle.c
	c++ -fPIC -c mingle_wrap.cxx -I/usr/include/python2.7
	c++ -shared mingle.o mingle_wrap.o -o _mingle.so
