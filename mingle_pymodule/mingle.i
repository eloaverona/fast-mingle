/* example.i */
%module mingle
%{
extern int my_mod(int x, int y);
%}

extern int my_mod(int x, int y);