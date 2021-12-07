This is a very simplified version of box2d-lite converted to C in order to compile it into Z80 using z88dk:

https://github.com/erincatto/box2d-lite

There is no support for joints and only rectangular, single-fixture bodies are supported.

The hope is to get a physics library to work on the ZX Spectrum (and other computers based on Z80)

Test programs (for Windows):
===========================

Compile with compile.bat for a console program (uses main.c). You should see the y-coordinate of the single body go down to 0.49

Compile with compile_win.bat for a Win32 OpenGL program showing a stack of crates on ground (uses win.c). This will wobble and collapse.

The compiler used in these test programs is gcc (32-bit mingw on Windows) as I think that is closer to z88dk. But you could equally compile with MSVC.

These test programs show the library works on Windows (x86) so hopefully it will work on Z80 as well. Of course, win.c should not be compiled for z80 machines!
main.c ought to work on z80 but you will have to link the stdio.h lib which might not be practical for small computers like the ZX Spectrum.

Next Steps
==========

Try to compile using z88dk. Explore various options for floating point arithmetic. (z88dk provides many of these!)

Then try fixed point arithmetic (in that case sizes and positions of bodies should be of order 1.0 and placed near the origin)

findArbiter, eraseArbiter, insertArbiter should be written as a proper map/hash function.