# Introduction

This is a very simplified version of box2d-lite converted to C in order to compile it into Z80 using z88dk:

https://github.com/erincatto/box2d-lite

There is no support for joints and only rectangular, single-fixture bodies are supported.

The hope is to get a physics library to work on the ZX Spectrum (and other computers based on Z80) so games like Angry Birds can be written.

# Conversion to C

To prove the C++ -> C conversion has worked, in Windows, compile with compile.bat for a console program (uses main.c) and run b2d_lite.exe. You should see the y-coordinate of the single body go down to 0.49

Compile with compile_win.bat for a Win32 OpenGL program showing a stack of crates on ground (uses win.c). This will wobble and collapse.

The compiler I used in these test programs was gcc (32-bit mingw on Windows) as I think that is closer to z88dk. But you could equally compile with MSVC.

These test programs show the library works on Windows (x86) so hopefully it will work on Z80 as well. Of course, win.c makes no sense for z80 machines!

main.c ought to work on z80 but you will have to link the stdio.h lib which might not be practical for small computers like the ZX Spectrum. In the next section I've just used a very simple print routine to write some unit test numbers on the Spectrum screen.

# Conversion to Z80 ASM

I have now converted some of the code to Z80 assembly suitable for the ZX Spectrum. It's not yet ready to show collapsing towers but I can present
some unit tests showing vector and matrix maths etc.

I have assembled and tested the code using ZX Spin, a
Spectrum emulator which has a built in assembler (https://spectrumcomputing.co.uk/tool/40/ZX_Spin). To run the code, run ZX Spin then select Tools > Z80 assembler. Then do File > Open
and open the file `unit.z80` (note that this includes other files). Then File > Assemble and Program > Run. The Spectrum screen will then show a series of unit test outputs with the number of the unit test at the bottom left in red. Press "S" to go to the next unit test. The numbers shown are floating point numbers in hex format. To convert to readable numbers run the Python scripts `f24.py` (float to hex) and `AHLtoFloat.py` (hex to float).

You can see what the correct numbers should be in unit.z80. For instance test "D" Is showing that the dot product of (4,2).(3,11) = 34.0 (represented as 0x441000). The most advanced function unit tested so far is `Collide` which returns the contact list between two passed-in bodies.

Since the Z80 contains no floating point support I have used the f24 library from z80float (https://github.com/Zeda/z80float/tree/master/f24) which uses 24 bit floats. I have adapted this code slightly to suit the ZX Spin compiler which only supports vanilla Z80 assembler directives as described here:

https://www.nongnu.org/z80asm/directives.html

# Next Steps

* Continue to convert the code to assembly language (still need to do b2d_lite.c) and lets see a pile of crates wobble and collapse!

* As an alternative I will try to compile the C code to Z80 asm using z88dk. I did try this at first but failed because z88dk does not support pass, return of structs by value which Box2d uses extensively. Later I will create a version with struct passing only by pointer and can hopefully compile. This will make the C code less readable and I was a bit surprised z88dk only supports a subsection of C. Within z88dk, I can explore various options for floating point arithmetic. (16 bit, 32 bit etc, z88dk has many flavours)

* If floating point arithmetic is too slow, I can then try fixed point arithmetic (in that case sizes and positions of bodies should be of order 1.0 and placed near the origin)

* Routines `findArbiter`, `eraseArbiter`, `insertArbiter` should be written as a proper map/hash function rather than just stupidly looking up the value. The original C++ code used std::map.