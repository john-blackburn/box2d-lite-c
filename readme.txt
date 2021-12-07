This is a very simplified version of box2d-lite converted to C in order to compile it into Z80 using z88dk:

https://github.com/erincatto/box2d-lite

There is no support for joints and only rectangular, single-fixture bodies are supported.

Test programs:

Compile with compile.bat for a console program. (You should see the y coordinate of the single body go down to 0.49)

Compile with compile_win.bat for a Win32 OpenGL program showing a stack of crates on ground. This will wobble and collapse.

The compiler used in these test programs is gcc (32-bit mingw on Windows) as I think that is closer to z88dk.