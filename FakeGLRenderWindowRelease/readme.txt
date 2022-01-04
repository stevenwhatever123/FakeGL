To compile on feng-linux / feng-gps:

module add qt/5.13.0
qmake -project QT+=opengl
qmake
make
--------------------------------------

To run on feng-linux / feng-gps:
./FakeGLRenderWindowRelease ../path_to/model.obj ../path_to/texture.ppm

---------------------------------------------
