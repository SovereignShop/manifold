[![Clojars Project](https://img.shields.io/clojars/v/org.clojars.cartesiantheatrics/manifold3d.svg)](https://clojars.org/org.clojars.cartesiantheatrics/manifold3d)

This fork Maintains java/JNI bindings to [Manifold](https://github.com/elalish/manifold). It supports nearly all the features, plus extends Manifold with support for Convex Hulls, Polyhedrons, and Lofts. It has builds for linux, mac and an experimental build for windows.


## Installation

You need to include a classifier for your platform and backend. For linux, TBB (Threading Building Blocks) and OMP (OpenMP) parallel backends are available with classifiers `linux-OMP-x86_64` or`linux-TBB-x86_64`. There is a also a build with cuda support enabled at`linux-TBB-cuda-x86_64`.

For mac, only `mac-TBB-x86_64` is available.

Windows is not currently well-supported. There is an experimental jar available in the github build artifacts that should work. It's not available in maven because it is too large (it currently is compiled with the dreaded `CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS=TRUE` flag).
