# PandarGeneral
## Clone
```
ssh://git@code.hesaitech.com:10022/ningshang/PandarGeneral.git  --recursive
```
## Build
```
cd <project>
mkdir build
cd build
cmake ..
make
```
## Add to your project
### Cmake
```
add_subdirectory(<path_to>PandarGeneral)

include_directories(
	<path_to>PandarGeneral/include
	<path_to>PandarGeneral/src/PandarGeneralRaw/include
)

target_link_libraries(<Your project>
  PandarGeneralSDK
)
```
### C++
```
#include "pandarGeneral_sdk//pandarGeneral_sdk.h"
```
