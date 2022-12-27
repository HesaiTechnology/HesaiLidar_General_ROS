/******************************************************************************
 * Copyright 2020 The Hesai Technology Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#ifndef VERSION_H_
#define VERSION_H_

#include <stdio.h>
#include <unistd.h>
#include <string>

#define VERSION "PandarGeneralROS_1.1.15"
#ifdef __cplusplus
extern "C" {
#endif

void printVersion() {
    printf("       ///////////////////////////////////////////////////////////////\n"
           "       //     PandarGeneralROS version: %s      //\n" 
           "       ///////////////////////////////////////////////////////////////\n",VERSION);
}

#ifdef __cplusplus
}
#endif

#endif  // VERSION_H_

