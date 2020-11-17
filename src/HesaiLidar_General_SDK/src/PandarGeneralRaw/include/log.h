/******************************************************************************
 * Copyright 2018 The Hesai Technology Authors. All Rights Reserved.
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

#include <stdio.h>
#include <unistd.h>
#include <thread>
#include <sys/time.h>
#include <sys/timeb.h>

class TranceFunc
{
public:
    TranceFunc(const char* file, const char* func){
        m_cFile = file;
        m_cFunc = func;
        struct tm *ptm;
        struct timeb stTimeb;
        ftime(&stTimeb);
        ptm = localtime(&stTimeb.time);
        printf("[T] %02d:%02d:%02d.%03d pid:%d tid:%10d ->[File:%s Function:%s ]\n", ptm->tm_hour, ptm->tm_min, ptm->tm_sec, stTimeb.millitm, getpid(), std::hash<std::thread::id>()(std::this_thread::get_id()), m_cFile, m_cFunc);
    }
    ~TranceFunc(){
        struct tm *ptm;
        struct timeb stTimeb;
        ftime(&stTimeb);
        ptm = localtime(&stTimeb.time);
        printf("[T] %02d:%02d:%02d.%03d pid:%d tid:%10d <-[File:%s Function:%s ]\n", ptm->tm_hour, ptm->tm_min, ptm->tm_sec, stTimeb.millitm, getpid(), std::hash<std::thread::id>()(std::this_thread::get_id()), m_cFile, m_cFunc);
    }
    const char* m_cFile;
    const char* m_cFunc;
};

#define LOG_D(format,...) {\
    struct tm *ptm;\
    struct timeb stTimeb;\
    ftime(&stTimeb);\
    ptm = localtime(&stTimeb.time);\
    printf("[D] %02d:%02d:%02d.%03d pid:%d tid:%10d File:%s Function:%s Line:%d " format"\n", ptm->tm_hour, ptm->tm_min, ptm->tm_sec, stTimeb.millitm, getpid(), std::hash<std::thread::id>()(std::this_thread::get_id()), __FILE__, __FUNCTION__, __LINE__, ##__VA_ARGS__);\
}

#define LOG_E(format,...) {\
    struct tm *ptm;\
    struct timeb stTimeb;\
    ftime(&stTimeb);\
    ptm = localtime(&stTimeb.time);\
    printf("[E] %02d:%02d:%02d.%03d pid:%d tid:%10d File:%s Function:%s Line:%d " format"\n", ptm->tm_hour, ptm->tm_min, ptm->tm_sec, stTimeb.millitm, getpid(), std::hash<std::thread::id>()(std::this_thread::get_id()), __FILE__, __FUNCTION__, __LINE__, ##__VA_ARGS__);\
}

#define LOG_FUNC() TranceFunc tf( __FILE__, __FUNCTION__)
    