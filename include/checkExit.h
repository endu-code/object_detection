#ifndef CHECKEXIT_H
#define CHECKEXIT_H

#include <atomic>
#include <iostream>

void checkExit(std::atomic<bool> *exitsignal){
    const int ESC = 27;
    char ch;

    while(std::cin.get() != 27){

    }
    ROS_WARN_STREAM("exit signal send");
    *exitsignal = true;
}

#endif