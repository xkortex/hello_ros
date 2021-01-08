//
// Created by mike on 07/01/2021.
//

#ifndef HELLO_CPP_MYNODELETCLASS_H
#define HELLO_CPP_MYNODELETCLASS_H

#include <nodelet/nodelet.h>

namespace hello_nodelet
{
    class MyNodeletClass : public nodelet::Nodelet {
    public:
        MyNodeletClass();
        ~MyNodeletClass();
        virtual void onInit();
};
}


#endif //HELLO_CPP_MYNODELETCLASS_H
