#include <boost/thread.hpp>
#include <rtai_lxrt.h>

#include <iostream>

using namespace boost;

void thread1(){
    std::cout << "test" << std::endl;
    rt_make_hard_real_time();
}

int main(){
    thread main;
    thread thread1_test(thread1);


    thread1_test.join();
}
