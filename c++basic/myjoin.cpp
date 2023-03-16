#include <iostream>
#include <thread>

#include <unistd.h>

using namespace std;

void thread_1()
{
    while (1)
    {
        cout << "sub thread: 1" << endl;
        sleep(5);
    }    
}

void thread_2(int x)
{
    while (1)
    {
        cout << "sub thread: 2 " << endl; 
        cout << "x:  = " << x << endl;  
        sleep(5); 
    }    
}

int main(int argc,char** argv)
{
    thread firstThread(thread_1);
    thread secondThread(thread_2,100);
    // thread thirdThread(thread_2,300);
    // cout << "main thread." << endl;

    firstThread.join();
    secondThread.join();
    while (1)
    {
        cout << "main thread." << endl;
        sleep(5);
    }    
}

// complie
// g++ mythread.cpp -o test -l pthread