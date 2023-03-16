#include <iostream>
#include <thread>

using namespace std;

void thread_1()
{
    cout << "sub thread: 1" << endl;
}

void thread_2(int x)
{
    cout << "sub thread: 2 " << endl; 
    cout << "x:  = " << x << endl;   
}

int main(int argc,char** argv)
{
    thread firstThread(thread_1);
    thread secondThread(thread_2,100);
    // thread thirdThread(thread_2,300);
    cout << "main thread." << endl;

    firstThread.join();
    secondThread.join();
    cout << "sub threads have end" << endl;
}

// complie
// g++ mythread.cpp -o test -l pthread