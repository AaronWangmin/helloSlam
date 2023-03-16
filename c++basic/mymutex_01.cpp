#include <iostream>
#include <thread>
#include <mutex>

using namespace std;

std::mutex mtx_1;
std::mutex mtx_2;
int test_num = 1;

void print_block_1(int n, char c)
{
    mtx_1.lock();
    for (int i = 0; i < n; i++)
    {
        cout << c;
        cout << "1" << endl;
    }
    cout << endl;
    mtx_1.unlock();
}

void print_block_2(int n, char c)
{
    mtx_2.lock();
    for (int i = 0; i < n; i++)
    {
        cout << c;
        cout << "2" << endl;
    }
    cout << endl;
    mtx_2.unlock();
}

int main()
{
    for (int i = 0; i < 1000; i++)
    {
        thread th1(print_block_1,10000,'*');
        thread th2(print_block_2,10000,'$');        
        th1.join();
        th2.join();
    }  

    return 0;
}