#include<stdio.h>
#include<string.h>

class SpgM01
{
private:
    /* data */
public:
    SpgM01(/* args */);
    ~SpgM01();
    int SetOpt(int fd,int nSpeed, int nBits, char nEvent, int nStop);
    int ParseData(unsigned char &data,int n){};
    
};

SpgM01::SpgM01(/* args */)
{
}

SpgM01::~SpgM01()
{
}

int SetOpt(int fd,int nSpeed, int nBits, char nEvent, int nStop){
    
}


// int ParseData(unsigned char &data,int n){

// }
