//
// Created by book on 2021/11/19.
//

#include <iostream>

using namespace std;

class Counter{
public:
    Counter();
    ~Counter();
    int getValue() const{
        return value;
    }
    void setValue(int x){
        value = x;
    }
    const Counter & operator++();
    const Counter operator++(int);

private:
    int value;
    int x;
};

Counter::Counter() :value(0),x(0){}
Counter::~Counter() {}

const Counter & Counter::operator++() {
    ++value;
    return(*this);
}
const Counter Counter::operator++(int) {
    Counter temp(*this);
    ++value;
    cout<<"pre addr = "<<&temp<<endl;
    cout<<"pre value = "<<temp.getValue()<<endl;


    cout<<"new addr = "<<this<<endl;
    cout<<"pre new = "<<this->getValue()<<endl;

    return temp;
}

int main(int argc,char ** argv){
    cout<<"ch1 ======= "<<endl;
    Counter c;
    Counter D = c++;
    cout<<" D = "<<&D<<endl;
    Counter E = ++c;
    cout<<" E = "<<&E<<endl;



    return 0;
}