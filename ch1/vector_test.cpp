//
// Created by book on 2021/11/24.
//


#include <iostream>
#include <vector>

using namespace std;

int main(int argc, char **argv){

    vector<int> obj;
    for(int i=0;i<10;i++) {
        obj.push_back(i);
        cout<<obj[i]<<" ";
    }

    return 0;
}