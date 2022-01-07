////
//// Created by book on 2021/11/24.
////
//
//
//#include <iostream>
//#include <vector>
//
//using namespace std;
//
//int main(int argc, char **argv){
//
//    vector<int> obj;
//    for(int i=0;i<10;i++) {
//        obj.push_back(i);
//        cout<<obj[i]<<" ";
//    }
//
//    return 0;
//}
#include<iostream>
#include<algorithm>
using namespace std;

bool cmp(int a,int b)
{
    return a<b;
}
int main()
{
    int num[]={2,3,1,6,4,5};
    cout<<"最小值是 "<<*min_element(num,num+6)<<endl;
    cout<<"最大值是 "<<*max_element(num,num+6)<<endl;
    cout<<"最小值是 "<<*min_element(num,num+6,cmp)<<endl;
    cout<<"最大值是 "<<*max_element(num,num+6,cmp)<<endl;
    return 0;
}