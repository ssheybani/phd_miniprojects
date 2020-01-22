////
// A auxiliary class for working with vectors and matrices
//
// Developed by Saber Sheybani - Spring 2017
////

#include <vector>
#include <list>
#include <iostream>
#include "VectorMatrix.h"


using namespace std;

class VecList{
    public:
        //VecList(){};
        VecList(vector<list<double> >::size_type n) {vec_size = n; data_array = vector<list<double> > (vec_size, list<double>(3,0));};
        void push_back(vector<double>);
        void push_back(TVector<double>);
        void display();

        vector<list<double> > data_array;
        vector<list<double> >::size_type vec_size;
};

void VecList::push_back(vector<double> x){
    // Push x into data_array
    if (x.size() != vec_size)
        cerr << "Line 22: Input vector is not of the same size as the object" << endl;
        cerr << "Size of X = " << sizeof(x) << ", Size of Vector of Lists = " << vec_size << endl;
        cin.get();
    int i = 0;
    for (vector<list<double> >::iterator vecit = data_array.begin(); vecit != data_array.end(); ++vecit){
        (*vecit).pop_front();
        (*vecit).push_back(x[i]);
        i++;
    }
}

void VecList::push_back(TVector<double> x){
    // Push x into data_array
    if (x.Size() != vec_size){
        cerr << "Line 36: Input vector is not of the same size as the object" << endl;
        cerr << "Size of X = " << x.Size() << ", Size of Vector of Lists = " << vec_size << endl;
        cin.get();
    }
    int i = 0;
    for (vector<list<double> >::iterator vecit = data_array.begin(); vecit != data_array.end(); ++vecit){
        (*vecit).pop_front();
        (*vecit).push_back(x[i]);
        i++;
    }
}

void VecList::display(){
        // Display data_array
    for (vector<list<double> >::iterator vecit = data_array.begin(); vecit != data_array.end(); ++vecit){
        for (list<double>::iterator listit = (*vecit).begin(); listit != (*vecit).end(); ++listit)
            cout << " " << *(listit) << endl;
        cout << "End of List" << endl;
    }
}
