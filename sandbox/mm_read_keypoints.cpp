#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <memory>



using namespace std;
int main(int argc, char ** argv)
{
    string filename = argv[1];

     ifstream file(filename, std::ios::in|std::ios::binary);

     int n_k;
     file.read(reinterpret_cast<char*>(&n_k), sizeof(int));

     cout << "number of points: " << n_k << endl;

     file.read(reinterpret_cast<char*>(&n_k), sizeof(int));

     cout << "number of points: " << n_k << endl;


     file.read(reinterpret_cast<char*>(&n_k), sizeof(int));

     cout << "number of points: " << n_k << endl;


     file.read(reinterpret_cast<char*>(&n_k), sizeof(int));

     cout << "number of points: " << n_k << endl;

     file.seekg(21);

     float a,b,c,d;

     file.read(reinterpret_cast<char*>(&a), sizeof(float));
     file.read(reinterpret_cast<char*>(&b), sizeof(float));
     file.read(reinterpret_cast<char*>(&c), sizeof(float));
     file.read(reinterpret_cast<char*>(&d), sizeof(float));

     cout << a << " " << b << " " << c << " " << d << endl;



}
