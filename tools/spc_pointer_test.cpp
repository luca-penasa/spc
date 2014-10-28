

#include <iostream>

int main()
{
  using namespace std;

// Declare and initialize a pointer.
  unsigned short int * pPointer = 0;
// Declare an integer variable and initialize it with 35698
  unsigned short int twoInt = 35698;
// Declare an integer variable and initialize it with 77
  unsigned short int oneInt = 77;
  *pPointer = 5;
//   cout << "ppointer points to the integer value: \t" << *pPointer << endl;
  // Use address-of operator & to assign a memory address of twoInt to a pointer
  pPointer = &twoInt;
// Pointer pPointer now holds a memory address of twoInt

// Print out associated memory addresses and its values
  cout << "pPointer's memory address:\t\t" << &pPointer << endl;
  cout << "Integer's oneInt memory address:\t" << &oneInt << "\tInteger value:\t" << oneInt << endl;
  cout << "Integer's twoInt memory address:\t" << &twoInt << "\tInteger value:\t" << twoInt << endl;
  cout << "pPointer is pointing to memory address:\t" << pPointer << "\tInteger value:\t" << *pPointer << endl;

return 0;
}