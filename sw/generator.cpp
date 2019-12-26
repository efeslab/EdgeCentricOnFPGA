#include <iostream>
#include <stdlib.h>
using namespace std;

#define NUM_V 256*3
#define FACTOR 4

int main() {
    for (int i = 0; i < NUM_V; i++) {
        for (int j = 0; j < FACTOR; j++) {
            int k = rand() % NUM_V;
            cout << i << " " << k << endl;
        }
    }
    return 0;
}

