#include <iostream>
using namespace std;

int main() {
    for (int i = 0; i < 3000; i++) {
        for (int j = 0; j < i; j+=(i%100+75)) {
            cout << i << " " << j << endl;
        }
    }
    return 0;
}
