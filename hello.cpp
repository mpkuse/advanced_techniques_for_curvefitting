#include <iostream>

using namespace std;

#include <Eigen/Core>
using namespace Eigen;

int main( int argc, char ** argv )
{
    cout << "Hello World\n";

    Vector3d s;
    s << 1.0, 2.3, 4.5;
    cout << s << endl;
    return 0;
}
