#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
using namespace std;

int main()
{
  vector <vector <string> > data;
  ifstream infile( "test1.csv" );

  while (infile)
  {
    string s;
    if (!getline( infile, s )) break;

    istringstream ss( s );
    vector <string> record;

    while (ss)
    {
      string s;
      if (!getline( ss, s, ',' )) break;
      record.push_back( s );
    }

    data.push_back( record );
  }
  if (!infile.eof())
  {
    cerr << "Fooey!\n";
  }

printf("ended up here %d %d", data[1], data[2]);

	return 0;
}
