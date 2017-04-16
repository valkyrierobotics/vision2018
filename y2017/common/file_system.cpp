#include "common/file_system.hpp"
#include <fstream>

// Checks if a file with the filename exists
bool is_file(::std::string filename)
{
  ::std::ifstream infile(filename);
  return infile.good();
}
