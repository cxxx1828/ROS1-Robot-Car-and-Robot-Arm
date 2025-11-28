
#include "glob.hpp"

#include <glob.h> // glob(), globfree()
#include <string.h> // memset()
#include <sstream>

vector<string> glob(const string& pattern) {
	// glob struct resides on the stack
	glob_t glob_result;
	memset(&glob_result, 0, sizeof(glob_result));
	
	vector<string> file_names;

	// do the glob operation
	int return_value = glob(pattern.c_str(), GLOB_TILDE, NULL, &glob_result);
	if(return_value != 0) {
		globfree(&glob_result);
#if 0
		stringstream oss;
		oss << "glob() failed with return_value " << return_value << endl;
		throw runtime_error(oss.str());
#else
		return file_names;
#endif
	}

	// Collect all the file names into vector.
	for(size_t i = 0; i < glob_result.gl_pathc; ++i) {
		file_names.emplace_back(glob_result.gl_pathv[i]);
	}

	// Cleanup.
	globfree(&glob_result);

	return file_names;
}
