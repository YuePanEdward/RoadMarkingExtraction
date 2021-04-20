//Roadmarking extraction, classification and vectorization module (dll)
//Contact: yuepan@student.ethz.ch

// The following ifdef block is the standard way of creating macros which make exporting
// from a DLL simpler. All files within this DLL are compiled with the ROADMARKINGDLL_EXPORTS
// symbol defined on the command line. This symbol should not be defined on any project
// that uses this DLL. This way any other project whose source files include this file see
// ROADMARKINGDLL_API functions as being imported from a DLL, whereas this DLL sees symbols
// defined with this macro as being exported.
#ifdef ROADMARKINGDLL_EXPORTS
#define ROADMARKINGDLL_API __declspec(dllexport)
#else
#define ROADMARKINGDLL_API __declspec(dllimport)
#endif

// This class is exported from the dll
class ROADMARKINGDLL_API Croadmarkingdll {
public:
	Croadmarkingdll(void);
	// TODO: add your methods here.
};

#include <string>

extern ROADMARKINGDLL_API int nroadmarkingdll;

ROADMARKINGDLL_API int fnroadmarkingdll(std::string &inputFilePath, std::string &outputFilePath, std::string &model_path, std::string & parm_list);


