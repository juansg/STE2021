#pragma once

#include <string>

struct CallSite {
  unsigned lineNumber;
  std::string fileName;
  CallSite () {
    lineNumber = 0;
    fileName = "<invalid>";
  }
  CallSite (unsigned lineNumber, std::string fileName)
    : lineNumber(lineNumber), fileName(fileName) {}
};
