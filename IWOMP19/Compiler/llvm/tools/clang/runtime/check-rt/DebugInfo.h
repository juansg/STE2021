#pragma once

#include <bfd.h>

struct DebugInfo {
  bfd_vma pc;
  const char *filename;
  const char *functionname;
  unsigned int line;
  bfd_boolean found;
  DebugInfo(bfd_vma pc);
};

void initDebugInfo();
void termDebugInfo();
void getDebugInfo(DebugInfo* DI);
