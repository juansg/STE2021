#pragma once

void collectDependencies(unsigned long numIterations,
    void* enterPC, void* exitPC);
void reportDependencies();
void MemRead(void *addr, unsigned long iterationID, void *PC);
void MemWrite(void *addr, unsigned long iterationID, void *PC);
