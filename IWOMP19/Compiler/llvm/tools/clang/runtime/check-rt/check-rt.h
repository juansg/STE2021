
#ifdef __cplusplus
extern "C" {
#endif

void __initCheckRuntime() __attribute__((nothrow));
void __termCheckRuntime() __attribute__((nothrow));

void __enterParallelRegion(int tID, int numThreads) __attribute__((nothrow));
void __exitParallelRegion() __attribute__((nothrow));

void __function_entry(int lineNumber, void* pc) __attribute__((nothrow));
void __function_exit() __attribute__((nothrow));

void __start_iter_prof(unsigned long iter) __attribute__((nothrow));
void __stop_iter_prof(unsigned long iter) __attribute__((nothrow));

void __check_dependence(const void *addr, unsigned char type) __attribute__((nothrow));

#ifdef __cplusplus
} // extern "C"
#endif
