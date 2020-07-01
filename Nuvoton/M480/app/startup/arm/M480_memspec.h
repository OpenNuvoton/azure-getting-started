/* Define memory specification for target application. */
#if !defined(ROM_APP_START)
#define ROM_APP_START               0x0
#endif

#if !defined(ROM_APP_SIZE)
#define ROM_APP_SIZE                0x80000
#endif

#if !defined(RAM_APP_START)
#define RAM_APP_START               0x20000000
#endif

#if !defined(RAM_APP_SIZE)
#define RAM_APP_SIZE                0x28000
#endif
