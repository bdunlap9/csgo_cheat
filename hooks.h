#pragma once

#include <Windows.h>
#include <vector>

namespace hooks {

    bool AddVEHHook(void* originalFunction, void* hookedFunction);
    LONG CALLBACK VehHandler(EXCEPTION_POINTERS* exceptionInfo);
    void InitializeVEHHooks();
    void RemoveVEHHooks();
    void MyHookedFunction();



}
